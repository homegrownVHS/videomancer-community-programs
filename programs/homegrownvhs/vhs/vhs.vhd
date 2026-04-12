-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: vhs.vhd - VHS Degradation Program for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-sdk
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Program Name:        VHS
-- Author:              Adam Pflanzer
-- Overview:
--   Simulates VHS tape playback artifacts: horizontal chroma blur, luma noise,
--   horizontal tracking jitter via BRAM scanline buffer, head-switching noise
--   band at the bottom of the frame, random dropout scanlines, and color
--   desaturation. Y-only tracking jitter uses a single BRAM scanline buffer
--   to displace luma horizontally per line; chroma passes through an IIR
--   lowpass filter for the characteristic VHS color smearing.
--
-- Resources:
--   5 BRAM (Y-only scanline buffer, 10-bit x 2048), ~2000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input reg + counters + IIR feed):  1 clock  -> T+1
--   Stage 1 (BRAM write/read + IIR output):     1 clock  -> T+2
--   Stage 2 (BRAM output + noise + zones):       1 clock  -> T+3
--   Stage 3 (apply noise + dropout + clamp):     1 clock  -> T+4
--   Stage 4 (output register):                   1 clock  -> T+5
--   interpolator_u x3 (wet/dry mix):             4 clocks -> T+9
--   Total: 9 clocks
--
-- Submodules:
--   variable_filter_s x2: IIR lowpass for chroma blur, 1 cycle each
--   lfsr16 x1: pseudo-random noise generation, free-running
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Chroma Blur (IIR cutoff for U/V)
--   Pot 2  (registers_in(1)):   Noise (overall noise level)
--   Pot 3  (registers_in(2)):   Tracking (horizontal jitter magnitude)
--   Pot 4  (registers_in(3)):   Head Switch (noise band at bottom of frame)
--   Pot 5  (registers_in(4)):   Dropout (random scanline dropout rate)
--   Pot 6  (registers_in(5)):   Color Loss (desaturation amount)
--   Tog 7  (registers_in(6)(0)): Tracking Mode (Scattered / Band)
--   Tog 8  (registers_in(6)(1)): Dropout Style (Snow / Black)
--   Tog 9  (registers_in(6)(2)): Color Noise (Mono / Color)
--   Tog 10 (registers_in(6)(3)): Sync Warp (Off / On)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 5 (inline stages)
--   C_SYNC_DELAY_CLKS       = 9 (total, including trailing interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture vhs of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 5;
    constant C_SYNC_DELAY_CLKS       : integer := 9;  -- 5 + 4 (interpolator)
    constant C_BUF_DEPTH             : integer := 11;  -- 2048 entries
    constant C_BUF_SIZE              : integer := 2**C_BUF_DEPTH;

    ---------------------------------------------------------------------------
    -- BRAM for Y scanline buffer (tracking jitter)
    ---------------------------------------------------------------------------
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal bram_y : t_bram := (others => (others => '0'));

    ---------------------------------------------------------------------------
    -- Control signals (from registers)
    ---------------------------------------------------------------------------
    signal s_chroma_blur    : unsigned(7 downto 0);   -- IIR cutoff (upper 8 bits of knob1)
    signal s_noise_amount   : unsigned(9 downto 0);   -- 0-1023
    signal s_tracking_amt   : unsigned(9 downto 0);   -- 0-1023
    signal s_head_switch    : unsigned(9 downto 0);   -- 0-1023
    signal s_dropout_rate   : unsigned(9 downto 0);   -- 0-1023
    signal s_color_loss     : unsigned(9 downto 0);   -- 0-1023
    signal s_tracking_band  : std_logic;              -- 0=scattered, 1=band
    signal s_dropout_black  : std_logic;              -- 0=snow, 1=black
    signal s_color_noise    : std_logic;              -- 0=mono, 1=color
    signal s_sync_warp      : std_logic;              -- 0=off, 1=sync warp on
    signal s_bypass_enable  : std_logic;
    signal s_mix_amount     : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Pixel and line counters
    ---------------------------------------------------------------------------
    signal s_pixel_count    : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal s_line_count     : unsigned(9 downto 0) := (others => '0');
    signal s_prev_avid      : std_logic := '0';
    signal s_prev_hsync_n   : std_logic := '1';
    signal s_prev_vsync_n   : std_logic := '1';

    ---------------------------------------------------------------------------
    -- Per-line jitter offset (sampled at start of each active line)
    ---------------------------------------------------------------------------
    signal s_jitter_offset  : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Tracking band position (slow accumulator, wraps per frame)
    ---------------------------------------------------------------------------
    signal s_band_pos       : unsigned(9 downto 0) := (others => '0');
    signal s_band_accum     : unsigned(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Sync warp state (smooth horizontal displacement)
    ---------------------------------------------------------------------------
    signal s_warp_phase     : unsigned(9 downto 0) := (others => '0');
    signal s_warp_frame_ofs : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Per-line dropout flag (sampled at start of each active line)
    ---------------------------------------------------------------------------
    signal s_dropout_active : std_logic := '0';

    ---------------------------------------------------------------------------
    -- LFSR output
    ---------------------------------------------------------------------------
    signal s_lfsr_out       : std_logic_vector(15 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 0 outputs
    ---------------------------------------------------------------------------
    signal s0_y             : unsigned(9 downto 0) := (others => '0');
    signal s0_u             : unsigned(9 downto 0) := (others => '0');
    signal s0_v             : unsigned(9 downto 0) := (others => '0');
    signal s0_avid          : std_logic := '0';
    signal s0_pixel_count   : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- IIR filter I/O
    ---------------------------------------------------------------------------
    signal s_filter_u_in    : signed(10 downto 0);  -- 11-bit signed (U - 512)
    signal s_filter_v_in    : signed(10 downto 0);
    signal s_filter_u_lp    : signed(10 downto 0);
    signal s_filter_v_lp    : signed(10 downto 0);
    signal s_filter_u_valid : std_logic;
    signal s_filter_v_valid : std_logic;

    ---------------------------------------------------------------------------
    -- Stage 1 outputs
    ---------------------------------------------------------------------------
    signal s1_bram_rd_y     : std_logic_vector(9 downto 0) := (others => '0');
    signal s1_blur_u        : unsigned(9 downto 0) := (others => '0');
    signal s1_blur_v        : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2 outputs
    ---------------------------------------------------------------------------
    signal s2_jittered_y    : unsigned(9 downto 0) := (others => '0');
    signal s2_blur_u        : unsigned(9 downto 0) := (others => '0');
    signal s2_blur_v        : unsigned(9 downto 0) := (others => '0');
    signal s2_y_noise       : signed(10 downto 0) := (others => '0');
    signal s2_uv_noise      : signed(10 downto 0) := (others => '0');
    signal s2_in_head_zone  : std_logic := '0';
    signal s2_dropout       : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 3 outputs
    ---------------------------------------------------------------------------
    signal s3_y             : unsigned(9 downto 0) := (others => '0');
    signal s3_u             : unsigned(9 downto 0) := (others => '0');
    signal s3_v             : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 4 outputs (wet signal for interpolator)
    ---------------------------------------------------------------------------
    signal s4_y             : unsigned(9 downto 0) := (others => '0');
    signal s4_u             : unsigned(9 downto 0) := (others => '0');
    signal s4_v             : unsigned(9 downto 0) := (others => '0');
    signal s4_valid         : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y          : unsigned(9 downto 0);
    signal s_mix_y_valid    : std_logic;
    signal s_mix_u          : unsigned(9 downto 0);
    signal s_mix_u_valid    : std_logic;
    signal s_mix_v          : unsigned(9 downto 0);
    signal s_mix_v_valid    : std_logic;

    ---------------------------------------------------------------------------
    -- Bypass / sync delay path
    ---------------------------------------------------------------------------
    signal s_y_delayed      : std_logic_vector(9 downto 0);
    signal s_u_delayed      : std_logic_vector(9 downto 0);
    signal s_v_delayed      : std_logic_vector(9 downto 0);

    ---------------------------------------------------------------------------
    -- Dry tap (input delayed by C_PROCESSING_DELAY_CLKS for interpolator)
    ---------------------------------------------------------------------------
    signal s_dry_y          : unsigned(9 downto 0);
    signal s_dry_u          : unsigned(9 downto 0);
    signal s_dry_v          : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Saturating add/sub helpers
    ---------------------------------------------------------------------------
    function sat_add_s(val : unsigned(9 downto 0); noise : signed(10 downto 0))
        return unsigned is
        variable v_sum : signed(11 downto 0);
    begin
        v_sum := signed(resize(val, 12)) + resize(noise, 12);
        if v_sum < 0 then
            return to_unsigned(0, 10);
        elsif v_sum > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v_sum(9 downto 0));
        end if;
    end function;

    function clamp_signed_to_u10(val : signed(10 downto 0))
        return unsigned is
    begin
        if val < 0 then
            return to_unsigned(0, 10);
        elsif val > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(val(9 downto 0));
        end if;
    end function;

begin
    ---------------------------------------------------------------------------
    -- Register mapping (concurrent)
    ---------------------------------------------------------------------------
    -- Chroma blur: knob at 0 = no blur, at 1023 = max blur
    -- IIR cutoff: higher value = more smoothing. Map knob directly.
    s_chroma_blur    <= unsigned(registers_in(0)(9 downto 2));
    s_noise_amount   <= unsigned(registers_in(1));
    s_tracking_amt   <= unsigned(registers_in(2));
    s_head_switch    <= unsigned(registers_in(3));
    s_dropout_rate   <= unsigned(registers_in(4));
    s_color_loss     <= unsigned(registers_in(5));
    s_tracking_band  <= registers_in(6)(0);
    s_dropout_black  <= registers_in(6)(1);
    s_color_noise    <= registers_in(6)(2);
    s_sync_warp      <= registers_in(6)(3);
    s_bypass_enable  <= registers_in(6)(4);
    s_mix_amount     <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- LFSR noise generator (free-running)
    ---------------------------------------------------------------------------
    u_lfsr : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"ACE1",
            load   => '0',
            q      => s_lfsr_out
        );

    ---------------------------------------------------------------------------
    -- IIR filters for chroma blur (U and V channels)
    -- Feed data_in directly (combinational) so filter output is ready at S1
    ---------------------------------------------------------------------------
    s_filter_u_in <= signed(resize(unsigned(data_in.u), 11)) - to_signed(512, 11);
    s_filter_v_in <= signed(resize(unsigned(data_in.v), 11)) - to_signed(512, 11);

    u_filter_u : entity work.variable_filter_s
        generic map (G_WIDTH => 11)
        port map (
            clk       => clk,
            enable    => data_in.avid,
            a         => s_filter_u_in,
            cutoff    => s_chroma_blur,
            low_pass  => s_filter_u_lp,
            high_pass => open,
            valid     => s_filter_u_valid
        );

    u_filter_v : entity work.variable_filter_s
        generic map (G_WIDTH => 11)
        port map (
            clk       => clk,
            enable    => data_in.avid,
            a         => s_filter_v_in,
            cutoff    => s_chroma_blur,
            low_pass  => s_filter_v_lp,
            high_pass => open,
            valid     => s_filter_v_valid
        );

    ---------------------------------------------------------------------------
    -- Stage 0: Input register, counters, per-line jitter sample
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
        variable v_jitter_raw   : unsigned(25 downto 0);  -- 16-bit lfsr * 10-bit tracking
        variable v_jitter_shift : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_band_dist    : unsigned(9 downto 0);
        variable v_tracking_scaled : unsigned(9 downto 0);
        variable v_warp_combined : unsigned(9 downto 0);
        variable v_warp_triangle : unsigned(9 downto 0);
        variable v_warp_product  : unsigned(19 downto 0);  -- 10-bit * 10-bit
        variable v_warp_offset   : unsigned(C_BUF_DEPTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Register input data
            s0_y    <= unsigned(data_in.y);
            s0_u    <= unsigned(data_in.u);
            s0_v    <= unsigned(data_in.v);
            s0_avid <= data_in.avid;

            -- Edge detection
            s_prev_avid    <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Pixel counter: reset at start of active line, increment during active
            if data_in.avid = '1' and s_prev_avid = '0' then
                -- Rising edge of avid: start of active pixels
                s_pixel_count <= (others => '0');
            elsif data_in.avid = '1' then
                s_pixel_count <= s_pixel_count + 1;
            end if;

            -- Line counter: reset on vsync falling edge, increment on hsync falling edge
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line_count <= (others => '0');
                -- Advance tracking band position per frame
                s_band_accum <= s_band_accum + resize(unsigned(s_lfsr_out(3 downto 0)), 16) + 1;
                s_band_pos   <= s_band_accum(15 downto 6);
                -- Advance warp phase offset per frame (slow organic drift)
                s_warp_frame_ofs <= s_warp_frame_ofs
                                  + resize(unsigned(s_lfsr_out(4 downto 0)), 10);
            elsif data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_line_count <= s_line_count + 1;
                -- Advance warp phase per line (~2 undulations per field)
                s_warp_phase <= s_warp_phase + 8;
            end if;

            -- Per-line jitter: sample at avid rising edge
            if data_in.avid = '1' and s_prev_avid = '0' then
                -- Compute base jitter from tracking mode
                v_jitter_shift := (others => '0');
                if s_tracking_amt /= 0 then
                    if s_tracking_band = '0' then
                        -- Scattered mode: every line gets random jitter
                        v_jitter_raw := unsigned(s_lfsr_out) * s_tracking_amt;
                        v_jitter_shift := v_jitter_raw(25 downto 25 - C_BUF_DEPTH + 1);
                    else
                        -- Band mode: only lines near band position get jitter
                        if s_line_count >= s_band_pos then
                            v_band_dist := s_line_count - s_band_pos;
                        else
                            v_band_dist := s_band_pos - s_line_count;
                        end if;
                        -- Within ~32 lines of band center: apply jitter
                        if v_band_dist < 32 then
                            v_tracking_scaled := s_tracking_amt;
                            v_jitter_raw := unsigned(s_lfsr_out)
                                          * v_tracking_scaled;
                            v_jitter_shift := v_jitter_raw(25 downto 25 - C_BUF_DEPTH + 1);
                        end if;
                    end if;
                end if;

                -- Compute sync warp: smooth triangle-wave displacement
                v_warp_offset := (others => '0');
                if s_sync_warp = '1' and s_tracking_amt /= 0 then
                    v_warp_combined := s_warp_phase + s_warp_frame_ofs;
                    -- Triangle wave: ramp up 0→1023 then fold back
                    if v_warp_combined(9) = '0' then
                        v_warp_triangle := v_warp_combined(8 downto 0) & '0';
                    else
                        v_warp_triangle := (to_unsigned(511, 9)
                                          - v_warp_combined(8 downto 0)) & '0';
                    end if;
                    -- Scale by tracking amount
                    v_warp_product := v_warp_triangle * s_tracking_amt;
                    v_warp_offset  := resize(v_warp_product(19 downto 10), C_BUF_DEPTH);
                end if;

                -- Combine jitter + warp
                s_jitter_offset <= v_jitter_shift + v_warp_offset;

                -- Per-line dropout decision: compare LFSR to dropout threshold
                -- Higher dropout_rate = more frequent dropouts
                if unsigned(s_lfsr_out(9 downto 0)) < s_dropout_rate then
                    s_dropout_active <= '1';
                else
                    s_dropout_active <= '0';
                end if;
            end if;

            -- Store pixel count for stage 1 addressing
            s0_pixel_count <= s_pixel_count;
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: BRAM write + read, capture IIR filter outputs
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_rd_addr : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_blur_u_sum : signed(10 downto 0);
        variable v_blur_v_sum : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- BRAM write: store current pixel's Y
            if s0_avid = '1' then
                bram_y(to_integer(s0_pixel_count)) <= std_logic_vector(s0_y);
            end if;

            -- BRAM read: shifted address for tracking jitter
            v_rd_addr := s0_pixel_count - s_jitter_offset;
            s1_bram_rd_y <= bram_y(to_integer(v_rd_addr));

            -- Capture IIR filter output: low_pass + 512 back to unsigned
            v_blur_u_sum := s_filter_u_lp + to_signed(512, 11);
            v_blur_v_sum := s_filter_v_lp + to_signed(512, 11);
            s1_blur_u <= clamp_signed_to_u10(v_blur_u_sum);
            s1_blur_v <= clamp_signed_to_u10(v_blur_v_sum);
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 2: BRAM read output + noise computation + zone detection
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_noise_raw     : unsigned(19 downto 0);  -- 10-bit * 10-bit
        variable v_noise_val     : signed(10 downto 0);
        variable v_uv_noise_raw  : unsigned(19 downto 0);
        variable v_uv_noise_val  : signed(10 downto 0);
        variable v_hs_lines      : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- Jittered Y from BRAM
            s2_jittered_y <= unsigned(s1_bram_rd_y);

            -- Align blurred U/V (1 extra clock delay to match BRAM Y path)
            s2_blur_u <= s1_blur_u;
            s2_blur_v <= s1_blur_v;

            -- Compute Y noise: lfsr(9:0) centered around 0, scaled by noise_amount
            -- noise = ((lfsr - 512) * noise_amount) >> 10
            v_noise_raw := unsigned(s_lfsr_out(9 downto 0)) * s_noise_amount;
            v_noise_val := signed(resize(v_noise_raw(19 downto 10), 11))
                         - to_signed(to_integer(s_noise_amount) / 2, 11);
            s2_y_noise <= v_noise_val;

            -- UV noise (optionally same as Y noise or different LFSR bits)
            if s_color_noise = '1' then
                v_uv_noise_raw := unsigned(s_lfsr_out(15 downto 6)) * s_noise_amount;
                v_uv_noise_val := signed(resize(v_uv_noise_raw(19 downto 10), 11))
                                - to_signed(to_integer(s_noise_amount) / 2, 11);
                s2_uv_noise <= v_uv_noise_val;
            else
                s2_uv_noise <= (others => '0');  -- mono noise: no UV noise
            end if;

            -- Head-switch zone detection (bottom of frame)
            -- Head switch size: scaled from 0 (none) to ~64 lines max
            v_hs_lines := "00" & s_head_switch(9 downto 2);  -- 0-255 range for line count

            if s_line_count > (to_unsigned(1023, 10) - v_hs_lines) and v_hs_lines /= 0 then
                s2_in_head_zone <= '1';
            else
                s2_in_head_zone <= '0';
            end if;

            -- Dropout (sampled per-line in stage 0)
            s2_dropout <= s_dropout_active;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 3: Apply noise, dropout, head-switch, color loss, clamp
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_y_out   : unsigned(9 downto 0);
        variable v_u_out   : unsigned(9 downto 0);
        variable v_v_out   : unsigned(9 downto 0);
        variable v_hs_noise_y  : signed(10 downto 0);
        variable v_hs_noise_uv : signed(10 downto 0);
        variable v_color_scale : unsigned(9 downto 0);
        variable v_u_centered  : signed(10 downto 0);
        variable v_v_centered  : signed(10 downto 0);
        variable v_u_scaled    : signed(21 downto 0);
        variable v_v_scaled    : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            if s2_dropout = '1' then
                -- Dropout line: replace with snow or black
                if s_dropout_black = '1' then
                    v_y_out := (others => '0');
                    v_u_out := to_unsigned(512, 10);
                    v_v_out := to_unsigned(512, 10);
                else
                    -- Snow: use LFSR as brightness
                    v_y_out := unsigned(s_lfsr_out(9 downto 0));
                    v_u_out := to_unsigned(512, 10);
                    v_v_out := to_unsigned(512, 10);
                end if;
            else
                -- Normal processing: start with jittered Y and blurred U/V
                v_y_out := s2_jittered_y;
                v_u_out := s2_blur_u;
                v_v_out := s2_blur_v;

                -- Add general noise to Y
                v_y_out := sat_add_s(v_y_out, s2_y_noise);

                -- Add noise to U/V if color noise enabled
                v_u_out := sat_add_s(v_u_out, s2_uv_noise);
                v_v_out := sat_add_s(v_v_out, s2_uv_noise);

                -- Head-switch zone: add heavy noise
                if s2_in_head_zone = '1' then
                    v_hs_noise_y := signed(resize(unsigned(s_lfsr_out(9 downto 0)), 11))
                                  - to_signed(512, 11);
                    v_hs_noise_uv := signed(resize(unsigned(s_lfsr_out(15 downto 6)), 11))
                                   - to_signed(512, 11);
                    v_y_out := sat_add_s(v_y_out, v_hs_noise_y);
                    v_u_out := sat_add_s(v_u_out, v_hs_noise_uv);
                    v_v_out := sat_add_s(v_v_out, v_hs_noise_uv);
                end if;

                -- Color loss: blend U/V toward 512 (neutral)
                -- output_u = 512 + ((blurred_u - 512) * (1023 - color_loss)) >> 10
                if s_color_loss /= 0 then
                    v_color_scale := to_unsigned(1023, 10) - s_color_loss;
                    v_u_centered := signed(resize(v_u_out, 11)) - to_signed(512, 11);
                    v_v_centered := signed(resize(v_v_out, 11)) - to_signed(512, 11);
                    v_u_scaled := v_u_centered * signed(resize(v_color_scale, 11));
                    v_v_scaled := v_v_centered * signed(resize(v_color_scale, 11));
                    v_u_out := clamp_signed_to_u10(
                        resize(v_u_scaled(20 downto 10), 11) + to_signed(512, 11));
                    v_v_out := clamp_signed_to_u10(
                        resize(v_v_scaled(20 downto 10), 11) + to_signed(512, 11));
                end if;
            end if;

            s3_y <= v_y_out;
            s3_u <= v_u_out;
            s3_v <= v_v_out;
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Stage 4: Output register (wet signal ready for interpolator)
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage4 : process(clk)
    begin
        if rising_edge(clk) then
            s4_y     <= s3_y;
            s4_u     <= s3_u;
            s4_v     <= s3_v;
            -- Valid derived from delayed avid (through processing chain)
            -- We use a simple shift register for valid tracking
        end if;
    end process p_stage4;

    ---------------------------------------------------------------------------
    -- Valid pipeline (track avid through processing stages)
    ---------------------------------------------------------------------------
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s4_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    ---------------------------------------------------------------------------
    -- Interpolators: Dry/wet mix
    -- Latency: 4 clocks each (parallel)
    ---------------------------------------------------------------------------
    u_interp_y : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s4_valid,
            a      => s_dry_y,
            b      => s4_y,
            t      => s_mix_amount,
            result => s_mix_y,
            valid  => s_mix_y_valid
        );

    u_interp_u : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s4_valid,
            a      => s_dry_u,
            b      => s4_u,
            t      => s_mix_amount,
            result => s_mix_u,
            valid  => s_mix_u_valid
        );

    u_interp_v : entity work.interpolator_u
        generic map (
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map (
            clk    => clk,
            enable => s4_valid,
            a      => s_dry_v,
            b      => s4_v,
            t      => s_mix_amount,
            result => s_mix_v,
            valid  => s_mix_v_valid
        );

    ---------------------------------------------------------------------------
    -- Delay lines: sync signals (full pipeline depth) and dry tap + bypass
    ---------------------------------------------------------------------------
    p_delay : process(clk)
        -- Sync delay (C_SYNC_DELAY_CLKS = 9)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (C_SYNC_DELAY_CLKS = 9)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (C_PROCESSING_DELAY_CLKS = 5, aligned with wet signal)
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Sync delay shift registers
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);

            -- Bypass data delay (full pipeline depth)
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap delay (aligned with end of inline stages, for interpolator)
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Output delayed signals
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            s_y_delayed <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_u_delayed <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_v_delayed <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    ---------------------------------------------------------------------------
    -- Output mux: bypass / processed
    ---------------------------------------------------------------------------
    data_out.y <= s_y_delayed when s_bypass_enable = '1' else
                  std_logic_vector(s_mix_y);

    data_out.u <= s_u_delayed when s_bypass_enable = '1' else
                  std_logic_vector(s_mix_u);

    data_out.v <= s_v_delayed when s_bypass_enable = '1' else
                  std_logic_vector(s_mix_v);

end architecture vhs;
