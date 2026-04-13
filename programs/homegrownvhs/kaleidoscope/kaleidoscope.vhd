-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: kaleidoscope.vhd - Kaleidoscope Program for Videomancer
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
-- Program Name:        Kaleidoscope
-- Author:              Adam Pflanzer
-- Overview:
--   Horizontal N-fold mirror/tile engine with diagonal tilt and rotation
--   animation. Divides each scanline into N power-of-2 segments and
--   mirrors or tiles one wedge across the others using BRAM line buffers.
--   Tilt shifts the fold axis per scanline to create diagonal mirror lines.
--   Rotation animates the fold pattern via a DDS phase accumulator.
--   Optional color cycling hue-shifts each segment for psychedelic effects.
--
-- Resources:
--   15 BRAM (3x line buffer, 10-bit x 2048), ~1500 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + counters + BRAM write):  1 clock  -> T+1
--   Stage 1 (fold addr compute + BRAM read launch):    1 clock  -> T+2
--   Stage 2 (brightness multiply + color cycling MUX): 1 clock  -> T+3
--   Stage 3 (saturation multiply + clamp):             1 clock  -> T+4
--   interpolator_u x3 (wet/dry mix):                   4 clocks -> T+8
--   Total: 8 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Segments (steps_4: 64/128/256/512 px width)
--   Pot 2  (registers_in(1)):   Tilt (diagonal fold shift per line)
--   Pot 3  (registers_in(2)):   Spin (rotation animation speed)
--   Pot 4  (registers_in(3)):   Offset (manual phase offset)
--   Pot 5  (registers_in(4)):   Brightness (output Y level)
--   Pot 6  (registers_in(5)):   Saturation (output color intensity)
--   Tog 7  (registers_in(6)(0)): Mirror / Tile mode
--   Tog 8  (registers_in(6)(1)): Color Cycle (hue shift per segment)
--   Tog 9  (registers_in(6)(2)): (unused)
--   Tog 10 (registers_in(6)(3)): (unused)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 3 (inline stages)
--   C_SYNC_DELAY_CLKS       = 7 (3 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture kaleidoscope of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 4;
    constant C_SYNC_DELAY_CLKS       : integer := 8;  -- 4 + 4 (interpolator)
    constant C_BUF_DEPTH             : integer := 11;  -- 2048 entries

    ---------------------------------------------------------------------------
    -- BRAM line buffers (Y, U, V)
    ---------------------------------------------------------------------------
    type t_line_buf is array (0 to 2047) of unsigned(9 downto 0);
    signal s_line_y : t_line_buf := (others => (others => '0'));
    signal s_line_u : t_line_buf := (others => (others => '0'));
    signal s_line_v : t_line_buf := (others => (others => '0'));

    ---------------------------------------------------------------------------
    -- Control signals
    ---------------------------------------------------------------------------
    signal s_seg_sel    : unsigned(1 downto 0);  -- Knob 1: segment width select
    signal s_tilt       : unsigned(9 downto 0);  -- Knob 2: tilt rate
    signal s_spin       : unsigned(9 downto 0);  -- Knob 3: rotation speed
    signal s_phase_off  : unsigned(9 downto 0);  -- Knob 4: manual phase offset
    signal s_brightness : unsigned(9 downto 0);  -- Knob 5: output brightness
    signal s_saturation : unsigned(9 downto 0);  -- Knob 6: output saturation
    signal s_mirror     : std_logic;             -- Toggle 7: mirror vs tile
    signal s_color_cyc  : std_logic;             -- Toggle 8: color cycling
    signal s_bypass     : std_logic;             -- Toggle 11: bypass
    signal s_mix        : unsigned(9 downto 0);  -- Fader: dry/wet

    ---------------------------------------------------------------------------
    -- Pixel / line counters
    ---------------------------------------------------------------------------
    signal s_pixel_count  : unsigned(10 downto 0) := (others => '0');
    signal s_line_count   : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_avid    : std_logic := '0';
    signal s_prev_vsync_n : std_logic := '1';

    ---------------------------------------------------------------------------
    -- Rotation DDS accumulator (20-bit, updated per frame)
    ---------------------------------------------------------------------------
    signal s_rotation_phase : unsigned(19 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Phase computation
    ---------------------------------------------------------------------------
    signal s_total_phase : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Segment parameters (derived from seg_sel)
    ---------------------------------------------------------------------------
    signal s_seg_shift : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal s_seg_mask  : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 0 outputs (registered input, write point)
    ---------------------------------------------------------------------------
    signal s0_y    : unsigned(9 downto 0) := (others => '0');
    signal s0_u    : unsigned(9 downto 0) := (others => '0');
    signal s0_v    : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Fold address (combinational → BRAM read port)
    ---------------------------------------------------------------------------
    signal s_fold_addr : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 1 outputs (BRAM read launched)
    ---------------------------------------------------------------------------
    signal s1_rd_y : unsigned(9 downto 0) := (others => '0');
    signal s1_rd_u : unsigned(9 downto 0) := (others => '0');
    signal s1_rd_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2 outputs (brightness + color cycling MUX)
    ---------------------------------------------------------------------------
    signal s2_y       : unsigned(9 downto 0) := (others => '0');
    signal s2_u_signed : signed(10 downto 0) := (others => '0');
    signal s2_v_signed : signed(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3 outputs (saturation multiply + clamp)
    ---------------------------------------------------------------------------
    signal s3_y : unsigned(9 downto 0) := (others => '0');
    signal s3_u : unsigned(9 downto 0) := (others => '0');
    signal s3_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Segment index (for color cycling)
    ---------------------------------------------------------------------------
    signal s1_seg_idx : unsigned(3 downto 0) := (others => '0');
    signal s2_seg_idx : unsigned(3 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Dry tap / bypass data / valid
    ---------------------------------------------------------------------------
    signal s_dry_y : unsigned(9 downto 0);
    signal s_dry_u : unsigned(9 downto 0);
    signal s_dry_v : unsigned(9 downto 0);

    signal s_bypass_y : std_logic_vector(9 downto 0);
    signal s_bypass_u : std_logic_vector(9 downto 0);
    signal s_bypass_v : std_logic_vector(9 downto 0);

    signal s_interp_valid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;

begin
    ---------------------------------------------------------------------------
    -- Register mapping
    ---------------------------------------------------------------------------
    s_seg_sel    <= unsigned(registers_in(0)(9 downto 8));  -- top 2 bits: 0-3
    s_tilt       <= unsigned(registers_in(1));
    s_spin       <= unsigned(registers_in(2));
    s_phase_off  <= unsigned(registers_in(3));
    s_brightness <= unsigned(registers_in(4));
    s_saturation <= unsigned(registers_in(5));
    s_mirror    <= registers_in(6)(0);
    s_color_cyc <= registers_in(6)(1);
    s_bypass    <= registers_in(6)(4);
    s_mix       <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- Segment width parameters (combinational from seg_sel)
    -- seg_sel 0: shift=6, width=64   (many narrow segments)
    -- seg_sel 1: shift=7, width=128
    -- seg_sel 2: shift=8, width=256
    -- seg_sel 3: shift=9, width=512  (few wide segments)
    ---------------------------------------------------------------------------
    s_seg_shift <= to_unsigned(6, 4) + resize(s_seg_sel, 4);

    -- Mask for local position within segment (seg_width - 1)
    process(s_seg_shift)
    begin
        case s_seg_shift is
            when "0110" => s_seg_mask <= "00000111111";  -- 63
            when "0111" => s_seg_mask <= "00001111111";  -- 127
            when "1000" => s_seg_mask <= "00011111111";  -- 255
            when "1001" => s_seg_mask <= "00111111111";  -- 511
            when others => s_seg_mask <= "00001111111";  -- 127 default
        end case;
    end process;

    ---------------------------------------------------------------------------
    -- BRAM write port: write current pixel at pixel_count address
    ---------------------------------------------------------------------------
    p_bram_write : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_line_y(to_integer(s_pixel_count)) <= unsigned(data_in.y);
                s_line_u(to_integer(s_pixel_count)) <= unsigned(data_in.u);
                s_line_v(to_integer(s_pixel_count)) <= unsigned(data_in.v);
            end if;
        end if;
    end process p_bram_write;

    ---------------------------------------------------------------------------
    -- Fold address computation (combinational)
    -- adjusted_pixel = pixel_count + total_phase
    -- local_pos = adjusted_pixel AND seg_mask
    -- seg_parity = bit above local_pos (odd segments → mirror)
    -- source = local_pos (even) or (seg_mask - local_pos) (odd + mirror)
    ---------------------------------------------------------------------------
    p_fold_addr : process(s_pixel_count, s_total_phase, s_seg_mask, s_seg_shift, s_mirror)
        variable v_adjusted  : unsigned(10 downto 0);
        variable v_local     : unsigned(10 downto 0);
        variable v_parity    : std_logic;
    begin
        v_adjusted := s_pixel_count + s_total_phase;
        v_local    := v_adjusted and s_seg_mask;

        -- Extract parity bit (segment index bit 0)
        case s_seg_shift is
            when "0110" => v_parity := v_adjusted(6);
            when "0111" => v_parity := v_adjusted(7);
            when "1000" => v_parity := v_adjusted(8);
            when "1001" => v_parity := v_adjusted(9);
            when others => v_parity := v_adjusted(7);
        end case;

        -- Mirror: odd segments read in reverse
        if s_mirror = '1' and v_parity = '1' then
            s_fold_addr <= s_seg_mask - v_local;
        else
            s_fold_addr <= v_local;
        end if;
    end process p_fold_addr;

    ---------------------------------------------------------------------------
    -- BRAM read port: read from fold_addr (registered, 1 clock latency)
    ---------------------------------------------------------------------------
    p_bram_read : process(clk)
    begin
        if rising_edge(clk) then
            s1_rd_y <= s_line_y(to_integer(s_fold_addr));
            s1_rd_u <= s_line_u(to_integer(s_fold_addr));
            s1_rd_v <= s_line_v(to_integer(s_fold_addr));
        end if;
    end process p_bram_read;

    ---------------------------------------------------------------------------
    -- Stage 0: Register input, pixel/line counters, frame detect, phase
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
        variable v_tilt_prod : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            s_prev_avid    <= data_in.avid;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Pixel counter
            if data_in.avid = '1' then
                s_pixel_count <= s_pixel_count + 1;
            end if;

            -- End of active line: reset pixel counter, advance line
            if data_in.avid = '0' and s_prev_avid = '1' then
                s_pixel_count <= (others => '0');
                s_line_count  <= s_line_count + 1;
            end if;

            -- Start of new frame: reset line counter, update rotation DDS
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line_count    <= (others => '0');
                s_rotation_phase <= s_rotation_phase + resize(s_spin, 20);
            end if;

            -- Total phase = rotation + tilt per line
            -- tilt_prod = line * tilt (10 x 10 = 20 bit)
            -- Take top 11 bits of tilt product + top 11 bits of rotation
            v_tilt_prod  := s_line_count * s_tilt;
            s_total_phase <= v_tilt_prod(19 downto 9)
                          + s_rotation_phase(19 downto 9)
                          + resize(s_phase_off, 11);
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: BRAM read result arrives (from p_bram_read registered process)
    -- Also pipeline the segment index for color cycling
    -- Latency: 1 clock (BRAM read was launched by fold_addr in prev cycle)
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_adjusted : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- Compute segment index for color cycling (delayed to match BRAM read)
            v_adjusted := s_pixel_count + s_total_phase;
            case s_seg_shift is
                when "0110" => s1_seg_idx <= v_adjusted(9 downto 6);
                when "0111" => s1_seg_idx <= v_adjusted(10 downto 7);
                when "1000" => s1_seg_idx <= '0' & v_adjusted(10 downto 8);
                when "1001" => s1_seg_idx <= "00" & v_adjusted(10 downto 9);
                when others => s1_seg_idx <= v_adjusted(10 downto 7);
            end case;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 2: Brightness multiply + color cycling MUX
    -- BRAM read data (s1_rd_y/u/v) is available from p_bram_read
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_y_prod   : unsigned(19 downto 0);
        variable v_u_signed : signed(10 downto 0);
        variable v_v_signed : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s2_seg_idx <= s1_seg_idx;

            -- Brightness: scale Y by knob 5 (0=black, 512=half, 1023=full)
            v_y_prod := s1_rd_y * s_brightness;
            s2_y <= v_y_prod(19 downto 10);

            -- Color cycling MUX: select and center U/V for saturation stage
            if s_color_cyc = '1' then
                case s1_seg_idx(1 downto 0) is
                    when "00" =>
                        v_u_signed := signed('0' & s1_rd_u) - 512;
                        v_v_signed := signed('0' & s1_rd_v) - 512;
                    when "01" =>
                        v_u_signed := signed('0' & s1_rd_v) - 512;
                        v_v_signed := 512 - signed('0' & s1_rd_u);
                    when "10" =>
                        v_u_signed := 512 - signed('0' & s1_rd_u);
                        v_v_signed := 512 - signed('0' & s1_rd_v);
                    when "11" =>
                        v_u_signed := signed('0' & s1_rd_v) - 512;
                        v_v_signed := signed('0' & s1_rd_u) - 512;
                    when others =>
                        v_u_signed := signed('0' & s1_rd_u) - 512;
                        v_v_signed := signed('0' & s1_rd_v) - 512;
                end case;
            else
                v_u_signed := signed('0' & s1_rd_u) - 512;
                v_v_signed := signed('0' & s1_rd_v) - 512;
            end if;

            s2_u_signed <= v_u_signed;
            s2_v_signed <= v_v_signed;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 3: Saturation multiply + clamp
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_u_prod : signed(21 downto 0);
        variable v_v_prod : signed(21 downto 0);
        variable v_u_sat  : signed(10 downto 0);
        variable v_v_sat  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- Pass brightness-scaled Y through
            s3_y <= s2_y;

            -- Apply saturation scaling from registered signed values
            v_u_prod := s2_u_signed * signed('0' & s_saturation);
            v_v_prod := s2_v_signed * signed('0' & s_saturation);
            v_u_sat := v_u_prod(20 downto 10);
            v_v_sat := v_v_prod(20 downto 10);

            -- Clamp U back to 0-1023 (centered at 512)
            if v_u_sat + 512 < 0 then
                s3_u <= (others => '0');
            elsif v_u_sat + 512 > 1023 then
                s3_u <= to_unsigned(1023, 10);
            else
                s3_u <= unsigned(resize(v_u_sat + 512, 10));
            end if;

            -- Clamp V back to 0-1023
            if v_v_sat + 512 < 0 then
                s3_v <= (others => '0');
            elsif v_v_sat + 512 > 1023 then
                s3_v <= to_unsigned(1023, 10);
            else
                s3_v <= unsigned(resize(v_v_sat + 512, 10));
            end if;
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Valid pipeline for interpolator enable
    ---------------------------------------------------------------------------
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    ---------------------------------------------------------------------------
    -- Interpolators: wet/dry mix (3 channels)
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
            enable => s_interp_valid,
            a      => s_dry_y,
            b      => s3_y,
            t      => s_mix,
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
            enable => s_interp_valid,
            a      => s_dry_u,
            b      => s3_u,
            t      => s_mix,
            result => s_mix_u,
            valid  => open
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
            enable => s_interp_valid,
            a      => s_dry_v,
            b      => s3_v,
            t      => s_mix,
            result => s_mix_v,
            valid  => open
        );

    ---------------------------------------------------------------------------
    -- Delay lines: sync signals, dry tap, bypass data
    ---------------------------------------------------------------------------
    p_delay : process(clk)
        -- Sync delay (full pipeline = 7)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (full pipeline = 7)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (processing depth = 3)
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Sync signal shift registers
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);

            -- Bypass data shift registers
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap shift registers
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Outputs
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    ---------------------------------------------------------------------------
    -- Output mux: bypass / processed
    ---------------------------------------------------------------------------
    data_out.y <= s_bypass_y when s_bypass = '1' else
                  std_logic_vector(s_mix_y);

    data_out.u <= s_bypass_u when s_bypass = '1' else
                  std_logic_vector(s_mix_u);

    data_out.v <= s_bypass_v when s_bypass = '1' else
                  std_logic_vector(s_mix_v);

end architecture kaleidoscope;
