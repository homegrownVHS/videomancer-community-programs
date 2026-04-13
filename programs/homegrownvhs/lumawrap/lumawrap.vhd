-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: lumawrap.vhd - Luma Wrap / Modulo Program for Videomancer
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
-- Program Name:        Luma Wrap
-- Author:              Adam Pflanzer
-- Overview:
--   Modulo/wrap processor for Y, U, V channels. Multiplies each channel by a
--   gain factor to push values past the 10-bit range, then extracts a sliding
--   10-bit window from the product to produce sawtooth-style wrapping. Creates
--   topographic contour banding that follows brightness/color levels. Per-channel
--   enable, adjustable wrap frequency (gain), window position (fine tune),
--   and optional offset bias.
--
--   Zero BRAM — pure arithmetic pipeline.
--
-- Resources:
--   0 BRAM, ~1200 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register):             1 clock  -> T+1
--   Stage 1 (multiply Y/U/V by gain):     1 clock  -> T+2
--   Stage 2 (bit-slice + channel enable):  1 clock  -> T+3
--   Stage 3 (offset bias + clamp):         1 clock  -> T+4
--   interpolator_u x3 (wet/dry mix):       4 clocks -> T+8
--   Total: 8 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Frequency (wrap gain multiplier)
--   Pot 2  (registers_in(1)):   Window (bit-slice offset for fine freq)
--   Pot 3  (registers_in(2)):   Offset (brightness bias before wrap)
--   Pot 4  (registers_in(3)):   Fold (triangle wave fold point)
--   Pot 5  (registers_in(4)):   UV Detune (chroma window offset)
--   Pot 6  (registers_in(5)):   Threshold (only wrap above this Y level)
--   Tog 7  (registers_in(6)(0)): Y Enable
--   Tog 8  (registers_in(6)(1)): U Enable
--   Tog 9  (registers_in(6)(2)): V Enable
--   Tog 10 (registers_in(6)(3)): Invert (flip wrapped output)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 4 (inline stages)
--   C_SYNC_DELAY_CLKS       = 8 (4 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture lumawrap of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 4;
    constant C_SYNC_DELAY_CLKS       : integer := 8;  -- 4 + 4 (interpolator)

    ---------------------------------------------------------------------------
    -- Control signals
    ---------------------------------------------------------------------------
    signal s_frequency  : unsigned(9 downto 0);  -- Knob 1: gain multiplier
    signal s_window     : unsigned(3 downto 0);  -- Knob 2: bit-slice position (0-10)
    signal s_offset     : unsigned(9 downto 0);  -- Knob 3: brightness bias
    signal s_fold       : unsigned(9 downto 0);  -- Knob 4: fold point (0=off, >0=triangle)
    signal s_uv_detune  : unsigned(3 downto 0);  -- Knob 5: UV window offset (0-15)
    signal s_threshold  : unsigned(9 downto 0);  -- Knob 6: wrap threshold (Y level)
    signal s_y_en      : std_logic;             -- Toggle 7: wrap Y
    signal s_u_en      : std_logic;             -- Toggle 8: wrap U
    signal s_v_en      : std_logic;             -- Toggle 9: wrap V
    signal s_invert    : std_logic;             -- Toggle 10: invert output
    signal s_bypass    : std_logic;             -- Toggle 11: bypass
    signal s_mix       : unsigned(9 downto 0);  -- Fader: dry/wet

    ---------------------------------------------------------------------------
    -- Stage 0 outputs
    ---------------------------------------------------------------------------
    signal s0_y    : unsigned(9 downto 0) := (others => '0');
    signal s0_u    : unsigned(9 downto 0) := (others => '0');
    signal s0_v    : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 1 outputs (products)
    ---------------------------------------------------------------------------
    signal s1_y_prod : unsigned(19 downto 0) := (others => '0');
    signal s1_u_prod : unsigned(19 downto 0) := (others => '0');
    signal s1_v_prod : unsigned(19 downto 0) := (others => '0');
    signal s1_y      : unsigned(9 downto 0) := (others => '0');
    signal s1_u      : unsigned(9 downto 0) := (others => '0');
    signal s1_v      : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2 outputs (bit-sliced / wrapped)
    ---------------------------------------------------------------------------
    signal s2_y : unsigned(9 downto 0) := (others => '0');
    signal s2_u : unsigned(9 downto 0) := (others => '0');
    signal s2_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3 outputs (offset + clamp)
    ---------------------------------------------------------------------------
    signal s3_y : unsigned(9 downto 0) := (others => '0');
    signal s3_u : unsigned(9 downto 0) := (others => '0');
    signal s3_v : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Dry tap (delayed to match processing depth at interpolator input)
    ---------------------------------------------------------------------------
    signal s_dry_y : unsigned(9 downto 0);
    signal s_dry_u : unsigned(9 downto 0);
    signal s_dry_v : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Bypass data (delayed full pipeline depth)
    ---------------------------------------------------------------------------
    signal s_bypass_y : std_logic_vector(9 downto 0);
    signal s_bypass_u : std_logic_vector(9 downto 0);
    signal s_bypass_v : std_logic_vector(9 downto 0);

    ---------------------------------------------------------------------------
    -- Valid for interpolator enable
    ---------------------------------------------------------------------------
    signal s_interp_valid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;

    ---------------------------------------------------------------------------
    -- Bit-slice extraction function
    -- Extracts 10 bits from a 20-bit product starting at bit position 'pos'
    ---------------------------------------------------------------------------
    function extract_window(
        prod : unsigned(19 downto 0);
        pos  : unsigned(3 downto 0)
    ) return unsigned is
        variable v_result : unsigned(9 downto 0);
    begin
        case pos is
            when "0000" => v_result := prod( 9 downto  0);
            when "0001" => v_result := prod(10 downto  1);
            when "0010" => v_result := prod(11 downto  2);
            when "0011" => v_result := prod(12 downto  3);
            when "0100" => v_result := prod(13 downto  4);
            when "0101" => v_result := prod(14 downto  5);
            when "0110" => v_result := prod(15 downto  6);
            when "0111" => v_result := prod(16 downto  7);
            when "1000" => v_result := prod(17 downto  8);
            when "1001" => v_result := prod(18 downto  9);
            when "1010" => v_result := prod(19 downto 10);
            when others => v_result := prod(19 downto 10);
        end case;
        return v_result;
    end function;

begin
    ---------------------------------------------------------------------------
    -- Register mapping
    ---------------------------------------------------------------------------
    s_frequency  <= unsigned(registers_in(0));
    s_window     <= unsigned(registers_in(1)(9 downto 6));  -- top 4 bits: 0-10
    s_offset     <= unsigned(registers_in(2));
    s_fold       <= unsigned(registers_in(3));
    s_uv_detune  <= unsigned(registers_in(4)(9 downto 6));  -- top 4 bits: 0-15
    s_threshold  <= unsigned(registers_in(5));
    s_y_en      <= registers_in(6)(0);
    s_u_en      <= registers_in(6)(1);
    s_v_en      <= registers_in(6)(2);
    s_invert    <= registers_in(6)(3);
    s_bypass    <= registers_in(6)(4);
    s_mix       <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- Stage 0: Register input
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: Multiply each channel by frequency (gain)
    -- 10 x 10 = 20 bits unsigned
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_y_prod <= s0_y * s_frequency;
            s1_u_prod <= s0_u * s_frequency;
            s1_v_prod <= s0_v * s_frequency;

            -- Pass through originals for channel-disable
            s1_y <= s0_y;
            s1_u <= s0_u;
            s1_v <= s0_v;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 2: Extract sliding 10-bit window from products
    -- Channel enable: if disabled, pass through original value
    -- Invert: bitwise NOT on wrapped channels
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_wrap_y : unsigned(9 downto 0);
        variable v_wrap_u : unsigned(9 downto 0);
        variable v_wrap_v : unsigned(9 downto 0);
        variable v_uv_win : unsigned(3 downto 0);
        variable v_fold_r : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Extract Y window using global window position
            v_wrap_y := extract_window(s1_y_prod, s_window);

            -- UV detune: offset window position for chroma channels
            v_uv_win := s_window + s_uv_detune;
            if v_uv_win > 10 then
                v_uv_win := to_unsigned(10, 4);
            end if;
            v_wrap_u := extract_window(s1_u_prod, v_uv_win);
            v_wrap_v := extract_window(s1_v_prod, v_uv_win);

            -- Fold: triangle wave. Reflect values above fold point.
            -- s_fold=0: no fold (sawtooth). s_fold=512: symmetric triangle.
            -- s_fold=1023: fold at 1023 (nearly all reflected).
            if s_fold > 0 then
                -- Fold Y
                if v_wrap_y > s_fold then
                    v_fold_r := signed("00" & s_fold) + signed("00" & s_fold)
                              - signed("00" & v_wrap_y);
                    if v_fold_r < 0 then
                        v_wrap_y := (others => '0');
                    else
                        v_wrap_y := unsigned(v_fold_r(9 downto 0));
                    end if;
                end if;
                -- Fold U
                if v_wrap_u > s_fold then
                    v_fold_r := signed("00" & s_fold) + signed("00" & s_fold)
                              - signed("00" & v_wrap_u);
                    if v_fold_r < 0 then
                        v_wrap_u := (others => '0');
                    else
                        v_wrap_u := unsigned(v_fold_r(9 downto 0));
                    end if;
                end if;
                -- Fold V
                if v_wrap_v > s_fold then
                    v_fold_r := signed("00" & s_fold) + signed("00" & s_fold)
                              - signed("00" & v_wrap_v);
                    if v_fold_r < 0 then
                        v_wrap_v := (others => '0');
                    else
                        v_wrap_v := unsigned(v_fold_r(9 downto 0));
                    end if;
                end if;
            end if;

            -- Invert if enabled
            if s_invert = '1' then
                v_wrap_y := not v_wrap_y;
                v_wrap_u := not v_wrap_u;
                v_wrap_v := not v_wrap_v;
            end if;

            -- Threshold: only wrap above this luma level
            -- Below threshold: all channels pass through original
            if s1_y < s_threshold then
                s2_y <= s1_y;
                s2_u <= s1_u;
                s2_v <= s1_v;
            else
                -- Per-channel enable
                if s_y_en = '1' then
                    s2_y <= v_wrap_y;
                else
                    s2_y <= s1_y;
                end if;

                if s_u_en = '1' then
                    s2_u <= v_wrap_u;
                else
                    s2_u <= s1_u;
                end if;

                if s_v_en = '1' then
                    s2_v <= v_wrap_v;
                else
                    s2_v <= s1_v;
                end if;
            end if;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 3: Apply brightness offset bias + clamp
    -- Offset center = 512 (no change), <512 = darken, >512 = brighten
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_bias   : signed(10 downto 0);
        variable v_y_sum  : signed(11 downto 0);
        variable v_u_sum  : signed(11 downto 0);
        variable v_v_sum  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Offset centered at 512: subtract 512 to get signed bias -512..+511
            v_bias := signed(resize(s_offset, 11)) - to_signed(512, 11);

            v_y_sum := signed('0' & s2_y & '0') + resize(v_bias, 12);
            if v_y_sum < 0 then
                s3_y <= (others => '0');
            elsif v_y_sum > 1023 then
                s3_y <= to_unsigned(1023, 10);
            else
                s3_y <= unsigned(v_y_sum(10 downto 1));
            end if;

            -- U/V: keep chroma centered — only apply offset if channel is wrapped
            if s_u_en = '1' then
                v_u_sum := signed('0' & s2_u & '0') + resize(v_bias, 12);
                if v_u_sum < 0 then
                    s3_u <= (others => '0');
                elsif v_u_sum > 1023 then
                    s3_u <= to_unsigned(1023, 10);
                else
                    s3_u <= unsigned(v_u_sum(10 downto 1));
                end if;
            else
                s3_u <= s2_u;
            end if;

            if s_v_en = '1' then
                v_v_sum := signed('0' & s2_v & '0') + resize(v_bias, 12);
                if v_v_sum < 0 then
                    s3_v <= (others => '0');
                elsif v_v_sum > 1023 then
                    s3_v <= to_unsigned(1023, 10);
                else
                    s3_v <= unsigned(v_v_sum(10 downto 1));
                end if;
            else
                s3_v <= s2_v;
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
        -- Sync delay (full pipeline = 8)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (full pipeline = 8)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (processing depth = 4)
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

end architecture lumawrap;
