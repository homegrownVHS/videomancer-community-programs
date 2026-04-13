-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: scope.vhd - Waveform Monitor / Chroma Scope for Videomancer
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
-- Program Name:        Scope
-- Author:              Adam Pflanzer
-- Overview:
--   Waveform monitor and chroma waveform display. In luma mode, each input
--   pixel's Y value is tested against a per-scanline target to render a
--   classic phosphor-trace waveform. In chroma mode, U and V waveforms are
--   overlaid with distinct colors (cyan for U, magenta for V). Graticule
--   reference lines at 0%, 25%, 50%, 75%, 100% of the luma range with
--   vertical markers every 128 pixels. Optional video-behind overlay mode
--   dims the input and superimposes the trace.
--
--   Zero BRAM — the waveform renders naturally: each scanline independently
--   tests all active pixels against the target luma for that line. A DDA
--   accumulator maps the 10-bit luma range (0-1023) across the active line
--   count, adapting to the timing mode via registers_in(8) lookup.
--
-- Resources:
--   0 BRAM, ~800 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + counter snapshot):      1 clock  -> T+1
--   Stage 1 (distance computation Y/U/V):             1 clock  -> T+2
--   Stage 2 (threshold + graticule detection):         1 clock  -> T+3
--   Stage 3 (color output mux):                        1 clock  -> T+4
--   interpolator_u x3 (wet/dry mix):                   4 clocks -> T+8
--   Total: 8 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Trace Intensity
--   Pot 2  (registers_in(1)):   Trace Width (top 6 bits -> 0-63 luma tolerance)
--   Pot 3  (registers_in(2)):   Graticule brightness
--   Pot 4  (registers_in(3)):   Trace Color (steps_4: Green/Amber/White/Blue)
--   Pot 5  (registers_in(4)):   Background brightness
--   Pot 6  (registers_in(5)):   (unused)
--   Tog 7  (registers_in(6)(0)): Display (Luma / Chroma)
--   Tog 8  (registers_in(6)(1)): Graticule (Off / On)
--   Tog 9  (registers_in(6)(2)): Background (Black / Video overlay)
--   Tog 10 (registers_in(6)(3)): (unused)
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

architecture scope of program_top is

    -- ========================================================================
    -- Constants
    -- ========================================================================
    constant C_VIDEO_DATA_WIDTH      : integer := 10;
    constant C_PROCESSING_DELAY_CLKS : integer := 4;
    constant C_SYNC_DELAY_CLKS       : integer := 8;  -- 4 + 4 (interpolator)

    -- DDA initial value: 1023 * 1024 = 1,047,552 (10.10 fixed point)
    constant C_DDA_INIT : unsigned(19 downto 0) := to_unsigned(1047552, 20);

    -- ========================================================================
    -- Parameter signals
    -- ========================================================================
    signal s_brightness  : unsigned(9 downto 0);
    signal s_thickness   : unsigned(5 downto 0);
    signal s_grat_bright : unsigned(9 downto 0);
    signal s_color_sel   : unsigned(1 downto 0);
    signal s_bg_bright   : unsigned(9 downto 0);
    signal s_chroma_mode : std_logic;
    signal s_graticule   : std_logic;
    signal s_overlay     : std_logic;
    signal s_bypass      : std_logic;
    signal s_mix         : unsigned(9 downto 0);

    -- ========================================================================
    -- DDA vertical mapping
    -- ========================================================================
    signal s_v_step         : unsigned(12 downto 0) := to_unsigned(1457, 13);
    signal s_target_luma_fp : unsigned(19 downto 0) := C_DDA_INIT;
    signal s_target_luma    : unsigned(9 downto 0)  := to_unsigned(1023, 10);

    -- ========================================================================
    -- Position counters
    -- ========================================================================
    signal s_pixel_x       : unsigned(11 downto 0) := (others => '0');
    signal s_line_had_avid : std_logic := '0';
    signal s_prev_hsync    : std_logic := '1';
    signal s_prev_vsync    : std_logic := '1';

    -- ========================================================================
    -- Stage 0 outputs (input register + snapshot)
    -- ========================================================================
    signal s0_y       : unsigned(9 downto 0) := (others => '0');
    signal s0_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_v       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_target  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s0_pixel_x : unsigned(11 downto 0) := (others => '0');

    -- ========================================================================
    -- Stage 1 outputs (distance computation)
    -- ========================================================================
    signal s1_dist_y  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s1_dist_u  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s1_dist_v  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s1_target  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s1_pixel_x : unsigned(11 downto 0) := (others => '0');
    signal s1_y       : unsigned(9 downto 0) := (others => '0');
    signal s1_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_v       : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Stage 2 outputs (threshold + graticule)
    -- ========================================================================
    signal s2_is_trace_y : std_logic := '0';
    signal s2_is_trace_u : std_logic := '0';
    signal s2_is_trace_v : std_logic := '0';
    signal s2_is_grat    : std_logic := '0';
    signal s2_y          : unsigned(9 downto 0) := (others => '0');
    signal s2_u          : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2_v          : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Stage 3 outputs (color output / wet signal)
    -- ========================================================================
    signal s3_y : unsigned(9 downto 0) := (others => '0');
    signal s3_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s3_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Dry tap / bypass / valid
    -- ========================================================================
    signal s_dry_y        : unsigned(9 downto 0);
    signal s_dry_u        : unsigned(9 downto 0);
    signal s_dry_v        : unsigned(9 downto 0);
    signal s_bypass_y     : std_logic_vector(9 downto 0);
    signal s_bypass_u     : std_logic_vector(9 downto 0);
    signal s_bypass_v     : std_logic_vector(9 downto 0);
    signal s_interp_valid : std_logic := '0';

    -- ========================================================================
    -- Interpolator outputs
    -- ========================================================================
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;

begin

    -- ========================================================================
    -- Parameter decode
    -- ========================================================================
    s_brightness  <= unsigned(registers_in(0)(9 downto 0));
    s_thickness   <= unsigned(registers_in(1)(9 downto 4));  -- top 6 bits: 0-63
    s_grat_bright <= unsigned(registers_in(2)(9 downto 0));
    s_color_sel   <= unsigned(registers_in(3)(9 downto 8));  -- steps_4: top 2 bits
    s_bg_bright   <= unsigned(registers_in(4)(9 downto 0));
    s_chroma_mode <= registers_in(6)(0);
    s_graticule   <= registers_in(6)(1);
    s_overlay     <= registers_in(6)(2);
    s_bypass      <= registers_in(6)(4);
    s_mix         <= unsigned(registers_in(7)(9 downto 0));

    -- ========================================================================
    -- DDA step selection based on timing ID
    -- Maps 1024 luma levels across the active line count for each mode.
    -- Step = 1023*1024 / (active_lines - 1)  in 10.10 fixed point.
    -- ========================================================================
    p_step_select : process(clk)
    begin
        if rising_edge(clk) then
            case registers_in(8)(3 downto 0) is
                when C_NTSC                                           => s_v_step <= to_unsigned(4329, 13);  -- 243 lines
                when C_PAL                                            => s_v_step <= to_unsigned(3651, 13);  -- 288 lines
                when C_480P                                           => s_v_step <= to_unsigned(2187, 13);  -- 480 lines
                when C_576P                                           => s_v_step <= to_unsigned(1822, 13);  -- 576 lines
                when C_720P50 | C_720P5994 | C_720P60                 => s_v_step <= to_unsigned(1457, 13);  -- 720 lines
                when C_1080I50 | C_1080I5994 | C_1080I60              => s_v_step <= to_unsigned(1944, 13);  -- 540 lines/field
                when others                                           => s_v_step <= to_unsigned(971, 13);   -- 1080 lines
            end case;
        end if;
    end process p_step_select;

    -- ========================================================================
    -- Position counters + vertical DDA
    -- Pixel counter for graticule vertical lines.
    -- DDA accumulator maps luma 1023 (top) to 0 (bottom) across active lines.
    -- ========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync <= data_in.hsync_n;
            s_prev_vsync <= data_in.vsync_n;

            -- Active pixel counter
            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;
                s_line_had_avid <= '1';
            end if;

            -- Line boundary (hsync falling edge)
            if s_prev_hsync = '1' and data_in.hsync_n = '0' then
                s_pixel_x <= (others => '0');
                if s_line_had_avid = '1' then
                    -- Decrement DDA for next active line
                    if s_target_luma_fp >= resize(s_v_step, 20) then
                        s_target_luma_fp <= s_target_luma_fp - resize(s_v_step, 20);
                    else
                        s_target_luma_fp <= (others => '0');
                    end if;
                end if;
                s_line_had_avid <= '0';
            end if;

            -- Frame boundary (vsync falling edge)
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                s_target_luma_fp <= C_DDA_INIT;
                s_line_had_avid <= '0';
            end if;

            -- Extract integer part of DDA (10.10 FP -> 10-bit integer)
            s_target_luma <= s_target_luma_fp(19 downto 10);
        end if;
    end process p_counters;

    -- ========================================================================
    -- Stage 0: Input register + position snapshot
    -- Latency: 1 clock
    -- ========================================================================
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_y       <= unsigned(data_in.y);
            s0_u       <= unsigned(data_in.u);
            s0_v       <= unsigned(data_in.v);
            s0_target  <= s_target_luma;
            s0_pixel_x <= s_pixel_x;
        end if;
    end process p_stage0;

    -- ========================================================================
    -- Stage 1: Distance computation
    -- Computes |channel - target| for Y, U, V.
    -- Y distance used in luma mode; U and V distances used in chroma mode.
    -- Latency: 1 clock
    -- ========================================================================
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_target  <= s0_target;
            s1_pixel_x <= s0_pixel_x;
            s1_y       <= s0_y;
            s1_u       <= s0_u;
            s1_v       <= s0_v;

            -- Y distance
            if s0_y >= s0_target then
                s1_dist_y <= s0_y - s0_target;
            else
                s1_dist_y <= s0_target - s0_y;
            end if;

            -- U distance
            if s0_u >= s0_target then
                s1_dist_u <= s0_u - s0_target;
            else
                s1_dist_u <= s0_target - s0_u;
            end if;

            -- V distance
            if s0_v >= s0_target then
                s1_dist_v <= s0_v - s0_target;
            else
                s1_dist_v <= s0_target - s0_v;
            end if;
        end if;
    end process p_stage1;

    -- ========================================================================
    -- Stage 2: Threshold + graticule detection
    -- Compares distance to thickness tolerance; detects graticule positions.
    -- Horizontal graticule at 0%, 25%, 50%, 75%, 100% of luma range.
    -- Vertical graticule every 128 pixels.
    -- Latency: 1 clock
    -- ========================================================================
    p_stage2 : process(clk)
        variable v_thick : unsigned(9 downto 0);
        variable v_grat  : std_logic;
    begin
        if rising_edge(clk) then
            s2_y <= s1_y;
            s2_u <= s1_u;
            s2_v <= s1_v;

            v_thick := resize(s_thickness, 10);

            -- Trace hit detection
            if s1_dist_y <= v_thick then
                s2_is_trace_y <= '1';
            else
                s2_is_trace_y <= '0';
            end if;

            if s1_dist_u <= v_thick then
                s2_is_trace_u <= '1';
            else
                s2_is_trace_u <= '0';
            end if;

            if s1_dist_v <= v_thick then
                s2_is_trace_v <= '1';
            else
                s2_is_trace_v <= '0';
            end if;

            -- Graticule detection
            v_grat := '0';
            if s_graticule = '1' then
                -- Horizontal reference lines at 0/25/50/75/100% luma
                -- Tolerance ±3 handles large DDA steps in SD modes
                if s1_target <= 3 or s1_target >= 1020 or
                   (s1_target >= 253 and s1_target <= 259) or
                   (s1_target >= 509 and s1_target <= 515) or
                   (s1_target >= 765 and s1_target <= 771) then
                    v_grat := '1';
                end if;

                -- Vertical reference lines every 128 pixels
                if s1_pixel_x(6 downto 0) = "0000000" then
                    v_grat := '1';
                end if;
            end if;

            s2_is_grat <= v_grat;
        end if;
    end process p_stage2;

    -- ========================================================================
    -- Stage 3: Color output
    -- Priority: trace > graticule > background.
    -- Luma mode: single trace in selectable phosphor color.
    -- Chroma mode: U trace (cyan) + V trace (magenta), overlap = white.
    -- Latency: 1 clock
    -- ========================================================================
    p_stage3 : process(clk)
    begin
        if rising_edge(clk) then
            if s_chroma_mode = '0' then
                -- ==========================================================
                -- Luma waveform mode
                -- ==========================================================
                if s2_is_trace_y = '1' then
                    -- Trace pixel: user-selected phosphor color
                    s3_y <= s_brightness;
                    case s_color_sel is
                        when "00"   => s3_u <= to_unsigned(173, 10); s3_v <= to_unsigned(83, 10);   -- Green
                        when "01"   => s3_u <= to_unsigned(85, 10);  s3_v <= to_unsigned(702, 10);  -- Amber
                        when "10"   => s3_u <= to_unsigned(512, 10); s3_v <= to_unsigned(512, 10);  -- White
                        when others => s3_u <= to_unsigned(800, 10); s3_v <= to_unsigned(200, 10);  -- Blue
                    end case;
                elsif s2_is_grat = '1' then
                    -- Graticule: neutral gray
                    s3_y <= s_grat_bright;
                    s3_u <= to_unsigned(512, 10);
                    s3_v <= to_unsigned(512, 10);
                elsif s_overlay = '1' then
                    -- Video behind trace at 25% brightness
                    s3_y <= "00" & s2_y(9 downto 2);
                    s3_u <= s2_u;
                    s3_v <= s2_v;
                else
                    -- Black (or dim) background
                    s3_y <= s_bg_bright;
                    s3_u <= to_unsigned(512, 10);
                    s3_v <= to_unsigned(512, 10);
                end if;
            else
                -- ==========================================================
                -- Chroma waveform mode: U (cyan) + V (magenta)
                -- ==========================================================
                if s2_is_trace_u = '1' and s2_is_trace_v = '1' then
                    -- Both traces overlap: white
                    s3_y <= s_brightness;
                    s3_u <= to_unsigned(512, 10);
                    s3_v <= to_unsigned(512, 10);
                elsif s2_is_trace_u = '1' then
                    -- U trace: cyan
                    s3_y <= s_brightness;
                    s3_u <= to_unsigned(800, 10);
                    s3_v <= to_unsigned(200, 10);
                elsif s2_is_trace_v = '1' then
                    -- V trace: magenta
                    s3_y <= s_brightness;
                    s3_u <= to_unsigned(200, 10);
                    s3_v <= to_unsigned(800, 10);
                elsif s2_is_grat = '1' then
                    -- Graticule
                    s3_y <= s_grat_bright;
                    s3_u <= to_unsigned(512, 10);
                    s3_v <= to_unsigned(512, 10);
                elsif s_overlay = '1' then
                    -- Video behind at 25%
                    s3_y <= "00" & s2_y(9 downto 2);
                    s3_u <= s2_u;
                    s3_v <= s2_v;
                else
                    -- Background
                    s3_y <= s_bg_bright;
                    s3_u <= to_unsigned(512, 10);
                    s3_v <= to_unsigned(512, 10);
                end if;
            end if;
        end if;
    end process p_stage3;

    -- ========================================================================
    -- Valid pipeline for interpolator enable
    -- ========================================================================
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    -- ========================================================================
    -- Interpolators: wet/dry mix (3 channels)
    -- ========================================================================
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

    -- ========================================================================
    -- Delay lines: sync signals, dry tap, bypass data
    -- ========================================================================
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(9 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(9 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Sync delay
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);

            -- Bypass data delay (full sync depth)
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap delay (processing depth only)
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Output sync signals
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            -- Bypass taps
            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            -- Dry taps (for interpolator a port)
            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    -- ========================================================================
    -- Output mux: bypass or interpolated result
    -- ========================================================================
    data_out.y <= s_bypass_y when s_bypass = '1' else std_logic_vector(s_mix_y);
    data_out.u <= s_bypass_u when s_bypass = '1' else std_logic_vector(s_mix_u);
    data_out.v <= s_bypass_v when s_bypass = '1' else std_logic_vector(s_mix_v);

end architecture scope;
