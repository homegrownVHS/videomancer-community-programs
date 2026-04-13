-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: slitscan.vhd - Slit-Scan Program for Videomancer
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
-- Program Name:        Slit-Scan
-- Author:              Adam Pflanzer
-- Overview:
--   Temporal slit-scan effect with 8 spatial mapping modes. Different pixel
--   positions in the output correspond to different moments in the input
--   stream, using BRAM-based variable delay lines. Modes include horizontal,
--   vertical, diagonal, diamond, radial, mirror, zigzag, and noise patterns.
--   Quantization ("slices") creates banded time steps. Y/UV temporal split
--   creates chromatic time offset.
--
-- Resources:
--   15 BRAM (3x variable_delay_u, 10-bit x 2048), ~2500 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + counters):      1 clock  -> T+1
--   Stage 1 (centered coords + abs + max/min):1 clock  -> T+2
--   Stage 1b (position metric per mode):      1 clock  -> T+3
--   Stage 2 (offset + bounce + register):     1 clock  -> T+4
--   Stage 2b (multiply position x amount):    1 clock  -> T+5
--   Stage 3 (reverse + quantize + clamp):     1 clock  -> T+6
--   Stage 3b (luma depth multiply):           1 clock  -> T+7
--   Stage 3c (add + clamp + Y/UV split):      1 clock  -> T+8
--   variable_delay_u (address gen + read):    2 clocks -> T+10
--   interpolator_u x3 (wet/dry mix):          4 clocks -> T+14
--   Total: 14 clocks
--
-- Submodules:
--   variable_delay_u x3: BRAM delay lines for Y/U/V, 2 clocks each
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--   lfsr16 x1: pseudo-random noise for noise mode, free-running
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Amount (temporal spread across frame)
--   Pot 2  (registers_in(1)):   Offset (phase offset of delay gradient)
--   Pot 3  (registers_in(2)):   Slices (quantization; 0=smooth, up=banded)
--   Pot 4  (registers_in(3)):   Mode (8 modes via steps_8)
--   Pot 5  (registers_in(4)):   Split (Y/UV temporal offset; center=none)
--   Pot 6  (registers_in(5)):   Luma Depth (video brightness modulates delay)
--   Tog 7  (registers_in(6)(0)): Reverse
--   Tog 8  (registers_in(6)(1)): Luma Only (delay Y only, pass U/V through)
--   Tog 9  (registers_in(6)(2)): Bounce (fold/mirror delay gradient)
--   Tog 10 (registers_in(6)(3)): Deep (2x delay range)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Modes (Pot 4, 8 positions):
--   0: Horizontal  — left-to-right delay ramp
--   1: Vertical    — top-to-bottom delay ramp
--   2: Diagonal    — corner-to-corner ramp
--   3: Diamond     — concentric diamonds from center (Manhattan distance)
--   4: Radial      — concentric circles from center (alpha-max-beta-min)
--   5: Mirror      — bilateral symmetric outward from vertical center
--   6: Zigzag      — alternating ramp direction per line group
--   7: Noise       — LFSR pseudo-random delay per pixel
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 10 (8 inline + 2 variable_delay_u)
--   C_SYNC_DELAY_CLKS       = 14 (10 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture slitscan of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 10;
    constant C_SYNC_DELAY_CLKS       : integer := 14; -- 10 + 4 (interpolator)
    constant C_BUF_DEPTH             : integer := 11;  -- 2048 entries per channel

    ---------------------------------------------------------------------------
    -- Control signals (from registers)
    ---------------------------------------------------------------------------
    signal s_amount  : unsigned(9 downto 0);  -- Knob 1: temporal spread
    signal s_offset  : unsigned(9 downto 0);  -- Knob 2: base delay
    signal s_quant   : unsigned(3 downto 0);  -- Knob 3: quantization shift 0-15
    signal s_mode    : unsigned(2 downto 0);  -- Knob 4: mode 0-7
    signal s_split   : unsigned(9 downto 0);  -- Knob 5: Y/UV split (512=none)
    signal s_luma_depth : unsigned(9 downto 0);  -- Knob 6: luma-driven delay depth
    signal s_reverse  : std_logic;             -- Toggle 7: reverse ramp
    signal s_y_only  : std_logic;             -- Toggle 8: luma only mode
    signal s_bounce  : std_logic;             -- Toggle 9: fold/mirror gradient
    signal s_deep    : std_logic;             -- Toggle 10: 2x delay range
    signal s_bypass  : std_logic;             -- Toggle 11: bypass enable
    signal s_mix     : unsigned(9 downto 0);  -- Fader: dry/wet mix

    ---------------------------------------------------------------------------
    -- Pixel and line counters
    ---------------------------------------------------------------------------
    signal s_pixel_count   : unsigned(10 downto 0) := (others => '0');
    signal s_line_count    : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_avid     : std_logic := '0';
    signal s_prev_vsync_n  : std_logic := '1';

    ---------------------------------------------------------------------------
    -- Frame dimension tracking (dynamic center for diamond/radial/mirror)
    ---------------------------------------------------------------------------
    signal s_cur_max_pixel : unsigned(10 downto 0) := (others => '0');
    signal s_max_pixel     : unsigned(10 downto 0) := to_unsigned(719, 11);
    signal s_max_line      : unsigned(9 downto 0)  := to_unsigned(269, 10);
    signal s_half_w        : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_half_h        : unsigned(9 downto 0)  := to_unsigned(135, 10);

    ---------------------------------------------------------------------------
    -- LFSR output (noise mode)
    ---------------------------------------------------------------------------
    signal s_lfsr_out : std_logic_vector(15 downto 0);

    ---------------------------------------------------------------------------
    -- Stage 0 outputs (registered input)
    ---------------------------------------------------------------------------
    signal s0_y    : unsigned(9 downto 0) := (others => '0');
    signal s0_u    : unsigned(9 downto 0) := (others => '0');
    signal s0_v    : unsigned(9 downto 0) := (others => '0');
    signal s0_avid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 1 intermediate outputs (abs/max/min registered)
    ---------------------------------------------------------------------------
    signal s1a_y         : unsigned(9 downto 0) := (others => '0');
    signal s1a_u         : unsigned(9 downto 0) := (others => '0');
    signal s1a_v         : unsigned(9 downto 0) := (others => '0');
    signal s1a_avid      : std_logic := '0';
    signal s1a_abs_dx    : unsigned(10 downto 0) := (others => '0');
    signal s1a_abs_dy    : unsigned(10 downto 0) := (others => '0');
    signal s1a_max_xy    : unsigned(10 downto 0) := (others => '0');
    signal s1a_min_xy    : unsigned(10 downto 0) := (others => '0');
    signal s1a_pixel     : unsigned(10 downto 0) := (others => '0');
    signal s1a_line      : unsigned(9 downto 0)  := (others => '0');
    signal s1a_max_pixel : unsigned(10 downto 0) := (others => '0');
    signal s1a_line_bit3 : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 1 outputs (position metric computed)
    ---------------------------------------------------------------------------
    signal s1_y        : unsigned(9 downto 0) := (others => '0');
    signal s1_u        : unsigned(9 downto 0) := (others => '0');
    signal s1_v        : unsigned(9 downto 0) := (others => '0');
    signal s1_avid     : std_logic := '0';
    signal s1_position : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2 outputs (delay multiplied)
    ---------------------------------------------------------------------------
    signal s2_y     : unsigned(9 downto 0) := (others => '0');
    signal s2_u     : unsigned(9 downto 0) := (others => '0');
    signal s2_v     : unsigned(9 downto 0) := (others => '0');
    signal s2_avid  : std_logic := '0';
    signal s2_delay : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2a intermediate (offset+bounce position registered before multiply)
    ---------------------------------------------------------------------------
    signal s2a_y    : unsigned(9 downto 0) := (others => '0');
    signal s2a_u    : unsigned(9 downto 0) := (others => '0');
    signal s2a_v    : unsigned(9 downto 0) := (others => '0');
    signal s2a_avid : std_logic := '0';
    signal s2a_pos  : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3 intermediate outputs (reverse + quantize, before luma depth)
    ---------------------------------------------------------------------------
    signal s3a_y          : unsigned(9 downto 0) := (others => '0');
    signal s3a_u          : unsigned(9 downto 0) := (others => '0');
    signal s3a_v          : unsigned(9 downto 0) := (others => '0');
    signal s3a_avid       : std_logic := '0';
    signal s3a_base_delay : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3b outputs (luma depth multiply registered)
    ---------------------------------------------------------------------------
    signal s3b_y          : unsigned(9 downto 0) := (others => '0');
    signal s3b_u          : unsigned(9 downto 0) := (others => '0');
    signal s3b_v          : unsigned(9 downto 0) := (others => '0');
    signal s3b_avid       : std_logic := '0';
    signal s3b_base_delay : unsigned(10 downto 0) := (others => '0');
    signal s3b_luma_prod  : unsigned(9 downto 0) := (others => '0');
    signal s3b_use_luma   : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 3 outputs (quantized, offset, clamped — feed BRAM)
    ---------------------------------------------------------------------------
    signal s3_y    : unsigned(9 downto 0) := (others => '0');
    signal s3_u    : unsigned(9 downto 0) := (others => '0');
    signal s3_v    : unsigned(9 downto 0) := (others => '0');
    signal s3_avid : std_logic := '0';
    signal s_delay_val    : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal s_delay_val_uv : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Wet (delayed) outputs from variable_delay_u
    ---------------------------------------------------------------------------
    signal s_wet_y       : unsigned(9 downto 0);
    signal s_wet_u       : unsigned(9 downto 0);
    signal s_wet_v       : unsigned(9 downto 0);
    signal s_wet_valid_y : std_logic;

    ---------------------------------------------------------------------------
    -- Valid signal for interpolator enable
    ---------------------------------------------------------------------------
    signal s_interp_valid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Dry tap (input delayed to align with wet at interpolator input)
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
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;
    signal s_mix_u_valid : std_logic;
    signal s_mix_v_valid : std_logic;

begin
    ---------------------------------------------------------------------------
    -- Register mapping (concurrent)
    ---------------------------------------------------------------------------
    s_amount  <= unsigned(registers_in(0));
    s_offset  <= unsigned(registers_in(1));
    s_quant   <= unsigned(registers_in(2)(9 downto 6));  -- top 4 bits: 0-15
    s_mode    <= unsigned(registers_in(3)(9 downto 7));  -- top 3 bits: 0-7
    s_split      <= unsigned(registers_in(4));              -- 0-1023, center=512
    s_luma_depth <= unsigned(registers_in(5));              -- 0-1023
    s_reverse    <= registers_in(6)(0);
    s_y_only  <= registers_in(6)(1);
    s_bounce  <= registers_in(6)(2);
    s_deep    <= registers_in(6)(3);
    s_bypass  <= registers_in(6)(4);
    s_mix     <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- LFSR noise generator (free-running, noise mode source)
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
    -- Stage 0: Register input, update pixel/line counters, track dimensions
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_y    <= unsigned(data_in.y);
            s0_u    <= unsigned(data_in.u);
            s0_v    <= unsigned(data_in.v);
            s0_avid <= data_in.avid;

            s_prev_avid    <= data_in.avid;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Pixel counter: increment during active video
            if data_in.avid = '1' then
                s_pixel_count <= s_pixel_count + 1;
                -- Track max pixel count for this line
                if s_pixel_count > s_cur_max_pixel then
                    s_cur_max_pixel <= s_pixel_count;
                end if;
            end if;

            -- End of active line: latch line width, reset pixel counter
            if data_in.avid = '0' and s_prev_avid = '1' then
                s_max_pixel     <= s_cur_max_pixel;
                s_half_w        <= '0' & s_cur_max_pixel(10 downto 1);
                s_cur_max_pixel <= (others => '0');
                s_pixel_count   <= (others => '0');
                s_line_count    <= s_line_count + 1;
            end if;

            -- Start of new frame: latch frame height, reset line counter
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_max_line   <= s_line_count;
                s_half_h     <= '0' & s_line_count(9 downto 1);
                s_line_count <= (others => '0');
            end if;
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: Compute centered coordinates, absolute values, max/min
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_dx      : signed(11 downto 0);
        variable v_dy      : signed(11 downto 0);
        variable v_abs_dx  : unsigned(10 downto 0);
        variable v_abs_dy  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- Pipeline data through
            s1a_y    <= s0_y;
            s1a_u    <= s0_u;
            s1a_v    <= s0_v;
            s1a_avid <= s0_avid;

            -- Pipe counters for mode case in Stage 1b
            s1a_pixel     <= s_pixel_count;
            s1a_line      <= s_line_count;
            s1a_max_pixel <= s_max_pixel;
            s1a_line_bit3 <= s_line_count(3);

            -- Centered coordinates
            v_dx := signed('0' & s_pixel_count) - signed('0' & s_half_w);
            v_dy := signed("00" & s_line_count) - signed("00" & s_half_h);

            -- Absolute values
            if v_dx < 0 then
                v_abs_dx := unsigned(-v_dx(10 downto 0));
            else
                v_abs_dx := unsigned(v_dx(10 downto 0));
            end if;
            if v_dy < 0 then
                v_abs_dy := unsigned(-v_dy(10 downto 0));
            else
                v_abs_dy := unsigned(v_dy(10 downto 0));
            end if;

            s1a_abs_dx <= v_abs_dx;
            s1a_abs_dy <= v_abs_dy;

            -- Max/min for radial approximation
            if v_abs_dx >= v_abs_dy then
                s1a_max_xy <= v_abs_dx;
                s1a_min_xy <= v_abs_dy;
            else
                s1a_max_xy <= v_abs_dy;
                s1a_min_xy <= v_abs_dx;
            end if;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 1b: Mode-dependent position metric from registered abs/max/min
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage1b : process(clk)
        variable v_pos11   : unsigned(10 downto 0);
        variable v_pos     : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s1_y    <= s1a_y;
            s1_u    <= s1a_u;
            s1_v    <= s1a_v;
            s1_avid <= s1a_avid;

            -- Mode-dependent position computation (using registered values)
            case s_mode is
                when "000" =>
                    -- Horizontal: left-to-right ramp
                    if s1a_pixel > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := s1a_pixel(9 downto 0);
                    end if;

                when "001" =>
                    -- Vertical: top-to-bottom ramp
                    v_pos := s1a_line;

                when "010" =>
                    -- Diagonal: top-left to bottom-right
                    v_pos11 := ('0' & s1a_pixel(10 downto 1)) + ('0' & s1a_line);
                    if v_pos11 > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := v_pos11(9 downto 0);
                    end if;

                when "011" =>
                    -- Diamond: Manhattan distance from center
                    v_pos11 := s1a_abs_dx + s1a_abs_dy;
                    if v_pos11 > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := v_pos11(9 downto 0);
                    end if;

                when "100" =>
                    -- Radial: alpha-max-beta-min distance from center
                    v_pos11 := s1a_max_xy
                             + ("00" & s1a_min_xy(10 downto 2))
                             + ("000" & s1a_min_xy(10 downto 3));
                    if v_pos11 > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := v_pos11(9 downto 0);
                    end if;

                when "101" =>
                    -- Mirror: bilateral symmetric from vertical centerline
                    v_pos11 := s1a_abs_dx(9 downto 0) & '0';
                    if v_pos11 > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := v_pos11(9 downto 0);
                    end if;

                when "110" =>
                    -- Zigzag: alternating ramp direction every 8 lines
                    if s1a_line_bit3 = '0' then
                        v_pos11 := s1a_pixel;
                    else
                        if s1a_max_pixel >= s1a_pixel then
                            v_pos11 := s1a_max_pixel - s1a_pixel;
                        else
                            v_pos11 := (others => '0');
                        end if;
                    end if;
                    if v_pos11 > 1023 then
                        v_pos := to_unsigned(1023, 10);
                    else
                        v_pos := v_pos11(9 downto 0);
                    end if;

                when "111" =>
                    -- Noise: LFSR pseudo-random per pixel
                    v_pos := unsigned(s_lfsr_out(9 downto 0));

                when others =>
                    v_pos := (others => '0');
            end case;

            s1_position <= v_pos;
        end if;
    end process p_stage1b;

    ---------------------------------------------------------------------------
    -- Stage 2: Multiply position by amount to compute raw delay
    -- Latency: 1 clock
    -- delay = (position * amount) >> 9, range 0-2045
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_pos       : unsigned(9 downto 0);
        variable v_offset_s  : signed(10 downto 0);
        variable v_shifted_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s2a_y    <= s1_y;
            s2a_u    <= s1_u;
            s2a_v    <= s1_v;
            s2a_avid <= s1_avid;

            -- Offset as signed phase shift (centered at 512)
            -- Saturating clamp eliminates wrap discontinuity
            v_offset_s  := signed(resize(s_offset, 11)) - to_signed(512, 11);
            v_shifted_s := signed('0' & s1_position) + resize(v_offset_s, 12);
            if v_shifted_s < 0 then
                v_pos := to_unsigned(0, 10);
            elsif v_shifted_s > 1023 then
                v_pos := to_unsigned(1023, 10);
            else
                v_pos := unsigned(v_shifted_s(9 downto 0));
            end if;

            -- Bounce: fold the position at midpoint (triangle pattern)
            if s_bounce = '1' and v_pos(9) = '1' then
                v_pos := not v_pos;
            end if;

            s2a_pos <= v_pos;
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 2b: Multiply registered position by amount
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2b : process(clk)
        variable v_mult : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s2_y    <= s2a_y;
            s2_u    <= s2a_u;
            s2_v    <= s2a_v;
            s2_avid <= s2a_avid;

            v_mult := s2a_pos * s_amount;

            -- Deep: 2x delay range (>> 8 instead of >> 9)
            if s_deep = '1' then
                s2_delay <= v_mult(18 downto 8);
            else
                s2_delay <= v_mult(19 downto 9);  -- >> 9 → 0-2045
            end if;
        end if;
    end process p_stage2b;

    ---------------------------------------------------------------------------
    -- Stage 3: Apply reverse, quantization, basic clamp
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_delay      : unsigned(10 downto 0);
        variable v_quantized  : unsigned(10 downto 0);
        variable v_total      : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s3a_y    <= s2_y;
            s3a_u    <= s2_u;
            s3a_v    <= s2_v;
            s3a_avid <= s2_avid;

            v_delay := s2_delay;

            -- Reverse: flip ramp direction
            if s_reverse = '1' then
                if (s_amount & '0') >= v_delay then
                    v_delay := (s_amount & '0') - v_delay;
                else
                    v_delay := (others => '0');
                end if;
            end if;

            -- Quantize: right-shift then left-shift (power-of-2 banding)
            case s_quant is
                when "0000" => v_quantized := v_delay;
                when "0001" => v_quantized := v_delay(10 downto 1) & "0";
                when "0010" => v_quantized := v_delay(10 downto 2) & "00";
                when "0011" => v_quantized := v_delay(10 downto 3) & "000";
                when "0100" => v_quantized := v_delay(10 downto 4) & "0000";
                when "0101" => v_quantized := v_delay(10 downto 5) & "00000";
                when "0110" => v_quantized := v_delay(10 downto 6) & "000000";
                when "0111" => v_quantized := v_delay(10 downto 7) & "0000000";
                when "1000" => v_quantized := v_delay(10 downto 8) & "00000000";
                when "1001" => v_quantized := v_delay(10 downto 9) & "000000000";
                when "1010" => v_quantized := v_delay(10) & "0000000000";
                when others => v_quantized := v_delay(10) & "0000000000";
            end case;

            -- Clamp to 2047
            v_total := '0' & v_quantized;
            if v_total > 2047 then
                s3a_base_delay <= to_unsigned(2047, 11);
            else
                s3a_base_delay <= v_total(10 downto 0);
            end if;
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Stage 3b: Luma depth multiply (register product)
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3b : process(clk)
        variable v_luma_prod : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s3b_y          <= s3a_y;
            s3b_u          <= s3a_u;
            s3b_v          <= s3a_v;
            s3b_avid       <= s3a_avid;
            s3b_base_delay <= s3a_base_delay;

            -- Compute luma depth product, register result
            v_luma_prod := s3a_y * s_luma_depth;
            s3b_luma_prod <= v_luma_prod(19 downto 10);

            if s_luma_depth > 0 then
                s3b_use_luma <= '1';
            else
                s3b_use_luma <= '0';
            end if;
        end if;
    end process p_stage3b;

    ---------------------------------------------------------------------------
    -- Stage 3c: Add luma product + clamp + Y/UV split
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage3c : process(clk)
        variable v_y_clamped  : unsigned(10 downto 0);
        variable v_luma_total : unsigned(11 downto 0);
        variable v_split_off  : signed(10 downto 0);
        variable v_uv_total   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s3_y    <= s3b_y;
            s3_u    <= s3b_u;
            s3_v    <= s3b_v;
            s3_avid <= s3b_avid;

            v_y_clamped := s3b_base_delay;

            -- Conditionally add registered luma product
            if s3b_use_luma = '1' then
                v_luma_total := ('0' & v_y_clamped) + ("00" & s3b_luma_prod);
                if v_luma_total > 2047 then
                    v_y_clamped := to_unsigned(2047, 11);
                else
                    v_y_clamped := v_luma_total(10 downto 0);
                end if;
            end if;

            s_delay_val <= v_y_clamped;

            -- Y/UV split: offset UV delay from Y delay
            v_split_off := signed(resize(s_split, 11)) - to_signed(512, 11);
            v_uv_total  := signed('0' & v_y_clamped) + resize(v_split_off, 12);
            if v_uv_total < 0 then
                s_delay_val_uv <= (others => '0');
            elsif v_uv_total > 2047 then
                s_delay_val_uv <= to_unsigned(2047, C_BUF_DEPTH);
            else
                s_delay_val_uv <= unsigned(v_uv_total(C_BUF_DEPTH - 1 downto 0));
            end if;
        end if;
    end process p_stage3c;

    ---------------------------------------------------------------------------
    -- BRAM variable delay lines (Y, U, V channels)
    -- Each: G_DEPTH=11 (2048 entries), G_WIDTH=10 -> 5 BRAMs per channel
    -- Y uses s_delay_val, U/V use s_delay_val_uv (split offset)
    -- Latency: 2 clocks (address gen + BRAM read)
    ---------------------------------------------------------------------------
    u_delay_y : entity work.variable_delay_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_BUF_DEPTH)
        port map (
            clk    => clk,
            enable => s3_avid,
            delay  => s_delay_val,
            a      => s3_y,
            result => s_wet_y,
            valid  => s_wet_valid_y
        );

    u_delay_u : entity work.variable_delay_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_BUF_DEPTH)
        port map (
            clk    => clk,
            enable => s3_avid,
            delay  => s_delay_val_uv,
            a      => s3_u,
            result => s_wet_u,
            valid  => open
        );

    u_delay_v : entity work.variable_delay_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_DEPTH => C_BUF_DEPTH)
        port map (
            clk    => clk,
            enable => s3_avid,
            delay  => s_delay_val_uv,
            a      => s3_v,
            result => s_wet_v,
            valid  => open
        );

    ---------------------------------------------------------------------------
    -- Valid pipeline: track avid through processing stages for interpolator
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
    -- Interpolators: wet/dry mix (3 channels, parallel)
    -- Latency: 4 clocks each
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
            b      => s_wet_y,
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
            b      => s_wet_u,
            t      => s_mix,
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
            enable => s_interp_valid,
            a      => s_dry_v,
            b      => s_wet_v,
            t      => s_mix,
            result => s_mix_v,
            valid  => s_mix_v_valid
        );

    ---------------------------------------------------------------------------
    -- Delay lines: sync signals, dry tap, and bypass data
    ---------------------------------------------------------------------------
    p_delay : process(clk)
        -- Sync delay (full pipeline depth = 10)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');

        -- Bypass data delay (full pipeline depth = 10)
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));

        -- Dry tap delay (processing depth = 6, aligned with wet at interp input)
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

            -- Bypass data shift registers (full pipeline depth)
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);

            -- Dry tap shift registers (processing depth only)
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Output delayed sync signals
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            -- Output bypass data
            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            -- Output dry tap
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
                  s_bypass_u when s_y_only = '1' else
                  std_logic_vector(s_mix_u);

    data_out.v <= s_bypass_v when s_bypass = '1' else
                  s_bypass_v when s_y_only = '1' else
                  std_logic_vector(s_mix_v);

end architecture slitscan;
