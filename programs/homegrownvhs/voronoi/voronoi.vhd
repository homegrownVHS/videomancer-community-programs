-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: voronoi.vhd - Voronoi Program for Videomancer
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
-- Program Name:        Voronoi
-- Author:              Adam Pflanzer
-- Overview:
--   Real-time Voronoi cell pattern generator with video modulation. The screen
--   is divided into a grid of cells, each with a center point jittered by a
--   deterministic XOR hash. For every pixel the Manhattan distance to the four
--   nearest cell centers is computed and the minimum is taken, producing the
--   classic Voronoi edge pattern. The distance value modulates the input video:
--   cell interiors pass through, edges are darkened or highlighted. An Edge
--   mode toggle switches between smooth distance falloff and hard cell borders.
--   Cell coloring mode tints each cell with a unique hue derived from the
--   winning cell's hash. Scroll X/Y animate the pattern via DDS accumulators.
--
-- Resources:
--   0 BRAM, ~3000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + counters):       1 clock  -> T+1
--   Stage 1 (grid cell + fractional coords):   1 clock  -> T+2
--   Stage 2 (hash 4 neighbors):                1 clock  -> T+3
--   Stage 2b (jitter multiply from hashes):    1 clock  -> T+4
--   Stage 3 (Manhattan distance to 4 centers): 1 clock  -> T+5
--   Stage 4 (min distance + winning cell ID):  1 clock  -> T+6
--   Stage 5 (distance scale + drive multiply): 1 clock  -> T+7
--   Stage 5a (add drive + invert + edge):      1 clock  -> T+8
--   Stage 5b (apply pattern to video):         1 clock  -> T+9
--   interpolator_u x3 (wet/dry mix):           4 clocks -> T+13
--   Total: 13 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Scale (cell size: 8 steps from 8px to 64px)
--   Pot 2  (registers_in(1)):   Edge Width (distance threshold for edge effect)
--   Pot 3  (registers_in(2)):   Scroll X (horizontal animation speed)
--   Pot 4  (registers_in(3)):   Scroll Y (vertical animation speed)
--   Pot 5  (registers_in(4)):   Jitter (randomness of cell center placement)
--   Pot 6  (registers_in(5)):   Video Drive (luma-responsive jitter)
--   Tog 7  (registers_in(6)(0)): Edge Mode (Smooth / Hard)
--   Tog 8  (registers_in(6)(1)): Cell Color (Off / On)
--   Tog 9  (registers_in(6)(2)): Invert (Normal / Inverted)
--   Tog 10 (registers_in(6)(3)): Outline (Multiply / Overlay)
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 9 (inline stages)
--   C_SYNC_DELAY_CLKS       = 13 (9 + 4 interpolator)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture voronoi of program_top is
    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : integer := 9;
    constant C_SYNC_DELAY_CLKS       : integer := 13; -- 9 + 4 (interpolator)

    ---------------------------------------------------------------------------
    -- Control signals (from registers)
    ---------------------------------------------------------------------------
    signal s_scale      : unsigned(2 downto 0);   -- 0-7: cell size selector
    signal s_edge_width : unsigned(9 downto 0);   -- 0-1023
    signal s_scroll_x   : unsigned(9 downto 0);   -- scroll speed X
    signal s_scroll_y   : unsigned(9 downto 0);   -- scroll speed Y
    signal s_jitter_amt : unsigned(9 downto 0);   -- 0-1023 jitter amount
    signal s_video_drive : unsigned(9 downto 0);   -- 0-1023 luma-driven jitter
    signal s_edge_mode  : std_logic;              -- 0=smooth, 1=hard
    signal s_cell_color : std_logic;              -- 0=off, 1=tint cells
    signal s_invert     : std_logic;              -- 0=normal, 1=inverted
    signal s_outline    : std_logic;              -- 0=multiply, 1=outline overlay
    signal s_bypass     : std_logic;
    signal s_mix        : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Scroll DDS accumulators (animate pattern)
    ---------------------------------------------------------------------------
    signal s_scroll_accum_x : unsigned(19 downto 0) := (others => '0');
    signal s_scroll_accum_y : unsigned(19 downto 0) := (others => '0');
    signal s_prev_vsync_n   : std_logic := '1';

    ---------------------------------------------------------------------------
    -- Pixel and line counters
    ---------------------------------------------------------------------------
    signal s_pixel_count : unsigned(10 downto 0) := (others => '0');
    signal s_line_count  : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_avid   : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 0 outputs
    ---------------------------------------------------------------------------
    signal s0_y    : unsigned(9 downto 0) := (others => '0');
    signal s0_u    : unsigned(9 downto 0) := (others => '0');
    signal s0_v    : unsigned(9 downto 0) := (others => '0');
    signal s0_avid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Stage 1 outputs: grid cell coords + fractional position
    ---------------------------------------------------------------------------
    signal s1_y    : unsigned(9 downto 0) := (others => '0');
    signal s1_u    : unsigned(9 downto 0) := (others => '0');
    signal s1_v    : unsigned(9 downto 0) := (others => '0');
    signal s1_avid : std_logic := '0';
    -- Cell coordinates (which grid cell the pixel is in)
    signal s1_cell_x : unsigned(7 downto 0) := (others => '0');
    signal s1_cell_y : unsigned(7 downto 0) := (others => '0');
    -- Fractional position within cell (0 to cell_size-1), 7 bits max (cell up to 64px)
    signal s1_frac_x : unsigned(6 downto 0) := (others => '0');
    signal s1_frac_y : unsigned(6 downto 0) := (others => '0');
    -- Cell size shift amount (3-6 for cells 8-64)
    signal s1_shift  : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2a outputs: registered hashes (before jitter multiply)
    ---------------------------------------------------------------------------
    signal s2a_y       : unsigned(9 downto 0) := (others => '0');
    signal s2a_u       : unsigned(9 downto 0) := (others => '0');
    signal s2a_v       : unsigned(9 downto 0) := (others => '0');
    signal s2a_avid    : std_logic := '0';
    signal s2a_frac_x  : unsigned(6 downto 0) := (others => '0');
    signal s2a_frac_y  : unsigned(6 downto 0) := (others => '0');
    signal s2a_hash_00 : unsigned(15 downto 0) := (others => '0');
    signal s2a_hash_10 : unsigned(15 downto 0) := (others => '0');
    signal s2a_hash_01 : unsigned(15 downto 0) := (others => '0');
    signal s2a_hash_11 : unsigned(15 downto 0) := (others => '0');
    signal s2a_shift   : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 2 outputs: jittered centers for 4 neighbors
    -- Jitter = hash-derived offset, scaled by jitter knob
    ---------------------------------------------------------------------------
    signal s2_y    : unsigned(9 downto 0) := (others => '0');
    signal s2_u    : unsigned(9 downto 0) := (others => '0');
    signal s2_v    : unsigned(9 downto 0) := (others => '0');
    signal s2_avid : std_logic := '0';
    signal s2_frac_x : unsigned(6 downto 0) := (others => '0');
    signal s2_frac_y : unsigned(6 downto 0) := (others => '0');
    -- Jittered center offsets for 4 cells (each 0 to cell_size-1)
    signal s2_jx_00 : unsigned(6 downto 0) := (others => '0');
    signal s2_jy_00 : unsigned(6 downto 0) := (others => '0');
    signal s2_jx_10 : unsigned(6 downto 0) := (others => '0');
    signal s2_jy_10 : unsigned(6 downto 0) := (others => '0');
    signal s2_jx_01 : unsigned(6 downto 0) := (others => '0');
    signal s2_jy_01 : unsigned(6 downto 0) := (others => '0');
    signal s2_jx_11 : unsigned(6 downto 0) := (others => '0');
    signal s2_jy_11 : unsigned(6 downto 0) := (others => '0');
    -- Hash values for cell coloring
    signal s2_hash_00 : unsigned(7 downto 0) := (others => '0');
    signal s2_hash_10 : unsigned(7 downto 0) := (others => '0');
    signal s2_hash_01 : unsigned(7 downto 0) := (others => '0');
    signal s2_hash_11 : unsigned(7 downto 0) := (others => '0');
    signal s2_shift   : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 3 outputs: Manhattan distances to 4 cell centers
    ---------------------------------------------------------------------------
    signal s3_y    : unsigned(9 downto 0) := (others => '0');
    signal s3_u    : unsigned(9 downto 0) := (others => '0');
    signal s3_v    : unsigned(9 downto 0) := (others => '0');
    signal s3_avid : std_logic := '0';
    signal s3_dist_00 : unsigned(7 downto 0) := (others => '0');
    signal s3_dist_10 : unsigned(7 downto 0) := (others => '0');
    signal s3_dist_01 : unsigned(7 downto 0) := (others => '0');
    signal s3_dist_11 : unsigned(7 downto 0) := (others => '0');
    signal s3_hash_00 : unsigned(7 downto 0) := (others => '0');
    signal s3_hash_10 : unsigned(7 downto 0) := (others => '0');
    signal s3_hash_01 : unsigned(7 downto 0) := (others => '0');
    signal s3_hash_11 : unsigned(7 downto 0) := (others => '0');
    signal s3_shift   : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 4 outputs: minimum distance + winning cell hash
    ---------------------------------------------------------------------------
    signal s4_y         : unsigned(9 downto 0) := (others => '0');
    signal s4_u         : unsigned(9 downto 0) := (others => '0');
    signal s4_v         : unsigned(9 downto 0) := (others => '0');
    signal s4_avid      : std_logic := '0';
    signal s4_min_dist  : unsigned(7 downto 0) := (others => '0');
    signal s4_win_hash  : unsigned(7 downto 0) := (others => '0');
    signal s4_shift     : unsigned(2 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 5 outputs: distance scaled + video drive multiply (registered)
    ---------------------------------------------------------------------------
    signal s5_dist_base  : unsigned(9 downto 0) := (others => '0');
    signal s5_drive_prod : unsigned(9 downto 0) := (others => '0');
    signal s5_use_drive  : std_logic := '0';
    signal s5_mid_y      : unsigned(9 downto 0) := (others => '0');
    signal s5_mid_u      : unsigned(9 downto 0) := (others => '0');
    signal s5_mid_v      : unsigned(9 downto 0) := (others => '0');
    signal s5_mid_avid   : std_logic := '0';
    signal s5_mid_hash   : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 5a outputs: invert + edge (before pattern apply)
    ---------------------------------------------------------------------------
    signal s5a_y         : unsigned(9 downto 0) := (others => '0');
    signal s5a_u         : unsigned(9 downto 0) := (others => '0');
    signal s5a_v         : unsigned(9 downto 0) := (others => '0');
    signal s5a_avid      : std_logic := '0';
    signal s5a_edge_val  : unsigned(9 downto 0) := (others => '0');
    signal s5a_win_hash  : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stage 5b outputs: modulated video (wet)
    ---------------------------------------------------------------------------
    signal s5_y    : unsigned(9 downto 0) := (others => '0');
    signal s5_u    : unsigned(9 downto 0) := (others => '0');
    signal s5_v    : unsigned(9 downto 0) := (others => '0');
    signal s5_avid : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Dry tap (input delayed by C_PROCESSING_DELAY_CLKS)
    ---------------------------------------------------------------------------
    signal s_dry_y : unsigned(9 downto 0);
    signal s_dry_u : unsigned(9 downto 0);
    signal s_dry_v : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Interpolator outputs
    ---------------------------------------------------------------------------
    signal s_mix_y       : unsigned(9 downto 0);
    signal s_mix_u       : unsigned(9 downto 0);
    signal s_mix_v       : unsigned(9 downto 0);
    signal s_mix_y_valid : std_logic;
    signal s_mix_u_valid : std_logic;
    signal s_mix_v_valid : std_logic;

    ---------------------------------------------------------------------------
    -- Hash function: simple XOR-fold hash for cell coordinate → jitter
    -- Returns 16-bit hash from two 8-bit cell coordinates
    ---------------------------------------------------------------------------
    function cell_hash(cx : unsigned(7 downto 0); cy : unsigned(7 downto 0))
        return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        -- XOR-shift hash: no multiplies, combinational-only
        v := cx & cy;
        v := v xor (v(11 downto 0) & v(15 downto 12));
        v := v xor ("00" & v(15 downto 2));
        v := v xor (v(7 downto 0) & v(15 downto 8));
        return v;
    end function;

begin
    ---------------------------------------------------------------------------
    -- Register mapping (concurrent)
    ---------------------------------------------------------------------------
    s_scale      <= unsigned(registers_in(0)(9 downto 7));  -- top 3 bits: 0-7
    s_edge_width <= unsigned(registers_in(1));
    s_scroll_x   <= unsigned(registers_in(2));
    s_scroll_y   <= unsigned(registers_in(3));
    s_jitter_amt  <= unsigned(registers_in(4));
    s_video_drive <= unsigned(registers_in(5));
    s_edge_mode  <= registers_in(6)(0);
    s_cell_color <= registers_in(6)(1);
    s_invert     <= registers_in(6)(2);
    s_outline    <= registers_in(6)(3);
    s_bypass     <= registers_in(6)(4);
    s_mix        <= unsigned(registers_in(7));

    ---------------------------------------------------------------------------
    -- Stage 0: Register input, pixel/line counters, scroll accumulators
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

            -- Pixel counter
            if data_in.avid = '1' then
                s_pixel_count <= s_pixel_count + 1;
            end if;
            if data_in.avid = '0' and s_prev_avid = '1' then
                s_pixel_count <= (others => '0');
                s_line_count  <= s_line_count + 1;
            end if;

            -- Frame reset + scroll update
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line_count <= (others => '0');
                -- DDS accumulators: add scroll speed each frame
                s_scroll_accum_x <= s_scroll_accum_x + resize(s_scroll_x, 20);
                s_scroll_accum_y <= s_scroll_accum_y + resize(s_scroll_y, 20);
            end if;
        end if;
    end process p_stage0;

    ---------------------------------------------------------------------------
    -- Stage 1: Compute grid cell coordinates and fractional position
    -- Latency: 1 clock
    --
    -- Cell size = 2^(3 + scale), giving sizes 8, 16, 32, 64 for scale 0-3
    -- (scale 4-7 clamp to 64)
    -- Pixel position includes scroll offset.
    ---------------------------------------------------------------------------
    p_stage1 : process(clk)
        variable v_px  : unsigned(10 downto 0);
        variable v_py  : unsigned(9 downto 0);
        variable v_sh  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            s1_y    <= s0_y;
            s1_u    <= s0_u;
            s1_v    <= s0_v;
            s1_avid <= s0_avid;

            -- Add scroll offset to pixel position
            v_px := s_pixel_count + s_scroll_accum_x(19 downto 9);
            v_py := s_line_count  + s_scroll_accum_y(19 downto 10);

            -- Clamp scale to 0-3 (cell sizes 8-64)
            if s_scale > 3 then
                v_sh := to_unsigned(3, 3);
            else
                v_sh := s_scale;
            end if;
            s1_shift <= v_sh;

            -- Compute cell coords and fractional based on shift
            case v_sh is
                when "000" =>  -- cell size 8
                    s1_cell_x <= v_px(10 downto 3);
                    s1_cell_y <= v_py(9 downto 3) & '0';
                    s1_frac_x <= "0000" & v_px(2 downto 0);
                    s1_frac_y <= "0000" & v_py(2 downto 0);
                when "001" =>  -- cell size 16
                    s1_cell_x <= '0' & v_px(10 downto 4);
                    s1_cell_y <= "00" & v_py(9 downto 4);
                    s1_frac_x <= "000" & v_px(3 downto 0);
                    s1_frac_y <= "000" & v_py(3 downto 0);
                when "010" =>  -- cell size 32
                    s1_cell_x <= "00" & v_px(10 downto 5);
                    s1_cell_y <= "000" & v_py(9 downto 5);
                    s1_frac_x <= "00" & v_px(4 downto 0);
                    s1_frac_y <= "00" & v_py(4 downto 0);
                when "011" =>  -- cell size 64
                    s1_cell_x <= "000" & v_px(10 downto 6);
                    s1_cell_y <= "0000" & v_py(9 downto 6);
                    s1_frac_x <= '0' & v_px(5 downto 0);
                    s1_frac_y <= '0' & v_py(5 downto 0);
                when others =>
                    s1_cell_x <= "000" & v_px(10 downto 6);
                    s1_cell_y <= "0000" & v_py(9 downto 6);
                    s1_frac_x <= '0' & v_px(5 downto 0);
                    s1_frac_y <= '0' & v_py(5 downto 0);
            end case;
        end if;
    end process p_stage1;

    ---------------------------------------------------------------------------
    -- Stage 2: Hash 4 neighbor cells (XOR-only, no multiplies)
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2 : process(clk)
        variable v_hash_00, v_hash_10, v_hash_01, v_hash_11 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s2a_y    <= s1_y;
            s2a_u    <= s1_u;
            s2a_v    <= s1_v;
            s2a_avid <= s1_avid;
            s2a_frac_x <= s1_frac_x;
            s2a_frac_y <= s1_frac_y;
            s2a_shift  <= s1_shift;

            -- Hash each of the 4 neighbor cells
            s2a_hash_00 <= cell_hash(s1_cell_x, s1_cell_y);
            s2a_hash_10 <= cell_hash(s1_cell_x + 1, s1_cell_y);
            s2a_hash_01 <= cell_hash(s1_cell_x, s1_cell_y + 1);
            s2a_hash_11 <= cell_hash(s1_cell_x + 1, s1_cell_y + 1);
        end if;
    end process p_stage2;

    ---------------------------------------------------------------------------
    -- Stage 2b: Jitter multiply from registered hashes
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage2b : process(clk)
        variable v_jx, v_jy : unsigned(13 downto 0);
        variable v_cell_mask : unsigned(6 downto 0);
        variable v_jitter_top7 : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            s2_y    <= s2a_y;
            s2_u    <= s2a_u;
            s2_v    <= s2a_v;
            s2_avid <= s2a_avid;
            s2_frac_x <= s2a_frac_x;
            s2_frac_y <= s2a_frac_y;
            s2_shift  <= s2a_shift;

            -- Jitter amount (top 7 bits of knob 5)
            v_jitter_top7 := s_jitter_amt(9 downto 3);

            -- Cell size mask
            case s2a_shift is
                when "000" => v_cell_mask := "0000111";
                when "001" => v_cell_mask := "0001111";
                when "010" => v_cell_mask := "0011111";
                when "011" => v_cell_mask := "0111111";
                when others => v_cell_mask := "0111111";
            end case;

            -- Save hash for cell coloring (bottom 8 bits)
            s2_hash_00 <= s2a_hash_00(7 downto 0);
            s2_hash_10 <= s2a_hash_10(7 downto 0);
            s2_hash_01 <= s2a_hash_01(7 downto 0);
            s2_hash_11 <= s2a_hash_11(7 downto 0);

            -- Jitter offsets from registered hash values
            v_jx := s2a_hash_00(6 downto 0) * v_jitter_top7;
            v_jy := s2a_hash_00(13 downto 7) * v_jitter_top7;
            s2_jx_00 <= v_jx(13 downto 7) and v_cell_mask;
            s2_jy_00 <= v_jy(13 downto 7) and v_cell_mask;

            v_jx := s2a_hash_10(6 downto 0) * v_jitter_top7;
            v_jy := s2a_hash_10(13 downto 7) * v_jitter_top7;
            s2_jx_10 <= v_jx(13 downto 7) and v_cell_mask;
            s2_jy_10 <= v_jy(13 downto 7) and v_cell_mask;

            v_jx := s2a_hash_01(6 downto 0) * v_jitter_top7;
            v_jy := s2a_hash_01(13 downto 7) * v_jitter_top7;
            s2_jx_01 <= v_jx(13 downto 7) and v_cell_mask;
            s2_jy_01 <= v_jy(13 downto 7) and v_cell_mask;

            v_jx := s2a_hash_11(6 downto 0) * v_jitter_top7;
            v_jy := s2a_hash_11(13 downto 7) * v_jitter_top7;
            s2_jx_11 <= v_jx(13 downto 7) and v_cell_mask;
            s2_jy_11 <= v_jy(13 downto 7) and v_cell_mask;
        end if;
    end process p_stage2b;

    ---------------------------------------------------------------------------
    -- Stage 3: Manhattan distance from pixel to each of 4 jittered centers
    -- Latency: 1 clock
    --
    -- Distance = |frac_x - jitter_x| + |frac_y - jitter_y| for cell (0,0)
    -- For neighbor cells (+1,0), the center is at (jitter_x - cell_size, jitter_y)
    -- relative to current cell origin, so distance uses cell-adjusted coords.
    ---------------------------------------------------------------------------
    p_stage3 : process(clk)
        variable v_dx, v_dy : signed(7 downto 0);
        variable v_abs_dx, v_abs_dy : unsigned(6 downto 0);
        variable v_dist : unsigned(7 downto 0);
        variable v_cell_size : unsigned(6 downto 0);
        variable v_fx_s, v_fy_s : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            s3_y    <= s2_y;
            s3_u    <= s2_u;
            s3_v    <= s2_v;
            s3_avid <= s2_avid;
            s3_hash_00 <= s2_hash_00;
            s3_hash_10 <= s2_hash_10;
            s3_hash_01 <= s2_hash_01;
            s3_hash_11 <= s2_hash_11;
            s3_shift   <= s2_shift;

            -- Cell size
            case s2_shift is
                when "000" => v_cell_size := to_unsigned(8, 7);
                when "001" => v_cell_size := to_unsigned(16, 7);
                when "010" => v_cell_size := to_unsigned(32, 7);
                when "011" => v_cell_size := to_unsigned(64, 7);
                when others => v_cell_size := to_unsigned(64, 7);
            end case;

            v_fx_s := signed('0' & s2_frac_x);
            v_fy_s := signed('0' & s2_frac_y);

            -- Cell (0,0): pixel at (frac_x, frac_y), center at (jx, jy)
            v_dx := v_fx_s - signed('0' & s2_jx_00);
            v_dy := v_fy_s - signed('0' & s2_jy_00);
            if v_dx < 0 then v_abs_dx := unsigned(-v_dx(6 downto 0)); else v_abs_dx := unsigned(v_dx(6 downto 0)); end if;
            if v_dy < 0 then v_abs_dy := unsigned(-v_dy(6 downto 0)); else v_abs_dy := unsigned(v_dy(6 downto 0)); end if;
            s3_dist_00 <= ('0' & v_abs_dx) + ('0' & v_abs_dy);

            -- Cell (1,0): center at (jx + cell_size, jy) relative to this cell
            v_dx := v_fx_s - signed(resize(s2_jx_10 + v_cell_size, 8));
            v_dy := v_fy_s - signed('0' & s2_jy_10);
            if v_dx < 0 then v_abs_dx := unsigned(-v_dx(6 downto 0)); else v_abs_dx := unsigned(v_dx(6 downto 0)); end if;
            if v_dy < 0 then v_abs_dy := unsigned(-v_dy(6 downto 0)); else v_abs_dy := unsigned(v_dy(6 downto 0)); end if;
            s3_dist_10 <= ('0' & v_abs_dx) + ('0' & v_abs_dy);

            -- Cell (0,1): center at (jx, jy + cell_size)
            v_dx := v_fx_s - signed('0' & s2_jx_01);
            v_dy := v_fy_s - signed(resize(s2_jy_01 + v_cell_size, 8));
            if v_dx < 0 then v_abs_dx := unsigned(-v_dx(6 downto 0)); else v_abs_dx := unsigned(v_dx(6 downto 0)); end if;
            if v_dy < 0 then v_abs_dy := unsigned(-v_dy(6 downto 0)); else v_abs_dy := unsigned(v_dy(6 downto 0)); end if;
            s3_dist_01 <= ('0' & v_abs_dx) + ('0' & v_abs_dy);

            -- Cell (1,1): center at (jx + cell_size, jy + cell_size)
            v_dx := v_fx_s - signed(resize(s2_jx_11 + v_cell_size, 8));
            v_dy := v_fy_s - signed(resize(s2_jy_11 + v_cell_size, 8));
            if v_dx < 0 then v_abs_dx := unsigned(-v_dx(6 downto 0)); else v_abs_dx := unsigned(v_dx(6 downto 0)); end if;
            if v_dy < 0 then v_abs_dy := unsigned(-v_dy(6 downto 0)); else v_abs_dy := unsigned(v_dy(6 downto 0)); end if;
            s3_dist_11 <= ('0' & v_abs_dx) + ('0' & v_abs_dy);
        end if;
    end process p_stage3;

    ---------------------------------------------------------------------------
    -- Stage 4: Find minimum distance among 4 cells, record winner hash
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage4 : process(clk)
        variable v_min_01   : unsigned(7 downto 0);
        variable v_hash_01  : unsigned(7 downto 0);
        variable v_min_23   : unsigned(7 downto 0);
        variable v_hash_23  : unsigned(7 downto 0);
        variable v_min_final : unsigned(7 downto 0);
        variable v_hash_final : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s4_y    <= s3_y;
            s4_u    <= s3_u;
            s4_v    <= s3_v;
            s4_avid <= s3_avid;
            s4_shift <= s3_shift;

            -- Compare pairs
            if s3_dist_00 <= s3_dist_10 then
                v_min_01  := s3_dist_00;
                v_hash_01 := s3_hash_00;
            else
                v_min_01  := s3_dist_10;
                v_hash_01 := s3_hash_10;
            end if;

            if s3_dist_01 <= s3_dist_11 then
                v_min_23  := s3_dist_01;
                v_hash_23 := s3_hash_01;
            else
                v_min_23  := s3_dist_11;
                v_hash_23 := s3_hash_11;
            end if;

            -- Compare winners
            if v_min_01 <= v_min_23 then
                v_min_final  := v_min_01;
                v_hash_final := v_hash_01;
            else
                v_min_final  := v_min_23;
                v_hash_final := v_hash_23;
            end if;

            s4_min_dist <= v_min_final;
            s4_win_hash <= v_hash_final;
        end if;
    end process p_stage4;

    ---------------------------------------------------------------------------
    -- Stage 5: Distance scaling + video drive multiply + edge computation
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage5 : process(clk)
        variable v_dist_10       : unsigned(9 downto 0);
        variable v_drive_prod_20 : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s5_mid_avid <= s4_avid;
            s5_mid_y    <= s4_y;
            s5_mid_u    <= s4_u;
            s5_mid_v    <= s4_v;
            s5_mid_hash <= s4_win_hash;

            -- Scale distance to 10-bit range (0-1023)
            case s4_shift is
                when "000" => v_dist_10 := s4_min_dist(2 downto 0) & "0000000";
                when "001" => v_dist_10 := s4_min_dist(3 downto 0) & "000000";
                when "010" => v_dist_10 := s4_min_dist(4 downto 0) & "00000";
                when "011" => v_dist_10 := s4_min_dist(5 downto 0) & "0000";
                when others => v_dist_10 := s4_min_dist(5 downto 0) & "0000";
            end case;
            if v_dist_10 > 1023 then
                v_dist_10 := to_unsigned(1023, 10);
            end if;

            -- Register base distance and video drive multiply product
            s5_dist_base <= v_dist_10;
            v_drive_prod_20 := s4_y * s_video_drive;
            s5_drive_prod <= v_drive_prod_20(19 downto 10);

            if s_video_drive > 0 then
                s5_use_drive <= '1';
            else
                s5_use_drive <= '0';
            end if;
        end if;
    end process p_stage5;

    ---------------------------------------------------------------------------
    -- Stage 5a: Add drive product + invert + edge computation
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage5a : process(clk)
        variable v_dist_10  : unsigned(9 downto 0);
        variable v_dist_adj : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s5a_avid     <= s5_mid_avid;
            s5a_y        <= s5_mid_y;
            s5a_u        <= s5_mid_u;
            s5a_v        <= s5_mid_v;
            s5a_win_hash <= s5_mid_hash;

            v_dist_10 := s5_dist_base;

            -- Add registered video drive product
            if s5_use_drive = '1' then
                v_dist_adj := resize(v_dist_10, 11) + ('0' & s5_drive_prod);
                if v_dist_adj > 1023 then
                    v_dist_10 := to_unsigned(1023, 10);
                else
                    v_dist_10 := v_dist_adj(9 downto 0);
                end if;
            end if;

            -- Invert if requested
            if s_invert = '1' then
                v_dist_10 := to_unsigned(1023, 10) - v_dist_10;
            end if;

            -- Compute edge value
            if s_edge_mode = '1' then
                if v_dist_10 < s_edge_width then
                    s5a_edge_val <= to_unsigned(0, 10);
                else
                    s5a_edge_val <= to_unsigned(1023, 10);
                end if;
            else
                s5a_edge_val <= v_dist_10;
            end if;
        end if;
    end process p_stage5a;

    ---------------------------------------------------------------------------
    -- Stage 5b: Apply pattern to video (multiply / outline / cell color)
    -- Latency: 1 clock
    ---------------------------------------------------------------------------
    p_stage5b : process(clk)
        variable v_y_mod  : unsigned(19 downto 0);
        variable v_cell_u : unsigned(9 downto 0);
        variable v_cell_v : unsigned(9 downto 0);
        variable v_y_sum  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s5_avid <= s5a_avid;

            if s_outline = '1' then
                -- Outline mode: video passes through, edges overlaid as bright lines
                if s5a_edge_val < s_edge_width then
                    v_y_sum := signed("00" & s5a_y)
                             + signed("00" & (s_edge_width - s5a_edge_val));
                    if v_y_sum > 1023 then
                        s5_y <= to_unsigned(1023, 10);
                    else
                        s5_y <= unsigned(v_y_sum(9 downto 0));
                    end if;
                else
                    s5_y <= s5a_y;
                end if;
                s5_u <= s5a_u;
                s5_v <= s5a_v;
            else
                -- Multiply mode: video * edge_val
                v_y_mod := s5a_y * s5a_edge_val;
                s5_y <= v_y_mod(19 downto 10);

                -- Cell coloring
                if s_cell_color = '1' then
                    v_cell_u := unsigned(s5a_win_hash(7 downto 4)) & "000000";
                    v_cell_v := unsigned(s5a_win_hash(3 downto 0)) & "000000";
                    if s5a_edge_val > 512 then
                        s5_u <= resize(
                            shift_right(resize(s5a_u, 11) + resize(v_cell_u, 11), 1), 10);
                        s5_v <= resize(
                            shift_right(resize(s5a_v, 11) + resize(v_cell_v, 11), 1), 10);
                    else
                        s5_u <= s5a_u;
                        s5_v <= s5a_v;
                    end if;
                else
                    s5_u <= s5a_u;
                    s5_v <= s5a_v;
                end if;
            end if;
        end if;
    end process p_stage5b;

    ---------------------------------------------------------------------------
    -- Dry tap: delay input by C_PROCESSING_DELAY_CLKS for interpolator
    ---------------------------------------------------------------------------
    p_dry_delay : process(clk)
        type t_delay_u10 is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of unsigned(9 downto 0);
        variable v_y : t_delay_u10 := (others => (others => '0'));
        variable v_u : t_delay_u10 := (others => (others => '0'));
        variable v_v : t_delay_u10 := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_y := unsigned(data_in.y) & v_y(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u := unsigned(data_in.u) & v_u(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v := unsigned(data_in.v) & v_v(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_dry_y <= v_y(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_dry_delay;

    ---------------------------------------------------------------------------
    -- Interpolators: wet/dry mix (4 clocks each)
    ---------------------------------------------------------------------------
    u_mix_y : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (
            clk    => clk,
            enable => '1',
            a      => s_dry_y,
            b      => s5_y,
            t      => s_mix,
            result => s_mix_y,
            valid  => s_mix_y_valid
        );

    u_mix_u : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (
            clk    => clk,
            enable => '1',
            a      => s_dry_u,
            b      => s5_u,
            t      => s_mix,
            result => s_mix_u,
            valid  => s_mix_u_valid
        );

    u_mix_v : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (
            clk    => clk,
            enable => '1',
            a      => s_dry_v,
            b      => s5_v,
            t      => s_mix,
            result => s_mix_v,
            valid  => s_mix_v_valid
        );

    ---------------------------------------------------------------------------
    -- Sync signal delay pipeline
    ---------------------------------------------------------------------------
    p_sync_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');
    begin
        if rising_edge(clk) then
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);
        end if;
    end process p_sync_delay;

    ---------------------------------------------------------------------------
    -- Output mux: bypass or mixed
    ---------------------------------------------------------------------------
    p_output : process(clk)
    begin
        if rising_edge(clk) then
            if s_bypass = '1' then
                data_out.y <= data_in.y;
                data_out.u <= data_in.u;
                data_out.v <= data_in.v;
            else
                data_out.y <= std_logic_vector(s_mix_y);
                data_out.u <= std_logic_vector(s_mix_u);
                data_out.v <= std_logic_vector(s_mix_v);
            end if;
        end if;
    end process p_output;

end architecture voronoi;
