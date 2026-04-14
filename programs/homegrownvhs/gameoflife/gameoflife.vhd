-- Videomancer Community Programs
-- Copyright (C) 2025 homegrownVHS
-- File: gameoflife.vhd - Conway's Game of Life for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-community-programs
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
-- Program Name:        Game of Life
-- Author:              homegrownVHS
-- Overview:
--   Conway's Game of Life cellular automaton rendered as a 16x16 toroidal
--   grid overlaid on the incoming video. Alive cells display as colored
--   blocks or reveal the source video; dead cells are darkened. The grid
--   evolves autonomously following the classic rules (birth at 3 neighbors,
--   survival at 2-3). Double-buffered register grids allow glitch-free
--   updates during vertical blanking. An LFSR provides random seeding
--   with adjustable density.
--
-- Resources:
--   0 BRAM, ~5000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (register input + grid coords):  1 clock  -> T+1
--   Stage 1 (cell lookup + grid line flag):   1 clock  -> T+2
--   Stage 2 (color output mux):              1 clock  -> T+3
--   interpolator_u (wet/dry mix):            4 clocks -> T+7
--   Total: 7 clocks
--
-- Submodules:
--   interpolator_u: linear blend, 4 clocks (x3 for Y/U/V)
--   lfsr16: pseudo-random noise, free-running
--
-- Parameters:
--   Pot 1  (registers_in(0)):     Speed       - evolution rate
--   Pot 2  (registers_in(1)):     Color       - alive cell hue (8 steps)
--   Pot 3  (registers_in(2)):     Brightness  - alive cell luma level (8 steps)
--   Pot 4  (registers_in(3)):     Dim         - dead cell darkness (8 steps)
--   Pot 5  (registers_in(4)):     Density     - random seed fill ratio
--   Pot 6  (registers_in(5)):     (unused)
--   Tog 7  (registers_in(6)(0)):  Video Reveal mode
--   Tog 8  (registers_in(6)(1)):  Pause
--   Tog 9  (registers_in(6)(2)):  Grid Lines
--   Tog 10 (registers_in(6)(3)):  Reseed
--   Tog 11 (registers_in(6)(4)):  Bypass
--   Fader  (registers_in(7)):     Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 3 (inline stages)
--   C_SYNC_DELAY_CLKS       = 7 (total, including trailing interpolator)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture gameoflife of program_top is

    -- ========================================================================
    -- Constants
    -- ========================================================================
    constant C_VIDEO_DATA_WIDTH      : integer := 10;
    constant C_PROCESSING_DELAY_CLKS : integer := 3;
    constant C_SYNC_DELAY_CLKS       : integer := 7;

    constant C_GRID_SIZE : integer := 16;
    constant C_GRID_BITS : integer := 4;

    -- ========================================================================
    -- Types
    -- ========================================================================
    type t_grid is array(0 to C_GRID_SIZE - 1) of std_logic_vector(C_GRID_SIZE - 1 downto 0);

    -- ========================================================================
    -- Grid storage (double-buffered)
    -- ========================================================================
    signal grid_a : t_grid := (others => (others => '0'));
    signal grid_b : t_grid := (others => (others => '0'));
    signal s_show_a : std_logic := '1';

    -- ========================================================================
    -- LFSR for random seeding
    -- ========================================================================
    signal s_lfsr_out : std_logic_vector(15 downto 0);

    -- ========================================================================
    -- Parameter signals
    -- ========================================================================
    signal s_speed_skip    : unsigned(5 downto 0);
    signal s_color_sel     : unsigned(2 downto 0);
    signal s_bright_sel    : unsigned(2 downto 0);
    signal s_dim_sel       : unsigned(2 downto 0);
    signal s_seed_density  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_video_reveal  : std_logic;
    signal s_pause         : std_logic;
    signal s_show_grid     : std_logic;
    signal s_reseed        : std_logic;
    signal s_bypass        : std_logic;
    signal s_mix_amount    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- ========================================================================
    -- Timing detection
    -- ========================================================================
    signal s_timing_id : t_video_timing_id;
    signal s_cell_w    : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_cell_h    : unsigned(6 downto 0) := to_unsigned(45, 7);

    -- ========================================================================
    -- Pixel counting and cell mapping
    -- ========================================================================
    signal s_subcell_col : unsigned(7 downto 0) := (others => '0');
    signal s_subcell_row : unsigned(6 downto 0) := (others => '0');
    signal s_render_col  : unsigned(C_GRID_BITS - 1 downto 0) := (others => '0');
    signal s_render_row  : unsigned(C_GRID_BITS - 1 downto 0) := (others => '0');
    signal s_is_grid_h   : std_logic := '0';
    signal s_is_grid_v   : std_logic := '0';
    signal s_prev_hsync  : std_logic := '1';
    signal s_prev_vsync  : std_logic := '1';
    signal s_line_had_avid : std_logic := '0';

    -- ========================================================================
    -- Game logic signals
    -- ========================================================================
    signal s_game_vsync    : std_logic := '1';
    signal s_prev_reseed   : std_logic := '0';
    signal s_frame_counter : unsigned(5 downto 0) := (others => '0');
    signal s_updating      : std_logic := '0';
    signal s_seeding       : std_logic := '1';
    signal s_update_col    : unsigned(C_GRID_BITS - 1 downto 0) := (others => '0');
    signal s_update_row    : unsigned(C_GRID_BITS - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- Pipeline stage 0 signals
    -- ========================================================================
    signal s0_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s0_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s0_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s0_col     : unsigned(C_GRID_BITS - 1 downto 0);
    signal s0_row     : unsigned(C_GRID_BITS - 1 downto 0);
    signal s0_is_grid : std_logic;

    -- ========================================================================
    -- Pipeline stage 1 signals
    -- ========================================================================
    signal s1_y        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s1_u        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s1_v        : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s1_is_alive : std_logic;
    signal s1_is_grid  : std_logic;

    -- ========================================================================
    -- Pipeline stage 2 signals
    -- ========================================================================
    signal s_proc_y     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_proc_u     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_proc_v     : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_proc_valid : std_logic := '0';

    -- ========================================================================
    -- Palette: 8 colors for alive cells (Y, U, V)
    -- ========================================================================
    type t_palette_entry is array(0 to 2) of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    type t_palette is array(0 to 7) of t_palette_entry;
    constant C_PALETTE : t_palette := (
        0 => (to_unsigned(940, 10), to_unsigned(512, 10), to_unsigned(512, 10)),
        1 => (to_unsigned(800, 10), to_unsigned(226, 10), to_unsigned(627, 10)),
        2 => (to_unsigned(600, 10), to_unsigned(339, 10), to_unsigned(282, 10)),
        3 => (to_unsigned(500, 10), to_unsigned(798, 10), to_unsigned(397, 10)),
        4 => (to_unsigned(700, 10), to_unsigned(282, 10), to_unsigned(454, 10)),
        5 => (to_unsigned(600, 10), to_unsigned(648, 10), to_unsigned(282, 10)),
        6 => (to_unsigned(650, 10), to_unsigned(571, 10), to_unsigned(724, 10)),
        7 => (to_unsigned(550, 10), to_unsigned(684, 10), to_unsigned(512, 10))
    );

    -- ========================================================================
    -- Delay pipeline signals
    -- ========================================================================
    signal s_avid_d    : std_logic := '0';
    signal s_hsync_n_d : std_logic := '1';
    signal s_vsync_n_d : std_logic := '1';
    signal s_field_n_d : std_logic := '1';
    signal s_y_d       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_u_d       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_v_d       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- Interpolator output signals
    -- ========================================================================
    signal s_mix_y_result : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mix_u_result : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mix_v_result : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mix_y_valid  : std_logic;
    signal s_mix_u_valid  : std_logic;
    signal s_mix_v_valid  : std_logic;

begin

    -- ========================================================================
    -- LFSR instance (free-running)
    -- ========================================================================
    u_lfsr : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => (others => '0'),
            load   => '0',
            q      => s_lfsr_out
        );

    -- ========================================================================
    -- Parameter decode
    -- ========================================================================
    p_params : process(clk)
    begin
        if rising_edge(clk) then
            s_speed_skip   <= not unsigned(registers_in(0)(9 downto 4));
            s_color_sel    <= unsigned(registers_in(1)(9 downto 7));
            s_bright_sel   <= unsigned(registers_in(2)(9 downto 7));
            s_dim_sel      <= unsigned(registers_in(3)(9 downto 7));
            s_seed_density <= unsigned(registers_in(4)(9 downto 0));
            s_video_reveal <= registers_in(6)(0);
            s_pause        <= registers_in(6)(1);
            s_show_grid    <= registers_in(6)(2);
            s_reseed       <= registers_in(6)(3);
            s_bypass       <= registers_in(6)(4);
            s_mix_amount   <= unsigned(registers_in(7)(9 downto 0));
            s_timing_id    <= registers_in(8)(3 downto 0);
        end if;
    end process p_params;

    -- ========================================================================
    -- Timing-dependent cell dimensions
    -- ========================================================================
    p_timing : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing_id = C_NTSC then
                s_cell_w <= to_unsigned(45, 8);
                s_cell_h <= to_unsigned(15, 7);
            elsif s_timing_id = C_PAL then
                s_cell_w <= to_unsigned(45, 8);
                s_cell_h <= to_unsigned(18, 7);
            elsif s_timing_id = C_480P then
                s_cell_w <= to_unsigned(45, 8);
                s_cell_h <= to_unsigned(30, 7);
            elsif s_timing_id = C_576P then
                s_cell_w <= to_unsigned(45, 8);
                s_cell_h <= to_unsigned(36, 7);
            elsif s_timing_id = C_720P50 or s_timing_id = C_720P5994 or s_timing_id = C_720P60 then
                s_cell_w <= to_unsigned(80, 8);
                s_cell_h <= to_unsigned(45, 7);
            elsif s_timing_id = C_1080I50 or s_timing_id = C_1080I5994 or s_timing_id = C_1080I60 then
                s_cell_w <= to_unsigned(120, 8);
                s_cell_h <= to_unsigned(33, 7);
            else
                s_cell_w <= to_unsigned(120, 8);
                s_cell_h <= to_unsigned(67, 7);
            end if;
        end if;
    end process p_timing;

    -- ========================================================================
    -- Pixel counting and cell coordinate mapping
    -- ========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync <= data_in.hsync_n;
            s_prev_vsync <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_line_had_avid <= '1';
                if s_subcell_col >= s_cell_w - 1 then
                    s_subcell_col <= (others => '0');
                    if s_render_col < to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                        s_render_col <= s_render_col + 1;
                    end if;
                else
                    s_subcell_col <= s_subcell_col + 1;
                end if;
            end if;

            if s_subcell_col = 0 then
                s_is_grid_h <= '1';
            else
                s_is_grid_h <= '0';
            end if;
            if s_subcell_row = 0 then
                s_is_grid_v <= '1';
            else
                s_is_grid_v <= '0';
            end if;

            if s_prev_hsync = '1' and data_in.hsync_n = '0' then
                s_subcell_col <= (others => '0');
                s_render_col  <= (others => '0');
                if s_line_had_avid = '1' then
                    if s_subcell_row >= s_cell_h - 1 then
                        s_subcell_row <= (others => '0');
                        if s_render_row < to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                            s_render_row <= s_render_row + 1;
                        end if;
                    else
                        s_subcell_row <= s_subcell_row + 1;
                    end if;
                end if;
                s_line_had_avid <= '0';
            end if;

            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                s_subcell_col   <= (others => '0');
                s_subcell_row   <= (others => '0');
                s_render_col    <= (others => '0');
                s_render_row    <= (others => '0');
                s_line_had_avid <= '0';
            end if;
        end if;
    end process p_counters;

    -- ========================================================================
    -- Game logic: update grid during vertical blanking
    -- ========================================================================
    p_game : process(clk)
        variable v_c, v_r     : integer range 0 to C_GRID_SIZE - 1;
        variable v_cl, v_cr   : integer range 0 to C_GRID_SIZE - 1;
        variable v_ru, v_rd   : integer range 0 to C_GRID_SIZE - 1;
        variable v_n           : integer range 0 to 8;
        variable v_row_above   : std_logic_vector(C_GRID_SIZE - 1 downto 0);
        variable v_row_here    : std_logic_vector(C_GRID_SIZE - 1 downto 0);
        variable v_row_below   : std_logic_vector(C_GRID_SIZE - 1 downto 0);
        variable v_alive_next  : std_logic;
    begin
        if rising_edge(clk) then
            s_game_vsync <= data_in.vsync_n;
            s_prev_reseed <= s_reseed;

            if s_seeding = '1' then
                v_c := to_integer(s_update_col);
                v_r := to_integer(s_update_row);

                if unsigned(s_lfsr_out(9 downto 0)) < s_seed_density then
                    v_alive_next := '1';
                else
                    v_alive_next := '0';
                end if;

                if s_show_a = '1' then
                    grid_a(v_r)(v_c) <= v_alive_next;
                else
                    grid_b(v_r)(v_c) <= v_alive_next;
                end if;

                if s_update_col = to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                    s_update_col <= (others => '0');
                    if s_update_row = to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                        s_seeding    <= '0';
                        s_update_col <= (others => '0');
                        s_update_row <= (others => '0');
                    else
                        s_update_row <= s_update_row + 1;
                    end if;
                else
                    s_update_col <= s_update_col + 1;
                end if;

            elsif s_updating = '1' then
                v_c  := to_integer(s_update_col);
                v_r  := to_integer(s_update_row);
                v_cl := to_integer(s_update_col - 1);
                v_cr := to_integer(s_update_col + 1);
                v_ru := to_integer(s_update_row - 1);
                v_rd := to_integer(s_update_row + 1);

                if s_show_a = '1' then
                    v_row_above := grid_a(v_ru);
                    v_row_here  := grid_a(v_r);
                    v_row_below := grid_a(v_rd);
                else
                    v_row_above := grid_b(v_ru);
                    v_row_here  := grid_b(v_r);
                    v_row_below := grid_b(v_rd);
                end if;

                v_n := 0;
                if v_row_above(v_cl) = '1' then v_n := v_n + 1; end if;
                if v_row_above(v_c)  = '1' then v_n := v_n + 1; end if;
                if v_row_above(v_cr) = '1' then v_n := v_n + 1; end if;
                if v_row_here(v_cl)  = '1' then v_n := v_n + 1; end if;
                if v_row_here(v_cr)  = '1' then v_n := v_n + 1; end if;
                if v_row_below(v_cl) = '1' then v_n := v_n + 1; end if;
                if v_row_below(v_c)  = '1' then v_n := v_n + 1; end if;
                if v_row_below(v_cr) = '1' then v_n := v_n + 1; end if;

                if v_n = 3 then
                    v_alive_next := '1';
                elsif v_n = 2 and v_row_here(v_c) = '1' then
                    v_alive_next := '1';
                else
                    v_alive_next := '0';
                end if;

                if s_show_a = '1' then
                    grid_b(v_r)(v_c) <= v_alive_next;
                else
                    grid_a(v_r)(v_c) <= v_alive_next;
                end if;

                if s_update_col = to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                    s_update_col <= (others => '0');
                    if s_update_row = to_unsigned(C_GRID_SIZE - 1, C_GRID_BITS) then
                        s_updating   <= '0';
                        s_update_col <= (others => '0');
                        s_update_row <= (others => '0');
                        s_show_a     <= not s_show_a;
                    else
                        s_update_row <= s_update_row + 1;
                    end if;
                else
                    s_update_col <= s_update_col + 1;
                end if;

            elsif s_game_vsync = '1' and data_in.vsync_n = '0' then
                if s_reseed = '1' and s_prev_reseed = '0' then
                    s_seeding    <= '1';
                    s_update_col <= (others => '0');
                    s_update_row <= (others => '0');
                elsif s_pause = '0' then
                    if s_frame_counter >= s_speed_skip then
                        s_frame_counter <= (others => '0');
                        s_updating      <= '1';
                        s_update_col    <= (others => '0');
                        s_update_row    <= (others => '0');
                    else
                        s_frame_counter <= s_frame_counter + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_game;

    -- ========================================================================
    -- Render Pipeline Stage 0: register input + grid coordinates
    -- ========================================================================
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_y   <= unsigned(data_in.y);
            s0_u   <= unsigned(data_in.u);
            s0_v   <= unsigned(data_in.v);
            s0_col <= s_render_col;
            s0_row <= s_render_row;
            if s_show_grid = '1' and (s_is_grid_h = '1' or s_is_grid_v = '1') then
                s0_is_grid <= '1';
            else
                s0_is_grid <= '0';
            end if;
        end if;
    end process p_stage0;

    -- ========================================================================
    -- Render Pipeline Stage 1: cell lookup from display grid
    -- ========================================================================
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_y <= s0_y;
            s1_u <= s0_u;
            s1_v <= s0_v;
            s1_is_grid <= s0_is_grid;
            if s_show_a = '1' then
                s1_is_alive <= grid_a(to_integer(s0_row))(to_integer(s0_col));
            else
                s1_is_alive <= grid_b(to_integer(s0_row))(to_integer(s0_col));
            end if;
        end if;
    end process p_stage1;

    -- ========================================================================
    -- Render Pipeline Stage 2: color output mux (shift-based, no multiply)
    -- ========================================================================
    p_stage2 : process(clk)
        variable v_pal_idx  : integer range 0 to 7;
        variable v_bright_y : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_dim_y    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_pal_idx := to_integer(s_color_sel);

            case s_bright_sel is
                when "000"  => v_bright_y := (others => '0');
                when "001"  => v_bright_y := "0000000" & s1_y(9 downto 7);
                when "010"  => v_bright_y := "00000" & s1_y(9 downto 5);
                when "011"  => v_bright_y := "000" & s1_y(9 downto 3);
                when "100"  => v_bright_y := "00" & s1_y(9 downto 2);
                when "101"  => v_bright_y := "0" & s1_y(9 downto 1);
                when "110"  => v_bright_y := s1_y;
                when others => v_bright_y := s1_y;
            end case;

            case s_dim_sel is
                when "000"  => v_dim_y := (others => '0');
                when "001"  => v_dim_y := "0000000" & s1_y(9 downto 7);
                when "010"  => v_dim_y := "00000" & s1_y(9 downto 5);
                when "011"  => v_dim_y := "000" & s1_y(9 downto 3);
                when "100"  => v_dim_y := "00" & s1_y(9 downto 2);
                when "101"  => v_dim_y := "0" & s1_y(9 downto 1);
                when "110"  => v_dim_y := s1_y;
                when others => v_dim_y := s1_y;
            end case;

            if s1_is_grid = '1' then
                s_proc_y <= to_unsigned(64, C_VIDEO_DATA_WIDTH);
                s_proc_u <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
                s_proc_v <= to_unsigned(512, C_VIDEO_DATA_WIDTH);
            elsif s1_is_alive = '1' then
                if s_video_reveal = '1' then
                    s_proc_y <= v_bright_y;
                    s_proc_u <= s1_u;
                    s_proc_v <= s1_v;
                else
                    s_proc_y <= C_PALETTE(v_pal_idx)(0);
                    s_proc_u <= C_PALETTE(v_pal_idx)(1);
                    s_proc_v <= C_PALETTE(v_pal_idx)(2);
                end if;
            else
                s_proc_y <= v_dim_y;
                s_proc_u <= s1_u;
                s_proc_v <= s1_v;
            end if;

            s_proc_valid <= '1';
        end if;
    end process p_stage2;

    -- ========================================================================
    -- Interpolator Stage: wet/dry mix (4 clocks each)
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
            enable => s_proc_valid,
            a      => unsigned(s_y_d),
            b      => s_proc_y,
            t      => s_mix_amount,
            result => s_mix_y_result,
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
            enable => s_proc_valid,
            a      => unsigned(s_u_d),
            b      => s_proc_u,
            t      => s_mix_amount,
            result => s_mix_u_result,
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
            enable => s_proc_valid,
            a      => unsigned(s_v_d),
            b      => s_proc_v,
            t      => s_mix_amount,
            result => s_mix_v_result,
            valid  => s_mix_v_valid
        );

    -- ========================================================================
    -- Sync and Data Delay Pipeline
    -- ========================================================================
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_avid_delay  : t_sync_delay := (others => '0');
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
        type t_data_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_y_delay : t_data_delay := (others => (others => '0'));
        variable v_u_delay : t_data_delay := (others => (others => '0'));
        variable v_v_delay : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid_delay  := data_in.avid    & v_avid_delay(0 to C_SYNC_DELAY_CLKS - 2);
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_SYNC_DELAY_CLKS - 2);

            s_avid_d    <= v_avid_delay(C_SYNC_DELAY_CLKS - 1);
            s_hsync_n_d <= v_hsync_delay(C_SYNC_DELAY_CLKS - 1);
            s_vsync_n_d <= v_vsync_delay(C_SYNC_DELAY_CLKS - 1);
            s_field_n_d <= v_field_delay(C_SYNC_DELAY_CLKS - 1);

            v_y_delay := data_in.y & v_y_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_delay := data_in.u & v_u_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_delay := data_in.v & v_v_delay(0 to C_PROCESSING_DELAY_CLKS - 2);

            s_y_d <= v_y_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_u_d <= v_u_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_v_d <= v_v_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    -- ========================================================================
    -- Output Assignment
    -- ========================================================================
    data_out.y <= std_logic_vector(s_mix_y_result);
    data_out.u <= std_logic_vector(s_mix_u_result);
    data_out.v <= std_logic_vector(s_mix_v_result);

    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hsync_n_d;
    data_out.vsync_n <= s_vsync_n_d;
    data_out.field_n <= s_field_n_d;

end architecture gameoflife;
