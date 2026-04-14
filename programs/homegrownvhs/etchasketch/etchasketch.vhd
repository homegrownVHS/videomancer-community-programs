-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: etchasketch.vhd - Etch-a-Sketch / Video Reveal for Videomancer
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
-- Program Name:        Etch-a-Sketch
-- Author:              Adam Pflanzer
-- Overview:
--   Two-knob etch-a-sketch with persistent bitmap canvas stored in BRAM.
--   Canvas is 64x32 pixels (2048 x 10-bit). Stamps cursor position each
--   vsync. BRAM pattern matches howler exactly (separate read/write procs).
--
-- Resources:
--   1 BRAM (10-bit x 2048), ~1500 LUTs
--
-- Pipeline:
--   Stage 0 (input register):         1 clock  -> T+1
--   BRAM read (parallel with stage0): 1 clock  -> T+1
--   Stage 1 (drawn/cursor detect):    1 clock  -> T+2
--   Stage 2 (color output mux):       1 clock  -> T+3
--   interpolator_u x3 (wet/dry mix):  4 clocks -> T+7
--   Total: 7 clocks
--
-- Parameters:
--   Pot 1  (registers_in(0)):   H Position (cursor X)
--   Pot 2  (registers_in(1)):   V Position (cursor Y)
--   Pot 3  (registers_in(2)):   Brush Size (unused for now)
--   Pot 4  (registers_in(3)):   Draw Color Hue (steps_8)
--   Pot 5  (registers_in(4)):   Draw Brightness
--   Tog 7  (registers_in(6)(0)): Mode (Color / Video Reveal)
--   Tog 8  (registers_in(6)(1)): Cursor (Off / On)
--   Tog 10 (registers_in(6)(3)): Clear Canvas
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 3 (inline stages)
--   C_SYNC_DELAY_CLKS       = 7 (3 + 4 interpolator)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture etchasketch of program_top is

    constant C_VIDEO_DATA_WIDTH      : integer := 10;
    constant C_PROCESSING_DELAY_CLKS : integer := 3;
    constant C_SYNC_DELAY_CLKS       : integer := 7;

    -- Canvas dimensions
    constant C_CANVAS_W_BITS : integer := 6;  -- 64 columns
    constant C_CANVAS_H_BITS : integer := 5;  -- 32 rows
    constant C_BRAM_DEPTH    : integer := 11;  -- 2^11 = 2048

    -- BRAM: 2048 x 10-bit (howler-style dual-port inference)
    type t_bram is array(0 to 2**C_BRAM_DEPTH - 1)
        of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal bram_canvas : t_bram := (others => (others => '0'));

    -- BRAM read port
    signal s_rd_addr : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_rd_data : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- BRAM write port
    signal s_wr_en   : std_logic := '0';
    signal s_wr_addr : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_wr_data : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Parameters (directly wired)
    signal s_cursor_col   : unsigned(C_CANVAS_W_BITS - 1 downto 0);
    signal s_cursor_row   : unsigned(C_CANVAS_H_BITS - 1 downto 0);
    signal s_color_sel    : unsigned(2 downto 0);
    signal s_draw_bright  : unsigned(9 downto 0);
    signal s_video_reveal : std_logic;
    signal s_show_cursor  : std_logic;
    signal s_clear        : std_logic;
    signal s_bypass       : std_logic;
    signal s_mix          : unsigned(9 downto 0);

    -- Timing detection
    signal s_active_width : unsigned(10 downto 0) := to_unsigned(1280, 11);
    signal s_active_lines : unsigned(9 downto 0)  := to_unsigned(720, 10);

    -- Pixel counters
    signal s_pixel_col     : unsigned(10 downto 0) := (others => '0');
    signal s_line_count    : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_hsync    : std_logic := '1';
    signal s_prev_vsync    : std_logic := '1';
    signal s_line_had_avid : std_logic := '0';

    -- Render mapping (pixel -> canvas coords)
    signal s_render_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_render_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');

    -- Drawing state
    signal s_clear_active : std_logic := '1';
    signal s_clear_addr   : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_prev_clear   : std_logic := '0';

    -- Pipeline stage 0
    signal s0_y   : unsigned(9 downto 0) := (others => '0');
    signal s0_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_v   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s0_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');

    -- Pipeline stage 1
    signal s1_y         : unsigned(9 downto 0) := (others => '0');
    signal s1_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_v         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_is_drawn  : std_logic := '0';
    signal s1_is_cursor : std_logic := '0';

    -- Pipeline stage 2
    signal s2_y : unsigned(9 downto 0) := (others => '0');
    signal s2_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Interpolator signals
    signal s_dry_y        : unsigned(9 downto 0);
    signal s_dry_u        : unsigned(9 downto 0);
    signal s_dry_v        : unsigned(9 downto 0);
    signal s_bypass_y     : std_logic_vector(9 downto 0);
    signal s_bypass_u     : std_logic_vector(9 downto 0);
    signal s_bypass_v     : std_logic_vector(9 downto 0);
    signal s_interp_valid : std_logic := '0';
    signal s_mix_y        : unsigned(9 downto 0);
    signal s_mix_u        : unsigned(9 downto 0);
    signal s_mix_v        : unsigned(9 downto 0);
    signal s_mix_y_valid  : std_logic;

    -- Color palette (8 entries)
    type t_color is record
        y : unsigned(9 downto 0);
        u : unsigned(9 downto 0);
        v : unsigned(9 downto 0);
    end record;
    type t_color_lut is array (0 to 7) of t_color;
    constant C_COLORS : t_color_lut := (
        (y => to_unsigned(800, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
        (y => to_unsigned(600, 10), u => to_unsigned(100, 10), v => to_unsigned(850, 10)),
        (y => to_unsigned(800, 10), u => to_unsigned(100, 10), v => to_unsigned(600, 10)),
        (y => to_unsigned(600, 10), u => to_unsigned(173, 10), v => to_unsigned(83, 10)),
        (y => to_unsigned(500, 10), u => to_unsigned(800, 10), v => to_unsigned(200, 10)),
        (y => to_unsigned(300, 10), u => to_unsigned(900, 10), v => to_unsigned(200, 10)),
        (y => to_unsigned(350, 10), u => to_unsigned(700, 10), v => to_unsigned(850, 10)),
        (y => to_unsigned(700, 10), u => to_unsigned(300, 10), v => to_unsigned(750, 10))
    );

begin

    -- =========================================================================
    -- Parameter decode (combinational)
    -- =========================================================================
    s_cursor_col   <= unsigned(registers_in(0)(9 downto 4));   -- 0-63
    s_cursor_row   <= unsigned(registers_in(1)(9 downto 5));   -- 0-31
    s_color_sel    <= unsigned(registers_in(3)(9 downto 7));
    s_draw_bright  <= unsigned(registers_in(4)(9 downto 0));
    s_video_reveal <= registers_in(6)(0);
    s_show_cursor  <= registers_in(6)(1);
    s_clear        <= registers_in(6)(3);
    s_bypass       <= registers_in(6)(4);
    s_mix          <= unsigned(registers_in(7)(9 downto 0));

    -- =========================================================================
    -- Timing detection
    -- =========================================================================
    p_timing : process(clk)
    begin
        if rising_edge(clk) then
            case registers_in(8)(3 downto 0) is
                when C_NTSC =>
                    s_active_width <= to_unsigned(720, 11);
                    s_active_lines <= to_unsigned(243, 10);
                when C_PAL =>
                    s_active_width <= to_unsigned(720, 11);
                    s_active_lines <= to_unsigned(288, 10);
                when C_480P =>
                    s_active_width <= to_unsigned(720, 11);
                    s_active_lines <= to_unsigned(480, 10);
                when C_576P =>
                    s_active_width <= to_unsigned(720, 11);
                    s_active_lines <= to_unsigned(576, 10);
                when C_720P50 | C_720P5994 | C_720P60 =>
                    s_active_width <= to_unsigned(1280, 11);
                    s_active_lines <= to_unsigned(720, 10);
                when C_1080I50 | C_1080I5994 | C_1080I60 =>
                    s_active_width <= to_unsigned(1920, 11);
                    s_active_lines <= to_unsigned(540, 10);
                when others =>
                    s_active_width <= to_unsigned(1920, 11);
                    s_active_lines <= to_unsigned(1023, 10);
            end case;
        end if;
    end process p_timing;

    -- =========================================================================
    -- Pixel/line counting
    -- =========================================================================
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync <= data_in.hsync_n;
            s_prev_vsync <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_pixel_col <= s_pixel_col + 1;
                s_line_had_avid <= '1';
            end if;

            if s_prev_hsync = '1' and data_in.hsync_n = '0' then
                s_pixel_col <= (others => '0');
                if s_line_had_avid = '1' then
                    s_line_count <= s_line_count + 1;
                end if;
                s_line_had_avid <= '0';
            end if;

            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                s_line_count    <= (others => '0');
                s_line_had_avid <= '0';
            end if;
        end if;
    end process p_counters;

    -- =========================================================================
    -- Map active pixel to canvas coords (registered)
    -- =========================================================================
    p_render_map : process(clk)
        variable v_col : unsigned(C_CANVAS_W_BITS - 1 downto 0);
        variable v_row : unsigned(C_CANVAS_H_BITS - 1 downto 0);
    begin
        if rising_edge(clk) then
            if s_active_width >= to_unsigned(1280, 11) then
                v_col := s_pixel_col(10 downto 5);
            else
                v_col := s_pixel_col(9 downto 4);
            end if;
            if v_col >= 64 then
                s_render_col <= to_unsigned(63, C_CANVAS_W_BITS);
            else
                s_render_col <= v_col;
            end if;

            if s_active_lines >= to_unsigned(540, 10) then
                v_row := s_line_count(9 downto 5);
            elsif s_active_lines >= to_unsigned(480, 10) then
                v_row := s_line_count(8 downto 4);
            else
                v_row := s_line_count(7 downto 3);
            end if;
            if v_row >= 32 then
                s_render_row <= to_unsigned(31, C_CANVAS_H_BITS);
            else
                s_render_row <= v_row;
            end if;
        end if;
    end process p_render_map;

    -- =========================================================================
    -- BRAM read address (combinational from render coords)
    -- =========================================================================
    s_rd_addr <= resize(s_render_col, C_BRAM_DEPTH - C_CANVAS_H_BITS)
               & s_render_row;

    -- =========================================================================
    -- BRAM read: dedicated process (howler pattern, exactly)
    -- =========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            s_rd_data <= bram_canvas(to_integer(s_rd_addr));
        end if;
    end process;

    -- =========================================================================
    -- BRAM write: dedicated process (howler pattern, exactly)
    -- =========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            if s_wr_en = '1' then
                bram_canvas(to_integer(s_wr_addr)) <= s_wr_data;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Drawing: stamp cursor on vsync, clear on toggle
    -- Very simple - one pixel written per vsync frame.
    -- =========================================================================
    p_draw : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_clear <= s_clear;
            s_wr_en <= '0';

            if s_clear = '1' and s_prev_clear = '0' then
                s_clear_active <= '1';
                s_clear_addr   <= (others => '0');
            end if;

            if s_clear_active = '1' then
                s_wr_en   <= '1';
                s_wr_addr <= s_clear_addr;
                s_wr_data <= (others => '0');
                if s_clear_addr = to_unsigned(2047, C_BRAM_DEPTH) then
                    s_clear_active <= '0';
                else
                    s_clear_addr <= s_clear_addr + 1;
                end if;
            elsif s_prev_vsync = '1' and data_in.vsync_n = '0' then
                s_wr_en   <= '1';
                s_wr_addr <= resize(s_cursor_col, C_BRAM_DEPTH - C_CANVAS_H_BITS)
                           & s_cursor_row;
                s_wr_data <= (others => '1');
            end if;
        end if;
    end process p_draw;

    -- =========================================================================
    -- Stage 0 (T+1): Register input data + canvas coords
    -- =========================================================================
    p_stage0 : process(clk)
    begin
        if rising_edge(clk) then
            s0_y   <= unsigned(data_in.y);
            s0_u   <= unsigned(data_in.u);
            s0_v   <= unsigned(data_in.v);
            s0_col <= s_render_col;
            s0_row <= s_render_row;
        end if;
    end process p_stage0;

    -- =========================================================================
    -- Stage 1 (T+2): Check BRAM result + cursor detect
    -- =========================================================================
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_y <= s0_y;
            s1_u <= s0_u;
            s1_v <= s0_v;

            if s_rd_data /= "0000000000" then
                s1_is_drawn <= '1';
            else
                s1_is_drawn <= '0';
            end if;

            if s_show_cursor = '1' and
               s0_col = s_cursor_col and s0_row = s_cursor_row then
                s1_is_cursor <= '1';
            else
                s1_is_cursor <= '0';
            end if;
        end if;
    end process p_stage1;

    -- =========================================================================
    -- Stage 2 (T+3): Color output mux
    -- =========================================================================
    p_stage2 : process(clk)
        variable v_idx : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_idx := to_integer(s_color_sel);
            if s1_is_cursor = '1' then
                s2_y <= to_unsigned(1023, 10);
                s2_u <= to_unsigned(512, 10);
                s2_v <= to_unsigned(512, 10);
            elsif s1_is_drawn = '1' then
                if s_video_reveal = '1' then
                    s2_y <= s1_y;
                    s2_u <= s1_u;
                    s2_v <= s1_v;
                else
                    s2_y <= s_draw_bright;
                    s2_u <= C_COLORS(v_idx).u;
                    s2_v <= C_COLORS(v_idx).v;
                end if;
            else
                s2_y <= "00" & s1_y(9 downto 2);
                s2_u <= s1_u;
                s2_v <= s1_v;
            end if;
        end if;
    end process p_stage2;

    -- =========================================================================
    -- Valid pipeline
    -- =========================================================================
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    -- =========================================================================
    -- Interpolators (wet/dry mix)
    -- =========================================================================
    u_interp_y : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_y, b => s2_y, t => s_mix,
                  result => s_mix_y, valid => s_mix_y_valid);

    u_interp_u : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_u, b => s2_u, t => s_mix,
                  result => s_mix_u, valid => open);

    u_interp_v : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_v, b => s2_v, t => s_mix,
                  result => s_mix_v, valid => open);

    -- =========================================================================
    -- Sync delay + bypass delay + dry tap
    -- =========================================================================
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
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);
            data_out.hsync_n <= v_hsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync_n(C_SYNC_DELAY_CLKS - 1);
            data_out.field_n <= v_field_n(C_SYNC_DELAY_CLKS - 1);
            data_out.avid    <= v_avid   (C_SYNC_DELAY_CLKS - 1);

            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            s_bypass_y <= v_y_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_u <= v_u_bypass(C_SYNC_DELAY_CLKS - 1);
            s_bypass_v <= v_v_bypass(C_SYNC_DELAY_CLKS - 1);

            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_dry_y <= v_y_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_u <= v_u_dry(C_PROCESSING_DELAY_CLKS - 1);
            s_dry_v <= v_v_dry(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    -- =========================================================================
    -- Output mux
    -- =========================================================================
    data_out.y <= s_bypass_y when s_bypass = '1' else std_logic_vector(s_mix_y);
    data_out.u <= s_bypass_u when s_bypass = '1' else std_logic_vector(s_mix_u);
    data_out.v <= s_bypass_v when s_bypass = '1' else std_logic_vector(s_mix_v);

end architecture etchasketch;
