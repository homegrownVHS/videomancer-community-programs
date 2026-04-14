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
--   Uses a true bitmap so arbitrary shapes draw correctly with no sweep/fill
--   artifacts. Canvas is 64x32 pixels (2048 entries x 10-bit) stored in a
--   single BRAM tile to guarantee inference.
--
--   During vertical blanking, Bresenham line + brush stamp writes "1023"
--   for drawn pixels. During active video, each pixel is mapped to a
--   canvas coordinate. If the stored value is non-zero -> drawn.
--
-- Resources:
--   1 BRAM (10-bit x 2048, same as variable_delay_u)
--   ~2000 LUTs (estimated)
--
-- Pipeline:
--   Stage 0 (input register + BRAM addr):       1 clock  -> T+1
--   Stage 1 (BRAM read result):                 1 clock  -> T+2
--   Stage 2 (drawn/cursor detect):              1 clock  -> T+3
--   Stage 3 (color output mux):                 1 clock  -> T+4
--   interpolator_u x3 (wet/dry mix):            4 clocks -> T+8
--   Total: 8 clocks
--
-- Submodules:
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   H Position (cursor X)
--   Pot 2  (registers_in(1)):   V Position (cursor Y)
--   Pot 3  (registers_in(2)):   Brush Size
--   Pot 4  (registers_in(3)):   Draw Color Hue (steps_8)
--   Pot 5  (registers_in(4)):   Draw Brightness
--   Pot 6  (registers_in(5)):   (unused)
--   Tog 7  (registers_in(6)(0)): Mode (Color / Video Reveal)
--   Tog 8  (registers_in(6)(1)): Cursor (Off / On)
--   Tog 9  (registers_in(6)(2)): (unused)
--   Tog 10 (registers_in(6)(3)): Clear Canvas
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   C_PROCESSING_DELAY_CLKS = 4 (inline stages)
--   C_SYNC_DELAY_CLKS       = 8 (4 + 4 interpolator)

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
    constant C_PROCESSING_DELAY_CLKS : integer := 4;
    constant C_SYNC_DELAY_CLKS       : integer := 8;

    -- Canvas: 64 x 32 = 2048 pixels stored as 10-bit words
    constant C_CANVAS_W      : integer := 64;
    constant C_CANVAS_H      : integer := 32;
    constant C_CANVAS_W_BITS : integer := 6;
    constant C_CANVAS_H_BITS : integer := 5;
    constant C_BRAM_DEPTH    : integer := 11;  -- 2^11 = 2048
    constant C_DRAWN_VAL     : std_logic_vector(9 downto 0) := "1111111111";
    constant C_CLEAR_VAL     : std_logic_vector(9 downto 0) := "0000000000";

    -- BRAM: 2048 x 10-bit (matches variable_delay_u / howler pattern)
    type t_bram is array(0 to 2**C_BRAM_DEPTH - 1)
        of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal bram_canvas : t_bram := (others => (others => '0'));

    -- Parameters
    signal s_cursor_x_raw : unsigned(9 downto 0);
    signal s_cursor_y_raw : unsigned(9 downto 0);
    signal s_brush_size   : unsigned(2 downto 0);
    signal s_color_sel    : unsigned(2 downto 0);
    signal s_draw_bright  : unsigned(9 downto 0);
    signal s_video_reveal : std_logic;
    signal s_show_cursor  : std_logic;
    signal s_clear        : std_logic;
    signal s_bypass       : std_logic;
    signal s_mix          : unsigned(9 downto 0);

    -- Cursor in canvas coords
    signal s_cursor_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_cursor_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');

    -- Timing
    signal s_active_width : unsigned(10 downto 0) := to_unsigned(1280, 11);
    signal s_active_lines : unsigned(9 downto 0)  := to_unsigned(720, 10);

    -- Pixel counters
    signal s_pixel_col     : unsigned(10 downto 0) := (others => '0');
    signal s_line_count    : unsigned(9 downto 0)  := (others => '0');
    signal s_prev_hsync    : std_logic := '1';
    signal s_prev_vsync    : std_logic := '1';
    signal s_line_had_avid : std_logic := '0';

    -- Clear
    signal s_clear_active : std_logic := '1';
    signal s_clear_addr   : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_prev_clear   : std_logic := '0';

    -- Bresenham + brush
    type t_brush_state is (BRUSH_IDLE, BRUSH_SETUP, BRUSH_DRAW);
    signal s_brush_state     : t_brush_state := BRUSH_IDLE;
    signal s_prev_cursor_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_prev_cursor_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');
    signal s_line_x          : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_line_y          : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');
    signal s_line_end_x      : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_line_end_y      : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');
    signal s_line_dx         : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_line_dy         : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');
    signal s_line_x_inc      : std_logic := '1';
    signal s_line_y_inc      : std_logic := '1';
    signal s_line_err        : signed(7 downto 0) := (others => '0');
    signal s_line_done       : std_logic := '0';

    -- Brush stamp
    signal s_stamp_active : std_logic := '0';
    signal s_stamp_bx     : signed(3 downto 0) := (others => '0');
    signal s_stamp_by     : signed(3 downto 0) := (others => '0');
    signal s_stamp_limit  : signed(3 downto 0) := (others => '0');

    -- BRAM write port
    signal s_wr_en   : std_logic := '0';
    signal s_wr_addr : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_wr_data : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- BRAM read port (separate signals)
    signal s_rd_addr : unsigned(C_BRAM_DEPTH - 1 downto 0) := (others => '0');
    signal s_rd_data : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Render mapping
    signal s_render_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s_render_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');

    -- Pipeline stages
    signal s0_y   : unsigned(9 downto 0) := (others => '0');
    signal s0_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_v   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_col : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s0_row : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');

    signal s1_drawn : std_logic := '0';
    signal s1_col   : unsigned(C_CANVAS_W_BITS - 1 downto 0) := (others => '0');
    signal s1_row   : unsigned(C_CANVAS_H_BITS - 1 downto 0) := (others => '0');
    signal s1_y     : unsigned(9 downto 0) := (others => '0');
    signal s1_u     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_v     : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s2_is_drawn  : std_logic := '0';
    signal s2_is_cursor : std_logic := '0';
    signal s2_y         : unsigned(9 downto 0) := (others => '0');
    signal s2_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2_v         : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s3_y : unsigned(9 downto 0) := (others => '0');
    signal s3_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s3_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Dry/bypass/interp
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

    -- Color LUT
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

    -- Parameter decode
    s_cursor_x_raw <= unsigned(registers_in(0)(9 downto 0));
    s_cursor_y_raw <= unsigned(registers_in(1)(9 downto 0));
    s_brush_size   <= unsigned(registers_in(2)(9 downto 7));
    s_color_sel    <= unsigned(registers_in(3)(9 downto 7));
    s_draw_bright  <= unsigned(registers_in(4)(9 downto 0));
    s_video_reveal <= registers_in(6)(0);
    s_show_cursor  <= registers_in(6)(1);
    s_clear        <= registers_in(6)(3);
    s_bypass       <= registers_in(6)(4);
    s_mix          <= unsigned(registers_in(7)(9 downto 0));

    -- Timing
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

    -- Cursor scale: knob(0..1023) -> canvas cols(0..63) / rows(0..31)
    p_cursor_scale : process(clk)
    begin
        if rising_edge(clk) then
            s_cursor_col <= s_cursor_x_raw(9 downto 4);
            s_cursor_row <= s_cursor_y_raw(9 downto 5);
        end if;
    end process p_cursor_scale;

    -- Map active pixel to canvas coords
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
            if v_col >= to_unsigned(C_CANVAS_W, C_CANVAS_W_BITS) then
                s_render_col <= to_unsigned(C_CANVAS_W - 1, C_CANVAS_W_BITS);
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
            if v_row >= to_unsigned(C_CANVAS_H, C_CANVAS_H_BITS) then
                s_render_row <= to_unsigned(C_CANVAS_H - 1, C_CANVAS_H_BITS);
            else
                s_render_row <= v_row;
            end if;
        end if;
    end process p_render_map;

    -- Pixel/line counting
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

    -- Bresenham line drawing + brush stamp + clear (single process)
    p_brush : process(clk)
        variable v_dx      : unsigned(C_CANVAS_W_BITS - 1 downto 0);
        variable v_dy      : unsigned(C_CANVAS_H_BITS - 1 downto 0);
        variable v_x_inc   : std_logic;
        variable v_y_inc   : std_logic;
        variable v_e2      : signed(8 downto 0);
        variable v_new_err : signed(7 downto 0);
        variable v_px_col  : signed(7 downto 0);
        variable v_px_row  : signed(6 downto 0);
        variable v_addr    : unsigned(C_BRAM_DEPTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_clear <= s_clear;
            -- Default: no BRAM write (write-enable off)
            s_wr_en <= '0';

            -- Clear request
            if s_clear = '1' and s_prev_clear = '0' then
                s_clear_active <= '1';
                s_clear_addr   <= (others => '0');
                s_brush_state  <= BRUSH_IDLE;
                s_stamp_active <= '0';
                s_wr_en <= '0';
            end if;

            -- Vsync -> trigger drawing
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                if s_clear_active = '0' then
                    s_brush_state <= BRUSH_SETUP;
                end if;
            end if;

            if s_clear_active = '1' then
                s_wr_en   <= '1';
                s_wr_data <= C_CLEAR_VAL;
                s_wr_addr <= s_clear_addr;
                if s_clear_addr = to_unsigned(2**C_BRAM_DEPTH - 1, C_BRAM_DEPTH) then
                    s_clear_active    <= '0';
                    s_prev_cursor_col <= s_cursor_col;
                    s_prev_cursor_row <= s_cursor_row;
                else
                    s_clear_addr <= s_clear_addr + 1;
                end if;
            else
                case s_brush_state is
                    when BRUSH_IDLE =>
                        null;

                    when BRUSH_SETUP =>
                        if s_cursor_col >= s_prev_cursor_col then
                            v_dx := s_cursor_col - s_prev_cursor_col;
                            v_x_inc := '1';
                        else
                            v_dx := s_prev_cursor_col - s_cursor_col;
                            v_x_inc := '0';
                        end if;
                        if s_cursor_row >= s_prev_cursor_row then
                            v_dy := s_cursor_row - s_prev_cursor_row;
                            v_y_inc := '1';
                        else
                            v_dy := s_prev_cursor_row - s_cursor_row;
                            v_y_inc := '0';
                        end if;

                        s_line_dx    <= v_dx;
                        s_line_dy    <= v_dy;
                        s_line_x_inc <= v_x_inc;
                        s_line_y_inc <= v_y_inc;
                        s_line_x     <= s_prev_cursor_col;
                        s_line_y     <= s_prev_cursor_row;
                        s_line_end_x <= s_cursor_col;
                        s_line_end_y <= s_cursor_row;
                        s_line_err   <= signed(resize(v_dx, 8)) - signed(resize(v_dy, 8));
                        s_line_done  <= '0';

                        s_stamp_active <= '1';
                        s_stamp_bx     <= -signed(resize(s_brush_size, 4));
                        s_stamp_by     <= -signed(resize(s_brush_size, 4));
                        s_stamp_limit  <= signed(resize(s_brush_size, 4));
                        s_brush_state  <= BRUSH_DRAW;

                    when BRUSH_DRAW =>
                        if s_stamp_active = '1' then
                            v_px_col := signed(resize(s_line_x, 8)) + resize(s_stamp_bx, 8);
                            v_px_row := signed(resize(s_line_y, 7)) + resize(s_stamp_by, 7);
                            if v_px_col >= 0 and v_px_col < C_CANVAS_W and
                               v_px_row >= 0 and v_px_row < C_CANVAS_H then
                                v_addr := unsigned(v_px_col(C_CANVAS_W_BITS - 1 downto 0))
                                        & unsigned(v_px_row(C_CANVAS_H_BITS - 1 downto 0));
                                s_wr_en   <= '1';
                                s_wr_data <= C_DRAWN_VAL;
                                s_wr_addr <= v_addr;
                            end if;

                            if s_stamp_bx < s_stamp_limit then
                                s_stamp_bx <= s_stamp_bx + 1;
                            else
                                s_stamp_bx <= -s_stamp_limit;
                                if s_stamp_by < s_stamp_limit then
                                    s_stamp_by <= s_stamp_by + 1;
                                else
                                    s_stamp_active <= '0';
                                end if;
                            end if;
                        else
                            if s_line_done = '1' then
                                s_brush_state     <= BRUSH_IDLE;
                                s_prev_cursor_col <= s_line_end_x;
                                s_prev_cursor_row <= s_line_end_y;
                            else
                                if s_line_x = s_line_end_x and s_line_y = s_line_end_y then
                                    s_line_done <= '1';
                                    s_stamp_active <= '1';
                                    s_stamp_bx <= -s_stamp_limit;
                                    s_stamp_by <= -s_stamp_limit;
                                else
                                    v_e2 := resize(s_line_err, 9) + resize(s_line_err, 9);
                                    v_new_err := s_line_err;
                                    if v_e2 > -signed(resize(s_line_dy, 9)) then
                                        v_new_err := v_new_err - signed(resize(s_line_dy, 8));
                                        if s_line_x_inc = '1' then
                                            s_line_x <= s_line_x + 1;
                                        else
                                            s_line_x <= s_line_x - 1;
                                        end if;
                                    end if;
                                    if v_e2 < signed(resize(s_line_dx, 9)) then
                                        v_new_err := v_new_err + signed(resize(s_line_dx, 8));
                                        if s_line_y_inc = '1' then
                                            s_line_y <= s_line_y + 1;
                                        else
                                            s_line_y <= s_line_y - 1;
                                        end if;
                                    end if;
                                    s_line_err <= v_new_err;
                                    s_stamp_active <= '1';
                                    s_stamp_bx <= -s_stamp_limit;
                                    s_stamp_by <= -s_stamp_limit;
                                end if;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_brush;

    -- BRAM write: conditional (like howler pattern)
    p_bram_write : process(clk)
    begin
        if rising_edge(clk) then
            if s_wr_en = '1' then
                bram_canvas(to_integer(s_wr_addr)) <= s_wr_data;
            end if;
        end if;
    end process p_bram_write;

    -- BRAM read: dedicated process for yosys inference (howler pattern)
    p_bram_read : process(clk)
    begin
        if rising_edge(clk) then
            s_rd_data <= bram_canvas(to_integer(s_rd_addr));
        end if;
    end process p_bram_read;

    -- BRAM read address: combinational from render coords
    s_rd_addr <= resize(s_render_col, C_BRAM_DEPTH - C_CANVAS_H_BITS)
               & s_render_row;

    -- Stage 0: input register
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

    -- Stage 1: BRAM result + pipeline
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            if s_rd_data /= C_CLEAR_VAL then
                s1_drawn <= '1';
            else
                s1_drawn <= '0';
            end if;
            s1_col <= s0_col;
            s1_row <= s0_row;
            s1_y   <= s0_y;
            s1_u   <= s0_u;
            s1_v   <= s0_v;
        end if;
    end process p_stage1;

    -- Stage 2: cursor detect
    p_stage2 : process(clk)
        variable v_dx : unsigned(C_CANVAS_W_BITS - 1 downto 0);
        variable v_dy : unsigned(C_CANVAS_H_BITS - 1 downto 0);
    begin
        if rising_edge(clk) then
            s2_y <= s1_y;
            s2_u <= s1_u;
            s2_v <= s1_v;
            s2_is_drawn <= s1_drawn;

            if s1_col >= s_cursor_col then
                v_dx := s1_col - s_cursor_col;
            else
                v_dx := s_cursor_col - s1_col;
            end if;
            if s1_row >= s_cursor_row then
                v_dy := s1_row - s_cursor_row;
            else
                v_dy := s_cursor_row - s1_row;
            end if;
            if s_show_cursor = '1' and v_dx <= 1 and v_dy <= 1 then
                s2_is_cursor <= '1';
            else
                s2_is_cursor <= '0';
            end if;
        end if;
    end process p_stage2;

    -- Stage 3: output color
    p_stage3 : process(clk)
        variable v_idx : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_idx := to_integer(s_color_sel);
            if s2_is_cursor = '1' then
                s3_y <= to_unsigned(1023, 10);
                s3_u <= to_unsigned(512, 10);
                s3_v <= to_unsigned(512, 10);
            elsif s2_is_drawn = '1' then
                if s_video_reveal = '1' then
                    s3_y <= s2_y;
                    s3_u <= s2_u;
                    s3_v <= s2_v;
                else
                    s3_y <= s_draw_bright;
                    s3_u <= C_COLORS(v_idx).u;
                    s3_v <= C_COLORS(v_idx).v;
                end if;
            else
                -- Undrawn: show dimmed input video (1/4 brightness)
                s3_y <= "00" & s2_y(9 downto 2);
                s3_u <= s2_u;
                s3_v <= s2_v;
            end if;
        end if;
    end process p_stage3;

    -- Valid pipeline
    p_valid : process(clk)
        type t_valid_pipe is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_valid : t_valid_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_valid := data_in.avid & v_valid(0 to C_PROCESSING_DELAY_CLKS - 2);
            s_interp_valid <= v_valid(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_valid;

    -- Interpolators (wet/dry mix)
    u_interp_y : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_y, b => s3_y, t => s_mix,
                  result => s_mix_y, valid => s_mix_y_valid);

    u_interp_u : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_u, b => s3_u, t => s_mix,
                  result => s_mix_u, valid => open);

    u_interp_v : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => s_interp_valid,
                  a => s_dry_v, b => s3_v, t => s_mix,
                  result => s_mix_v, valid => open);

    -- Sync delay + bypass + dry tap
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');
        type t_data_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic_vector(9 downto 0);
        variable v_y_bypass : t_data_delay := (others => (others => '0'));
        variable v_u_bypass : t_data_delay := (others => (others => '0'));
        variable v_v_bypass : t_data_delay := (others => (others => '0'));
        type t_dry_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of unsigned(9 downto 0);
        variable v_y_dry : t_dry_delay := (others => (others => '0'));
        variable v_u_dry : t_dry_delay := (others => (others => '0'));
        variable v_v_dry : t_dry_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_SYNC_DELAY_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_SYNC_DELAY_CLKS - 2);
            v_y_bypass := data_in.y & v_y_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_u_bypass := data_in.u & v_u_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_v_bypass := data_in.v & v_v_bypass(0 to C_SYNC_DELAY_CLKS - 2);
            v_y_dry := unsigned(data_in.y) & v_y_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_dry := unsigned(data_in.u) & v_u_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_dry := unsigned(data_in.v) & v_v_dry(0 to C_PROCESSING_DELAY_CLKS - 2);
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

    -- Output mux
    data_out.y <= s_bypass_y when s_bypass = '1' else std_logic_vector(s_mix_y);
    data_out.u <= s_bypass_u when s_bypass = '1' else std_logic_vector(s_mix_u);
    data_out.v <= s_bypass_v when s_bypass = '1' else std_logic_vector(s_mix_v);

end architecture etchasketch;
