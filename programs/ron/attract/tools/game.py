#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ATTRACT game program for the DODECA board's CPU (see dasm.py for the ISA).

Layout of the code:
  lib_*      shared routines: sprites, tiles, text, score, palettes, random
  shell      the attract cycle: static burst -> title card -> READY! ->
             demo play -> GAME OVER -> next game
  dm_*       DOT MATRIX (maze chase)

Conventions: X is the actor index inside actor loops; t0..t7 are scratch;
routines document what they clobber.  Screen = 32x30 tiles, rows 0-1 and 29
are the HUD; sprite x/y are canvas pixels (x may run -16..255 via bit 8).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dasm      # noqa: E402
import assets    # noqa: E402
from dasm import (P_TADDR, P_TDATA, P_CADDR, P_CDATA, P_SIDX, P_SPRY,   # noqa: E402
                  P_SPRA, P_YSCR, P_STAR, P_SWAP, P_SEED, I_TDATA, I_K1,
                  I_K4, I_SW, I_P12, I_RAND, I_FRAME)


def c332(r, g, b):
    return (r << 5) | (g << 2) | b


# palettes (3-3-2).  Tile palettes 0-3 at ctl 0..15, sprite 0-7 at 32..63.
BLACK = 0
TILE_PALS = [
    [BLACK, c332(0, 2, 3), c332(0, 0, 2), c332(3, 5, 3)],   # 0 walls (style colour)
    [c332(1, 1, 1), c332(7, 5, 2), c332(5, 3, 1), c332(7, 7, 3)],   # 1 dots (entry 4 = star col 1)
    [c332(4, 4, 2), c332(7, 7, 3), c332(4, 4, 2), c332(7, 2, 1)],   # 2 text white
    [c332(2, 2, 3), c332(7, 6, 0), c332(7, 2, 0), c332(3, 7, 3)],   # 3 HUD gold
]
SPR_PALS = [
    [0, c332(1, 6, 3), c332(0, 0, 1), c332(7, 7, 3)],   # 0 BLIP
    [0, c332(7, 4, 0), c332(3, 1, 0), c332(7, 7, 2)],   # 1 moth orange
    [0, c332(5, 2, 3), c332(2, 0, 1), c332(7, 6, 3)],   # 2 moth violet
    [0, c332(0, 5, 2), c332(0, 2, 1), c332(6, 7, 3)],   # 3 moth teal
    [0, c332(7, 3, 2), c332(3, 1, 1), c332(7, 6, 3)],   # 4 moth rose
    [0, c332(3, 4, 3), c332(1, 1, 2), c332(7, 7, 3)],   # 5 pale (dazzled) moth
    [0, c332(7, 5, 0), c332(4, 2, 0), c332(7, 7, 3)],   # 6 items / sparkle
    [0, c332(7, 2, 0), c332(7, 6, 0), c332(7, 7, 3)],   # 7 effects / ember
]
WALL_COLS = [   # (main, shade, highlight) per stage
    (c332(0, 2, 3), c332(0, 0, 2), c332(3, 5, 3)),      # electric blue
    (c332(6, 0, 2), c332(3, 0, 1), c332(7, 4, 3)),      # magenta
    (c332(7, 3, 0), c332(3, 1, 0), c332(7, 6, 1)),      # orange
    (c332(0, 5, 1), c332(0, 2, 0), c332(4, 7, 2)),      # emerald
]

# sprite attribute bits (word 1)
S16 = 1 << 14
VF = 1 << 13
HF = 1 << 12


def spal(p):
    return ((p & 3) << 10) | (((p >> 2) & 1) << 9)


def tpal(p):
    return (p & 3) << 8


class Prog(dasm.Asm):
    def __init__(self, T):
        super().__init__()
        self.T = T
        self.ov_base = None
        self.ov_max = 0

    def overlay(self):
        """Start a game's private variables: every game's vars share the
        same RAM after the library's (only one game runs at a time)."""
        if self.ov_base is None:
            self.ov_base = self.nvar
        self.ov_max = max(self.ov_max, self.nvar)
        self.nvar = self.ov_base

    def overlay_end(self):
        self.ov_max = max(self.ov_max, self.nvar)
        self.nvar = self.ov_max

    # ---- small macros
    def lda(self, v):
        if isinstance(v, int) and -1024 <= v <= 1023:
            self.ldi(v)
        elif isinstance(v, int):
            v &= 0xFFFF
            self.ldi(v & 255)
            self.ldh(v >> 8)
        else:
            self.ldi(('lo', v))
            self.ldh(('hi', v))

    def ldxa(self, label):           # X = label address
        self.lda(label)
        self.tax()

    def inp(self, p):
        self.emit('IN', p)

    def outp(self, p):
        self.emit('OUT', p)

    def sta(self, var, v):
        self.lda(v)
        self.st(var)

    def inc(self, var, d=1):
        if d == 1:
            self.incm(var)
        elif d == -1:
            self.decm(var)
        else:
            self.ld(var)
            self.addi(d)
            self.st(var)

    def dec(self, var):
        self.decm(var)

    def string(self, label, col, row, pal, text):
        """ROM string: taddr, attr, len, chars."""
        self.table(label, row * 32 + col, tpal(pal), len(text), *[ord(ch) for ch in text])

    def frame(self):
        # finish the frame: show the sprites written this tick, wait, and
        # watch K1 (frame_sub may jump straight to another game)
        self.call('frame_sub')

    def call_puts(self, label):
        self.ldxa(label)
        self.call('puts')


# ======================================================================
def build_lib(a):
    T = a.T
    for i in range(8):
        a.var('t%d' % i)
    a.var('sp_x'); a.var('sp_y'); a.var('sp_a')
    a.var('p_n'); a.var('p_attr')
    a.var('tm_c'); a.var('tm_r')
    a.var('score', 6)       # BCD digits, score+0 = ones
    a.var('hi', 6)
    a.var('lives')
    a.var('stage')
    a.var('timer')
    a.var('tick')

    # ---- puts: X = ROM string {taddr, attr, len, chars...}
    a.label('puts')
    a.ldc(0); a.outp(P_TADDR)
    a.ldc(1); a.st('p_attr')
    a.ldc(2); a.st('p_n')
    a.adx(3)
    a.label('puts_l')
    a.ldc(0); a.addi(-32); a.emit('OR', 'p_attr'); a.outp(P_TDATA)
    a.inx()
    a.decm('p_n'); a.jnz('puts_l')
    a.ret()

    # ---- spr_put: (sp_x, sp_y, sp_a) -> next sprite slot
    a.label('spr_put')
    a.ld('sp_y'); a.andi(255); a.swapb(); a.st('t7')
    a.ld('sp_x'); a.andi(255); a.emit('OR', 't7'); a.outp(P_SPRY)
    a.ld('sp_x'); a.andi(256); a.shl4(); a.shl(); a.shl(); a.shl()
    a.emit('OR', 'sp_a'); a.outp(P_SPRA)
    a.ret()

    # ---- load_pals: X = ROM table {ctl addr, count, values...}
    a.label('load_pals')
    a.ldc(0); a.outp(P_CADDR)
    a.ldc(1); a.st('p_n')
    a.adx(2)
    a.label('lp_l')
    a.ldc(0); a.outp(P_CDATA); a.inx()
    a.decm('p_n'); a.jnz('lp_l')
    a.ret()

    # ---- cls: whole tilemap to blank (pal 0), row scroll 0, y scroll 0
    a.label('cls')
    a.cla(); a.outp(P_TADDR)
    a.lda(1024); a.tax()
    a.label('cls_l')
    a.cla(); a.outp(P_TDATA)
    a.loop('cls_l')
    a.lda(64); a.outp(P_CADDR)
    a.lxi(32)
    a.label('cls_r')
    a.cla(); a.outp(P_CDATA)
    a.loop('cls_r')
    a.cla(); a.outp(P_YSCR)
    a.ret()

    # ---- rowscroll_set: rows 2..28 = A (playfield), HUD rows 0
    a.label('rowscroll_pf')
    a.st('t6')
    a.lda(64); a.outp(P_CADDR)
    a.cla(); a.outp(P_CDATA); a.outp(P_CDATA)
    a.lxi(27)
    a.label('rsp_l')
    a.ld('t6'); a.outp(P_CDATA)
    a.loop('rsp_l')
    a.ret()

    # ---- tile address: A = (tm_r)*32 + tm_c  (screen tile coords)
    a.label('taddr')
    a.ld('tm_r'); a.shl4(); a.shl(); a.emit('ADD', 'tm_c')
    a.ret()

    # ---- wait_ticks: wait A ticks (keeps sprites as last drawn? no: the
    # hardware clears the sprite table each tick, so callers that want
    # sprites must redraw; this one shows none)
    a.label('wait_ticks')
    a.st('timer')
    a.label('wt_l')
    a.frame()
    a.decm('timer'); a.jnz('wt_l')
    a.ret()

    # ---- score: add BCD value in A (4 digits) to score
    a.var('as_v'); a.var('as_c'); a.var('as_t'); a.var('as_x')
    a.label('add_score')
    a.st('as_v')
    a.txa(); a.st('as_x')
    a.lxi(0)
    a.clr('as_c')            # carry
    a.label('as_l')
    a.ld('as_v'); a.andi(15); a.emit('ADD', 'as_c'); a.st('as_t')
    a.ldx('score'); a.emit('ADD', 'as_t'); a.st('as_t')
    a.clr('as_c')
    a.ld('as_t'); a.cmpi(10); a.jc('as_nc')
    a.addi(-10); a.st('as_t'); a.ldi(1); a.st('as_c')
    a.label('as_nc')
    a.ld('as_t'); a.stx('score')
    a.ld('as_v'); a.shr4(); a.st('as_v')
    a.inx()
    a.txa(); a.cmpi(6); a.jnz('as_l')
    a.ldxm('as_x')
    a.ret()

    # ---- draw_score: score digits at row 1 cols 1..6, blank leading
    # zeros but always the last two
    a.label('draw_score')
    a.lda(32 + 1); a.outp(P_TADDR)
    a.lxi(5)
    a.clr('t3')           # seen a non-zero digit
    a.label('ds_l')
    a.ldx('score'); a.st('t2')
    a.jnz('ds_nz')
    a.ld('t3'); a.jnz('ds_dig')
    a.txa(); a.cmpi(2); a.jc('ds_dig')     # x < 2: always shown
    a.lda(tpal(3)); a.outp(P_TDATA); a.jmp('ds_next')
    a.label('ds_nz')
    a.ldi(1); a.st('t3')
    a.label('ds_dig')
    a.ld('t2'); a.addi(ord('0') - 32); a.ori(tpal(3)); a.outp(P_TDATA)
    a.label('ds_next')
    a.dex()
    a.txa(); a.cmpi(-1); a.jnz('ds_l')
    # high score at cols 13..18
    a.lda(32 + 13); a.outp(P_TADDR)
    a.lxi(5)
    a.clr('t3')
    a.label('dh_l')
    a.ldx('hi'); a.st('t2')
    a.jnz('dh_nz')
    a.ld('t3'); a.jnz('dh_dig')
    a.txa(); a.cmpi(2); a.jc('dh_dig')
    a.lda(tpal(3)); a.outp(P_TDATA); a.jmp('dh_next')
    a.label('dh_nz')
    a.ldi(1); a.st('t3')
    a.label('dh_dig')
    a.ld('t2'); a.addi(ord('0') - 32); a.ori(tpal(3)); a.outp(P_TDATA)
    a.label('dh_next')
    a.dex()
    a.txa(); a.cmpi(-1); a.jnz('dh_l')
    a.ret()

    # ---- hi_update: if score > hi, copy
    a.label('hi_update')
    a.lxi(5)
    a.label('hu_l')
    a.ldx('score'); a.st('t0'); a.ldx('hi'); a.st('t1')
    a.ld('t0'); a.cmp('t1'); a.jc('hu_done'); a.jnz('hu_copy')
    a.dex(); a.txa(); a.cmpi(-1); a.jnz('hu_l')
    a.ret()
    a.label('hu_copy')
    a.lxi(5)
    a.label('hu_c')
    a.ldx('score'); a.stx('hi')
    a.dex(); a.txa(); a.cmpi(-1); a.jnz('hu_c')
    a.label('hu_done')
    a.ret()

    # ---- draw_lives: life icons on row 29 from col 2 (lives-1 icons)
    a.label('draw_lives')
    a.lda(29 * 32 + 2); a.outp(P_TADDR)
    a.lxi(4)
    a.ld('lives'); a.addi(-1); a.st('t0')
    a.label('dl_l')
    a.ld('t0'); a.jz('dl_blank'); a.jn('dl_blank')
    a.lda(T['life'] | tpal(3)); a.outp(P_TDATA)
    a.decm('t0'); a.jmp('dl_next')
    a.label('dl_blank')
    a.cla(); a.outp(P_TDATA)
    a.label('dl_next')
    a.loop('dl_l')
    a.ret()

    # ---- hud labels
    a.string('s_1up', 2, 0, 3, '1UP')
    a.string('s_hi', 11, 0, 3, 'HIGH SCORE')
    a.string('s_ready', 13, 17, 3, 'READY!')
    a.string('s_gameover', 11, 15, 3, 'GAME  OVER')
    a.string('s_copy', 9, 22, 3, '@ 1981 DODECA')
    a.string('s_insert', 10, 26, 3, 'INSERT COIN')
    a.string('s_credit', 11, 29, 3, 'CREDIT 0')
    a.string('s_ramok', 2, 4, 3, 'RAM OK')
    a.string('s_romok', 2, 6, 3, 'ROM OK')
    a.string('s_noready', 13, 17, 0, '      ')

    # ---- BLIP's CRT power-off at (sp_x, sp_y): hit-stop, then collapse
    # to a line, a dot, a glow; poweron runs it backwards.  Clobbers t5.
    a.table('off_frames', *[S16 | T['blip_off%d' % i] for i in (0, 1, 2, 2, 3, 3, 4, 4)])
    a.label('poweroff')
    a.clr('t5')
    a.label('po_l')
    a.cla(); a.outp(P_SIDX)
    a.ld('t5'); a.shr(); a.shr(); a.shr(); a.tax()
    a.ldc('off_frames'); a.st('sp_a'); a.call('spr_put')
    a.frame()
    a.incm('t5'); a.cmpi(48); a.jnz('po_l')
    a.ldi(30); a.jmp('wait_ticks')
    a.label('poweron')
    a.ldi(40); a.st('t5')
    a.label('pn_l')
    a.cla(); a.outp(P_SIDX)
    a.ld('t5'); a.shr(); a.shr(); a.shr(); a.tax()
    a.ldc('off_frames'); a.st('sp_a'); a.call('spr_put')
    a.frame()
    a.decm('t5'); a.jnz('pn_l')
    a.ret()

    # ---- new_game: score 0, 3 lives, level seed = K4*16 | A
    a.label('new_game')
    a.st('t0')
    a.ldi(3); a.st('lives')
    a.lxi(6)
    a.label('ng_l')
    a.cla(); a.stx(('score', -1)); a.loop('ng_l')
    a.inp(I_K4); a.shl4(); a.emit('OR', 't0'); a.outp(P_SEED)
    a.ret()

    # ---- game_over: HUD, high score, GAME OVER card (attract mode only)
    a.label('game_over')
    a.call('hud'); a.call('hi_update'); a.call('draw_score')
    a.inp(I_SW); a.andi(16); a.jz('go_ret')
    a.call_puts('s_gameover')
    a.ldi(120); a.call('wait_ticks')
    a.label('go_ret')
    a.ret()

    # ---- K1 zone -> A: 0 = Cycle, 1..NGAMES = that game.  Labels divide the
    # knob evenly; the firmware may hand a labelled default over as the
    # label INDEX (< NGAMES+1), so small values are taken as an index.
    a.var('k1_v'); a.var('k1_z'); a.var('k1_t'); a.var('k1_pend'); a.var('k1_cnt')
    a.var('game')
    a.label('k1_zone')
    a.inp(I_K1); a.st('k1_v')
    a.cmpi(NGAMES + 1); a.jc('kz_idx')
    a.clr('k1_z')
    a.lxi(0)
    a.label('kz_l')
    a.ldc('k1_zones'); a.st('k1_t')
    a.ld('k1_v'); a.cmp('k1_t'); a.jc('kz_done')
    a.incm('k1_z'); a.inx(); a.txa(); a.cmpi(NGAMES); a.jnz('kz_l')
    a.label('kz_done')
    a.ld('k1_z')
    a.ret()
    a.label('kz_idx')
    a.ld('k1_v')
    a.ret()

    # ---- frame_sub: show this tick's sprites, wait for the tick, then
    # watch K1: a different game held for 8 ticks switches immediately.
    a.label('frame_sub')
    a.outp(P_SWAP)
    a.emit('SYS', dasm.SYSC['WAIT'])
    a.call('k1_zone'); a.jz('fs_clr')              # Cycle: never interrupts
    a.addi(-1); a.cmp('game'); a.jz('fs_clr')      # already playing it
    a.cmp('k1_pend'); a.jz('fs_cnt')
    a.st('k1_pend'); a.clr('k1_cnt'); a.ret()
    a.label('fs_cnt')
    a.incm('k1_cnt'); a.cmpi(8); a.jc('fs_ret')
    a.ld('k1_pend'); a.st('game'); a.clr('k1_cnt')
    a.jmp('k1_switch')
    a.label('fs_clr')
    a.clr('k1_cnt')
    a.label('fs_ret')
    a.ret()

    a.label('hud')
    a.call_puts('s_1up')
    a.call_puts('s_hi')
    a.call('draw_score')
    a.call('draw_lives')
    a.ret()

    # ---- palettes
    tp = [c for p in TILE_PALS for c in p]
    a.table('pal_default', 0, 16, *tp)
    sp = [c for p in SPR_PALS for c in p]
    a.table('pal_sprites', 32, 32, *sp)

    # ---- static burst + self test (the channel change between games)
    a.label('static')
    a.ldi(14); a.call('static_burst')
    a.call('cls')
    a.call_puts('s_ramok')
    a.ldi(20); a.call('wait_ticks')
    a.call_puts('s_romok')
    a.ldi(30); a.call('wait_ticks')
    a.jmp('cls')

    # A frames of channel-change static: random tiles, vertical roll
    a.label('static_burst')
    a.st('t5')
    a.label('st_frame')
    a.cla(); a.outp(P_TADDR)
    a.lda(960); a.tax()
    a.label('st_l')
    a.inp(I_RAND); a.andi(0x3FF); a.outp(P_TDATA)
    a.loop('st_l')
    a.inp(I_RAND); a.andi(255); a.outp(P_YSCR)
    a.frame()
    a.decm('t5'); a.jnz('st_frame')
    a.ret()


# ======================================================================
# DOT MATRIX -- maze chase
# Maze = 27x27 tiles at screen tile (2,2); node (i,j) at maze tile
# (2i+1, 2j+1), i,j in 0..12.  Actors move in maze pixels (tile centre =
# 8*t+4).  Screen sprite x = px + 12 (the playfield rows scroll -4 px to
# centre the 27-tile maze), y = py + 8.
# ======================================================================
NGAMES = 3
DM_N = 5                  # actors: 0 = BLIP, 1..4 = moths
DIRX = [0, 1, 0, -1]      # up right down left
DIRY = [-1, 0, 1, 0]


def build_dotmatrix(a):
    T = a.T
    a.overlay()
    WALLMARK = T['fill1']
    for n in ('ax', 'ay', 'ad', 'aspd', 'ast', 'atim', 'aanim'):
        a.var(n, 8)
    for n in ('dm_dots', 'dm_pow', 'dm_style', 'dm_vis', 'dm_cur', 'dm_nb',
              'dm_tx', 'dm_ty', 'dm_best', 'dm_bestd', 'dm_dir', 'dm_chain',
              'dm_dead', 'dm_clear', 'dm_mode', 'dm_modet', 'dm_k', 'dm_gx', 'dm_gy',
              'dm_score_d', 'dm_item', 'dm_itemt', 'dm_eaten', 'dm_found'):
        a.var(n)

    # dx/dy tables in ROM (sign-extended words)
    a.table('dirx', *[v & 0xFFFF for v in DIRX])
    a.table('diry', *[v & 0xFFFF for v in DIRY])
    a.table('dm_moth_pal', spal(1), spal(2), spal(3), spal(4))
    a.table('dm_corner_x', 25 * 8 + 4, 1 * 8 + 4, 25 * 8 + 4, 1 * 8 + 4)
    a.table('dm_corner_y', 1 * 8 + 4, 1 * 8 + 4, 25 * 8 + 4, 25 * 8 + 4)
    a.table('dm_wallpal', *[c for trip in WALL_COLS for c in trip])
    a.table('dm_bits', 1, 2, 4, 8)
    a.table('dm_mdir', 0, 1, 3, 2)
    a.table('dm_mtim', 40, 80, 120, 160)
    a.table('dm_walbase', T['wall_tube_0'], T['wall_outline_0'], T['wall_tube_0'], T['wall_outline_0'])

    # ---- mt_addr: A = screen tile address of maze tile (tm_c, tm_r)
    a.label('mt_addr')
    a.ld('tm_r'); a.shl4(); a.shl(); a.emit('ADD', 'tm_c'); a.addi(66)
    a.ret()

    # ---- mt_get: A = tile index (low 8 bits) at maze (tm_c, tm_r);
    # outside the maze reads as wall
    a.label('mt_get')
    a.ld('tm_c'); a.cmpi(27); a.jnc('mtg_out')
    a.ld('tm_r'); a.cmpi(27); a.jnc('mtg_out')
    a.call('mt_addr'); a.outp(P_TADDR); a.inp(I_TDATA); a.andi(255)
    a.ret()
    a.label('mtg_out')
    a.lda(WALLMARK)
    a.ret()

    # ---- mt_set: maze tile (tm_c, tm_r) = A (full 12-bit word)
    a.label('mt_set')
    a.st('t7')
    a.call('mt_addr'); a.outp(P_TADDR)
    a.ld('t7'); a.outp(P_TDATA)
    a.ret()

    # ---- generation ------------------------------------------------------
    # 1. fill with wall marks; 2. Aldous-Broder random walk over the left
    # half nodes (i 0..6) carving into path; 3. braid dead ends; 4. mirror;
    # 5. convert: walls -> join tiles, paths -> dots, corners -> power.
    a.label('dm_gen')
    a.clr('tm_r')
    a.label('g_fr')
    a.clr('tm_c')
    a.call('mt_addr'); a.outp(P_TADDR)
    a.lxi(27)
    a.label('g_fc')
    a.lda(WALLMARK); a.outp(P_TDATA)
    a.loop('g_fc')
    a.incm('tm_r'); a.cmpi(27); a.jnz('g_fr')
    # start node: (3, 6); carve it
    a.ldi(3); a.st('dm_tx'); a.ldi(6); a.st('dm_ty')
    a.ldi(7); a.st('tm_c'); a.ldi(13); a.st('tm_r')
    a.cla(); a.call('mt_set')
    a.ldi(1); a.st('dm_vis')
    a.label('g_walk')
    a.inp(I_RAND); a.andi(3); a.st('dm_dir'); a.tax()
    a.ldc('dirx'); a.emit('ADD', 'dm_tx'); a.st('t0')
    a.jn('g_walk'); a.cmpi(7); a.jnc('g_walk')
    a.ldc('diry'); a.emit('ADD', 'dm_ty'); a.st('t1')
    a.jn('g_walk'); a.cmpi(13); a.jnc('g_walk')
    # neighbour tile = (2*t0+1, 2*t1+1)
    a.ld('t0'); a.shl(); a.addi(1); a.st('tm_c')
    a.ld('t1'); a.shl(); a.addi(1); a.st('tm_r')
    a.call('mt_get'); a.cmpi(WALLMARK); a.jnz('g_move')
    # unvisited: carve it and the wall between
    a.cla(); a.call('mt_set')
    a.ld('dm_tx'); a.emit('ADD', 't0'); a.addi(1); a.st('tm_c')
    a.ld('dm_ty'); a.emit('ADD', 't1'); a.addi(1); a.st('tm_r')
    a.cla(); a.call('mt_set')
    a.inc('dm_vis')
    a.label('g_move')
    a.ld('t0'); a.st('dm_tx'); a.ld('t1'); a.st('dm_ty')
    a.ld('dm_vis'); a.cmpi(91); a.jnz('g_walk')

    # braid: every left-half node with exactly 1 exit opens another
    a.clr('dm_ty')
    a.label('b_row')
    a.clr('dm_tx')
    a.label('b_col')
    a.clr('dm_nb')
    a.lxi(0)
    a.label('b_cnt')
    a.txa(); a.st('t3')
    a.ldc('dirx'); a.emit('ADD', 'dm_tx'); a.emit('ADD', 'dm_tx'); a.addi(1); a.st('tm_c')
    a.ldc('diry'); a.emit('ADD', 'dm_ty'); a.emit('ADD', 'dm_ty'); a.addi(1); a.st('tm_r')
    a.call('mt_get'); a.cmpi(WALLMARK); a.jz('b_cw')
    a.inc('dm_nb')
    a.label('b_cw')
    a.ldxm('t3'); a.inx(); a.txa(); a.cmpi(4); a.jnz('b_cnt')
    a.ld('dm_nb'); a.cmpi(1); a.jnz('b_next')
    # dead end: try random directions until one opens an interior wall
    a.ldi(12); a.st('t4')
    a.label('b_try')
    a.inp(I_RAND); a.andi(3); a.tax()
    a.ldc('dirx'); a.emit('ADD', 'dm_tx'); a.st('t0')
    a.jn('b_fail'); a.cmpi(7); a.jnc('b_fail')
    a.ldc('diry'); a.emit('ADD', 'dm_ty'); a.st('t1')
    a.jn('b_fail'); a.cmpi(13); a.jnc('b_fail')
    a.ld('dm_tx'); a.emit('ADD', 't0'); a.addi(1); a.st('tm_c')
    a.ld('dm_ty'); a.emit('ADD', 't1'); a.addi(1); a.st('tm_r')
    a.call('mt_get'); a.cmpi(WALLMARK); a.jnz('b_fail')
    a.cla(); a.call('mt_set'); a.jmp('b_next')
    a.label('b_fail')
    a.decm('t4'); a.jnz('b_try')
    a.label('b_next')
    a.incm('dm_tx'); a.cmpi(7); a.jnz('b_col')
    a.incm('dm_ty'); a.cmpi(13); a.jnz('b_row')

    # mirror columns 0..12 onto 26..14
    a.clr('tm_r')
    a.label('m_row')
    a.clr('dm_tx')
    a.label('m_col')
    a.ld('dm_tx'); a.st('tm_c'); a.call('mt_get'); a.st('t5')
    a.ldi(26); a.emit('SUB', 'dm_tx'); a.st('tm_c'); a.ld('t5'); a.call('mt_set')
    a.incm('dm_tx'); a.cmpi(13); a.jnz('m_col')
    a.incm('tm_r'); a.cmpi(27); a.jnz('m_row')

    # convert: walls to joins (style base + NESW mask), paths to dots
    a.ld('dm_style'); a.andi(3); a.tax(); a.ldc('dm_walbase'); a.st('t6')
    a.clr('dm_dots')
    a.clr('dm_ty')
    a.label('c_row')
    a.clr('dm_tx')
    a.label('c_col')
    a.ld('dm_tx'); a.st('tm_c'); a.ld('dm_ty'); a.st('tm_r')
    a.call('mt_get'); a.cmpi(WALLMARK); a.jnz('c_path')
    # wall: mask of wall neighbours (inside the maze only), N=1 E=2 S=4 W=8
    a.clr('t5')
    a.lxi(0)
    a.label('c_nb')
    a.stxm('t3')
    a.ldc('dirx'); a.emit('ADD', 'dm_tx'); a.st('tm_c'); a.cmpi(27); a.jnc('c_nbn')
    a.ldc('diry'); a.emit('ADD', 'dm_ty'); a.st('tm_r'); a.cmpi(27); a.jnc('c_nbn')
    a.call('mt_get'); a.st('t4')
    a.cmpi(WALLMARK); a.jz('c_nbw')
    a.ld('t4'); a.emit('SUB', 't6'); a.cmpi(16); a.jnc('c_nbn')
    a.label('c_nbw')
    a.ldxm('t3'); a.ldc('dm_bits'); a.emit('OR', 't5'); a.st('t5')
    a.label('c_nbn')
    a.ldxm('t3'); a.inx(); a.txa(); a.cmpi(4); a.jnz('c_nb')
    a.ld('dm_tx'); a.st('tm_c'); a.ld('dm_ty'); a.st('tm_r')
    a.ld('t5'); a.emit('ADD', 't6'); a.call('mt_set')
    a.jmp('c_next')
    a.label('c_path')
    a.lda(T['dot'] | tpal(1)); a.call('mt_set')
    a.inc('dm_dots')
    a.label('c_next')
    a.incm('dm_tx'); a.cmpi(27); a.jnz('c_col')
    a.incm('dm_ty'); a.cmpi(27); a.jnz('c_row')
    # power dots near the corners, clear BLIP's start and the moth den
    for (mc, mr) in ((1, 3), (25, 3), (1, 23), (25, 23)):
        a.ldi(mc); a.st('tm_c'); a.ldi(mr); a.st('tm_r')
        a.call('mt_get'); a.cmpi(T['dot']); skip = a.uid('pw'); a.jnz(skip)
        a.lda(T['power'] | tpal(1)); a.call('mt_set')
        a.label(skip)
    for (mc, mr) in ((13, 13), (13, 21)):
        a.ldi(mc); a.st('tm_c'); a.ldi(mr); a.st('tm_r')
        a.call('mt_get'); a.cmpi(T['dot']); skip = a.uid('cl'); a.jnz(skip)
        a.cla(); a.call('mt_set'); a.decm('dm_dots')
        a.label(skip)
    a.ret()

    # ---- wall colours for the stage
    a.label('dm_wallcols')
    a.ldi(1); a.outp(P_CADDR)
    a.ld('dm_style'); a.andi(3); a.st('t0'); a.shl(); a.emit('ADD', 't0'); a.tax()
    a.ldc('dm_wallpal'); a.outp(P_CDATA)
    a.ldc(('dm_wallpal', 1)); a.outp(P_CDATA)
    a.ldc(('dm_wallpal', 2)); a.outp(P_CDATA)
    a.ret()

    # ---- reset actors: BLIP at node (6,10) heading left, moths at the den
    a.label('dm_reset_actors')
    a.lda(13 * 8 + 4); a.st('ax'); a.lda(21 * 8 + 4); a.st('ay')
    a.ldi(3); a.st('ad'); a.clr('ast'); a.clr('aanim')
    a.lxi(DM_N - 1)
    a.label('ra_l')
    a.lda(13 * 8 + 4); a.stx('ax'); a.stx('ay')
    a.ldc(('dm_mdir', -1)); a.stx('ad')
    a.cla(); a.stx('ast')
    a.ldc(('dm_mtim', -1)); a.stx('atim')
    a.loop('ra_l')
    a.clr('dm_pow'); a.clr('dm_dead'); a.clr('dm_mode')
    a.lda(420); a.st('dm_modet')
    a.ret()

    # ---- at_center: Z set if actor X is at a tile centre
    # tile_ahead: A = tile index ahead of actor X in direction dm_dir
    a.label('dm_ahead')
    a.ldx('ax'); a.shr(); a.shr(); a.shr(); a.st('tm_c')
    a.ldx('ay'); a.shr(); a.shr(); a.shr(); a.st('tm_r')
    a.txa(); a.st('t3')
    a.ldxm('dm_dir')
    a.ldc('dirx'); a.emit('ADD', 'tm_c'); a.st('tm_c')
    a.ldc('diry'); a.emit('ADD', 'tm_r'); a.st('tm_r')
    a.ldxm('t3')
    a.call('mt_get')
    a.ret()

    # passable: Z clear (A=1) when tile in A is not a wall join
    a.label('dm_pass')
    a.emit('SUB', 'dm_k')            # dm_k = wall base for this style
    a.cmpi(16); a.jc('dp_no')
    a.ldi(1); a.ret()
    a.label('dp_no')
    a.cla(); a.ret()

    # ---- choose direction at a centre for actor X.  Score = -(manhattan
    # distance from the tile ahead to the target) [moths] or dot/danger
    # heuristic [BLIP]; reverse disallowed unless it is the only way.
    a.label('dm_choose')
    a.txa(); a.st('dm_cur')
    a.lda(-30000); a.st('dm_bestd')
    a.ldx('ad'); a.st('dm_best')
    a.clr('dm_found')
    a.clr('t2')                       # candidate dir
    a.label('ch_l')
    a.ld('t2'); a.st('dm_dir')
    a.ld('dm_cur'); a.tax(); a.jz('ch_rev')    # BLIP may turn back
    a.ldx('ad'); a.xori(2); a.cmp('t2'); a.jz('ch_next')     # moths never reverse
    a.label('ch_rev')
    a.call('dm_ahead'); a.call('dm_pass'); a.jz('ch_next')
    # score this exit
    a.ld('dm_cur'); a.tax()
    a.jnz('ch_moth')
    a.call('dm_blip_score'); a.jmp('ch_cmp')
    a.label('ch_moth')
    a.call('dm_moth_score')
    a.label('ch_cmp')
    a.st('t4')
    a.cmp('dm_bestd'); a.jn('ch_next')
    a.ld('t4'); a.st('dm_bestd'); a.ld('t2'); a.st('dm_best')
    a.ldi(1); a.st('dm_found')
    a.label('ch_next')
    a.incm('t2'); a.cmpi(4); a.jnz('ch_l')
    a.ld('dm_found'); a.jnz('ch_set')
    # dead end: reverse
    a.ldxm('dm_cur'); a.ldx('ad'); a.xori(2); a.st('dm_best')
    a.label('ch_set')
    a.ldxm('dm_cur'); a.ld('dm_best'); a.stx('ad')
    a.ret()

    # moth score: -(|tx - ahead_x| + |ty - ahead_y|) (+ random if pale)
    a.label('dm_moth_score')
    a.ld('tm_c'); a.shl(); a.shl(); a.shl(); a.addi(4); a.emit('SUB', 'dm_gx')
    a.jp('ms_ax'); a.neg()
    a.label('ms_ax')
    a.st('t5')
    a.ld('tm_r'); a.shl(); a.shl(); a.shl(); a.addi(4); a.emit('SUB', 'dm_gy')
    a.jp('ms_ay'); a.neg()
    a.label('ms_ay')
    a.emit('ADD', 't5'); a.neg()
    a.st('t5')
    a.ldxm('dm_cur'); a.ldx('ast'); a.cmpi(1); a.jnz('ms_done')
    # pale: random wander
    a.inp(I_RAND); a.andi(63); a.st('t5')
    a.label('ms_done')
    a.ld('t5')
    a.ret()

    # BLIP score: dot ahead +40, power +60, moth danger along the corridor,
    # small random jitter (the "sometimes greedy" demo pilot)
    a.label('dm_blip_score')
    a.inp(I_RAND); a.andi(15); a.st('t6')
    # tile ahead was left in A by dm_pass? recompute
    a.call('dm_ahead'); a.st('t5')
    a.cmpi(T['dot']); a.jnz('bs_np')
    a.ld('t6'); a.addi(40); a.st('t6')
    a.label('bs_np')
    a.ld('t5'); a.cmpi(T['power']); a.jnz('bs_npw')
    a.ld('t6'); a.addi(60); a.st('t6')
    a.label('bs_npw')
    # danger: each normal moth within 40 px that lies in this direction
    a.ldi(1); a.st('t1')
    a.label('bs_m')
    a.ldxm('t1'); a.ldx('ast'); a.jnz('bs_mn')          # only hunting moths
    a.ldx('ax'); a.emit('SUB', 'ax'); a.st('t3')               # moth - blip
    a.ldx('ay'); a.emit('SUB', 'ay'); a.st('t4')
    # |dx|+|dy| < 48 ?
    a.ld('t3'); a.jp('bs_a1'); a.neg()
    a.label('bs_a1')
    a.st('t0')
    a.ld('t4'); a.jp('bs_a2'); a.neg()
    a.label('bs_a2')
    a.emit('ADD', 't0'); a.cmpi(48); a.jnc('bs_mn')
    # in this direction? dot(dir, delta) > 0
    a.ldxm('dm_dir')
    a.ldc('dirx'); a.jz('bs_vy')
    a.jn('bs_xn')
    a.ld('t3'); a.jn('bs_mn'); a.jmp('bs_hit')
    a.label('bs_xn')
    a.ld('t3'); a.jp('bs_mn'); a.jmp('bs_hit')
    a.label('bs_vy')
    a.ldc('diry'); a.jn('bs_yn')
    a.ld('t4'); a.jn('bs_mn'); a.jmp('bs_hit')
    a.label('bs_yn')
    a.ld('t4'); a.jp('bs_mn')
    a.label('bs_hit')
    a.ld('t6'); a.addi(-200); a.st('t6')
    a.label('bs_mn')
    a.incm('t1'); a.cmpi(DM_N); a.jnz('bs_m')
    a.ld('ad'); a.xori(2); a.cmp('dm_dir'); a.jnz('bs_nr')
    a.ld('t6'); a.addi(-30); a.st('t6')
    a.label('bs_nr')
    a.ld('t6')
    a.ret()

    # ---- moth target for actor X -> dm_gx, dm_gy
    a.label('dm_target')
    a.txa(); a.st('t3')
    a.ldx('ast'); a.cmpi(2); a.jnz('tg_n2')
    # ember: home to the den
    a.lda(13 * 8 + 4); a.st('dm_gx'); a.st('dm_gy'); a.ret()
    a.label('tg_n2')
    a.ld('dm_mode'); a.jz('tg_chase')
    # scatter: own corner
    a.ld('t3'); a.addi(-1); a.tax()
    a.ldc('dm_corner_x'); a.st('dm_gx'); a.ldc('dm_corner_y'); a.st('dm_gy')
    a.ldxm('t3'); a.ret()
    a.label('tg_chase')
    a.ld('ax'); a.st('dm_gx'); a.ld('ay'); a.st('dm_gy')
    a.ld('t3'); a.cmpi(2); a.jnz('tg_n3')
    # ambusher: 32 px ahead of BLIP
    a.ldxm('ad')
    a.ldc('dirx'); a.shl4(); a.shl(); a.emit('ADD', 'dm_gx'); a.st('dm_gx')
    a.ldc('diry'); a.shl4(); a.shl(); a.emit('ADD', 'dm_gy'); a.st('dm_gy')
    a.jmp('tg_out')
    a.label('tg_n3')
    a.cmpi(3); a.jnz('tg_n4')
    # flanker: 2*BLIP - chaser
    a.ld('ax'); a.shl(); a.emit('SUB', ('ax', 1)); a.st('dm_gx')
    a.ld('ay'); a.shl(); a.emit('SUB', ('ay', 1)); a.st('dm_gy')
    a.jmp('tg_out')
    a.label('tg_n4')
    a.cmpi(4); a.jnz('tg_out')
    # wanderer: chase when far, its corner when within 64 px
    a.ld(('ax', 4)); a.emit('SUB', 'ax'); a.jp('tg_w1'); a.neg()
    a.label('tg_w1')
    a.st('t0')
    a.ld(('ay', 4)); a.emit('SUB', 'ay'); a.jp('tg_w2'); a.neg()
    a.label('tg_w2')
    a.emit('ADD', 't0'); a.cmpi(64); a.jnc('tg_out')
    a.ldi(3); a.tax(); a.ldc('dm_corner_x'); a.st('dm_gx'); a.ldc('dm_corner_y'); a.st('dm_gy')
    a.label('tg_out')
    a.ldxm('t3')
    a.ret()

    # ---- step actor X by one pixel (with direction decisions at centres)
    a.label('dm_step')
    a.ldx('ax'); a.andi(7); a.cmpi(4); a.jnz('st_go')
    a.ldx('ay'); a.andi(7); a.cmpi(4); a.jnz('st_go')
    # at a tile centre: tunnel wrap is not used; decide or stop
    a.ldx('ax'); a.shr(); a.shr(); a.shr(); a.andi(1); a.jz('st_fwd')
    a.ldx('ay'); a.shr(); a.shr(); a.shr(); a.andi(1); a.jz('st_fwd')
    # node tile: choose
    a.txa(); a.jz('st_blip')
    a.call('dm_target')
    a.label('st_blip')
    a.call('dm_choose')
    a.label('st_fwd')
    a.ldx('ad'); a.st('dm_dir')
    a.call('dm_ahead'); a.call('dm_pass'); a.jnz('st_go')
    # blocked: reverse (corridor end) for safety
    a.ldx('ad'); a.xori(2); a.stx('ad')
    a.ret()
    a.label('st_go')
    a.txa(); a.st('t3')
    a.ldx('ad'); a.tax()
    a.ldc('dirx'); a.st('t0'); a.ldc('diry'); a.st('t1')
    a.ldxm('t3')
    a.ldx('ax'); a.emit('ADD', 't0'); a.stx('ax')
    a.ldx('ay'); a.emit('ADD', 't1'); a.stx('ay')
    a.ret()

    # ---- eat: BLIP at a tile centre eats the dot under him
    a.label('dm_eat')
    a.ld('ax'); a.andi(7); a.cmpi(4); a.jnz('e_none')
    a.ld('ay'); a.andi(7); a.cmpi(4); a.jnz('e_none')
    a.ld('ax'); a.shr(); a.shr(); a.shr(); a.st('tm_c')
    a.ld('ay'); a.shr(); a.shr(); a.shr(); a.st('tm_r')
    a.call('mt_get'); a.st('t0')
    a.cmpi(T['dot']); a.jnz('e_np')
    a.cla(); a.call('mt_set')
    a.ldi(0x10); a.call('add_score')
    a.decm('dm_dots')
    a.ldi(6); a.st('dm_eaten')
    a.ret()
    a.label('e_np')
    a.ld('t0'); a.cmpi(T['power']); a.jnz('e_none')
    a.cla(); a.call('mt_set')
    a.ldi(0x50); a.call('add_score')
    a.decm('dm_dots')
    a.lda(360); a.st('dm_pow')
    a.ldi(0); a.st('dm_chain')
    # hunting moths turn pale and reverse
    a.ldi(1); a.st('t1')
    a.label('e_pl')
    a.ldxm('t1'); a.ldx('ast'); a.jnz('e_pn')
    a.ldi(1); a.stx('ast'); a.ldx('ad'); a.xori(2); a.stx('ad')
    a.label('e_pn')
    a.incm('t1'); a.cmpi(DM_N); a.jnz('e_pl')
    a.label('e_none')
    a.ret()

    # ---- collisions BLIP vs moths
    a.label('dm_collide')
    a.ldi(1); a.st('t1')
    a.label('co_l')
    a.ldxm('t1')
    a.ldx('atim'); a.jnz('co_n')                 # still in the den
    a.ldx('ax'); a.emit('SUB', 'ax'); a.jp('co_a'); a.neg()
    a.label('co_a')
    a.cmpi(7); a.jnc('co_n')
    a.ldx('ay'); a.emit('SUB', 'ay'); a.jp('co_b'); a.neg()
    a.label('co_b')
    a.cmpi(7); a.jnc('co_n')
    a.ldx('ast'); a.jz('co_die')
    a.cmpi(1); a.jnz('co_n')
    # eat a pale moth: it becomes an ember heading home
    a.ldi(2); a.stx('ast')
    a.ldxm('dm_chain'); a.ldc('dm_chainpts'); a.call('add_score')
    a.ld('dm_chain'); a.cmpi(3); a.jz('co_cmax'); a.inc('dm_chain')
    a.label('co_cmax')
    a.ldi(1); a.st('dm_eaten')
    a.jmp('co_n')
    a.label('co_die')
    a.ldi(1); a.st('dm_dead')
    a.ret()
    a.label('co_n')
    a.incm('t1'); a.cmpi(DM_N); a.jnz('co_l')
    a.ret()
    a.table('dm_chainpts', 0x200, 0x400, 0x800, 0x1600)

    # ---- draw sprites
    a.label('dm_draw')
    a.cla(); a.outp(P_SIDX)
    # BLIP: frame by direction, walk anim
    a.ld('ax'); a.addi(12); a.st('sp_x'); a.ld('ay'); a.addi(8); a.st('sp_y')
    a.ldxm('ad'); a.ldc('dm_blipfr'); a.st('t0')
    a.ld('aanim'); a.shr(); a.shr(); a.shr(); a.andi(1); a.jz('dr_b0')
    a.ldc(('dm_blipfr', 4)); a.st('t0')
    a.label('dr_b0')
    a.ld('ad'); a.cmpi(3); a.jnz('dr_bnf2')
    a.lda(HF); a.emit('OR', 't0'); a.st('t0')
    a.label('dr_bnf2')
    a.ld('dm_eaten'); a.jz('dr_bnh')
    a.lda(S16 | spal(0) | T['blip_happy']); a.st('t0')
    a.label('dr_bnh')
    a.ld('t0'); a.st('sp_a'); a.call('spr_put')
    # moths
    a.ldi(1); a.st('t1')
    a.label('dr_m')
    a.ldxm('t1')
    a.ldx('ax'); a.addi(12); a.st('sp_x')
    a.ldx('ay'); a.addi(8); a.st('sp_y')
    a.ldx('ast'); a.cmpi(2); a.jnz('dr_mn')
    # ember: 8x8 sprite, centred
    a.ld('sp_x'); a.addi(4); a.st('sp_x'); a.ld('sp_y'); a.addi(4); a.st('sp_y')
    a.lda(spal(7) | T['ember']); a.st('sp_a'); a.call('spr_put'); a.jmp('dr_mnext')
    a.label('dr_mn')
    a.ld('tick'); a.shr(); a.shr(); a.shr(); a.andi(1); a.jz('dr_f0')
    a.lda(S16 | T['moth1']); a.st('t0'); a.jmp('dr_f1')
    a.label('dr_f0')
    a.lda(S16 | T['moth0']); a.st('t0')
    a.label('dr_f1')
    a.ldxm('t1'); a.ldx('ast'); a.jz('dr_norm')
    # pale; flash own colour in the last 2 s
    a.ld('dm_pow'); a.cmpi(120); a.jnc('dr_pale')
    a.ld('tick'); a.andi(8); a.jz('dr_norm')
    a.label('dr_pale')
    a.lda(S16 | spal(5) | T['moth_pale0']); a.st('sp_a'); a.call('spr_put'); a.jmp('dr_mnext')
    a.label('dr_norm')
    a.ld('t1'); a.addi(-1); a.tax(); a.ldc('dm_moth_pal'); a.emit('OR', 't0'); a.st('sp_a')
    a.call('spr_put')
    a.label('dr_mnext')
    a.incm('t1'); a.cmpi(DM_N); a.jnz('dr_m')
    a.ret()
    a.table('dm_blipfr', S16 | T['blip_back0'], S16 | T['blip_side0'], S16 | T['blip_front0'], S16 | T['blip_side0'],
           S16 | T['blip_back1'], S16 | T['blip_side1'], S16 | T['blip_front1'], S16 | T['blip_side1'])

    # ---- power-off death animation (BLIP frames off0..off4 over ~50 ticks)
    a.label('dm_death')
    a.ldi(40); a.st('t5')
    a.label('dd_freeze')                                   # hit-stop, static face
    a.call('dm_draw')
    a.frame()
    a.decm('t5'); a.jnz('dd_freeze')
    a.ld('ax'); a.addi(12); a.st('sp_x'); a.ld('ay'); a.addi(8); a.st('sp_y')
    a.jmp('poweroff')

    # respawn: power-off in reverse
    a.label('dm_respawn')
    a.ld('ax'); a.addi(12); a.st('sp_x'); a.ld('ay'); a.addi(8); a.st('sp_y')
    a.jmp('poweron')

    # ---- stage: generate + draw HUD + READY!
    a.label('dm_stage')
    # hide the maze while it is built (walls invisible)
    a.ldi(1); a.outp(P_CADDR); a.cla(); a.outp(P_CDATA); a.outp(P_CDATA); a.outp(P_CDATA)
    a.call('dm_gen')
    a.lda(-4 & 0xFF); a.call('rowscroll_pf')
    a.call('dm_wallcols')
    a.ld('dm_style'); a.andi(3); a.tax(); a.ldc('dm_walbase'); a.st('dm_k')
    a.call('hud')
    a.ret()

    # ---- one game of DOT MATRIX (returns at game over)
    a.label('dm_game')
    a.ldi(1); a.call('new_game')
    a.clr('dm_style')
    a.ldi(1); a.st('stage')
    a.label('dg_stage')
    # level seed: K4 + stage
    a.inp(I_K4); a.shl4(); a.emit('ADD', 'stage'); a.ori(1); a.outp(P_SEED)
    a.call('dm_stage')
    a.label('dg_life')
    a.call('dm_reset_actors')
    a.call('hud')
    a.inp(I_SW); a.andi(16); a.jz('dg_go')
    a.call_puts('s_ready')
    a.call('dm_respawn')
    a.ldi(50); a.st('t5')
    a.label('dg_rdy')
    a.call('dm_draw')
    a.frame()
    a.decm('t5'); a.jnz('dg_rdy')
    a.call_puts('s_noready')
    a.label('dg_go')
    # ---- play loop
    a.label('dg_tick')
    a.inc('tick')
    a.ld('dm_eaten'); a.jz('dg_ne'); a.addi(-1); a.st('dm_eaten')
    a.label('dg_ne')
    # scatter / chase phases
    a.decm('dm_modet'); a.jnz('dg_nm')
    a.ld('dm_mode'); a.xori(1); a.st('dm_mode')
    a.jz('dg_mchase')
    a.lda(300); a.st('dm_modet'); a.jmp('dg_nm')
    a.label('dg_mchase')
    a.lda(900); a.st('dm_modet')
    a.label('dg_nm')
    # power timer
    a.ld('dm_pow'); a.jz('dg_np')
    a.addi(-1); a.st('dm_pow'); a.jnz('dg_np')
    a.ldi(1); a.st('t1')
    a.label('dg_unpale')
    a.ldxm('t1'); a.ldx('ast'); a.cmpi(1); a.jnz('dg_up')
    a.cla(); a.stx('ast')
    a.label('dg_up')
    a.incm('t1'); a.cmpi(DM_N); a.jnz('dg_unpale')
    a.label('dg_np')
    # BLIP: 1 px every tick except 1 in 8
    a.ld('tick'); a.andi(7); a.jz('dg_bskip')
    a.lxi(0); a.call('dm_step')
    a.ld('aanim'); a.addi(1); a.st('aanim')
    a.call('dm_eat')
    a.label('dg_bskip')
    # moths
    a.ldi(1); a.st('dm_cur')
    a.label('dg_m')
    a.ldxm('dm_cur')
    a.ldx('atim'); a.jz('dg_mgo')
    a.addi(-1); a.stx('atim'); a.jmp('dg_mnext')
    a.label('dg_mgo')
    # speed: hunting 7/8, pale 1/2, ember 2 px
    a.ldx('ast'); a.jz('dg_mh')
    a.cmpi(1); a.jz('dg_mp')
    a.call('dm_step'); a.ldxm('dm_cur'); a.call('dm_step')
    # ember home?
    a.ldxm('dm_cur')
    a.ldx('ax'); a.cmpi(13 * 8 + 4); a.jnz('dg_mnext')
    a.ldx('ay'); a.cmpi(13 * 8 + 4); a.jnz('dg_mnext')
    a.cla(); a.stx('ast'); a.ldi(30); a.stx('atim')
    a.jmp('dg_mnext')
    a.label('dg_mp')
    a.ld('tick'); a.andi(1); a.jz('dg_mnext')
    a.call('dm_step'); a.jmp('dg_mnext')
    a.label('dg_mh')
    a.ld('tick'); a.andi(7); a.cmpi(3); a.jz('dg_mnext')
    a.call('dm_step')
    a.label('dg_mnext')
    a.incm('dm_cur'); a.cmpi(DM_N); a.jnz('dg_m')
    a.call('dm_collide')
    a.call('draw_score')
    # 1UP blinks
    a.ld('tick'); a.andi(16); a.jz('dg_1on')
    a.ldi(2); a.outp(P_TADDR); a.cla(); a.outp(P_TDATA); a.outp(P_TDATA); a.outp(P_TDATA)
    a.jmp('dg_1done')
    a.label('dg_1on')
    a.call_puts('s_1up')
    a.label('dg_1done')
    # power dots blink (palette entry 7 = dot highlight)
    a.ldi(6); a.outp(P_CADDR)
    a.ld('tick'); a.andi(8); a.jz('dg_pd')
    a.lda(c332(7, 5, 2)); a.outp(P_CDATA); a.jmp('dg_pd2')
    a.label('dg_pd')
    a.lda(c332(1, 1, 0)); a.outp(P_CDATA)
    a.label('dg_pd2')
    a.call('dm_draw')
    a.frame()
    a.ld('dm_dead'); a.jnz('dg_died')
    a.ld('dm_dots'); a.jnz('dg_tick')
    # ---- stage clear: walls flash white 4 times
    a.ldi(8); a.st('t5')
    a.label('dg_flash')
    a.ldi(1); a.outp(P_CADDR)
    a.ld('t5'); a.andi(1); a.jz('dg_fw')
    a.call('dm_wallcols'); a.jmp('dg_fd')
    a.label('dg_fw')
    a.lda(c332(7, 7, 3)); a.outp(P_CDATA); a.outp(P_CDATA); a.outp(P_CDATA)
    a.label('dg_fd')
    a.ldi(12); a.call('wait_ticks')
    a.decm('t5'); a.jnz('dg_flash')
    a.inc('stage'); a.inc('dm_style')
    a.jmp('dg_stage')
    # ---- death
    a.label('dg_died')
    a.call('dm_death')
    a.decm('lives')
    a.jnz('dg_life')
    a.jmp('game_over')



# ======================================================================
# SKYLINE SIEGE -- marching invaders over a destructible city.
# Formation: 4 rows x 6 columns of moths in 32x24 cells; rows: LUNA (30),
# moth (20), moth (20), grub (10).  Cellophane: every sprite takes the
# colour of its horizontal band (red top, white middle, green bottom).
# Buildings are tiles (roof / windows / wall) that erode tile by tile.
# ======================================================================
SS_COLS, SS_ROWS = 6, 4
SS_N = SS_COLS * SS_ROWS
SS_TY = 208                    # turret sprite y (below the city)
SS_SPR_PALS = [
    [0, c332(1, 7, 1), c332(0, 3, 0), c332(5, 7, 3)],   # 0 turret / bottom band green
    [0, c332(7, 1, 2), c332(4, 0, 1), c332(7, 5, 3)],   # 1 top band red-magenta
    [0, c332(6, 6, 3), c332(3, 3, 2), c332(7, 7, 3)],   # 2 middle band white
    [0, c332(1, 7, 1), c332(0, 3, 0), c332(5, 7, 3)],   # 3 bottom band green
    [0, c332(7, 3, 0), c332(7, 6, 0), c332(7, 7, 3)],   # 4 explosions
    [0, c332(7, 1, 2), c332(4, 0, 1), c332(7, 6, 3)],   # 5 MOTHER
    [0, c332(7, 7, 3), c332(5, 5, 2), c332(7, 7, 3)],   # 6 shots
    [0, c332(7, 7, 3), c332(7, 7, 3), c332(7, 7, 3)],   # 7 bolt
]
SS_TILE_PAL0 = [BLACK, c332(1, 6, 1), c332(0, 2, 0), c332(5, 7, 2)]   # green city


def build_skyline(a):
    T = a.T
    a.overlay()
    a.var('ss_alive', SS_N)
    for n in ('ss_fx', 'ss_fy', 'ss_fd', 'ss_n', 'ss_st', 'ss_anim', 'ss_tx',
              'ss_tgt', 'ss_bx', 'ss_by', 'ss_bon', 'ss_ex', 'ss_ey', 'ss_et',
              'ss_ux', 'ss_ud', 'ss_ut', 'ss_dead', 'ss_wave', 'ss_c', 'ss_r',
              'ss_i', 'ss_hx', 'ss_hy', 'ss_tt'):
        a.var(n)
    for n in ('ss_mx', 'ss_my', 'ss_mon'):
        a.var(n, 3)

    a.table('ss_pals', 32, 32, *[c for p in SS_SPR_PALS for c in p])
    a.table('ss_tpal', 0, 4, *SS_TILE_PAL0)
    a.table('ss_rowspr', S16 | T['luna0'], S16 | T['moth0'], S16 | T['moth0'], S16 | T['grub0'])
    a.table('ss_rowpts', 0x30, 0x20, 0x20, 0x10)
    a.table('ss_bldx', 4, 11, 18, 25)      # building left tile columns

    # band palette for a sprite at y = A -> A = spal bits
    a.label('ss_band')
    a.cmpi(88); a.jc('sb_top')
    a.cmpi(160); a.jc('sb_mid')
    a.lda(spal(3)); a.ret()
    a.label('sb_top'); a.lda(spal(1)); a.ret()
    a.label('sb_mid'); a.lda(spal(2)); a.ret()

    # tile under pixel (ss_hx, ss_hy) -> A (tile index), TADDR set
    a.label('ss_tile')
    a.ld('ss_hy'); a.shr(); a.shr(); a.shr(); a.st('tm_r')
    a.ld('ss_hx'); a.shr(); a.shr(); a.shr(); a.st('tm_c')
    a.call('taddr'); a.st('t7'); a.outp(P_TADDR); a.inp(I_TDATA); a.andi(255)
    a.ret()

    # erode the building tile under (ss_hx, ss_hy): Z set (A=0) if no building
    a.label('ss_erode')
    a.call('ss_tile'); a.st('t6')
    a.cmpi(T['bldg_dmg']); a.jz('se_gone')
    a.cmpi(T['roof']); a.jz('se_dmg')
    a.cmpi(T['bldg']); a.jz('se_dmg')
    a.cmpi(T['win_lit']); a.jz('se_dmg')
    a.cmpi(T['win_dark']); a.jz('se_dmg')
    a.cla(); a.ret()
    a.label('se_dmg')
    a.ld('t7'); a.outp(P_TADDR); a.lda(T['bldg_dmg']); a.outp(P_TDATA)
    a.ldi(1); a.ret()
    a.label('se_gone')
    a.ld('t7'); a.outp(P_TADDR); a.cla(); a.outp(P_TDATA)
    a.ldi(1); a.ret()

    # ---- city: 4 buildings of random height (3..6 tiles), ground row 27
    a.label('ss_city')
    a.lda(25 * 32); a.outp(P_TADDR)
    a.lxi(32)
    a.label('sc_g')
    a.lda(T['ground']); a.outp(P_TDATA); a.loop('sc_g')
    a.lda(28 * 32); a.outp(P_TADDR)
    a.lxi(32)
    a.label('sc_g2')
    a.lda(T['ground']); a.outp(P_TDATA); a.loop('sc_g2')
    a.clr('ss_i')
    a.label('sc_b')
    a.inp(I_RAND); a.andi(3); a.addi(3); a.st('ss_r')       # height
    a.ldxm('ss_i'); a.ldc('ss_bldx'); a.st('tm_c')
    a.ldi(24); a.st('tm_r')
    a.label('sc_row')
    a.call('taddr'); a.outp(P_TADDR)
    a.lxi(4)
    a.label('sc_col')
    a.ld('ss_r'); a.cmpi(1); a.jnz('sc_nroof')
    a.lda(T['roof']); a.jmp('sc_put')
    a.label('sc_nroof')
    a.inp(I_RAND); a.andi(3); a.jz('sc_dark')
    a.cmpi(1); a.jz('sc_wall')
    a.lda(T['win_lit']); a.jmp('sc_put')
    a.label('sc_dark'); a.lda(T['win_dark']); a.jmp('sc_put')
    a.label('sc_wall'); a.lda(T['bldg'])
    a.label('sc_put')
    a.outp(P_TDATA); a.loop('sc_col')
    a.decm('tm_r')
    a.decm('ss_r'); a.jnz('sc_row')
    a.incm('ss_i'); a.cmpi(4); a.jnz('sc_b')
    a.ret()

    # ---- new wave: everyone alive, formation at the top (lower each wave)
    a.label('ss_newwave')
    a.lxi(SS_N)
    a.label('sw_l')
    a.ldi(1); a.stx(('ss_alive', -1)); a.loop('sw_l')
    a.ldi(SS_N); a.st('ss_n')
    a.ldi(32); a.st('ss_fx')
    a.ld('ss_wave'); a.andi(3); a.shl(); a.shl(); a.shl(); a.addi(32); a.st('ss_fy')
    a.ldi(4); a.st('ss_fd')
    a.ldi(10); a.st('ss_st')
    a.ret()

    # ---- formation step (period shrinks as moths die)
    a.label('ss_march')
    a.decm('ss_st'); a.jnz('sm_out')
    a.ld('ss_n'); a.shr(); a.addi(2); a.st('ss_st')
    a.incm('ss_anim')
    a.ld('ss_fx'); a.emit('ADD', 'ss_fd'); a.st('t0')
    a.cmpi(8); a.jn('sm_turn')
    a.addi(SS_COLS * 32); a.cmpi(250); a.jnc('sm_turn')
    a.ld('t0'); a.st('ss_fx')
    a.jmp('sm_crush')
    a.label('sm_turn')
    a.ld('ss_fd'); a.neg(); a.st('ss_fd')
    a.ld('ss_fy'); a.addi(8); a.st('ss_fy')
    # crush building tiles under every live moth's feet
    a.label('sm_crush')
    a.clr('ss_i')
    a.label('sm_cl')
    a.ldxm('ss_i'); a.ldx('ss_alive'); a.jz('sm_cn')
    a.call('ss_cellxy')
    a.ld('ss_hx'); a.addi(16); a.st('ss_hx')
    a.ld('ss_hy'); a.addi(14); a.st('ss_hy')
    a.cmpi(SS_TY + 8); a.jc('sm_ok')
    a.ldi(1); a.st('ss_dead')            # they landed: game over
    a.label('sm_ok')
    a.call('ss_erode')
    a.label('sm_cn')
    a.incm('ss_i'); a.cmpi(SS_N); a.jnz('sm_cl')
    a.label('sm_out')
    a.ret()

    # cell origin of moth ss_i -> ss_hx, ss_hy (also ss_c, ss_r)
    a.label('ss_cellxy')
    a.ld('ss_i'); a.st('t0'); a.clr('ss_r')
    a.label('scx_l')
    a.ld('t0'); a.cmpi(SS_COLS); a.jc('scx_d')
    a.addi(-SS_COLS); a.st('t0'); a.incm('ss_r'); a.jmp('scx_l')
    a.label('scx_d')
    a.ld('t0'); a.st('ss_c')
    a.shl4(); a.shl(); a.emit('ADD', 'ss_fx'); a.st('ss_hx')
    a.ld('ss_r'); a.shl(); a.emit('ADD', 'ss_r'); a.shl(); a.shl(); a.shl()   # r*24
    a.emit('ADD', 'ss_fy'); a.st('ss_hy')
    a.ret()

    # ---- bolt: up 4 px/tick; hits moths, MOTHER, buildings
    a.label('ss_bolt')
    a.ld('ss_bon'); a.jz('sbo_out')
    a.ld('ss_by'); a.addi(-4); a.st('ss_by')
    a.cmpi(18); a.jc('sbo_off')
    # buildings
    a.ld('ss_bx'); a.addi(4); a.st('ss_hx'); a.ld('ss_by'); a.st('ss_hy')
    a.call('ss_erode'); a.jnz('sbo_off')
    # MOTHER
    a.ld('ss_ut'); a.jnz('sbo_nm')
    a.ld('ss_by'); a.cmpi(40); a.jnc('sbo_nm')
    a.ld('ss_bx'); a.emit('SUB', 'ss_ux'); a.addi(4); a.cmpi(16); a.jnc('sbo_nm')
    a.ld('ss_ux'); a.st('ss_ex'); a.ldi(24); a.st('ss_ey'); a.ldi(20); a.st('ss_et')
    a.lda(0x150); a.call('add_score')
    a.lda(1500); a.st('ss_ut')
    a.jmp('sbo_off')
    a.label('sbo_nm')
    # moths: which cell?
    a.clr('ss_i')
    a.label('sbo_l')
    a.ldxm('ss_i'); a.ldx('ss_alive'); a.jz('sbo_n')
    a.call('ss_cellxy')
    a.ld('ss_bx'); a.addi(4 - 8); a.emit('SUB', 'ss_hx'); a.cmpi(16); a.jnc('sbo_n')
    a.ld('ss_by'); a.emit('SUB', 'ss_hy'); a.cmpi(16); a.jnc('sbo_n')
    # hit
    a.ldxm('ss_i'); a.cla(); a.stx('ss_alive')
    a.decm('ss_n')
    a.ld('ss_hx'); a.addi(8); a.st('ss_ex'); a.ld('ss_hy'); a.st('ss_ey'); a.ldi(10); a.st('ss_et')
    a.ldxm('ss_r'); a.ldc('ss_rowpts'); a.call('add_score')
    a.jmp('sbo_off')
    a.label('sbo_n')
    a.incm('ss_i'); a.cmpi(SS_N); a.jnz('sbo_l')
    a.ret()
    a.label('sbo_off')
    a.clr('ss_bon')
    a.label('sbo_out')
    a.ret()

    # ---- bombs: fall 2 px/tick; hit the city or the turret
    a.label('ss_bombs')
    a.lxi(0)
    a.label('sbm_l')
    a.stxm('ss_i')
    a.ldx('ss_mon'); a.jz('sbm_spawn')
    a.ldx('ss_my'); a.addi(2); a.stx('ss_my')
    a.cmpi(SS_TY + 12); a.jnc('sbm_off')
    a.ldx('ss_mx'); a.addi(4); a.st('ss_hx'); a.ldx('ss_my'); a.addi(7); a.st('ss_hy')
    a.call('ss_erode'); a.ldxm('ss_i'); a.jnz('sbm_off')
    # turret hit?
    a.ldx('ss_my'); a.cmpi(SS_TY); a.jc('sbm_n')
    a.ldx('ss_mx'); a.emit('SUB', 'ss_tx'); a.addi(2); a.cmpi(14); a.jnc('sbm_n')
    a.ldi(1); a.st('ss_dead')
    a.label('sbm_off')
    a.ldxm('ss_i'); a.cla(); a.stx('ss_mon')
    a.jmp('sbm_n')
    a.label('sbm_spawn')
    a.inp(I_RAND); a.andi(63); a.jnz('sbm_n')
    # drop from the lowest live moth of a random column
    a.inp(I_RAND); a.andi(7); a.cmpi(SS_COLS); a.jnc('sbm_n')
    a.st('ss_i'); a.addi((SS_ROWS - 1) * SS_COLS); a.st('ss_i')
    a.label('sbm_find')
    a.ldxm('ss_i'); a.ldx('ss_alive'); a.jnz('sbm_got')
    a.ld('ss_i'); a.addi(-SS_COLS); a.st('ss_i'); a.jn('sbm_n'); a.jmp('sbm_find')
    a.label('sbm_got')
    a.call('ss_cellxy')
    a.ld('ss_hx'); a.addi(12); a.st('t0'); a.ld('ss_hy'); a.addi(14); a.st('t1')
    a.lxi(0)
    a.label('sbm_slot')
    a.ldx('ss_mon'); a.jz('sbm_put')
    a.inx(); a.txa(); a.cmpi(3); a.jnz('sbm_slot')
    a.jmp('sbm_n')
    a.label('sbm_put')
    a.ld('t0'); a.stx('ss_mx'); a.ld('t1'); a.stx('ss_my'); a.ldi(1); a.stx('ss_mon')
    a.label('sbm_n')
    a.ldxm('ss_i') if False else None
    a.ldxm('ss_tt') if False else None
    a.ret()

    # ---- turret autopilot: drift under a target moth, dodge, fire
    a.label('ss_pilot')
    a.ld('tick'); a.andi(63); a.jnz('sp_keep')
    # new target: a random live moth's column centre
    a.inp(I_RAND); a.andi(31); a.cmpi(SS_N); a.jnc('sp_keep')
    a.st('ss_i'); a.tax(); a.ldx('ss_alive'); a.jz('sp_keep')
    a.call('ss_cellxy'); a.ld('ss_hx'); a.addi(8); a.st('ss_tgt')
    a.label('sp_keep')
    a.ld('ss_tgt'); a.emit('SUB', 'ss_tx'); a.st('t0')
    # dodge: a bomb within 48 px above and 14 px across pushes away
    a.lxi(2)
    a.label('sp_d')
    a.ldx('ss_mon'); a.jz('sp_dn')
    a.ldx('ss_my'); a.cmpi(SS_TY - 56); a.jc('sp_dn')
    a.ldx('ss_mx'); a.emit('SUB', 'ss_tx'); a.addi(10); a.cmpi(24); a.jnc('sp_dn')
    a.ldx('ss_mx'); a.emit('SUB', 'ss_tx'); a.jn('sp_dr')
    a.lda(-40); a.st('t0'); a.jmp('sp_dn')
    a.label('sp_dr')
    a.ldi(40); a.st('t0')
    a.label('sp_dn')
    a.dex(); a.txa(); a.cmpi(-1); a.jnz('sp_d')
    a.ld('t0'); a.jz('sp_fire'); a.jn('sp_left')
    a.ld('ss_tx'); a.cmpi(236); a.jnc('sp_fire'); a.incm('ss_tx'); a.jmp('sp_fire')
    a.label('sp_left')
    a.ld('ss_tx'); a.cmpi(6); a.jc('sp_fire'); a.decm('ss_tx')
    a.label('sp_fire')
    a.ld('ss_bon'); a.jnz('sp_out')
    a.ld('t0'); a.addi(6); a.cmpi(12); a.jnc('sp_out')
    a.ld('ss_tx'); a.addi(4); a.st('ss_bx'); a.ldi(SS_TY - 6); a.st('ss_by'); a.ldi(1); a.st('ss_bon')
    a.label('sp_out')
    a.ret()

    # ---- MOTHER flyover
    a.label('ss_mother')
    a.ld('ss_ut'); a.jz('smo_fly')
    a.decm('ss_ut'); a.jnz('smo_out')
    a.inp(I_RAND); a.andi(1); a.jz('smo_r')
    a.lda(-16 & 0x1FF); a.st('ss_ux'); a.ldi(1); a.st('ss_ud'); a.ret()
    a.label('smo_r')
    a.ldi(255); a.st('ss_ux'); a.ldi(-1); a.st('ss_ud'); a.ret()
    a.label('smo_fly')
    a.ld('tick'); a.andi(1); a.jz('smo_out')
    a.ld('ss_ux'); a.emit('ADD', 'ss_ud'); a.andi(511); a.st('ss_ux')
    a.cmpi(256); a.jc('smo_out'); a.cmpi(496); a.jnc('smo_out')
    a.lda(1500); a.st('ss_ut')
    a.label('smo_out')
    a.ret()

    # ---- draw everything
    a.label('ss_draw')
    a.cla(); a.outp(P_SIDX)
    # turret
    a.ld('ss_tx'); a.st('sp_x'); a.ldi(SS_TY); a.st('sp_y')
    a.lda(S16 | spal(0) | T['blip_turret']); a.st('sp_a'); a.call('spr_put')
    # bolt
    a.ld('ss_bon'); a.jz('sd_nb')
    a.ld('ss_bx'); a.st('sp_x'); a.ld('ss_by'); a.st('sp_y')
    a.lda(spal(7) | T['bolt']); a.st('sp_a'); a.call('spr_put')
    a.label('sd_nb')
    # bombs
    a.lxi(0)
    a.label('sd_m')
    a.stxm('ss_i')
    a.ldx('ss_mon'); a.jz('sd_mn')
    a.ldx('ss_mx'); a.st('sp_x'); a.ldx('ss_my'); a.st('sp_y')
    a.ld('tick'); a.andi(4); a.jz('sd_z0')
    a.lda(spal(6) | T['zig1']); a.jmp('sd_z1')
    a.label('sd_z0'); a.lda(spal(6) | T['zig0'])
    a.label('sd_z1'); a.st('sp_a'); a.call('spr_put')
    a.label('sd_mn')
    a.ldxm('ss_i'); a.inx(); a.txa(); a.cmpi(3); a.jnz('sd_m')
    # MOTHER
    a.ld('ss_ut'); a.jnz('sd_nu')
    a.ld('ss_ux'); a.st('sp_x'); a.ldi(18); a.st('sp_y')
    a.ld('tick'); a.andi(8); a.jz('sd_u0')
    a.lda(S16 | spal(5) | T['luna1']); a.jmp('sd_u1')
    a.label('sd_u0'); a.lda(S16 | spal(5) | T['luna0'])
    a.label('sd_u1'); a.st('sp_a'); a.call('spr_put')
    a.label('sd_nu')
    # explosion
    a.ld('ss_et'); a.jz('sd_ne')
    a.decm('ss_et')
    a.ld('ss_ex'); a.addi(-8); a.st('sp_x'); a.ld('ss_ey'); a.st('sp_y')
    a.lda(S16 | spal(4) | T['boom0']); a.st('sp_a'); a.call('spr_put')
    a.label('sd_ne')
    # formation
    a.clr('ss_i')
    a.label('sd_f')
    a.ldxm('ss_i'); a.ldx('ss_alive'); a.jz('sd_fn')
    a.call('ss_cellxy')
    a.ld('ss_hx'); a.addi(8); a.st('sp_x'); a.ld('ss_hy'); a.st('sp_y')
    a.call('ss_band'); a.st('t0')
    a.ldxm('ss_r'); a.ldc('ss_rowspr'); a.emit('OR', 't0'); a.st('t0')
    a.ld('ss_anim'); a.andi(1); a.jz('sd_fa')
    a.ld('t0'); a.addi(4); a.st('t0')          # frame 1 follows frame 0
    a.label('sd_fa')
    a.ld('t0'); a.st('sp_a'); a.call('spr_put')
    a.label('sd_fn')
    a.incm('ss_i'); a.cmpi(SS_N); a.jnz('sd_f')
    a.ret()

    # ---- one game
    a.label('ss_game')
    a.ldxa('ss_pals'); a.call('load_pals')
    a.ldxa('ss_tpal'); a.call('load_pals')
    a.lda(0x100); a.outp(P_STAR)                      # starfield on
    a.ldi(7); a.call('new_game')
    a.clr('ss_wave')
    a.call('ss_city')
    a.label('sg_wave')
    a.call('ss_newwave')
    a.label('sg_life')
    a.call('hud')
    a.ldi(120); a.st('ss_tx'); a.st('ss_tgt')
    a.clr('ss_bon'); a.clr('ss_dead'); a.clr('ss_et')
    a.lda(900); a.st('ss_ut')
    a.lxi(3)
    a.label('sg_cm')
    a.cla(); a.stx(('ss_mon', -1)); a.loop('sg_cm')
    a.label('sg_tick')
    a.incm('tick')
    a.call('ss_march')
    a.call('ss_pilot')
    a.call('ss_bolt')
    a.call('ss_bombs')
    a.call('ss_mother')
    a.call('draw_score')
    a.call('ss_draw')
    a.frame()
    a.ld('ss_dead'); a.jnz('sg_died')
    a.ld('ss_n'); a.jnz('sg_tick')
    a.incm('ss_wave')
    a.jmp('sg_wave')
    a.label('sg_died')
    # turret wreck: BOOM frames alternate for ~1 s, then BLIP's power-off
    a.ldi(60); a.st('t5')
    a.label('sg_wreck')
    a.cla(); a.outp(P_SIDX)
    a.ld('ss_tx'); a.st('sp_x'); a.ldi(SS_TY); a.st('sp_y')
    a.ld('t5'); a.andi(4); a.jz('sg_w0')
    a.lda(S16 | spal(4) | T['boom1']); a.jmp('sg_w1')
    a.label('sg_w0'); a.lda(S16 | spal(4) | T['boom0'])
    a.label('sg_w1'); a.st('sp_a'); a.call('spr_put')
    a.frame()
    a.decm('t5'); a.jnz('sg_wreck')
    a.decm('lives'); a.jnz('sg_reset')
    a.jmp('game_over')
    a.label('sg_reset')
    # a death clears the sky but the formation holds its place
    a.jmp('sg_life')


# ======================================================================
# CROSSTALK -- lane crosser.  Lanes are tile rows slid by the per-row
# scroll registers: lane L (0 = start strip, 1-5 road, 6 median, 7-11
# river, 12 = docks) has its top tile row at 26 - 2L.  Cars and logs are
# tile blocks in the lane rows; BLIP is the only sprite.
# ======================================================================
CR_TPALS = [
    [BLACK, c332(4, 1, 3), c332(2, 0, 1), c332(6, 3, 3)],   # 0 curb / median purple
    [c332(0, 0, 1), c332(0, 1, 3), c332(0, 0, 2), c332(2, 4, 3)],   # 1 river blue (entry 4 = river ground)
    [c332(1, 2, 0), c332(5, 2, 0), c332(2, 1, 0), c332(7, 5, 2)],   # 2 logs / hedge
]
CR_DOCKC = [3, 9, 15, 21, 27]


def build_crosstalk(a):
    T = a.T
    a.overlay()
    for n in ('cr_pos', 'cr_spd', 'cr_sc', 'cr_old'):
        a.var(n, 13)
    for n in ('cr_l', 'cr_x', 'cr_hop', 'cr_wait', 'cr_tgt', 'cr_dead',
              'cr_docks', 'cr_i', 'cr_row', 'cr_t', 'cr_k'):
        a.var(n)
    a.table('cr_tpal', 0, 12, *[c for p in CR_TPALS for c in p])
    a.table('cr_dockx', *[c * 8 for c in CR_DOCKC])
    a.table('cr_dockbit', 1, 2, 4, 8, 16)

    # A = top tile row of lane cr_i  (26 - 2L)
    a.label('cr_lrow')
    a.ld('cr_i'); a.shl(); a.neg(); a.addi(26)
    a.ret()

    # fill tile row cr_row with tile word A (32 cells)
    a.label('cr_fillrow')
    a.st('cr_t')
    a.ld('cr_row'); a.shl4(); a.shl(); a.outp(P_TADDR)
    a.lxi(32)
    a.label('cfr_l')
    a.ld('cr_t'); a.outp(P_TDATA); a.loop('cfr_l')
    a.ret()

    # ---- lay out the field
    a.label('cr_field')
    a.ldi(2); a.st('cr_row'); a.lda(T['fill1'] | tpal(2)); a.call('cr_fillrow')
    a.ldi(3); a.st('cr_row'); a.lda(T['fill1'] | tpal(2)); a.call('cr_fillrow')
    a.lxi(4)
    a.label('cf_d')
    a.stxm('cr_k')
    a.ldc('cr_dockx'); a.shr(); a.shr(); a.shr(); a.addi(2 * 32); a.st('t0')
    for r in (0, 32):
        a.ld('t0'); a.addi(r); a.outp(P_TADDR)
        a.lda(T['dock'] | tpal(0)); a.outp(P_TDATA); a.outp(P_TDATA)
    a.ldxm('cr_k'); a.dex(); a.txa(); a.cmpi(-1); a.jnz('cf_d')
    for row in (14, 15, 26, 27):
        a.ldi(row); a.st('cr_row'); a.lda(T['curb'] | tpal(0)); a.call('cr_fillrow')
    # lanes
    a.ldi(1); a.st('cr_i')
    a.label('cf_lane')
    a.ld('cr_i'); a.cmpi(6); a.jz('cf_next')
    a.call('cr_lrow'); a.st('cr_row')
    # speed: 6..21 sixteenths of a pixel, direction alternating per lane
    a.inp(I_RAND); a.andi(15); a.addi(6); a.st('t0')
    a.ld('cr_i'); a.andi(1); a.jz('cf_pos')
    a.ld('t0'); a.neg(); a.st('t0')
    a.label('cf_pos')
    a.ldxm('cr_i'); a.ld('t0'); a.stx('cr_spd')
    a.ld('cr_i'); a.cmpi(7); a.jc('cf_road')
    # river: water, then two or three logs of 3-6 tiles
    a.lda(T['water0'] | tpal(1)); a.call('cr_fillrow')
    a.incm('cr_row'); a.lda(T['water0'] | tpal(1) | (1 << 11)); a.call('cr_fillrow')
    a.decm('cr_row')
    a.clr('cr_k')
    a.label('cf_log')
    a.inp(I_RAND); a.andi(3); a.addi(3); a.st('t1')            # length
    a.inp(I_RAND); a.andi(3); a.emit('ADD', 'cr_k'); a.st('t2')    # start col
    for half, vf in ((0, 0), (32, 1 << 11)):
        a.ld('cr_row'); a.shl4(); a.shl(); a.emit('ADD', 't2'); a.addi(half); a.outp(P_TADDR)
        a.lda(T['log_l'] | tpal(2) | vf); a.outp(P_TDATA)
        a.ld('t1'); a.addi(-2); a.tax()
        lbl = a.uid('lm')
        a.label(lbl)
        a.lda(T['log_m'] | tpal(2) | vf); a.outp(P_TDATA); a.loop(lbl)
        a.lda(T['log_r'] | tpal(2) | vf); a.outp(P_TDATA)
    a.ld('cr_k'); a.addi(11); a.st('cr_k'); a.cmpi(30); a.jc('cf_log')
    a.jmp('cf_next')
    a.label('cf_road')
    a.cla(); a.call('cr_fillrow'); a.incm('cr_row'); a.cla(); a.call('cr_fillrow'); a.decm('cr_row')
    a.clr('cr_k')
    a.label('cf_car')
    a.inp(I_RAND); a.andi(3); a.emit('ADD', 'cr_k'); a.st('t2')
    a.ld('cr_i'); a.andi(1); a.xori(1); a.st('t1')   # even lanes slide left: face left (hflip)
    for half, pa, pb in ((0, 0, 1), (32, 2, 3)):
        a.ld('cr_row'); a.shl4(); a.shl(); a.emit('ADD', 't2'); a.addi(half); a.outp(P_TADDR)
        nf = a.uid('nf')
        done = a.uid('cd')
        a.ld('t1'); a.jz(nf)
        a.lda((T['car'] + pb) | tpal(3) | (1 << 10)); a.outp(P_TDATA)
        a.lda((T['car'] + pa) | tpal(3) | (1 << 10)); a.outp(P_TDATA)
        a.jmp(done)
        a.label(nf)
        a.lda((T['car'] + pa) | tpal(3)); a.outp(P_TDATA)
        a.lda((T['car'] + pb) | tpal(3)); a.outp(P_TDATA)
        a.label(done)
    a.ld('cr_k'); a.addi(8); a.st('cr_k'); a.cmpi(30); a.jc('cf_car')
    a.label('cf_next')
    a.incm('cr_i'); a.cmpi(12); a.jnz('cf_lane')
    a.ret()

    # ---- slide the lanes: pos += speed, scroll = pos >> 4 on both rows
    a.label('cr_slide')
    a.ldi(1); a.st('cr_i')
    a.label('cs_l')
    a.ldxm('cr_i')
    a.ldx('cr_sc'); a.stx('cr_old')
    a.ldx('cr_pos'); a.st('t0'); a.ldx('cr_spd'); a.emit('ADD', 't0'); a.stx('cr_pos')
    a.shr4(); a.andi(255); a.stx('cr_sc'); a.st('t0')
    a.call('cr_lrow'); a.addi(64); a.outp(P_CADDR)
    a.ld('t0'); a.outp(P_CDATA); a.outp(P_CDATA)
    a.incm('cr_i'); a.cmpi(12); a.jnz('cs_l')
    a.ret()

    # tile at BLIP-relative x offset A in lane cr_i (scrolled) -> A
    a.label('cr_tileat')
    a.emit('ADD', 'cr_x'); a.st('t0')
    a.ldxm('cr_i'); a.ldx('cr_sc'); a.emit('ADD', 't0'); a.andi(255)
    a.shr(); a.shr(); a.shr(); a.st('t0')
    a.call('cr_lrow'); a.shl4(); a.shl(); a.emit('ADD', 't0')
    a.outp(P_TADDR); a.inp(I_TDATA); a.andi(255)
    a.ret()

    # danger at lane cr_i for BLIP at cr_x: Z clear (A=1) if deadly
    a.label('cr_danger')
    a.ld('cr_i'); a.cmpi(7); a.jc('cd_road')
    a.cmpi(12); a.jc('cd_river')
    a.cla(); a.ret()
    a.label('cd_river')
    # both feet on a log where it will be when the hop lands (8 ticks:
    # the lane slides speed/2 px)
    a.ldxm('cr_i'); a.ldx('cr_spd'); a.asr(); a.addi(4); a.st('t3')
    a.call('cr_tileat'); a.addi(-T['log_l']); a.cmpi(3); a.jnc('cd_bad')
    a.ld('t3'); a.addi(8); a.call('cr_tileat'); a.addi(-T['log_l']); a.cmpi(3); a.jnc('cd_bad')
    a.cla(); a.ret()
    a.label('cd_feet')
    a.ldi(4); a.call('cr_tileat'); a.addi(-T['log_l']); a.cmpi(3); a.jnc('cd_bad')
    a.ldi(12); a.call('cr_tileat'); a.addi(-T['log_l']); a.cmpi(3); a.jnc('cd_bad')
    a.cla(); a.ret()
    a.label('cd_road')
    a.cmpi(1); a.jc('cd_ok')
    a.cmpi(6); a.jz('cd_ok')
    for off in (3, 8, 13):
        a.ldi(off); a.call('cr_tileat'); a.addi(-T['car']); a.cmpi(4); a.jc('cd_bad')
    # upstream: the traffic covers |speed| px in the 16 ticks of a hop and
    # the next look, so check that stretch on the side the cars come from
    a.ldxm('cr_i'); a.ldx('cr_spd'); a.st('t4'); a.jn('cd_left')
    a.addi(13); a.st('t4'); a.ldi(18); a.st('t3')
    a.label('cd_rl')
    a.ld('t3'); a.cmp('t4'); a.jp('cd_ok')
    a.ld('t3'); a.call('cr_tileat'); a.addi(-T['car']); a.cmpi(4); a.jc('cd_bad')
    a.ld('t3'); a.addi(5); a.st('t3'); a.jmp('cd_rl')
    a.label('cd_left')
    a.addi(3); a.st('t4'); a.ldi(-2); a.st('t3')
    a.label('cd_ll')
    a.ld('t4'); a.cmp('t3'); a.jp('cd_ok')
    a.ld('t3'); a.call('cr_tileat'); a.addi(-T['car']); a.cmpi(4); a.jc('cd_bad')
    a.ld('t3'); a.addi(-5); a.st('t3'); a.jmp('cd_ll')
    a.label('cd_ok')
    a.cla(); a.ret()
    a.label('cd_bad')
    a.ldi(1); a.ret()

    # actual collision in lane cr_l: car under BLIP, or water under his centre
    a.label('cr_hit')
    a.ld('cr_l'); a.st('cr_i'); a.cmpi(7); a.jnc('ch_riv')
    a.cmpi(1); a.jc('ch_ok'); a.cmpi(6); a.jz('ch_ok')
    for off in (3, 8, 13):
        a.ldi(off); a.call('cr_tileat'); a.addi(-T['car']); a.cmpi(4); a.jc('cd_bad')
    a.label('ch_ok')
    a.cla(); a.ret()
    a.label('ch_riv')
    a.cmpi(12); a.jnc('ch_ok')
    a.ldi(8); a.call('cr_tileat'); a.addi(-T['log_l']); a.cmpi(3); a.jnc('cd_bad')
    a.cla(); a.ret()

    # next empty dock's x -> cr_tgt
    a.label('cr_pick')
    a.lxi(0)
    a.label('cp_l')
    a.ldc('cr_dockbit'); a.emit('AND', 'cr_docks'); a.jz('cp_got')
    a.inx(); a.txa(); a.cmpi(5); a.jnz('cp_l')
    a.lxi(0)
    a.label('cp_got')
    a.ldc('cr_dockx'); a.st('cr_tgt')
    a.ret()

    # ---- BLIP: hop, ride, autopilot, deaths
    a.label('cr_blip')
    a.ld('cr_hop'); a.jz('cb_stand')
    a.jn('cb_hneg')
    a.decm('cr_hop'); a.ret()
    a.label('cb_hneg')
    a.incm('cr_hop'); a.ret()
    a.label('cb_stand')
    # ride the river
    a.ld('cr_l'); a.st('cr_i'); a.cmpi(7); a.jc('cb_noride'); a.cmpi(12); a.jnc('cb_noride')
    a.ldxm('cr_i'); a.ldx('cr_sc'); a.st('t0'); a.ldx('cr_old'); a.emit('SUB', 't0')
    a.addi(128); a.andi(255); a.addi(-128); a.emit('ADD', 'cr_x'); a.st('cr_x')
    a.label('cb_noride')
    a.ld('cr_x'); a.addi(4); a.cmpi(248); a.jnc('cb_die')         # carried off
    a.call('cr_hit'); a.jnz('cb_die')
    a.ld('cr_l'); a.cmpi(12); a.jnz('cb_auto')
    # reached the bank: a dock?
    a.lxi(0)
    a.label('cb_dk')
    a.ldc('cr_dockx'); a.emit('SUB', 'cr_x'); a.addi(6); a.cmpi(13); a.jc('cb_dock')
    a.inx(); a.txa(); a.cmpi(5); a.jnz('cb_dk')
    a.jmp('cb_die')
    a.label('cb_dock')
    a.ldc('cr_dockbit'); a.st('t0'); a.emit('AND', 'cr_docks'); a.jnz('cb_die')
    a.ld('t0'); a.emit('OR', 'cr_docks'); a.st('cr_docks')
    a.ldc('cr_dockx'); a.shr(); a.shr(); a.shr(); a.addi(3 * 32); a.outp(P_TADDR)
    a.lda(T['life'] | tpal(3)); a.outp(P_TDATA); a.outp(P_TDATA)
    a.lda(0x50); a.call('add_score')
    a.ld('cr_docks'); a.cmpi(31); a.jnz('cr_respawn')
    a.lda(0x1000); a.call('add_score')
    a.ldi(2); a.st('cr_dead')                       # 2 = stage clear
    a.ret()
    a.label('cb_auto')
    a.incm('cr_wait')
    # on the start strip / median walk toward the target dock's column
    a.ld('cr_l'); a.jz('cb_walk'); a.cmpi(6); a.jnz('cb_look')
    a.label('cb_walk')
    a.ld('cr_x'); a.emit('SUB', 'cr_tgt'); a.jz('cb_look')
    a.jn('cb_right'); a.decm('cr_x'); a.ret()
    a.label('cb_right')
    a.incm('cr_x'); a.ret()
    a.label('cb_look')
    # trouble where he stands?  (a car closing in, or the log near an edge)
    a.clr('cr_k')
    a.ld('cr_l'); a.st('cr_i'); a.cmpi(7); a.jc('cb_rd')
    a.ld('cr_x'); a.addi(-12); a.cmpi(216); a.jc('cb_fwd')
    a.ldi(1); a.st('cr_k'); a.jmp('cb_fwd')
    a.label('cb_rd')
    a.call('cr_danger'); a.st('cr_k')
    a.label('cb_fwd')
    # is the next lane safe where BLIP will land?  (impatient after 3 s)
    a.ld('cr_l'); a.addi(1); a.st('cr_i')
    a.ld('cr_wait'); a.cmpi(180); a.jnc('cb_hop')
    a.ld('cr_i'); a.cmpi(12); a.jnz('cb_lnx')
    # the bank: any empty dock lined up?
    a.lxi(0)
    a.label('cb_anyd')
    a.ldc('cr_dockbit'); a.emit('AND', 'cr_docks'); a.jnz('cb_anyn')
    a.ldc('cr_dockx'); a.emit('SUB', 'cr_x'); a.addi(5); a.cmpi(11); a.jc('cb_hop')
    a.label('cb_anyn')
    a.inx(); a.txa(); a.cmpi(5); a.jnz('cb_anyd')
    a.jmp('cb_back')
    a.label('cb_lnx')
    a.call('cr_danger'); a.jz('cb_hop')
    # can't go on: if in trouble, hop back a lane when that is safe
    a.label('cb_back')
    a.ld('cr_k'); a.jz('cb_wait')
    a.ld('cr_l'); a.jz('cb_wait'); a.addi(-1); a.st('cr_i')
    a.call('cr_danger'); a.jnz('cb_wait')
    a.decm('cr_l'); a.ldi(-8); a.st('cr_hop'); a.clr('cr_wait')
    a.ret()
    a.label('cb_hop')
    a.incm('cr_l'); a.ldi(8); a.st('cr_hop'); a.clr('cr_wait')
    a.label('cb_wait')
    a.ret()
    a.label('cb_die')
    a.ldi(1); a.st('cr_dead')
    a.ret()
    a.label('cr_respawn')
    a.clr('cr_l'); a.ldi(120); a.st('cr_x'); a.clr('cr_hop'); a.clr('cr_wait')
    a.jmp('cr_pick')

    # sprite position of BLIP -> sp_x, sp_y
    a.label('cr_spxy')
    a.ld('cr_x'); a.st('sp_x')
    a.ld('cr_l'); a.st('cr_i'); a.call('cr_lrow'); a.shl(); a.shl(); a.shl()
    a.st('t0'); a.ld('cr_hop'); a.shl(); a.emit('ADD', 't0'); a.st('sp_y')
    a.ret()

    a.label('cr_draw')
    a.cla(); a.outp(P_SIDX)
    a.call('cr_spxy')
    a.ld('cr_hop'); a.jz('crd_s')
    a.lda(S16 | spal(0) | T['blip_back1']); a.jmp('crd_p')
    a.label('crd_s')
    a.lda(S16 | spal(0) | T['blip_back0'])
    a.label('crd_p')
    a.st('sp_a'); a.call('spr_put')
    a.ret()

    # ---- one game
    a.label('cr_game')
    a.ldxa('cr_tpal'); a.call('load_pals')
    a.ldi(11); a.call('new_game')
    a.label('cg_stage')
    a.clr('cr_docks')
    a.call('cr_field')
    a.label('cg_life')
    a.call('hud')
    a.call('cr_respawn')
    a.clr('cr_dead')
    a.call('cr_spxy'); a.call('poweron')
    a.label('cg_tick')
    a.incm('tick')
    a.call('cr_slide')
    a.call('cr_blip')
    a.call('draw_score')
    # river shimmer: cycle the water highlight
    a.ldi(7); a.outp(P_CADDR)
    a.ld('tick'); a.andi(16); a.jz('cg_w0')
    a.lda(c332(2, 4, 3)); a.jmp('cg_w1')
    a.label('cg_w0'); a.lda(c332(0, 2, 3))
    a.label('cg_w1'); a.outp(P_CDATA)
    a.call('cr_draw')
    a.frame()
    a.ld('cr_dead'); a.jz('cg_tick')
    a.cmpi(2); a.jz('cg_clear')
    a.call('cr_spxy'); a.call('poweroff')
    a.decm('lives'); a.jnz('cg_life')
    a.jmp('game_over')
    a.label('cg_clear')
    # stage clear: every lane a little faster
    a.ldi(60); a.call('wait_ticks')
    a.jmp('cg_stage')


# ======================================================================
def build_shell(a):
    a.label('vid_reset')
    a.cla(); a.outp(P_STAR); a.outp(P_YSCR)
    a.ldxa('pal_sprites'); a.call('load_pals')
    a.ldxa('pal_default'); a.jmp('load_pals')

    a.label('reset')
    a.call('vid_reset')
    a.lxi(5)
    a.label('rs_hi')
    a.cla(); a.stx('hi'); a.dex(); a.txa(); a.cmpi(-1); a.jnz('rs_hi')
    a.clr('game')
    a.label('cycle')
    a.call('k1_zone')
    a.jz('cy_auto')
    a.addi(-1); a.st('game')
    a.label('cy_auto')
    a.inp(I_SW); a.andi(16); a.jz('cy_endless')
    a.call('static')
    # title card
    a.call('cls')
    a.ldxm('game'); a.ldc('titles'); a.tax(); a.call('puts')
    a.call_puts('s_copy')
    a.call_puts('s_insert')
    a.call_puts('s_credit')
    a.ldi(120); a.call('wait_ticks')
    a.label('cy_endless')
    a.call('cls')
    a.label('cy_play')
    a.ld('game'); a.jnz('cy_g1')
    a.call('dm_game'); a.jmp('cy_next')
    a.label('cy_g1')
    a.cmpi(1); a.jnz('cy_g2')
    a.call('ss_game'); a.jmp('cy_next')
    a.label('cy_g2')
    a.call('cr_game')
    a.label('cy_next')
    a.call('vid_reset')
    a.incm('game'); a.cmpi(NGAMES); a.jnz('cycle')
    a.clr('game')
    a.jmp('cycle')

    # K1 turned to another game mid-play: frame_sub jumps here (whatever the
    # call depth -- the return stack is circular, so the leftovers are
    # harmless).  A short burst of static, then straight into the new game.
    a.label('k1_switch')
    a.call('vid_reset')
    a.ldi(8); a.call('static_burst')
    a.call('cls')
    a.jmp('cy_play')
    a.string('s_title_dm', 11, 10, 3, 'DOT MATRIX')
    a.string('s_title_ss', 9, 10, 3, 'SKYLINE SIEGE')
    a.string('s_title_cr', 11, 10, 3, 'CROSSTALK')
    a.table('titles', 's_title_dm', 's_title_ss', 's_title_cr')
    a.table('k1_zones', *[(1024 * (i + 1)) // (NGAMES + 1) for i in range(NGAMES)])


def build(only_dm=False):
    pats, T = assets.load()
    a = Prog(T)
    a.jmp('reset')
    build_lib(a)
    build_dotmatrix(a)
    if not only_dm:
        build_skyline(a)
        build_crosstalk(a)
    else:
        a.label('ss_game')
        a.ret()
    a.overlay_end()
    build_shell(a)
    rom = a.assemble()
    return rom, {'ram': a.nvar, 'labels': a.addr, 'vars': a.vars}


if __name__ == '__main__':
    rom, info = build()
    print('program %d words, RAM %d words' % (len(rom), info['ram']))
