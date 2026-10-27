#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
WOODGRAIN - minimal dasm-compatible 6502 assembler (enough for the 8bitworkshop
VCS examples and the dasm vcs.h / macro.h headers).

  python3 asm6502.py source.a out.bin [-I dir] [--size 4096] [--sym out.sym]

Supported: include, processor, seg / seg.u (per-segment origins), org, equ / = /
set, labels (column 0, optional ':'), '.local' labels scoped by 'subroutine' and
per macro invocation, mac/endm with {n} arguments, if/else/endif, ifconst /
ifnconst, byte / word / dc.b / dc.w / ds, align, the documented 6502 set and the
stable illegal opcodes in isa6502.py.  Multi-pass until every symbol is stable,
so forward references pick zero-page vs absolute correctly.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isa6502 import ENC, MODES_LEN  # noqa: E402

ALIASES = {'ISC': 'ISB', 'INS': 'ISB', 'AXS': 'SBX', 'DCM': 'DCP', 'LSE': 'SRE',
           'ASO': 'SLO', 'AAX': 'SAX', 'ASR': 'ALR', 'AXA': 'SAX', 'OAL': 'LXA',
           'ATX': 'LXA', 'ANC2': 'ANC', 'SKB': 'NOP', 'SKW': 'NOP', 'DOP': 'NOP',
           'TOP': 'NOP'}
NOP_EXTRA = {'imm': 0x80, 'zp': 0x04, 'zpx': 0x14, 'abs': 0x0C, 'abx': 0x1C}


class Unknown(Exception):
    pass


class AsmError(Exception):
    pass


def load(path, incdirs, out):
    """Read a file with includes expanded -> list of (text, where)."""
    for d in [os.path.dirname(path)] + incdirs:
        p = os.path.join(d, os.path.basename(path)) if d else path
        if os.path.exists(p):
            path = p
            break
    with open(path, encoding='latin-1') as f:
        for n, line in enumerate(f, 1):
            m = re.match(r'^\s+include\s+"?([^"\s;]+)"?', line, re.I)
            if m:
                load(m.group(1), incdirs + [os.path.dirname(path)], out)
            else:
                out.append((line.rstrip('\n'), '%s:%d' % (os.path.basename(path), n)))
    return out


def strip_comment(s):
    out, q = [], None
    for ch in s:
        if q:
            out.append(ch)
            if ch == q:
                q = None
        elif ch in '"\'':
            q = ch
            out.append(ch)
        elif ch == ';':
            break
        else:
            out.append(ch)
    return ''.join(out).rstrip()


TOK = re.compile(r'\s*(\$[0-9A-Fa-f]+|%[01]+|[0-9]+|\'.\'?|[A-Za-z_.][A-Za-z0-9_.]*'
                 r'|<<|>>|<=|>=|==|!=|&&|\|\||[-+*/%<>&|^~!()\[\]=#,])')


class Asm:
    def __init__(self):
        self.sym = {}
        self.macros = {}

    # ---------------- expressions ----------------
    def lookup(self, name):
        if name.startswith('.') and name != '.':
            key = self.scope + name
        else:
            key = name
        if key in self.sym:
            return self.sym[key]
        if key.upper() in self.sym:
            return self.sym[key.upper()]
        self.unknown = True
        raise Unknown(name)

    def defined(self, name):
        key = self.scope + name if name.startswith('.') else name
        return key in self.sym

    def eval(self, text):
        toks = [t for t in TOK.findall(text) if t.strip()]
        self.toks, self.tp = toks, 0
        v = self.e_binop(0)
        if self.tp != len(self.toks):
            raise AsmError('junk in expression: %r' % text)
        return v

    PREC = [('||',), ('&&',), ('|',), ('^',), ('&',), ('==', '!=', '='),
            ('<', '>', '<=', '>='), ('<<', '>>'), ('+', '-'), ('*', '/', '%')]

    def peek(self):
        return self.toks[self.tp] if self.tp < len(self.toks) else None

    def e_binop(self, lvl):
        if lvl == len(self.PREC):
            return self.e_unary()
        v = self.e_binop(lvl + 1)
        while self.peek() in self.PREC[lvl]:
            op = self.toks[self.tp]
            self.tp += 1
            w = self.e_binop(lvl + 1)
            v = {'||': lambda a, b: int(bool(a) or bool(b)),
                 '&&': lambda a, b: int(bool(a) and bool(b)),
                 '|': lambda a, b: a | b, '^': lambda a, b: a ^ b, '&': lambda a, b: a & b,
                 '==': lambda a, b: int(a == b), '=': lambda a, b: int(a == b),
                 '!=': lambda a, b: int(a != b), '<': lambda a, b: int(a < b),
                 '>': lambda a, b: int(a > b), '<=': lambda a, b: int(a <= b),
                 '>=': lambda a, b: int(a >= b), '<<': lambda a, b: a << b,
                 '>>': lambda a, b: a >> b, '+': lambda a, b: a + b, '-': lambda a, b: a - b,
                 '*': lambda a, b: a * b, '/': lambda a, b: a // b if b else 0,
                 '%': lambda a, b: a % b if b else 0}[op](v, w)
        return v

    def e_unary(self):
        t = self.peek()
        if t is None:
            raise AsmError('empty expression')
        self.tp += 1
        if t == '-':
            return -self.e_unary()
        if t == '+':
            return self.e_unary()
        if t == '~':
            return ~self.e_unary() & 0xFFFF
        if t == '!':
            return int(not self.e_unary())
        if t == '<':
            return self.e_unary() & 0xFF
        if t == '>':
            return (self.e_unary() >> 8) & 0xFF
        if t == '#':
            return self.e_unary()
        if t in '([':
            v = self.e_binop(0)
            close = self.peek()
            if close not in (')', ']'):
                raise AsmError('missing )')
            self.tp += 1
            return v
        if t.startswith('$'):
            return int(t[1:], 16)
        if t.startswith('%'):
            return int(t[1:], 2)
        if t[0].isdigit():
            return int(t)
        if t.startswith("'"):
            return ord(t[1])
        if t in ('*', '.'):
            return self.pc
        return self.lookup(t)

    def ev(self, text):
        """Evaluate; unknown symbols give None (and flag another pass)."""
        try:
            return self.eval(text)
        except Unknown:
            self.unknown = True
            return None

    # ---------------- output ----------------
    def emit(self, *bs):
        for b in bs:
            if not self.uninit:
                self.out[self.pc & 0xFFFF] = b & 0xFF
            self.pc += 1

    def define(self, label, value):
        key = self.scope + label if label.startswith('.') else label
        if value is None:
            self.unknown = True
            return
        if self.sym.get(key) != value:
            self.changed = True
        self.sym[key] = value

    # ---------------- passes ----------------
    def run_pass(self, lines):
        self.out, self.pc, self.uninit = {}, 0, False
        self.segs = {'': [0, False]}
        self.seg = ''
        self.scope = 'G0'
        self.nscope = 0
        self.unknown = False
        self.changed = False
        self.errors = []
        self.process(lines)

    def process(self, lines):
        conds = []          # stack of (active, taken)
        i = 0
        while i < len(lines):
            raw, where = lines[i]
            i += 1
            line = strip_comment(raw)
            if not line.strip():
                continue
            label = None
            if not line[0].isspace():
                parts = line.split(None, 1)
                label = parts[0].rstrip(':')
                rest = parts[1] if len(parts) > 1 else ''
            else:
                rest = line.strip()
            parts = rest.split(None, 1)
            op = parts[0] if parts else ''
            arg = parts[1].strip() if len(parts) > 1 else ''
            opu = op.upper().lstrip('.')
            active = all(c[0] for c in conds)
            # conditionals are processed even when inactive (nesting)
            if opu in ('IF', 'IFCONST', 'IFNCONST'):
                if not active:
                    conds.append((False, True))
                    continue
                if opu == 'IF':
                    v = self.ev(arg)
                    t = bool(v) if v is not None else False
                elif opu == 'IFCONST':
                    t = self.defined(arg.strip())
                else:
                    t = not self.defined(arg.strip())
                conds.append((t, t))
                continue
            if opu == 'ELSE':
                a, taken = conds.pop()
                parent = all(c[0] for c in conds)
                conds.append((parent and not taken, True))
                continue
            if opu in ('ENDIF', 'EIF'):
                conds.pop()
                continue
            if not active:
                if opu == 'MAC':
                    while i < len(lines) and strip_comment(lines[i][0]).split()[:1] != ['ENDM'] \
                            and not re.match(r'^\s*endm\b', lines[i][0], re.I):
                        i += 1
                    i += 1
                continue
            if opu == 'MAC':
                name = arg.split()[0].upper()
                body = []
                while not re.match(r'^\s*endm\b', lines[i][0], re.I):
                    body.append(lines[i])
                    i += 1
                i += 1
                self.macros[name] = body
                continue
            try:
                self.statement(label, op, opu, arg, where)
            except (AsmError, Unknown) as e:
                if isinstance(e, Unknown):
                    self.unknown = True
                else:
                    self.errors.append('%s: %s' % (where, e))

    def statement(self, label, op, opu, arg, where):
        # label definition
        if opu in ('EQU', '=', 'SET'):
            self.define(label, self.ev(arg))
            return
        if label:
            if opu == 'SUBROUTINE':
                pass
            else:
                self.define(label, self.pc)
        if not op:
            return
        if opu == 'SUBROUTINE':
            self.nscope += 1
            self.scope = 'S%d' % self.nscope
            if label:
                self.define(label, self.pc)
            return
        if opu in ('PROCESSOR', 'ECHO', 'LIST', 'TITLE', 'ERR', 'INCDIR'):
            if opu == 'ERR':
                raise AsmError('ERR directive')
            return
        if opu in ('SEG', 'SEG.U'):
            self.segs[self.seg] = [self.pc, self.uninit]
            name = arg.strip()
            self.seg = name
            pc, _ = self.segs.get(name, [0, False])
            self.pc, self.uninit = pc, opu == 'SEG.U'
            return
        if opu == 'ORG' or opu == 'RORG':
            v = self.ev(arg.split(',')[0])
            if v is not None:
                self.pc = v
            if label:
                self.define(label, self.pc)
            return
        if opu == 'ALIGN':
            n = self.ev(arg) or 1
            while self.pc % n:
                self.emit(0)
            return
        if opu in ('DS', 'DS.B'):
            a = [s.strip() for s in arg.split(',')]
            n = self.ev(a[0]) if a[0] else 1
            fill = self.ev(a[1]) if len(a) > 1 else 0
            for _ in range(n or 0):
                self.emit(fill or 0)
            return
        if opu in ('BYTE', 'DC.B', 'DB', 'DC', 'HEX') or op.lower() in ('.byte', '.db'):
            for item in split_args(arg) or ['0']:
                if item.startswith('"'):
                    for ch in item.strip('"'):
                        self.emit(ord(ch))
                else:
                    v = self.ev(item)
                    self.emit(v if v is not None else 0)
            return
        if opu in ('WORD', 'DC.W', 'DW') or op.lower() in ('.word', '.dw'):
            for item in split_args(arg) or ['0']:
                v = self.ev(item)
                v = v if v is not None else 0
                self.emit(v & 0xFF, v >> 8)
            return
        if opu in self.macros:
            self.expand(opu, arg)
            return
        self.instruction(opu, arg)

    def expand(self, name, arg):
        args = split_args(arg)
        body = []
        for raw, where in self.macros[name]:
            for n, a in enumerate(args, 1):
                raw = raw.replace('{%d}' % n, a)
            body.append((raw, where))
        saved = self.scope
        self.nscope += 1
        self.scope = 'M%d' % self.nscope
        self.process(body)
        self.scope = saved

    def instruction(self, mn, arg):
        mn = ALIASES.get(mn, mn)
        a = arg.strip()
        mode, expr = None, None
        if a == '' or a.upper() == 'A':
            mode = 'acc' if (mn, 'acc') in ENC else 'imp'
        elif a.startswith('#'):
            mode, expr = 'imm', a[1:]
        elif a.startswith('(') and re.search(r',\s*[xX]\s*\)$', a):
            mode, expr = 'izx', re.sub(r',\s*[xX]\s*\)$', '', a[1:])
        elif a.startswith('(') and re.search(r'\)\s*,\s*[yY]$', a):
            mode, expr = 'izy', re.sub(r'\)\s*,\s*[yY]$', '', a[1:])
        elif mn == 'JMP' and a.startswith('(') and a.endswith(')'):
            mode, expr = 'ind', a[1:-1]
        else:
            m = re.match(r'^(.*?),\s*([xXyY])$', a)
            idx = None
            if m:
                expr, idx = m.group(1), m.group(2).lower()
            else:
                expr = a
            v = self.ev(expr)
            branch = (mn, 'rel') in ENC
            if branch:
                mode = 'rel'
            else:
                small = v is not None and 0 <= v < 256
                if idx is None:
                    cand = ['zp', 'abs'] if small else ['abs', 'zp']
                elif idx == 'x':
                    cand = ['zpx', 'abx'] if small else ['abx', 'zpx']
                else:
                    cand = ['zpy', 'aby'] if small else ['aby', 'zpy']
                for c in cand:
                    if (mn, c) in ENC or (mn == 'NOP' and c in NOP_EXTRA):
                        mode = c
                        break
                if mode is None:
                    raise AsmError('no mode for %s %s' % (mn, arg))
        op = ENC.get((mn, mode))
        if op is None and mn == 'NOP':
            op = NOP_EXTRA.get(mode)
        if op is None:
            raise AsmError('bad instruction %s (%s)' % (mn, mode))
        n = MODES_LEN[mode]
        v = self.ev(expr) if expr is not None else 0
        if mode == 'rel':
            if v is None:
                self.emit(op, 0)
                return
            off = v - (self.pc + 2)
            if not -128 <= off <= 127 and not self.unknown:
                raise AsmError('branch out of range')
            self.emit(op, off & 0xFF)
            return
        v = 0 if v is None else v
        if n == 1:
            self.emit(op)
        elif n == 2:
            self.emit(op, v & 0xFF)
        else:
            self.emit(op, v & 0xFF, (v >> 8) & 0xFF)


def split_args(arg):
    out, cur, depth, q = [], [], 0, None
    for ch in arg:
        if q:
            cur.append(ch)
            if ch == q:
                q = None
        elif ch in '"\'':
            q = ch
            cur.append(ch)
        elif ch in '([':
            depth += 1
            cur.append(ch)
        elif ch in ')]':
            depth -= 1
            cur.append(ch)
        elif ch == ',' and depth == 0:
            out.append(''.join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if cur or out:
        out.append(''.join(cur).strip())
    return [o.lstrip('#') if not o.startswith('"') else o for o in out if o != '']


def assemble(path, incdirs=(), size=4096):
    lines = load(path, list(incdirs), [])
    asm = Asm()
    for _ in range(12):
        asm.run_pass(lines)
        if not asm.unknown and not asm.changed:
            break
    else:
        raise AsmError('did not converge (unknown symbols?)')
    if asm.errors:
        raise AsmError('\n'.join(asm.errors))
    rom = bytearray(size)
    for addr, b in asm.out.items():
        if addr >= 0x1000 or True:
            rom[addr % size] = b
    return bytes(rom), asm.sym


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('out')
    ap.add_argument('-I', action='append', default=[])
    ap.add_argument('--size', type=int, default=4096)
    ap.add_argument('--sym')
    a = ap.parse_args()
    rom, sym = assemble(a.src, a.I, a.size)
    with open(a.out, 'wb') as f:
        f.write(rom)
    if a.sym:
        with open(a.sym, 'w') as f:
            for k in sorted(sym, key=lambda k: sym[k]):
                if not k[:1] in 'GSM' or '.' not in k:
                    f.write('%-24s %04X\n' % (k, sym[k] & 0xFFFF))
    print('%s: %d bytes' % (a.out, len(rom)))
