#!/usr/bin/env python3
import math
def arr(name, vals, width, per=8):
    lines=[f"    constant {name} : t_{name.lower()} := ("]
    body=[]
    for i in range(0,len(vals),per):
        body.append('        '+', '.join(f"to_unsigned({v},{width})" for v in vals[i:i+per]))
    lines.append(',\n'.join(body)+');')
    return '\n'.join(lines)
out=[]
atan=[round(math.atan(2.0**-i)/(2*math.pi)*2**17) for i in range(16)]
out.append("    type t_c_atan is array(0 to 15) of integer;")
out.append("    constant C_ATAN : t_c_atan := ("+', '.join(str(v) for v in atan)+");")
# CORDIC angle from the direction bits (d_i = sign of y going into iteration i, 1 = negative):
# z = sum(d_i ? -atan_i : +atan_i) mod 2^16.  d0..d7 address a 256-entry ROM (bit 7 = d0),
# d8..d11 a 16-entry constant (bit 3 = d8) added on the final clock.
zp=[]
for a in range(256):
    v=sum(-atan[i] if (a>>(7-i))&1 else atan[i] for i in range(8))
    zp.append(v & 0xFFFF)
zq=[sum(-atan[8+i] if (a>>(3-i))&1 else atan[8+i] for i in range(4)) for a in range(16)]
out.append("    -- CORDIC angle from the direction bits: z = sum over the 12 iterations of +-atan(i)")
out.append("    -- (minus where y < 0), mod 2^16.  The first eight directions address a 256-entry ROM")
out.append("    -- (one EBR), the last four a 16-entry constant added on the final clock.")
out.append("    type t_zrom is array(0 to 255) of std_logic_vector(15 downto 0);")
out.append("    constant C_ZP : t_zrom := (")
out.append(",\n".join("        "+", ".join(f'x"{v:04X}"' for v in zp[i:i+8]) for i in range(0,256,8))+");")
out.append("    type t_zq is array(0 to 15) of integer;")
out.append("    constant C_ZQ : t_zq := ("+", ".join(str(v) for v in zq)+");")
# packed log ROM: V (12 bits) | S4 (4 bits), S = 16 + 2*S4 ~ (V[m+1]-V[m])/16*32
V=[round(math.log2(1+m/256)*4096) for m in range(257)]
W=[]
for m in range(256):
    S=(V[m+1]-V[m])/16*32
    S4=max(0,min(15,round((S-16)/2)))
    W.append(V[m]*16+S4)
# FUN ROM: 0..127 sin(2*pi*i/512) Q15 (quarter wave, 128 entries) ; 128..255 (2^(i/128)-1) Q16
sn=[min(32767,round(math.sin(2*math.pi*i/512)*32768)) for i in range(128)]
ex=[min(65535,round((2**(i/128)-1)*65536)) for i in range(128)]
# 256..511: packed log ROM (V | S4) for the sequencer's log2
out.append("    type t_c_fun is array(0 to 511) of unsigned(15 downto 0);")
out.append(arr('C_FUN',sn+ex+W,16))
print('\n'.join(out))
