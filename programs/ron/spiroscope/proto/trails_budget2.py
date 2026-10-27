"""v6.4 Trails: rows over the line budget per config, scanning live first
(an overrun then drops the oldest ghost's segments)."""
import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import model, cycles
from cycles import simulate_row
from cycles2 import chunks_for

cases=[('spiro 5:-8','spiro',5,-8,.4,0),('spiro 16:-15','spiro',16,-15,.5,0),
       ('spiro 16:-2','spiro',16,-2,.23,1.2),('harm 3:-2','harm',3,-2,.5,1.2),('harm 2:3','harm',2,3,.3,.6)]
modes={'1080p':(1920,1080,500,2200),'720p':(1280,720,336,1650),'NTSC':(720,486,226,858)}
cfgs=[('1080p',1,4096,3,False),('1080p',2,2048,3,False),('1080p',4,1024,3,False),('1080p',2,1024,8,False),('1080p',4,1024,8,False),('1080p',2,2048,3,True),('1080p',4,1024,3,True),
      ('720p',1,4096,3,False),('720p',2,2048,3,False),('720p',4,1024,3,False),('720p',2,1024,3,True),
      ('NTSC',1,1024,3,False),('NTSC',2,1024,3,False),('NTSC',4,1024,3,False),('NTSC',2,512,3,False)]
for mname,E,N,wt,glow in cfgs:
    W,H,rb,clk=modes[mname]
    out=[]
    for nm,mach,a,b,arm,psi in cases:
        model.N=N; model.DEC=N/2.18; model.CYC=18.0
        reach = ((wt+8)//2) if glow else ((wt+2)//2+1)
        slots=[]
        for e in range(E):
            slots.append(chunks_for(model.gen(mach,a,b,arm,psi+0.08*e,n=N,rbase=rb),W,H,reach,16,1))
        allc=sum(slots,[])
        over=0; lost_live=0; rows=0; worst=0
        for R in range(0,H,4):
            t=simulate_row(allc,R,2)
            if any(c[0]<=R<=c[1] for c in allc): rows+=1
            worst=max(worst,t)
            if t>clk:
                over+=1
                # does the live slot alone (scanned first) finish?
                tl=simulate_row(slots[0],R,2)
                if tl>clk: lost_live+=1
        out.append(f'{nm}: worst {worst:4d} over {100*over/max(rows,1):4.1f}% live-lost {lost_live}')
    print(f'{mname} E{E}x{N} w{wt}{"g" if glow else "c"} (line {clk}):  ' + ' | '.join(out))
