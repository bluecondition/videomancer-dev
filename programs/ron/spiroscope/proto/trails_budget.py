"""Per-row budget for the v6.4 Trails plan: E slots (live + ghosts), each a full
figure of N/E points, 16-point chunks, wide deltas.  Worst case = all slots
scanned (coincident or drifted ghosts cost the same).  Prints worst row clocks."""
import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import model, cycles
from cycles import simulate_row
from cycles2 import chunks_for

cases=[('spiro 5:-8','spiro',5,-8,.4,0),('spiro 16:-15','spiro',16,-15,.5,0),
       ('spiro 16:-2','spiro',16,-2,.23,1.2),('harm 3:-2','harm',3,-2,.5,1.2),('harm 2:3','harm',2,3,.3,.6)]
modes=(('1080p',1920,1080,500,2200),('720p',1280,720,336,1650),('NTSC',720,486,226,858))
cfgs=[(1,4096),(2,2048),(4,1024),(2,1024),(4,512),(4,256),(2,512)]
for mname,W,H,rb,clk in modes:
    print(f'--- {mname} (line {clk} clk)')
    for nm,mach,a,b,arm,psi in cases:
        row=[]
        for E,N in cfgs:
            for wt,glow in ((3,False),(8,False),(3,True)):
                model.N=N; model.DEC=N/2.18; model.CYC=18.0
                cps=model.gen(mach,a,b,arm,psi,n=N,rbase=rb)
                # E slots: ghosts drifted in twist so they are distinct
                allc=[]
                for e in range(E):
                    allc+=model.gen(mach,a,b,arm,psi+0.08*e,n=N,rbase=rb)
                reach = ((wt+8)//2) if glow else ((wt+2)//2+1)
                ch=chunks_for(allc,W,H,reach,16,1)
                w=max(simulate_row(ch,R,2) for R in range(0,H,8))
                row.append(f'{w:5d}')
        print(f'{nm:13s}', '  '.join(f'E{E}x{N}:' + '/'.join(row[i*3:i*3+3]) for i,(E,N) in enumerate(cfgs)))
print('cols per cfg: w3 crisp / w8 crisp / w3 glow')
