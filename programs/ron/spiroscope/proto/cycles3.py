import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import model, cycles
from cycles import simulate_row
from cycles2 import chunks_for
W,H,rb=1920,1080,500
for nm,a,b,arm,psi in (('harm 3:-2',3,-2,.5,1.2),('harm 2:3',2,3,.3,.6),('harm 1:1',1,1,.5,1.5)):
    row=[]
    for N,C,wt in ((4096,16,3),(8192,32,2),(8192,32,3)):
        model.N=N; model.DEC=N/2.18; model.CYC=18.0
        cps=model.gen('harm',a,b,arm,psi,n=N,rbase=rb)
        reach=(wt+2)//2+1
        ch=chunks_for(cps,W,H,reach,C,1)
        row.append(f"{N//1024}k w{wt}: {max(simulate_row(ch,R,2) for R in range(0,H,8)):5d}")
    print(nm, ' | '.join(row))
