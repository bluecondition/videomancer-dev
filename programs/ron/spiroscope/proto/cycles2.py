"""Row-pipeline cycle model for the v6.1 plan: N points, C-point chunks, zoom
(figure scaled about the centre, off-screen x segments culled after the scan).
4-px words, 2-deep FIFO, 1 word/clk.  Prints worst row clocks."""
import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
import model
from model import gen
from cycles import simulate_row

def chunks_for(copies, W, H, reach, C, zoom):
    cx, cy = W//2, H//2
    out=[]
    for cp in copies:
        pts=cp*zoom+np.array([cx,cy])
        for c in range(0,len(pts)-1,C):
            blk=pts[c:c+C+1]
            segs=[]
            for k in range(len(blk)-1):
                (x0,y0),(x1,y1)=blk[k],blk[k+1]
                xl,xh=min(x0,x1)-reach,max(x0,x1)+reach
                if xh<0 or xl>W-1:            # off-screen in x: culled, costs its scan slot
                    segs.append((1e9,-1e9,0,0)); continue
                wl=int(max(0,xl))//4; wh=int(min(W-1,xh))//4
                segs.append((min(y0,y1)-reach, max(y0,y1)+reach, wl, wh))
            out.append((blk[:,1].min()-reach, blk[:,1].max()+reach, segs))
    return out

cases=[('spiro 5:-8','spiro',5,-8,.4,0,1),('spiro 15:-16','spiro',15,-16,.5,0,1),
       ('spiro 16:-15 a.8','spiro',16,-15,.8,0,1),('harm 3:-2','harm',3,-2,.5,1.2,1),
       ('kal8 5:-8','spiro',5,-8,.4,0,8)]
W,H,rb,clk=1920,1080,500,2200
if __name__ == "__main__":
  for nm,mach,a,b,arm,psi,nc in cases:
      row=[]
      for N,C,wt,glow,zoom in ((4096,16,3,False,1),(8192,32,2,False,1),(8192,32,3,False,1),(8192,32,4,False,1),
                               (8192,32,3,True,1),(8192,32,3,False,2),(8192,32,3,False,3)):
          model.N=N
          cps=gen(mach,a,b,arm,psi,ncopy=nc,rbase=rb,n=N)
          reach = ((wt+8)//2) if glow else ((wt+2)//2+1)
          ch=chunks_for(cps,W,H,reach,C,zoom)
          worst=max(simulate_row(ch,R,2) for R in range(0,H,8))
          row.append(f"{N//1024}k/C{C} w{wt}{'g' if glow else ''} z{zoom}:{worst:5d}")
      print(f"{nm:16s} " + ' | '.join(row))
  print('line', clk)
