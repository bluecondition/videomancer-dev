"""Cycle model of the v6 row renderer: prescan (1 chunk/clk, 1-entry hit buffer),
vertex issue (16 clk per hit chunk) -> D-deep pipe -> F-deep job FIFO (pipe stalls
when FIFO count >= F) -> word iterator (1 word/clk).  Reports worst row clocks."""
import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
from model import gen, CH

def row_jobs(copies, W, H, reach, wpx):
    cx, cy = W//2, H//2
    chunks=[]   # per chunk: (ymin, ymax, [ (ylo,yhi, wl, wh) per segment ])
    for cp in copies:
        pts=cp+np.array([cx,cy])
        for c in range(0,len(pts)-1,CH):
            blk=pts[c:c+CH+1]
            segs=[]
            for k in range(len(blk)-1):
                (x0,y0),(x1,y1)=blk[k],blk[k+1]
                wl=int(max(0,min(x0,x1)-reach))//wpx; wh=int(min(W-1,max(x0,x1)+reach))//wpx
                segs.append((min(y0,y1)-reach, max(y0,y1)+reach, wl, wh))
            chunks.append((blk[:,1].min()-reach, blk[:,1].max()+reach, segs))
    return chunks

def simulate_row(chunks, R, F, D=10):
    hits=[c for c in chunks if c[0]<=R<=c[1]]
    # vertex-scan stream: each hit chunk -> 16 items (job or None); prescan finds
    # hits >= 1 per clk, never the bottleneck when chunks <= 256 and hits are dense
    stream=[]
    for c in hits:
        for (ylo,yhi,wl,wh) in c[2]:
            stream.append((wh-wl+1) if ylo<=R<=yhi else 0)
    t=0; i=0; pipe=[0]*D; fifo=[]; cur=0; done=False
    pres_floor=len(chunks)    # prescan must read every chunk at least once
    while True:
        stall = len(fifo) >= F
        if not stall:
            out=pipe.pop(0)
            if out: fifo.append(out)
            pipe.append(stream[i] if i<len(stream) else 0)
            if i<len(stream): i+=1
        if cur==0 and fifo: cur=fifo.pop(0)
        if cur: cur-=1
        t+=1
        if i>=len(stream) and not any(pipe) and not fifo and cur==0: break
    return max(t, pres_floor)

cases=[('spiro 5:-8','spiro',5,-8,.4,0,1),('spiro 15:-16','spiro',15,-16,.5,0,1),
       ('harm 2:3','harm',2,3,.3,.6,1),('harm 3:-2','harm',3,-2,.5,1.2,1),
       ('kal8 5:-8','spiro',5,-8,.4,0,8),('kal8 harm 2:3','harm',2,3,.5,.6,8)]
for mode,W,H,rb,clk in (('1080p',1920,1080,500,2200),('720p',1280,720,336,1650)):
  for nm,mach,a,b,arm,psi,nc in cases:
    cps=gen(mach,a,b,arm,psi,ncopy=nc,rbase=rb)
    res=[]
    for wt,glow in ((3,False),(8,False),(3,True),(8,True)):
        reach = ((wt+8)//2) if glow else ((wt+2)//2+1)
        row=[]
        for wpx,F in ((8,4),(4,4),(4,2)):
            ch=row_jobs(cps,W,H,reach,wpx)
            worst=max(simulate_row(ch,R,F) for R in range(0,H,6))
            row.append(worst)
        res.append(f"w{wt}{'g' if glow else 'c'} " + '/'.join(f'{v:4d}' for v in row))
    print(f"{mode:5s} {nm:14s} [8px F4 / 4px F4 / 4px F2]  " + '  '.join(res) + f"   line {clk}")
