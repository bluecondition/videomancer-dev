import sys, numpy as np
sys.argv=[sys.argv[0],'/dev/null']
from model import gen, N, CH
def rowcost(copies, W, H, reach, wpx):
    hits=np.zeros(H); words=np.zeros(H)
    cx,cy=W//2,H//2
    for cp in copies:
        pts=cp+np.array([cx,cy])
        for c in range(0,len(pts)-1,CH):
            blk=pts[c:c+CH+1]
            lo=int(max(0,blk[:,1].min()-reach)); hi=int(min(H-1,blk[:,1].max()+reach))
            if lo<=hi: hits[lo:hi+1]+=1
            for k in range(len(blk)-1):
                (x0,y0),(x1,y1)=blk[k],blk[k+1]
                ylo=int(max(0,min(y0,y1)-reach)); yhi=int(min(H-1,max(y0,y1)+reach))
                if ylo>yhi: continue
                wl=int(max(0,min(x0,x1)-reach))//wpx; wh=int(min(W-1,max(x0,x1)+reach))//wpx
                words[ylo:yhi+1]+=wh-wl+1
    scan=np.maximum(256, 16*hits)
    tot=scan+np.maximum(0, words-16*hits)
    return int(tot.max()), int((16*hits).max()), int(words.max())
cases=[('spiro 5:-8','spiro',5,-8,.4,0,1),('spiro 15:-16','spiro',15,-16,.5,0,1),
       ('harm 2:3','harm',2,3,.3,.6,1),('harm 3:-2','harm',3,-2,.5,1.2,1),('harm 1:1','harm',1,1,.5,1.5,1),
       ('kal8 5:-8','spiro',5,-8,.4,0,8),('kal8 harm 2:3','harm',2,3,.5,.6,8)]
for mode,W,H,rb,clk,sc in (('1080p',1920,1080,500,2200,1.0),('720p',1280,720,340,1650,0.67)):
  for nm,mach,a,b,arm,psi,nc in cases:
    cps=gen(mach,a,b,arm,psi,ncopy=nc,rbase=rb)
    r=[]
    for wt,glow in ((8,False),(3,True),(8,True)):
        reach=int(round(((wt+1)//2 + (3 if glow else 1))*sc))
        t,s,w=rowcost(cps,W,H,reach,8); r.append(f'{t:5d}(s{s:4d} w{w:4d})')
    print(f'{mode:5s} {nm:14s} crisp8 {r[0]} | glow3 {r[1]} | glow8 {r[2]}  line {clk}')
