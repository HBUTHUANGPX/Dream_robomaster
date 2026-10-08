"""Rasterize original STL into free-space support layers and a traversability graph."""
from pathlib import Path
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label
import trimesh

ROOT=Path(__file__).resolve().parents[1]
RES=.1

class Terrain:
    def __init__(self, rebuild=False):
        path=ROOT/'assets/arena/terrain.npz'
        if rebuild or not path.exists(): self.build(path)
        d=np.load(path)
        self.heights=d['heights']; self.valid=d['valid']; self.clearance=d['clearance']
        self.rows,self.cols=self.heights.shape
        self.width=self.cols*RES;self.height=self.rows*RES
        self.spawn=np.array([7.5,4.,self.height_at(7.5,4.)])
        self.component,_=label(self.valid)
        self.reachable=self.component==self.component[self.cell(*self.spawn[:2])]
        self.reachable &= self.valid

    @staticmethod
    def build(path):
        mesh=trimesh.load(ROOT/'assets/arena/rmuc2023.stl')
        cols,rows=np.ceil(mesh.bounds[1,:2]/RES).astype(int)
        hits={}
        for tri,normal in zip(mesh.triangles,mesh.face_normals):
            if abs(normal[2])<1e-5:continue
            low=np.maximum(0,np.floor(tri[:,:2].min(0)/RES-.5).astype(int))
            high=np.minimum([cols-1,rows-1],np.ceil(tri[:,:2].max(0)/RES-.5).astype(int))
            xs=np.arange(low[0],high[0]+1);ys=np.arange(low[1],high[1]+1)
            xx,yy=np.meshgrid(xs,ys);xy=np.c_[(xx.ravel()+.5)*RES,(yy.ravel()+.5)*RES]
            a=tri[1,:2]-tri[0,:2];b=tri[2,:2]-tri[0,:2]
            det=a[0]*b[1]-a[1]*b[0]
            if abs(det)<1e-10:continue
            delta=xy-tri[0,:2]
            u=(delta[:,0]*b[1]-delta[:,1]*b[0])/det
            v=(a[0]*delta[:,1]-a[1]*delta[:,0])/det
            inside=(u>=-1e-6)&(v>=-1e-6)&(u+v<=1+1e-6)
            zs=tri[0,2]+u*(tri[1,2]-tri[0,2])+v*(tri[2,2]-tri[0,2])
            for x,y,z in zip(xx.ravel()[inside],yy.ravel()[inside],zs[inside]):
                hits.setdefault((y,x),[]).append((float(z),float(normal[2])))
        h=np.full((rows,cols),np.nan);ceilings=np.zeros_like(h)
        multilayer=[]
        for (y,x),values in hits.items():
            values.sort()
            supports=[]
            for z,nz in values:
                if nz<.01 or z<.09:continue
                above=[(zz,nn) for zz,nn in values if zz>z+.025]
                clearance=above[0][0]-z if above else 10.
                if above and above[0][1]>0:continue  # inside a solid, not free space
                if clearance<.65:continue
                if not supports or abs(supports[-1][0]-z)>.025:supports.append((z,clearance))
            if supports:
                # Top-down goal selects uppermost support. Underpasses are recorded separately.
                h[y,x],ceilings[y,x]=supports[-1]
                if len(supports)>1:multilayer.append([x,y,*[s[0] for s in supports]])
        finite=np.isfinite(h)
        bad=~finite
        for dy,dx in [(0,1),(1,0),(1,1),(1,-1)]:
            other=np.roll(h,(dy,dx),(0,1))
            jump=np.abs(h-other)>math.hypot(dy,dx)*RES*math.tan(math.radians(28))+.008
            bad |= jump | np.roll(jump,(-dy,-dx),(0,1))
        bad[[0,-1],:]=True;bad[:,[0,-1]]=True
        distance=distance_transform_edt(~bad)*RES
        valid=finite&(distance>=.38)
        np.savez_compressed(path,heights=h,valid=valid,clearance=distance)
        print('terrain',h.shape,'safe',int(valid.sum()),'multi-layer cells',len(multilayer))
        (ROOT/'assets/arena/multilayer_cells.txt').write_text(str(multilayer[:30]))

    def cell(self,x,y):
        return int(np.clip(y/RES,0,self.rows-1)),int(np.clip(x/RES,0,self.cols-1))
    def height_at(self,x,y):
        return float(self.heights[self.cell(x,y)])
    def point(self,cell):
        y,x=cell;return [(x+.5)*RES,(y+.5)*RES,float(self.heights[y,x])]
    def plan(self,start,goal):
        a=self.cell(*start[:2]);b=self.cell(*goal[:2])
        if not self.valid[b]:raise ValueError('目标不满足底盘净空、边缘余量或坡度要求')
        if not self.valid[a]:
            ys,xs=np.where(self.valid);i=np.argmin((xs-a[1])**2+(ys-a[0])**2)
            if math.hypot(xs[i]-a[1],ys[i]-a[0])*RES>.5:raise ValueError('当前位置偏离可通行区域，已停车')
            a=(int(ys[i]),int(xs[i]))
        queue=[(0.,a)];cost={a:0.};parent={}
        while queue:
            _,p=heapq.heappop(queue)
            if p==b:break
            for dy,dx in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                q=(p[0]+dy,p[1]+dx)
                if not(0<=q[0]<self.rows and 0<=q[1]<self.cols and self.valid[q]):continue
                if dy and dx and not(self.valid[p[0]+dy,p[1]] and self.valid[p[0],p[1]+dx]):continue
                dz=abs(self.heights[q]-self.heights[p]);step=math.hypot(dx,dy)*RES
                if dz>step*math.tan(math.radians(28))+.008:continue
                c=cost[p]+step+2*dz+.006/max(self.clearance[q],.1)
                if c<cost.get(q,math.inf):
                    cost[q]=c;parent[q]=p
                    heapq.heappush(queue,(c+math.hypot(q[0]-b[0],q[1]-b[1])*RES,q))
        if b not in cost:raise ValueError('目标与机器人之间没有满足底盘约束的连续通路')
        chain=[b]
        while chain[-1]!=a:chain.append(parent[chain[-1]])
        return np.array([self.point(p) for p in reversed(chain)])

if __name__=='__main__':
    t=Terrain(rebuild=True)
    from PIL import Image
    h=np.nan_to_num(t.heights)
    rgb=np.stack([60+80*h,75+90*h,95+65*h],-1).clip(0,255).astype('uint8')
    rgb[t.reachable]=[65,165,125]
    Image.fromarray(rgb[::-1]).resize((604,1116)).save(ROOT/'output/terrain.png')
    print('reachable',t.reachable.sum(), 'heights',np.unique(np.round(t.heights[t.reachable],2)))
