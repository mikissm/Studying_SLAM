"""Conservative circular-robot grid evaluation and terrain-aware 8-connected A*."""
import heapq
import time
import numpy as np
from scipy import ndimage

REASONS={1:'지면 미확인',2:'몸체 높이 구간 장애물',4:'머리 위 공간 부족',8:'몸체 반경·안전 여유 충돌',16:'국소 경사 초과',32:'몸체 아래 미관측 지면',64:'몸체 주변 경사 여유'}


def config(p):
    bounds={'radius':(.18,0,2),'margin':(.03,0,1),'height':(.5,.1,3),'bodyclearance':(.15,.02,1),
            'maxstep':(.12,0,1),'maxslope':(25,0,85),'inferredcost':(3,0,50),'recoveredcost':(1,0,50)}
    out={}
    for key,(default,lo,hi) in bounds.items():
        value=float(p.get(key,default))
        if not np.isfinite(value) or not lo<=value<=hi:raise ValueError(f'{key}: {lo}~{hi} 범위가 필요합니다')
        out[key]=value
    if out['bodyclearance']>=out['height']:raise ValueError('몸체 하부 여유는 전체 높이보다 작아야 합니다')
    out['strictunknown']=bool(p.get('strictunknown',True))
    return out


def evaluate(result,robot,points,rotation):
    centers,classes,counts,rough,ground,meta=result
    p=config(robot);res=float(meta['resolution']);origin=np.asarray(meta['origin_xy']);ny,nx=ground.shape
    known=np.isfinite(ground);codes=np.zeros(ground.shape,np.uint8);codes[~known]=1
    if not known.any():raise ValueError('선택된 지면이 없습니다')
    _,near=ndimage.distance_transform_edt(~known,return_indices=True)
    reference=ground[tuple(near)]
    full=points.astype(np.float64)@rotation(meta['parameters']).T
    ij=np.floor((full[:,:2]-origin)/res).astype(int)
    inside=(ij[:,0]>=0)&(ij[:,0]<nx)&(ij[:,1]>=0)&(ij[:,1]<ny)
    full=full[inside];ij=ij[inside];ids=ij[:,1]*nx+ij[:,0]
    dz=full[:,2]-reference.ravel()[ids]
    body=(dz>p['bodyclearance'])&(dz<=p['height'])
    hits=np.bincount(ids[body],minlength=ground.size).reshape(ground.shape)
    occupied=hits>=int(meta['parameters'].get('obstaclepoints',4))
    overhead=np.full(ground.size,np.inf)
    np.minimum.at(overhead,ids[dz>p['bodyclearance']],dz[dz>p['bodyclearance']])
    clearance=overhead.reshape(ground.shape)
    # Any supported collision is blocked. Headroom is reported separately when
    # a low return exists over an otherwise observed terrain surface.
    band_min=np.full(ground.size,np.inf);band_max=np.full(ground.size,-np.inf)
    np.minimum.at(band_min,ids[body],dz[body]);np.maximum.at(band_max,ids[body],dz[body])
    # A narrow vertical return band suggests a low overhead surface. A broad
    # column suggests a wall/body collision. This is geometric evidence only.
    width=(band_max-band_min).reshape(ground.shape)
    head=occupied&known&(clearance<p['height'])&(width<=.08)
    codes[occupied&~head]|=2;codes[head]|=4
    # Least-squares local terrain plane via masked convolution (3x3 cells).
    z=np.where(known,ground,0).astype(float)
    kernel=np.ones((3,3));oy,ox=np.mgrid[-1:2,-1:2]*res
    conv=lambda a,k:ndimage.correlate(a,k,mode='constant',cval=0)
    n=conv(known.astype(float),kernel);sx=conv(known.astype(float),ox);sy=conv(known.astype(float),oy)
    sz=conv(z,kernel);sxx=conv(known.astype(float),ox*ox);syy=conv(known.astype(float),oy*oy);sxy=conv(known.astype(float),ox*oy)
    sxz=conv(z,ox);syz=conv(z,oy);den=np.maximum(n,1)
    a=sxx-sx*sx/den;b=sxy-sx*sy/den;c=syy-sy*sy/den;d=sxz-sx*sz/den;e=syz-sy*sz/den
    determinant=a*c-b*b;good=(n>=3)&(determinant>1e-12)
    gx=np.divide(d*c-b*e,determinant,out=np.zeros_like(z),where=good)
    gy=np.divide(a*e-b*d,determinant,out=np.zeros_like(z),where=good)
    slope=np.rad2deg(np.arctan(np.hypot(gx,gy)))
    steep=good&known&(slope>p['maxslope']+1e-7);codes[steep]|=16
    # Distance to occupied CELL AREA, not just its center. Pad the map so the
    # circular footprint also respects the map boundary / unknown terrain.
    radius=p['radius']+p['margin'];cell_half_diag=res/np.sqrt(2)
    sources={}
    if occupied.any():
        distance,indices=ndimage.distance_transform_edt(~occupied,return_indices=True)
        sources[8]=indices
        distance=distance*res
        inflated=distance<=radius+cell_half_diag+1e-9
        codes[inflated&~occupied]|=8
    if p['strictunknown']:
        padded=np.pad(known,1,constant_values=False)
        distance,indices=ndimage.distance_transform_edt(padded,return_indices=True)
        sources[32]=indices[:,1:-1,1:-1]-1
        distance=distance[1:-1,1:-1]*res
        codes[known&(distance<=radius+cell_half_diag+1e-9)]|=32
    # The complete body footprint must fit on allowed slopes as well.
    if steep.any() and radius:
        distance,indices=ndimage.distance_transform_edt(~steep,return_indices=True)
        sources[64]=indices
        codes[(distance*res<=radius+cell_half_diag)&~steep]|=64
    traversable=known&(codes==0)
    xy=np.floor((centers[:,:2]-origin)/res).astype(int)
    cellcodes=codes[xy[:,1],xy[:,0]]
    # Retain the actual nearest cause, including unknown cells beyond map bounds.
    causes=[]
    for x,y in xy:
        items=[]
        for bit,label in REASONS.items():
            if not int(codes[y,x])&bit:continue
            sy,sx=(sources[bit][:,y,x] if bit in sources else (y,x))
            valid=0<=sy<ny and 0<=sx<nx
            height=float(reference[sy,sx]) if valid else float(reference[y,x])
            item={'bit':bit,'label':label,'source':[float(origin[0]+(sx+.5)*res),float(origin[1]+(sy+.5)*res),height],
                  'distance_m':float(np.hypot(sx-x,sy-y)*res)}
            if bit in (16,64):item['slope_deg']=float(slope[sy,sx])
            if bit in (2,4,8):item['point_count']=int(hits[sy,sx])
            items.append(item)
        causes.append(items)
    quality=np.zeros(ground.shape,np.uint8);quality[xy[:,1],xy[:,0]]=classes
    return {'robot':p,'codes':codes,'slope':slope,'allowed':traversable,'quality':quality,
            'cell_causes':causes,'footprint_limit_m':radius+cell_half_diag,'cell_codes':cellcodes,'clearance':clearance,'ground':ground,'meta':meta}


def transition(ev,a,b):
    ay,ax=a;by,bx=b
    if not ev['allowed'][by,bx]:return False
    # Both constraints apply, intentionally conservative at sharp step edges.
    rise=abs(float(ev['ground'][by,bx]-ev['ground'][ay,ax]))
    return rise<=ev['robot']['maxstep']+1e-7


def plan(ev,start,goal):
    started=time.perf_counter();g=ev['ground'];meta=ev['meta'];res=meta['resolution'];origin=np.asarray(meta['origin_xy'])
    def locate(point,label):
        if not isinstance(point,list) or len(point)!=2 or not np.isfinite(point).all():raise ValueError(label+'은 유한한 XY 좌표여야 합니다')
        x,y=np.floor((np.asarray(point)-origin)/res).astype(int)
        if not (0<=y<g.shape[0] and 0<=x<g.shape[1]):raise ValueError(label+'이 지도 밖입니다')
        if not ev['allowed'][y,x]:
            reasons=[text for bit,text in REASONS.items() if int(ev['codes'][y,x])&bit]
            return None,{'ok':False,'error':label+' 선택 불가: '+', '.join(reasons),'failure':label,'reasons':reasons}
        return (int(y),int(x)),None
    a,error=locate(start,'출발점')
    if error:return error
    b,error=locate(goal,'목표점')
    if error:return error
    def heuristic(v):return float(np.sqrt(((v[0]-b[0])*res)**2+((v[1]-b[1])*res)**2+(float(g[v])-float(g[b]))**2))
    distances={a:0.};parent={};queue=[(heuristic(a),0.,a)];visited=0
    dirs=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    edge_rejections=0
    while queue:
        _,cost,v=heapq.heappop(queue)
        if cost>distances.get(v,np.inf)+1e-12:continue
        visited+=1
        if v==b:break
        y,x=v
        for dy,dx in dirs:
            n=(y+dy,x+dx)
            if not (0<=n[0]<g.shape[0] and 0<=n[1]<g.shape[1]):continue
            if not transition(ev,v,n):edge_rejections+=1;continue
            if dx and dy:
                sides=[(y,x+dx),(y+dy,x)]
                if not all(transition(ev,v,s) and transition(ev,s,n) for s in sides):continue
            dz=float(g[n])-float(g[v]);length=float(np.sqrt(res**2*(dx*dx+dy*dy)+dz*dz))
            q=ev['quality'][n];penalty=ev['robot']['inferredcost'] if q==5 else ev['robot']['recoveredcost'] if q==7 else 0
            candidate=cost+length*(1+penalty)
            if candidate<distances.get(n,np.inf)-1e-12:
                distances[n]=candidate;parent[n]=v;heapq.heappush(queue,(candidate+heuristic(n),candidate,n))
    else:
        return {'ok':False,'error':'연결된 경로가 없습니다. 몸체 여유·미관측 영역·경사·단차 제한을 확인하세요.',
                'visited':visited,'edge_rejections':edge_rejections,'elapsed_ms':round((time.perf_counter()-started)*1000)}
    path=[b]
    while path[-1]!=a:path.append(parent[path[-1]])
    path.reverse();indices=np.array(path);yy,xx=indices.T
    xyz=np.column_stack((origin[0]+(xx+.5)*res,origin[1]+(yy+.5)*res,g[yy,xx]))
    length=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
    return {'ok':True,'path':xyz.tolist(),'length_m':round(length,3),'weighted_cost':distances[b],
            'max_slope_deg':float(ev['slope'][yy,xx].max()),'max_step_m':float(np.abs(np.diff(xyz[:,2])).max()) if len(path)>1 else 0,
            'inferred_ratio':float((ev['quality'][yy,xx]==5).mean()),'recovered_ratio':float((ev['quality'][yy,xx]==7).mean()),
            'visited':visited,'elapsed_ms':round((time.perf_counter()-started)*1000)}
