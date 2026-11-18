#!/usr/bin/env python3
"""Local PCD elevation laboratory. Run from the workspace root; no ROS build needed."""
import argparse
import io
import json
import threading
import time
import uuid
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from scipy import ndimage
from scipy.spatial import Delaunay, QhullError
from layered import extract_layered
from planner import evaluate as evaluate_robot, plan as plan_route, REASONS as ROBOT_REASONS

ROOT = Path(__file__).resolve().parent
MAX_UPLOAD = 512 * 1024**2


def lzf_decode(data, expected):
    out = bytearray()
    i = 0
    while i < len(data):
        ctrl = data[i]; i += 1
        if ctrl < 32:
            size = ctrl + 1
            if i + size > len(data): raise ValueError('잘못된 LZF 데이터')
            out.extend(data[i:i+size]); i += size
        else:
            size = ctrl >> 5
            offset = (ctrl & 31) << 8
            if size == 7:
                size += data[i]; i += 1
            offset += data[i]; i += 1
            ref = len(out) - offset - 1
            if ref < 0: raise ValueError('잘못된 LZF 참조')
            for k in range(size + 2): out.append(out[ref+k])
        if len(out) > expected: raise ValueError('LZF 크기 초과')
    if len(out) != expected: raise ValueError('LZF 크기 불일치')
    return out


def read_pcd(blob):
    stream = io.BytesIO(blob)
    header = {}
    for _ in range(100):
        line = stream.readline().decode('ascii').strip()
        if not line or line.startswith('#'): continue
        parts = line.split(); header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == 'DATA': break
    else: raise ValueError('PCD 헤더를 찾을 수 없습니다')
    fields = header.get('FIELDS', header.get('FIELD', []))
    if not all(v in fields for v in ('x','y','z')): raise ValueError('x, y, z 필드가 필요합니다')
    sizes = list(map(int, header['SIZE']))
    types = header['TYPE']; counts = list(map(int, header.get('COUNT',['1']*len(fields))))
    if not len(fields) == len(sizes) == len(types) == len(counts): raise ValueError('필드 정의 불일치')
    n = int(header.get('POINTS',[str(int(header['WIDTH'][0])*int(header.get('HEIGHT',['1'])[0]))])[0])
    if n <= 0 or n > 30_000_000: raise ValueError('포인트 수는 1~30,000,000 범위여야 합니다')
    dtype = np.dtype([(name, np.dtype({'F':'<f','I':'<i','U':'<u'}[kind]+str(size)), (count,)) for name,size,kind,count in zip(fields,sizes,types,counts)])
    encoding = header['DATA'][0].lower()
    if encoding == 'ascii':
        data = np.loadtxt(stream, dtype=np.float64, ndmin=2)
        if data.shape != (n,sum(counts)): raise ValueError('ASCII 포인트 수 또는 열 수가 헤더와 다릅니다')
        offsets = np.cumsum([0]+counts[:-1]).tolist()
        xyz = data[:,[offsets[fields.index(k)] for k in ('x','y','z')]]
    elif encoding == 'binary':
        raw = stream.read()
        if len(raw) < n*dtype.itemsize: raise ValueError('PCD 데이터가 잘렸습니다')
        data = np.frombuffer(raw, dtype=dtype, count=n)
        xyz = np.column_stack([data[k][:,0] for k in ('x','y','z')])
    elif encoding == 'binary_compressed':
        lengths = np.frombuffer(stream.read(8),dtype='<u4')
        if len(lengths)!=2: raise ValueError('압축 헤더 오류')
        compressed, expected = map(int,lengths)
        if expected != n*dtype.itemsize or expected > 1024**3: raise ValueError('압축 해제 크기 오류')
        raw = lzf_decode(stream.read(compressed), expected)
        columns={}; offset=0
        for field,size,kind,count in zip(fields,sizes,types,counts):
            length=n*size*count
            if field in ('x','y','z'):
                columns[field]=np.frombuffer(raw,dtype={'F':'<f','I':'<i','U':'<u'}[kind]+str(size),count=n*count,offset=offset).reshape(n,count)[:,0]
            offset += length
        xyz=np.column_stack([columns[k] for k in ('x','y','z')])
    else: raise ValueError('지원 형식: ascii, binary, binary_compressed')
    xyz = np.asarray(xyz[np.isfinite(xyz).all(axis=1)],dtype=np.float32)
    if not len(xyz): raise ValueError('유효한 좌표가 없습니다')
    return xyz, encoding, n-len(xyz)


def rotation(p):
    a,b=np.deg2rad([float(p['roll']),float(p['pitch'])])
    rx=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    ry=np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    return ry@rx


def interpolate_ground(grid, observed_cells, rejected_cells, res, parameters):
    """One-pass linear interpolation inside short, gently sloped ground triangles.

    Cells listed in observed_cells cannot be overwritten; callers may omit soft
    rejects to request recovery proposals. Original ground is the sole support.
    rejected_cells are hard barriers, including triangle interiors and edges.
    """
    gap=float(parameters.get('maxgap', .6))
    delta=float(parameters.get('maxheight', .15))
    slope=float(parameters.get('maxslope', 20))
    if not .02 <= gap <= 3 or not 0 <= delta <= 3 or not 0 <= slope <= 75:
        raise ValueError('보간 거리/높이/경사 범위 오류')
    y,x=np.where(np.isfinite(grid))
    if len(x)<3: return np.empty((0,3), dtype=np.float64)
    support=np.column_stack((x,y))*res
    heights=grid[y,x].astype(np.float64)
    try: tri=Delaunay(support)
    except QhullError: return np.empty((0,3), dtype=np.float64)
    vertices=support[tri.simplices]; z=heights[tri.simplices]
    edges=np.stack((vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0],vertices[:,2]-vertices[:,1]),axis=1)
    eligible=np.linalg.norm(edges,axis=2).max(axis=1)<=gap+1e-9
    eligible &= np.ptp(z,axis=1)<=delta+1e-9
    # Barycentric transforms yield the exact slope of each triangle plane.
    dz=z[:,:2]-z[:,2,None]
    gradient=np.einsum('ni,nij->nj',dz,tri.transform[:,:2,:])
    eligible &= np.linalg.norm(gradient,axis=1)<=np.tan(np.deg2rad(slope))+1e-9
    blocked=np.zeros(grid.shape,bool); blocked.ravel()[rejected_cells]=True
    by,bx=np.where(blocked)
    if len(bx):
        simplex=tri.find_simplex(np.column_stack((bx,by))*res)
        eligible[np.unique(simplex[simplex>=0])]=False
        # Check triangle edges too, including obstacles lying on shared edges.
        ids=np.flatnonzero(eligible)
        for a,b in ((0,1),(1,2),(2,0)):
            for t in np.linspace(0,1,max(2,int(np.ceil(gap/res*2))+1)):
                xy=(vertices[ids,a]*(1-t)+vertices[ids,b]*t)/res
                ix=np.clip(np.rint(xy[:,0]).astype(int),0,grid.shape[1]-1)
                iy=np.clip(np.rint(xy[:,1]).astype(int),0,grid.shape[0]-1)
                eligible[ids[blocked[iy,ix]]]=False
    unknown=np.ones(grid.shape,bool); unknown.ravel()[observed_cells]=False
    uy,ux=np.where(unknown)
    candidates=np.column_stack((ux,uy))*res
    simplex=tri.find_simplex(candidates)
    safe=simplex>=0
    safe[safe] &= eligible[simplex[safe]]
    candidates=candidates[safe]; simplex=simplex[safe]
    if not len(candidates): return np.empty((0,3), dtype=np.float64)
    bary=np.einsum('nij,nj->ni',tri.transform[simplex,:2,:],candidates-tri.transform[simplex,2,:])
    weights=np.column_stack((bary,1-bary.sum(axis=1)))
    values=np.einsum('ni,ni->n',weights,heights[tri.simplices[simplex]])
    return np.column_stack((ux[safe],uy[safe],values))


def detect_obstacles(xyz, ids, cells, counts, low, res, p):
    """Classify low vertical surfaces and raised objects, ignoring high ceilings.

    Estimate a nearby lower surface and inspect only the robot-height band above
    it. Covariance uses all points in that band (not the grid representative).
    """
    clearance=float(p.get('obstacleheight', .22))
    ceiling=float(p.get('robotheight', 1.5))
    radius=float(p.get('wallradius', .4))
    angle=float(p.get('wallangle', 55))
    if not .05<=clearance<=1 or not clearance<ceiling<=3 or not .1<=radius<=2 or not 20<=angle<=85:
        raise ValueError('벽 판별: 최소 높이 < 검사 높이, 반경/각도 범위를 확인하세요')
    # Avoid letting a single low outlier determine all neighbouring ground.
    reliable=np.full(low.shape,np.inf)
    reliable.ravel()[cells[counts>=3]]=low.ravel()[cells[counts>=3]]
    width=2*int(np.ceil(radius/res))+1
    reference=ndimage.minimum_filter(reliable,size=width)
    fallback=ndimage.minimum_filter(np.where(np.isfinite(low),low,np.inf),size=width)
    reference=np.where(np.isfinite(reference),reference,fallback)
    floor=reference.ravel()[ids]
    band=(xyz[:,2]>=floor-.08)&(xyz[:,2]<=floor+ceiling)
    high=band&(xyz[:,2]>floor+clearance)
    elevated=np.bincount(ids[high],minlength=low.size)[cells]
    # Covariance per XY cell in the lower band; smallest eigenvector is normal.
    index=np.searchsorted(cells,ids[band]); pts=xyz[band]
    n=np.bincount(index,minlength=len(cells)).astype(float)
    sums=np.column_stack([np.bincount(index,weights=pts[:,j],minlength=len(cells)) for j in range(3)])
    mean=sums/np.maximum(n[:,None],1)
    covariance=np.zeros((len(cells),3,3))
    for a in range(3):
        for b in range(a,3):
            moment=np.bincount(index,weights=pts[:,a]*pts[:,b],minlength=len(cells))/np.maximum(n,1)-mean[:,a]*mean[:,b]
            covariance[:,a,b]=covariance[:,b,a]=moment
    eigen,vectors=np.linalg.eigh(covariance)
    stable=(n>=6)&(eigen[:,1]>res**2*.002)&(eigen[:,0]/np.maximum(eigen[:,1],1e-12)<.3)
    vertical=stable&(np.abs(vectors[:,2,0])<np.cos(np.deg2rad(angle)))
    # Several returns above ground also block flat furniture tops / mixed cells.
    ground_rise=float(p.get('groundrise',.5))
    if not .1<=ground_rise<=5: raise ValueError('기준 바닥 대비 최대 높이는 0.1~5m입니다')
    ground_reference=float(np.percentile(low[np.isfinite(low)],35))
    unsupported_high=low.ravel()[cells]>ground_reference+ground_rise
    obstacles=(elevated>=int(p.get('obstaclepoints',4)))|vertical|unsupported_high
    mask=np.zeros(low.shape,bool);mask.ravel()[cells]=obstacles
    padding=float(p.get('wallpadding',0))
    if not 0<=padding<=.5: raise ValueError('벽 여유 거리는 0~0.5m입니다')
    if padding:
        distance=ndimage.distance_transform_edt(~mask)*res
        mask |= distance<=padding+1e-9
    return mask,reference,vertical


def extract(points,p):
    if p.get('layered',False):
        return extract_layered(points,p,rotation,interpolate_ground)
    # All filters are explicit. Reason is the first filter to reject each cell.
    res=float(p['resolution']); q=float(p['percentile']); radius=float(p['radius'])
    if not .02 <= res <= 2 or not 0 <= q <= 50 or not 0 <= radius <= 3: raise ValueError('격자/분위수/반경 범위 오류')
    zmin,zmax=float(p['zmin']),float(p['zmax'])
    if zmin>=zmax: raise ValueError('Z 최솟값은 최댓값보다 작아야 합니다')
    xyz=points.astype(np.float64)@rotation(p).T
    xyz=xyz[(xyz[:,2]>=zmin)&(xyz[:,2]<=zmax)]
    if not len(xyz): raise ValueError('Z 범위 안에 점이 없습니다')
    origin=np.floor(xyz[:,:2].min(axis=0)/res)*res
    ij=np.floor((xyz[:,:2]-origin)/res).astype(np.int64)
    nx,ny=map(int,ij.max(axis=0)+1)
    if nx*ny>2_000_000: raise ValueError('격자가 200만 셀을 넘습니다. 해상도(m)를 키우거나 Z 범위를 좁혀 주세요')
    ids=ij[:,1]*nx+ij[:,0]
    # One sort by cell then height allows vectorized, exact linear quantiles.
    order=np.lexsort((xyz[:,2],ids)); sorted_z=xyz[order,2]
    cells,starts,counts=np.unique(ids[order],return_index=True,return_counts=True)
    def quantile(v):
        pos=(counts-1)*v/100; lo=np.floor(pos).astype(int); hi=np.ceil(pos).astype(int)
        return sorted_z[starts+lo]*(1-(pos-lo))+sorted_z[starts+hi]*(pos-lo)
    heights=quantile(q); rough=quantile(min(q+20,100))-heights
    shape=(ny,nx); low=np.full(shape,np.nan); low.ravel()[cells]=heights
    reason=np.zeros(len(cells),np.uint8)
    barrier=np.zeros(shape,bool)
    reference=low.copy()
    if p.get('wallfilter',False):
        barrier,reference,vertical=detect_obstacles(xyz,ids,cells,counts,low,res,p)
        reason[barrier.ravel()[cells]]=6
    reason[(reason==0)&(counts<int(p['minpoints']))]=1
    if p['envelope']:
        size=2*int(np.ceil(radius/res))+1
        envelope=ndimage.minimum_filter(np.where(np.isfinite(low),low,np.inf),size=size)
        reason[(reason==0)&(heights>envelope.ravel()[cells]+float(p['tolerance']))]=2
    if p['roughfilter']:
        reason[(reason==0)&(rough>float(p['roughness']))]=3
    if p['components']:
        mask=np.zeros(shape,bool); mask.ravel()[cells[reason==0]]=True
        labels,n=ndimage.label(mask,structure=np.ones((3,3)))
        sizes=np.bincount(labels.ravel()); keep=sizes>=int(p['mincells']); keep[0]=False
        if p['seedfilter'] and n:
            threshold=float(np.percentile(heights,float(p['seedpercentile'])))+float(p['seedmargin'])
            # Same 20th-percentile anchor rule as the original prototype.
            label_ids=labels.ravel()[cells]
            for label in np.flatnonzero(keep):
                if np.percentile(heights[label_ids==label],20)>threshold: keep[label]=False
        accepted=keep[labels.ravel()[cells]]
        reason[(reason==0)&~accepted]=4
    y,x=np.divmod(cells,nx)
    centers=np.column_stack((origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,heights)).astype(np.float32)
    initial_good=reason==0
    grid=np.full(shape,np.nan,dtype=np.float32); grid.ravel()[cells[initial_good]]=heights[initial_good]
    interpolated=np.empty((0,3)); recovered_count=0
    if p.get('interpolate',False):
        recover=p.get('recover',False) and p.get('wallfilter',False)
        hard=np.flatnonzero(barrier)
        if recover:
            # A high rejected surface is a barrier, not a missing floor sample.
            high=(reason!=0)&((heights-reference.ravel()[cells])>float(p.get('recoverheight',.15)))
            hard=np.union1d(hard,cells[high])
            protected=np.union1d(cells[initial_good],hard)
            proposals=interpolate_ground(grid,protected,hard,res,p)
        else:
            hard=np.union1d(hard,cells[reason!=0])
            proposals=interpolate_ground(grid,np.union1d(cells,hard),hard,res,p)
        cell_lookup=np.full(grid.size,-1,dtype=int);cell_lookup[cells]=np.arange(len(cells))
        if len(proposals):
            fx,fy=proposals[:,:2].astype(int).T
            existing=cell_lookup[fy*nx+fx]
            restore=existing>=0
            if restore.any():
                indices=existing[restore]
                predictions=proposals[restore,2]
                allowed=(reason[indices]!=6)&(np.abs(heights[indices]-predictions)<=float(p.get('recoverheight',.15)))
                indices=indices[allowed]
                # Recover an actual measured representative, only if it agrees
                # with a supported plane. Not recursively used for interpolation.
                reason[indices]=7;grid.ravel()[cells[indices]]=heights[indices]
                recovered_count=len(indices)
            interpolated=proposals[~restore]
    if len(interpolated):
        fx,fy=interpolated[:,:2].astype(int).T
        grid[fy,fx]=interpolated[:,2]
        additions=np.column_stack((origin[0]+(fx+.5)*res,origin[1]+(fy+.5)*res,interpolated[:,2])).astype(np.float32)
        centers=np.vstack((centers,additions))
        reason=np.concatenate((reason,np.full(len(additions),5,np.uint8)))
        counts=np.concatenate((counts,np.zeros(len(additions),dtype=counts.dtype)))
        rough=np.concatenate((rough,np.zeros(len(additions))))
    # Include padded obstacle cells with no returns, so the mask is visible and
    # retained in exports. These have a nearby-ground reference height only.
    occupied=np.zeros(grid.shape,bool);occupied.ravel()[cells]=True
    py,px=np.where(barrier&~occupied)
    if len(px):
        z=reference[py,px];z=np.where(np.isfinite(z),z,0)
        centers=np.vstack((centers,np.column_stack((origin[0]+(px+.5)*res,origin[1]+(py+.5)*res,z))))
        reason=np.r_[reason,np.full(len(px),6,np.uint8)]
        counts=np.r_[counts,np.zeros(len(px),dtype=counts.dtype)];rough=np.r_[rough,np.zeros(len(px))]
    finite=grid[np.isfinite(grid)]
    meta={'resolution':res,'origin_xy':origin.tolist(),'shape_yx':[ny,nx],
          'valid_cells':len(finite),'observed_cells':int(initial_good.sum()),
          'interpolated_cells':len(interpolated),'recovered_cells':recovered_count,
          'obstacle_cells':int(barrier.sum()),'area_m2':round(len(finite)*res**2,3),
          'input_points':len(points),'cropped_points':len(xyz),
          'z_range':[float(finite.min()),float(finite.max())] if len(finite) else None,
          'reasons':np.bincount(reason,minlength=8).tolist(),'parameters':p,
          'frame':'R_y(pitch) @ R_x(roll) @ source_xyz; metres assumed; Z up',
          'unknown':'NaN = unobserved or rejected; not free space',
          'interpolation_method':'One-pass Delaunay linear within edge/height/slope limits; obstacles block triangles; optional measured-candidate recovery after wall classification',
          'interpolation_warning':'Inferred surface, not a measurement or proof of traversability'}
    return centers,reason,counts,rough,grid,meta



class State:
    def __init__(self,path):
        self.lock=threading.Lock(); self.points=None; self.name=None; self.revision=0; self.result=None
        if path: self.load(path.read_bytes(),path.name)
    def load(self,blob,name):
        points,encoding,dropped=read_pcd(blob)
        with self.lock:
            self.points=points; self.name=Path(name).name; self.revision+=1; self.result=None
            return self.info(encoding,dropped)
    def info(self,encoding=None,dropped=0):
        if self.points is None: return {'loaded':False}
        return {'loaded':True,'name':self.name,'points':len(self.points),'bounds':[self.points.min(axis=0).tolist(),self.points.max(axis=0).tolist()],'revision':self.revision,'encoding':encoding,'dropped':dropped,'display_points':min(len(self.points),300000)}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs): super().__init__(*args,directory=str(ROOT/'static'),**kwargs)
    def send(self,data,kind='application/json',status=200,filename=None):
        if kind=='application/json': data=json.dumps(data,allow_nan=False).encode()
        self.send_response(status); self.send_header('Content-Type',kind); self.send_header('Content-Length',str(len(data))); self.send_header('Cache-Control','no-store')
        if filename: self.send_header('Content-Disposition',f'attachment; filename="{filename}"')
        self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        try:
            path=urlparse(self.path).path
            if path=='/api/info':
                with self.server.state.lock: return self.send(self.server.state.info())
            if path=='/api/points':
                with self.server.state.lock:
                    points=self.server.state.points
                    if points is None: raise ValueError('PCD를 먼저 선택하세요')
                    # Evenly sample by index for display only; extraction always uses all points.
                    subset=points[np.linspace(0,len(points)-1,min(len(points),300000),dtype=int)]
                return self.send(subset.astype('<f4').tobytes(),'application/octet-stream')
            return super().do_GET()
        except (ValueError,KeyError) as e: self.send({'error':str(e)},status=400)
        except (BrokenPipeError,ConnectionResetError): pass
    def do_POST(self):
        try:
            # Mutations accept only same-origin browser requests (or local CLI without Origin).
            origin=self.headers.get('Origin')
            if origin and origin != 'http://'+self.headers.get('Host',''): return self.send({'error':'다른 출처의 요청은 허용하지 않습니다'},status=403)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=MAX_UPLOAD: raise ValueError('파일/요청 크기는 512MB 이하여야 합니다')
            body=self.rfile.read(size); path=urlparse(self.path).path; state=self.server.state
            if path=='/api/load':
                name=self.headers.get('X-Filename','selected.pcd')
                return self.send(state.load(body,name))
            request=json.loads(body)
            if path=='/api/process':
                with state.lock: points,revision,name=state.points,state.revision,state.name
                if points is None: raise ValueError('PCD를 먼저 선택하세요')
                if request['revision']!=revision: raise ValueError('맵이 변경됐습니다. 다시 불러오세요')
                t=time.perf_counter(); result=extract(points,request['parameters'])
                centers,reason,counts,rough,grid,meta=result
                token=uuid.uuid4().hex; meta.update(source=name,elapsed_ms=round((time.perf_counter()-t)*1000),revision=revision)
                with state.lock:
                    if revision!=state.revision: raise ValueError('맵이 변경됐습니다')
                    state.result=(token,result)
                return self.send({'token':token,'meta':meta,'cells':centers.tolist(),'reason':reason.tolist(),'counts':counts.tolist(),'roughness':rough.tolist(),'blocked':meta.get('cell_blocked',(reason==6).tolist()),'clearance':meta.get('cell_clearance',[None]*len(centers))})
            if path in ('/api/robot','/api/plan'):
                with state.lock: saved=state.result; points=state.points; revision=state.revision
                if not saved or saved[0]!=request.get('token'):raise ValueError('지면 결과가 변경됐습니다. 최신 결과에서 다시 계획하세요')
                ev=evaluate_robot(saved[1],request.get('robot',{}),points,rotation)
                response={'robot':ev['robot'],'cell_codes':ev['cell_codes'].tolist(),
                          'cell_slopes':ev['slope'][np.floor((saved[1][0][:,1]-ev['meta']['origin_xy'][1])/ev['meta']['resolution']).astype(int),np.floor((saved[1][0][:,0]-ev['meta']['origin_xy'][0])/ev['meta']['resolution']).astype(int)].tolist(),
                          'allowed_cells':int(ev['allowed'].sum()),'reason_labels':ROBOT_REASONS}
                labels,n=ndimage.label(ev['allowed'])
                suggestions=[]
                if n:
                    sizes=np.bincount(labels.ravel());sizes[0]=0
                    ys,xs=np.where(labels==int(np.argmax(sizes)))
                    if len(xs)>1:
                        center=np.array([ys.mean(),xs.mean()]);xy=np.column_stack((ys,xs))
                        a=int(np.argmax(np.linalg.norm(xy-center,axis=1)));b=int(np.argmax(np.linalg.norm(xy-xy[a],axis=1)))
                        for i in [a,b]:suggestions.append([float(ev['meta']['origin_xy'][0]+(xs[i]+.5)*ev['meta']['resolution']),float(ev['meta']['origin_xy'][1]+(ys[i]+.5)*ev['meta']['resolution']),float(ev['ground'][ys[i],xs[i]])])
                response['suggested']=suggestions
                if path=='/api/plan':response.update(plan_route(ev,request.get('start'),request.get('goal')))
                with state.lock:
                    if revision!=state.revision or state.result is None or state.result[0]!=saved[0]:raise ValueError('계산 중 지면 결과가 변경됐습니다')
                return self.send(response)
            if path=='/api/export':
                with state.lock: saved=state.result
                if not saved or saved[0]!=request['token']: raise ValueError('결과가 변경됐습니다. 현재 결과로 다시 저장하세요')
                centers,reason,counts,rough,grid,meta=saved[1]; selected=(reason==0)|(reason==5)|(reason==7); xyz=centers[selected]; inferred=(reason[selected]==5).astype(np.uint32); recovered=(reason[selected]==7).astype(np.uint32)
                kind=request['format']; output=io.BytesIO()
                blocked_values=np.asarray(meta.get('cell_blocked',(reason==6).tolist()),dtype=np.uint32)[selected]
                clear_values=np.asarray([np.nan if v is None else v for v in meta.get('cell_clearance',[None]*len(centers))],dtype=np.float32)[selected]
                if kind=='npz':
                    interpolated_mask=np.zeros(grid.shape,bool)
                    xy=np.floor((centers[reason==5,:2]-np.array(meta['origin_xy']))/meta['resolution']).astype(int)
                    if len(xy): interpolated_mask[xy[:,1],xy[:,0]]=True
                    recovered_mask=np.zeros(grid.shape,bool); obstacle_mask=np.zeros(grid.shape,bool)
                    for code,mask in [(7,recovered_mask),(6,obstacle_mask)]:
                        xy=np.floor((centers[reason==code,:2]-np.array(meta['origin_xy']))/meta['resolution']).astype(int)
                        if len(xy): mask[xy[:,1],xy[:,0]]=True
                    if 'cell_blocked' in meta:
                        xy=np.floor((centers[:,:2]-np.array(meta['origin_xy']))/meta['resolution']).astype(int)
                        obstacle_mask[xy[:,1],xy[:,0]]=np.asarray(meta['cell_blocked'],bool)
                    clearance_grid=np.full(grid.shape,np.nan,dtype=np.float32)
                    if 'cell_clearance' in meta:
                        xy=np.floor((centers[:,:2]-np.array(meta['origin_xy']))/meta['resolution']).astype(int)
                        clearance_grid[xy[:,1],xy[:,0]]=[np.nan if v is None else v for v in meta['cell_clearance']]
                    np.savez_compressed(output,elevation=grid,valid=np.isfinite(grid),observed=np.isfinite(grid)&~interpolated_mask&~recovered_mask,interpolated=interpolated_mask,recovered=recovered_mask,obstacle=obstacle_mask,traversable=np.isfinite(grid)&~obstacle_mask,clearance=clearance_grid,resolution=meta['resolution'],origin_xy=meta['origin_xy'],metadata_json=json.dumps(meta))
                    return self.send(output.getvalue(),'application/octet-stream',filename='elevation.npz')
                if kind=='json': return self.send(json.dumps(meta,indent=2).encode(),'application/json; charset=utf-8',filename='parameters.json')
                if kind=='csv':
                    text=io.StringIO(); np.savetxt(text,np.column_stack((xyz,inferred,recovered,blocked_values,clear_values)),delimiter=',',header='x_m,y_m,z_m,interpolated,recovered,blocked,clearance_m',comments='',fmt=['%.6f','%.6f','%.6f','%d','%d','%d','%.6f'])
                    return self.send(text.getvalue().encode(),'text/csv',filename='elevation.csv')
                if kind=='pcd':
                    header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z interpolated recovered blocked clearance\nSIZE 4 4 4 4 4 4 4\nTYPE F F F U U U F\nCOUNT 1 1 1 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
                    pcd=np.empty(len(xyz),dtype=[('xyz','<f4',(3,)),('interpolated','<u4'),('recovered','<u4'),('blocked','<u4'),('clearance','<f4')]); pcd['xyz']=xyz; pcd['interpolated']=inferred; pcd['recovered']=recovered; pcd['blocked']=blocked_values; pcd['clearance']=clear_values
                    return self.send(header.encode()+pcd.tobytes(),'application/octet-stream',filename='ground_surface.pcd')
                raise ValueError('알 수 없는 출력 형식')
            self.send({'error':'Not found'},status=404)
        except (BrokenPipeError,ConnectionResetError): pass
        except Exception as e:
            self.send({'error':str(e)},status=400)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,help='Optional initial PCD; otherwise choose a file in the browser')
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    server.state=State(args.map if args.map and args.map.exists() else None)
    print(f'Elevation Lab: http://127.0.0.1:{args.port}',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()

if __name__=='__main__': main()
