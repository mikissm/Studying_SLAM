"""Layer-aware ground extraction, independent full-cloud clearance checks.

One selected terrain sheet is returned; this is not a multifloor planner.
Ground elevation and collision occupancy are deliberately separate products.
"""
import heapq
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def number(p,key,default,lo,hi):
    v=float(p.get(key,default))
    if not np.isfinite(v) or not lo<=v<=hi: raise ValueError(f'{key}: {lo}~{hi} 범위가 필요합니다')
    return v


def extract_layered(points,p,rotation,interpolate_ground):
    res=number(p,'resolution',.1,.02,2)
    gap=number(p,'surfacegap',.08,.02,.3)
    thickness=number(p,'surfacethickness',.12,.04,.3)
    slope=number(p,'surfaceslope',25,5,60)
    linkstep=number(p,'linkstep',.15,0,.5)
    linkslope=number(p,'linkslope',25,5,60)
    maxgap=number(p,'maxgap',1.2,.02,3)
    minpoints=int(number(p,'minpoints',3,1,100))
    percentile=number(p,'percentile',10,0,50)
    zmin=number(p,'zmin',-1,-1e6,1e6);zmax=number(p,'zmax',5,-1e6,1e6)
    if zmin>=zmax: raise ValueError('바닥 후보 Z 최솟값은 최댓값보다 작아야 합니다')
    full=points.astype(np.float64)@rotation(p).T
    origin=np.floor(full[:,:2].min(axis=0)/res)*res
    ij=np.floor((full[:,:2]-origin)/res).astype(np.int64)
    nx,ny=map(int,ij.max(axis=0)+1);shape=(ny,nx)
    if nx*ny>2_000_000: raise ValueError('격자가 200만 셀을 넘습니다. 격자 크기를 늘려 주세요')
    ids=ij[:,1]*nx+ij[:,0]
    order=np.lexsort((full[:,2],ids));sorted_points=full[order];sorted_ids=ids[order]
    cells,starts,counts=np.unique(sorted_ids,return_index=True,return_counts=True)
    # Candidates hold (cell,height,point_count,spread,plane_quality).
    candidates=[];cell_candidates={};vertical_cells=set()
    cos_limit=np.cos(np.deg2rad(slope))
    for cell,start,count in zip(cells,starts,counts):
        pts=sorted_points[start:start+count]
        # Partition complete vertical columns into disconnected height clusters.
        cuts=np.r_[0,np.flatnonzero(np.diff(pts[:,2])>gap)+1,len(pts)]
        for a,b in zip(cuts[:-1],cuts[1:]):
            cluster=pts[a:b]
            # A continuous wall can connect floor to ceiling. Inspect narrow
            # overlapping bands rather than assigning one height to that column.
            if np.ptp(cluster[:,2])>thickness:
                bands=np.floor((cluster[:,2]-cluster[0,2])/(thickness*.5)).astype(int)
                spans=[]
                fine=np.floor((cluster[:,2]-cluster[0,2])/max(.01,thickness/6)).astype(int)
                modes,freq=np.unique(fine,return_counts=True)
                # Dense horizontal peaks preserve the floor even where a
                # continuous vertical wall touches the very same XY column.
                for peak in modes[np.argsort(freq)[-3:]]:
                    local=cluster[fine==peak]
                    if len(local)>=minpoints:spans.append(local)
                for band in np.unique(bands):
                    first=np.searchsorted(bands,band)
                    last=np.searchsorted(bands,band+2)
                    if last>first:spans.append(cluster[first:last])
            else: spans=[cluster]
            for sample in spans:
                if len(sample)<minpoints:continue
                position=(len(sample)-1)*percentile/100
                left=int(position);right=min(left+1,len(sample)-1)
                h=float(sample[left,2]*(1-(position-left))+sample[right,2]*(position-left))
                if h<zmin or h>zmax:continue
                spread=float(np.ptp(sample[:,2]))
                reliable=False;gradient=np.zeros(2)
                if len(sample)>=3:
                    selected=sample if len(sample)<=80 else sample[np.linspace(0,len(sample)-1,80,dtype=int)]
                    centered=selected-selected.mean(axis=0)
                    eigen,vectors=np.linalg.eigh(centered.T@centered/max(len(selected),1))
                    stable=eigen[1]>(res*.06)**2
                    if stable:
                        normal=abs(vectors[2,0])
                        if normal<cos_limit:
                            vertical_cells.add(int(cell));continue
                        # Large normal residuals indicate a volume, not a surface.
                        if eigen[0]>.15*max(eigen[1],1e-12):continue
                        reliable=True
                        gradient=-vectors[:2,0]/vectors[2,0]
                    elif spread>thickness*.45:continue
                elif spread>thickness*.45:continue
                # Collapse overlapping bands describing the same physical sheet.
                existing=cell_candidates.setdefault(int(cell),[])
                if any(abs(candidates[j][1]-h)<thickness*.55 for j in existing):continue
                existing.append(len(candidates))
                candidates.append((int(cell),h,len(sample),spread,reliable,float(gradient[0]),float(gradient[1])))
    if not candidates:raise ValueError('바닥 표면 후보가 없습니다. 후보 Z 범위·최소 점 수·표면 기울기를 확인하세요')
    c=np.asarray(candidates,dtype=float);cid=c[:,0].astype(int)
    cy,cx=np.divmod(cid,nx);xy=np.column_stack((cx,cy))*res
    heights=c[:,1];reliable=c[:,4]>0;has_reliable=bool(reliable.any())
    seedwidth=max(.05,thickness*.5)
    if p.get('seedauto',True):
        pool=heights[reliable] if reliable.sum()>=3 else heights
        bins=np.floor(pool/seedwidth).astype(int)
        values,freq=np.unique(bins,return_counts=True)
        significant=freq>=max(3,int(freq.max()*.20))
        seedbin=values[significant][0] if significant.any() else values[np.argmax(freq)]
        nearby=pool[(pool>=seedbin*seedwidth)&(pool<(seedbin+2)*seedwidth)]
        seed=float(np.median(nearby))
    else:seed=number(p,'seedheight',0,-1e6,1e6)
    seedtol=number(p,'seedradius',.5,.02,2)
    # Strong seeds stay near the chosen terrain level; the height tolerance
    # permits finding the closest observed seed but never seeds every shelf.
    eligible=np.flatnonzero(np.abs(heights-seed)<=seedtol)
    if not len(eligible):raise ValueError('기준 바닥 높이 근처에 표면이 없습니다. 기준 높이 또는 탐색 여유를 조정하세요')
    strong=eligible[reliable[eligible]]
    if not len(strong):strong=eligible
    distance_to_seed=np.abs(heights[strong]-seed)
    best=float(distance_to_seed.min())
    seeds=strong[distance_to_seed<=best+min(.10,seedtol)]
    # k-nearest sheet graph. Candidate-to-candidate edges never connect two
    # different heights in the same XY cell. Missing returns may be bridged.
    # A surface inside a vertical/occupied column may be retained, but cannot
    # be used as a stepping stone up the wall onto furniture or ceiling sheets.
    start_by_cell=dict(zip(map(int,cells),zip(starts,counts)))
    can_expand=np.ones(len(c),bool)
    # This is a geometric continuity check, not the robot collision band.
    # Changing robot height must not change the observed ground sheet.
    low_clearance=max(thickness,.15);body_height=max(thickness*4,.6)
    required=4
    allowweak=bool(p.get('recover',True))
    recovery_tolerance=number(p,'recoverheight',.15,.02,.4)
    for i,cell in enumerate(cid):
        start,count=start_by_cell[int(cell)]
        zs=sorted_points[start:start+count,2]
        occupied=np.searchsorted(zs,heights[i]+body_height,side='right')-np.searchsorted(zs,heights[i]+low_clearance,side='right')
        can_expand[i]=occupied<required
    tree=cKDTree(xy)
    k=min(24,len(c))
    distances,neighbors=tree.query(xy,k=k,distance_upper_bound=maxgap+1e-9)
    if k==1:distances=distances[:,None];neighbors=neighbors[:,None]
    costs=np.full(len(c),np.inf);queue=[]
    for i in seeds:
        costs[i]=abs(heights[i]-seed);heapq.heappush(queue,(costs[i],int(i)))
    tanlink=np.tan(np.deg2rad(linkslope))
    while queue:
        cost,i=heapq.heappop(queue)
        if cost>costs[i]+1e-12 or not can_expand[i]:continue
        for dist,j in zip(distances[i],neighbors[i]):
            if j>=len(c) or not np.isfinite(dist) or dist<res*.5 or cid[j]==cid[i]:continue
            dz=abs(heights[j]-heights[i])
            if dz>max(linkstep,dist*tanlink)+1e-9:continue
            displacement=xy[j]-xy[i]
            if not reliable[j] and (not allowweak or abs(heights[j]-heights[i]-c[i,5:7]@displacement)>recovery_tolerance):continue
            # Two flat but differently elevated shelves must not be connected
            # merely because a long diagonal edge makes the apparent slope low.
            if reliable[i] and abs(heights[j]-heights[i]-c[i,5:7]@displacement)>linkstep+1e-9:continue
            if reliable[j] and abs(heights[i]-heights[j]+c[j,5:7]@displacement)>linkstep+1e-9:continue
            score=cost+dist+dz*2+(0 if reliable[j] else .05)
            if score<costs[j]:
                costs[j]=score
                # Sparse candidates may be recovered from a supported surface,
                # but cannot form a ladder up a vertical wall to another sheet.
                if reliable[j] or not has_reliable:heapq.heappush(queue,(score,int(j)))
    # Select one reached terrain sheet per column. This is deliberately not a
    # stack of floors: a second floor requires an explicit different seed.
    chosen={}
    for i in np.flatnonzero(np.isfinite(costs)):
        if not reliable[i] and not allowweak:continue
        cell=int(cid[i]);prior=chosen.get(cell)
        if prior is None or costs[i]<costs[prior]:chosen[cell]=int(i)
    ground=np.full(shape,np.nan,dtype=np.float32)
    surface_counts=np.zeros(shape,np.int32);surface_spread=np.zeros(shape,np.float32)
    classes=np.full(shape,4,np.uint8)
    for cell,i in chosen.items():
        ground.ravel()[cell]=heights[i]
        surface_counts.ravel()[cell]=int(c[i,2]);surface_spread.ravel()[cell]=c[i,3]
        classes.ravel()[cell]=0 if reliable[i] else 7
    # A floor under a wall/furniture remains present in `ground`. Occupancy is
    # evaluated separately against ALL transformed input points, not Z-cropped.
    clearance_min=number(p,'obstacleheight',.22,.05,1)
    robotheight=number(p,'robotheight',1.5,.1,3)
    if robotheight<=clearance_min:raise ValueError('장애물 검사 높이는 시작 높이보다 커야 합니다')
    obstaclepoints=int(number(p,'obstaclepoints',4,1,100))
    known=np.isfinite(ground)
    if not known.any():raise ValueError('연결된 지면이 없습니다')
    _,nearest=ndimage.distance_transform_edt(~known,return_indices=True)
    reference=ground[tuple(nearest)]
    dz=full[:,2]-reference.ravel()[ids]
    body=(dz>clearance_min)&(dz<=robotheight)
    occupancy=np.bincount(ids[body],minlength=ground.size).reshape(shape)>=obstaclepoints
    # Thin observed vertical surfaces with insufficient returns still block if
    # they actually enter the low collision band (not high ceiling sidewalls).
    for cell in vertical_cells:
        if np.any(body[order[np.searchsorted(sorted_ids,cell):np.searchsorted(sorted_ids,cell,side='right')]]):occupancy.ravel()[cell]=True
    # Unselected elevated surfaces are not automatically obstacles at ground
    # level: overhangs above robotheight remain traversable below.
    padding=number(p,'wallpadding',0,0,.5)
    if padding and occupancy.any():occupancy|=ndimage.distance_transform_edt(~occupancy)*res<=padding+1e-9
    # Clearance is the first measured return above the ground tolerance. A lack
    # of such returns is unknown, not an infinite measured clearance.
    overhead=np.full(ground.size,np.inf)
    np.minimum.at(overhead,ids[dz>clearance_min],dz[dz>clearance_min])
    clearance=overhead.reshape(shape)
    # Surface fill is conservative across occupied cells, while observed floor
    # in those same cells is retained as a separate, blocked surface.
    if p.get('interpolate',False):
        supports=ground.copy();supports[occupancy]=np.nan
        protected=np.flatnonzero(known|occupancy)
        proposals=interpolate_ground(supports,protected,np.flatnonzero(occupancy),res,p)
        if len(proposals):
            fx,fy=proposals[:,:2].astype(int).T
            ground[fy,fx]=proposals[:,2];classes[fy,fx]=5
    surface=np.isfinite(ground)
    # Re-evaluate full-cloud collision and clearance against filled heights too.
    ref=np.where(surface,ground,reference)
    dz=full[:,2]-ref.ravel()[ids]
    body=(dz>clearance_min)&(dz<=robotheight)
    occupancy|=np.bincount(ids[body],minlength=ground.size).reshape(shape)>=obstaclepoints
    overhead=np.full(ground.size,np.inf);np.minimum.at(overhead,ids[dz>clearance_min],dz[dz>clearance_min])
    clearance=overhead.reshape(shape)
    # Occupancy and surface can overlap. Cells without a selected surface are
    # drawn as obstacles only if full-cloud points provide collision evidence.
    display=surface|occupancy
    y,x=np.where(display)
    z=np.where(surface[y,x],ground[y,x],reference[y,x])
    centers=np.column_stack((origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,z)).astype(np.float32)
    reason=np.where(surface[y,x],classes[y,x],6).astype(np.uint8)
    blocked=occupancy[y,x]
    gaps=clearance[y,x]
    count_values=surface_counts[y,x];rough=surface_spread[y,x]
    observed=(reason==0);recovered=reason==7;interpolated=reason==5
    traversable=surface&~occupancy
    finite=ground[surface]
    meta={'algorithm':'layered_surface_v2','obstacle_source':'full_cloud','resolution':res,
          'origin_xy':origin.tolist(),'shape_yx':[ny,nx],'seed_height_m':seed,
          'candidate_surfaces':len(c),'surface_cells':int(surface.sum()),
          'valid_cells':int(surface.sum()),'observed_cells':int(observed.sum()),
          'recovered_cells':int(recovered.sum()),'interpolated_cells':int(interpolated.sum()),
          'obstacle_cells':int(occupancy.sum()),'blocked_surface_cells':int((surface&occupancy).sum()),
          'traversable_cells':int(traversable.sum()),'area_m2':round(float(surface.sum()*res**2),3),
          'traversable_area_m2':round(float(traversable.sum()*res**2),3),
          'input_points':len(points),'cropped_points':int(((full[:,2]>=zmin)&(full[:,2]<=zmax)).sum()),
          'z_range':[float(finite.min()),float(finite.max())],
          'reasons':np.bincount(reason,minlength=8).tolist(),'parameters':p,
          'classification_counts':{'observed':int(observed.sum()),'recovered':int(recovered.sum()),'interpolated':int(interpolated.sum()),'no_surface_obstacle':int((reason==6).sum())},
          'frame':'R_y(pitch) @ R_x(roll) @ source_xyz; metres assumed; Z up',
          'unknown':'No selected floor = unknown; missing ceiling returns do not prove clearance',
          'interpolation_method':'Single-sheet surface connectivity + one-pass barrier-aware Delaunay fill',
          'interpolation_warning':'Surface presence and traversability are different; this is not a robot safety certificate',
          'cell_blocked':blocked.tolist(),
          'cell_clearance':[float(v) if np.isfinite(v) else None for v in gaps]}
    return centers,reason,count_values,rough,ground,meta
