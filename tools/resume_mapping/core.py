"""ROS-independent geometry for saved-map continuation (metres, radians)."""
from pathlib import Path
import os
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'elevation_viewer'))
from server import read_pcd


def transform(points, pose):
    return np.asarray(points) @ pose[:3,:3].T + pose[:3,3]


def pose_matrix(x=0., y=0., z=0., yaw=0.):
    out=np.eye(4);out[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();out[:3,3]=[x,y,z]
    return out


def voxel(points, resolution):
    points=np.asarray(points,dtype=np.float64)
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points):return np.empty((0,3))
    _,indices=np.unique(np.floor(points/resolution).astype(np.int64),axis=0,return_index=True)
    return points[np.sort(indices)]


def align(scan, target, guess, correspondence=.6, min_overlap=.55, max_rmse=.15):
    """Gated point-to-point ICP, initialized near the known starting pose.

    Gate coverage against ALL scan points, not only matched points. This is a
    geometric acceptance test, not proof of correct global localization.
    """
    scan=voxel(scan,.12);target=voxel(target,.12)
    if len(scan)>6000:scan=scan[np.linspace(0,len(scan)-1,6000).astype(int)]
    if len(scan)<30 or len(target)<30:raise ValueError('정합 점 부족 (최소 30점)')
    tree=cKDTree(target);pose=guess.copy()
    for _ in range(40):
        moved=transform(scan,pose);dist,idx=tree.query(moved)
        keep=dist<correspondence
        if keep.sum()<30:raise ValueError('초기 위치 주변의 대응점 부족')
        # Trim the worst matches to reduce influence of newly observed surfaces.
        keep &= dist<=np.quantile(dist[keep],.85)
        a=moved[keep];b=target[idx[keep]];ac=a.mean(axis=0);bc=b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ac).T@(b-bc));rot=vt.T@u.T
        if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
        delta=np.eye(4);delta[:3,:3]=rot;delta[:3,3]=bc-rot@ac;pose=delta@pose
        if np.linalg.norm(delta[:3,3])<1e-4 and Rotation.from_matrix(rot).magnitude()<1e-4:break
    dist,_=tree.query(transform(scan,pose));keep=dist<correspondence
    overlap=float(keep.mean());rmse=float(np.sqrt(np.mean(dist[keep]**2))) if keep.any() else float('inf')
    correction=pose@np.linalg.inv(guess)
    if overlap<min_overlap or rmse>max_rmse:raise ValueError(f'정합 거부: overlap={overlap:.2f}, RMSE={rmse:.3f}m')
    if np.linalg.norm(pose[:3,3]-guess[:3,3])>2 or Rotation.from_matrix(correction[:3,:3]).magnitude()>np.deg2rad(30):
        raise ValueError('초기 위치 보정량 초과 (2m / 30도); 초기 위치를 다시 지정하세요')
    return pose,{'overlap':overlap,'rmse_m':rmse}


class MapStore:
    def __init__(self, source, resolution=.1, max_voxels=2000000):
        if resolution<=0 or max_voxels<1:raise ValueError('잘못된 맵 크기 설정')
        self.source=Path(source).resolve();self.resolution=resolution;self.max_voxels=max_voxels
        loaded=read_pcd(self.source.read_bytes())
        points=loaded[0] if isinstance(loaded,tuple) else loaded
        self.cells={};self.add(np.asarray(points)[:,:3])

    def add(self, points):
        points=voxel(points,self.resolution)
        keys=np.floor(points/self.resolution).astype(np.int64)
        updates={tuple(k):p for k,p in zip(keys,points)}
        if len(self.cells)+sum(k not in self.cells for k in updates)>self.max_voxels:
            raise ValueError('맵 셀 한도 초과; 누적 중지. 저장 후 더 큰 한도로 재시작하세요')
        # Keep original samples in already observed voxels.
        for k,p in updates.items():self.cells.setdefault(k,p)

    def points(self):return np.asarray(list(self.cells.values()),dtype=np.float32).reshape(-1,3)

    def save(self, path):
        path=Path(path).expanduser().resolve()
        if path==self.source:raise ValueError('원본 지도에는 덮어쓸 수 없습니다')
        if path.exists():raise ValueError('출력 파일이 이미 있습니다; 다른 이름을 지정하세요')
        path.parent.mkdir(parents=True,exist_ok=True);points=self.points().astype('<f4')
        header=f'# resumed XYZ map\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'.encode()
        # Exclusive create protects source and existing outputs, including races.
        with path.open('xb') as f:f.write(header);f.write(points.tobytes());f.flush();os.fsync(f.fileno())
        return str(path)
