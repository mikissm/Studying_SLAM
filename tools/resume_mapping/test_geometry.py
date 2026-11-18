import tempfile
import unittest
from pathlib import Path
import numpy as np
from core import MapStore, align, transform, pose_matrix, read_pcd


def write(path,points):
    data=np.asarray(points,dtype='<f4')
    path.write_bytes(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(data)}\nHEIGHT 1\nPOINTS {len(data)}\nDATA binary\n'.encode()+data.tobytes())


class ResumeTests(unittest.TestCase):
    def test_alignment_and_odom_composition(self):
        rng=np.random.default_rng(31);scan=rng.uniform(-3,3,(2500,3))
        truth=pose_matrix(2,1,.3,.2);target=transform(scan,truth)
        aligned,stats=align(scan,target,pose_matrix(2.1,.9,.35,.23))
        np.testing.assert_allclose(aligned,truth,atol=.015)
        odom=pose_matrix(4,-2,.1,.4);offset=aligned@np.linalg.inv(odom)
        np.testing.assert_allclose(transform(scan,offset@odom),target,atol=.05)
        self.assertGreater(stats['overlap'],.95)

    def test_unrelated_scan_rejected(self):
        rng=np.random.default_rng(2)
        with self.assertRaises(ValueError):align(rng.random((100,3)),rng.random((100,3))+20,np.eye(4))

    def test_merge_roundtrip_and_source_protection(self):
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'original.pcd';dst=Path(tmp)/'new.pcd'
            write(src,[[0,0,0],[1,0,0]]);original=src.read_bytes();store=MapStore(src)
            store.add([[0.01,0,0],[2,0,0],[np.nan,0,0]])
            self.assertEqual(len(store.cells),3)
            with self.assertRaises(ValueError):store.save(src)
            store.save(dst);self.assertEqual(len(read_pcd(dst.read_bytes())[0]),3)
            self.assertEqual(src.read_bytes(),original)
            with self.assertRaises(ValueError):store.save(dst)

    def test_capacity_rejection_is_atomic(self):
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'original.pcd';write(src,[[0,0,0]])
            store=MapStore(src,max_voxels=2)
            with self.assertRaises(ValueError):store.add([[1,0,0],[2,0,0]])
            self.assertEqual(len(store.cells),1)

if __name__=='__main__':unittest.main()
