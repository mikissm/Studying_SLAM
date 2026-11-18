import unittest
import numpy as np
from planner import evaluate,plan


def fixture(n=11,obstacles=(),quality=None):
    y,x=np.indices((n,n));ground=np.zeros((n,n),np.float32)
    centers=np.column_stack(((x.ravel()+.5)*.1,(y.ravel()+.5)*.1,ground.ravel()))
    classes=np.zeros(n*n,np.uint8) if quality is None else quality.ravel()
    points=centers.copy().tolist()
    for yy,xx in obstacles:points.extend([[xx*.1+.05,yy*.1+.05,z] for z in [.2,.3,.4,.45]])
    meta={'resolution':.1,'origin_xy':[0,0],'parameters':{'obstaclepoints':3},'shape_yx':[n,n]}
    return (centers,classes,np.ones(n*n),np.zeros(n*n),ground,meta),np.array(points)


def ev(n=11,obstacles=(),**params):
    r,p=fixture(n,obstacles)
    return evaluate(r,{'radius':0,'margin':0,'strictunknown':False,**params},p,lambda _:np.eye(3))

class PlannerTests(unittest.TestCase):
    def test_flat_path_and_same_start_goal(self):
        e=ev();r=plan(e,[.15,.15],[.85,.85]);self.assertTrue(r['ok']);self.assertAlmostEqual(r['length_m'],np.sqrt(.7**2+.7**2),places=3)
        same=plan(e,[.15,.15],[.15,.15]);self.assertTrue(same['ok']);self.assertEqual(same['length_m'],0)
    def test_no_corner_cutting(self):
        e=ev(n=2,obstacles=[(0,1),(1,0)])
        self.assertFalse(plan(e,[.05,.05],[.15,.15])['ok'])
    def test_obstacle_and_body_radius(self):
        e=ev(obstacles=[(5,5)],radius=.15)
        self.assertTrue(e['codes'][5,5]&2);self.assertTrue(e['codes'][5,6]&8)
        self.assertFalse(plan(e,[.55,.55],[.85,.85])['ok'])
    def test_unknown_footprint_and_map_boundary(self):
        e=ev(radius=.15,strictunknown=True)
        self.assertTrue(e['codes'][0,0]&32);self.assertTrue(e['allowed'][5,5])
    def test_height_and_max_step(self):
        e=ev();e['ground'][:,6:]=.2
        self.assertFalse(plan(e,[.15,.55],[.85,.55])['ok'])
        e['robot']['maxstep']=.25
        self.assertTrue(plan(e,[.15,.55],[.85,.55])['ok'])
    def test_inference_penalty_prefers_observed_detour(self):
        r,p=fixture();r[1].reshape(11,11)[5,2:9]=5
        e=evaluate(r,{'radius':0,'margin':0,'strictunknown':False,'inferredcost':20},p,lambda _:np.eye(3))
        path=plan(e,[.15,.55],[.95,.55]);self.assertTrue(path['ok']);self.assertEqual(path['inferred_ratio'],0)
    def test_overhead_robot_height_changes_collision(self):
        r,p=fixture();p=np.vstack((p,[[.55,.55,.7]]*4))
        low=evaluate(r,{'radius':0,'margin':0,'height':.5,'strictunknown':False},p,lambda _:np.eye(3))
        tall=evaluate(r,{'radius':0,'margin':0,'height':1.,'strictunknown':False},p,lambda _:np.eye(3))
        self.assertTrue(low['allowed'][5,5]);self.assertTrue(tall['codes'][5,5]&4)
    def test_local_ramp_rejected(self):
        r,p=fixture();r[4][:]=np.indices((11,11))[1]*.1
        p=r[0].copy();p[:,2]=r[4].ravel()
        e=evaluate(r,{'radius':0,'margin':0,'maxslope':25,'strictunknown':False},p,lambda _:np.eye(3))
        self.assertTrue(e['codes'][5,5]&16)

if __name__=='__main__':unittest.main()
