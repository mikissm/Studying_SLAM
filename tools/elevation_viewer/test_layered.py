import io
import json
import threading
from urllib.request import Request,urlopen
import unittest
import numpy as np
from server import extract, State, Handler, ThreadingHTTPServer
from test_core import PARAMS


def patch(x,y,z,n=9):
    return [[x+dx,y+dy,z] for dx in np.linspace(-.025,.025,3) for dy in np.linspace(-.025,.025,3)][:n]


def fixture():
    points=[]
    for x in [.05,.15,.25,.35,.45]:
        for y in [.05,.15,.25,.35,.45]:
            points+=patch(x,y,0)
            points+=patch(x,y,2.8)
    points+=patch(.25,.25,.7)
    points+=[[.35,.35,float(z)] for z in np.linspace(0,1.2,30)]
    return np.array(points,dtype=np.float32)


P={**PARAMS,'layered':True,'minpoints':3,'seedauto':False,'seedheight':0,'seedradius':.3,'interpolate':False,'maxgap':.4}

class LayeredTests(unittest.TestCase):
    def test_floor_survives_wall_and_low_overhang(self):
        centers,reason,counts,rough,ground,meta=extract(fixture(),P)
        self.assertEqual(meta['surface_cells'],25)
        np.testing.assert_allclose(ground,0,atol=1e-7)
        self.assertEqual(meta['blocked_surface_cells'],2)
        self.assertEqual(meta['traversable_cells'],23)
        self.assertEqual(meta['obstacle_source'],'full_cloud')
        self.assertEqual(sum(meta['cell_blocked']),2)
        self.assertTrue(all(v==0 for v in reason))
    def test_candidate_z_crop_does_not_remove_obstacles(self):
        a=extract(fixture(),{**P,'zmax':3})
        b=extract(fixture(),{**P,'zmax':.1})
        np.testing.assert_array_equal(a[4],b[4])
        self.assertEqual(a[-1]['cell_blocked'],b[-1]['cell_blocked'])
        np.testing.assert_allclose(a[-1]['cell_clearance'],b[-1]['cell_clearance'])
    def test_ceiling_above_robot_is_not_blocked(self):
        points=np.array(patch(.05,.05,0)+patch(.05,.05,2.8),dtype=np.float32)
        r=extract(points,P)
        self.assertEqual(r[-1]['surface_cells'],1)
        self.assertEqual(r[-1]['blocked_surface_cells'],0)
        self.assertAlmostEqual(r[-1]['cell_clearance'][0],2.8,places=5)
    def test_vertical_wall_without_floor_is_not_surface(self):
        pts=patch(.05,.05,0)+patch(.15,.05,0)
        pts+=[[.25,y,z] for y in np.linspace(.025,.075,6) for z in np.linspace(.2,1.4,18)]
        r=extract(np.array(pts,dtype=np.float32),P)
        self.assertTrue(np.isnan(r[4][0,2]))
        self.assertIn(6,r[1])
    def test_manual_seed_selects_separate_upper_surface(self):
        points=np.array(patch(.05,.05,0)+patch(.05,.05,2.8),dtype=np.float32)
        r=extract(points,{**P,'seedheight':2.8})
        self.assertAlmostEqual(float(r[4][0,0]),2.8,places=5)
    def test_robot_height_changes_occupancy_not_observed_floor(self):
        low=extract(fixture(),{**P,'robotheight':.5})
        high=extract(fixture(),{**P,'robotheight':1.5})
        np.testing.assert_array_equal(low[4],high[4])
        self.assertLess(low[-1]['blocked_surface_cells'],high[-1]['blocked_surface_cells'])

    def test_flat_shelf_is_not_reached_by_long_diagonal(self):
        points=np.array(patch(.05,.05,0)+patch(.15,.05,0)+patch(.55,.05,.4),dtype=np.float32)
        r=extract(points,{**P,'maxgap':1,'linkslope':60})
        self.assertTrue(np.isnan(r[4][0,5]))
        self.assertEqual(r[-1]['surface_cells'],2)
    def test_continuous_ramp_can_leave_seed_height_band(self):
        points=[]
        for x in np.arange(11)*.1+.05:
            for y in [.05,.15,.25]:
                for point in patch(x,y,0):
                    point[2]=.2*(point[0]-.05)
                    points.append(point)
        r=extract(np.array(points,dtype=np.float32),{**P,'seedradius':.1,'maxgap':.25})
        self.assertEqual(r[-1]['surface_cells'],33)
        self.assertGreater(float(np.nanmax(r[4])),.17)

    def test_api_exports_keep_blocked_floor_separate(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.state=State(None)
        server.state.points=fixture();server.state.name='layered.pcd';server.state.revision=1
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        def post(path,data):
            with urlopen(Request(f'http://127.0.0.1:{server.server_port}'+path,json.dumps(data).encode(),{'Content-Type':'application/json'})) as response:return response.read()
        try:
            result=json.loads(post('/api/process',{'revision':1,'parameters':P}))
            self.assertEqual(sum(result['blocked']),2)
            data=post('/api/export',{'token':result['token'],'format':'npz'})
            with np.load(io.BytesIO(data)) as saved:
                self.assertEqual(int(saved['valid'].sum()),25)
                self.assertEqual(int(saved['traversable'].sum()),23)
                self.assertEqual(int((saved['valid']&saved['obstacle']).sum()),2)
                self.assertAlmostEqual(float(saved['elevation'][2,2]),0)
                self.assertAlmostEqual(float(saved['clearance'][2,2]),.7,places=5)
            csv=post('/api/export',{'token':result['token'],'format':'csv'}).decode().splitlines()
            self.assertIn('blocked,clearance_m',csv[0])
            self.assertEqual(sum(row.split(',')[5]=='1' for row in csv[1:]),2)
        finally:server.shutdown();server.server_close();thread.join()

    def test_no_fake_infinite_clearance(self):
        r=extract(np.array(patch(.05,.05,0),dtype=np.float32),P)
        self.assertIsNone(r[-1]['cell_clearance'][0])

if __name__=='__main__':unittest.main()
