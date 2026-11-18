"""PCD interoperability and extraction invariants, no GPU required."""
import io
import json
import threading
import unittest
from urllib.request import Request,urlopen
import numpy as np
from server import read_pcd,extract,interpolate_ground,State,Handler,ThreadingHTTPServer

PARAMS=dict(resolution=.1,percentile=10,minpoints=1,zmin=-2,zmax=5,roll=0,pitch=0,envelope=True,radius=.2,tolerance=.15,roughfilter=False,roughness=.1,components=False,mincells=1,seedfilter=False,seedpercentile=35,seedmargin=.15)

class CoreTests(unittest.TestCase):
    def test_pcd_formats_and_extra_fields(self):
        # COUNT > 1 before XYZ exercises byte and ASCII-column offsets.
        header=b'VERSION 0.7\nFIELDS normal x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 3 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n'
        rows=np.array([[0,0,1,1,2,3],[0,0,1,4,5,6]],dtype='<f4')
        expected=rows[:,3:]
        binary=header+b'DATA binary\n'+rows.tobytes()
        np.testing.assert_equal(read_pcd(binary)[0],expected)
        ascii_data=io.StringIO();np.savetxt(ascii_data,rows)
        np.testing.assert_equal(read_pcd(header+b'DATA ascii\n'+ascii_data.getvalue().encode())[0],expected)
        soa=rows[:,:3].tobytes()+b''.join(rows[:,i].tobytes() for i in range(3,6))
        # Valid LZF literal blocks suffice to exercise compressed layout.
        payload=b''.join(bytes([len(soa[i:i+32])-1])+soa[i:i+32] for i in range(0,len(soa),32))
        compressed=header+b'DATA binary_compressed\n'+np.array([len(payload),len(soa)],dtype='<u4').tobytes()+payload
        np.testing.assert_equal(read_pcd(compressed)[0],expected)
    def test_filters_and_unknown_cells(self):
        points=np.array([[.05,.05,0],[.15,.05,0],[.25,.05,1],[.45,.05,0]],dtype=np.float32)
        centers,reasons,_,_,grid,meta=extract(points,PARAMS)
        self.assertEqual(meta['valid_cells'],3)
        self.assertEqual(reasons.tolist(),[0,0,2,0])
        self.assertTrue(np.isnan(grid[0,2]))
        self.assertTrue(np.isnan(grid[0,3]))  # unobserved stays unknown
        self.assertEqual(extract(points,{**PARAMS,'envelope':False})[-1]['valid_cells'],4)
    def test_quantile_and_frame(self):
        points=np.array([[.05,.05,z] for z in [0,.1,.2,1]],dtype=np.float32)
        out=extract(points,{**PARAMS,'envelope':False})
        self.assertAlmostEqual(out[0][0,2],.03,places=6)
        rotated=extract(np.array([[.05,.05,1]],dtype=np.float32),{**PARAMS,'pitch':90,'envelope':False})
        self.assertAlmostEqual(rotated[0][0,2],-.05,places=6)
    def test_interpolation_plane_limits_and_obstacles(self):
        grid=np.full((7,7),np.nan,dtype=np.float32)
        for y,x in [(1,1),(1,5),(5,1),(5,5)]: grid[y,x]=.01*x+.005*y
        cells=np.flatnonzero(np.isfinite(grid))
        p=dict(maxgap=1,maxheight=.3,maxslope=20)
        filled=interpolate_ground(grid,cells,[],.1,p)
        self.assertEqual(len(filled),21)
        self.assertTrue(((filled[:,:2]>=1)&(filled[:,:2]<=5)).all())
        np.testing.assert_allclose(filled[:,2],filled[:,0]*.01+filled[:,1]*.005,atol=1e-7)
        self.assertEqual(len(interpolate_ground(grid,cells,[],.1,{**p,'maxgap':.3})),0)
        self.assertEqual(len(interpolate_ground(grid,cells,[],.1,{**p,'maxheight':.01})),0)
        self.assertEqual(len(interpolate_ground(grid,cells,[],.1,{**p,'maxslope':1})),0)
        blocked=np.array([3*7+3])
        self.assertEqual(len(interpolate_ground(grid,np.r_[cells,blocked],blocked,.1,p)),0)

    def test_interpolation_export_masks(self):
        points=np.array([[x+.05,y+.05,0] for x,y in [(0,0),(.4,0),(0,.4),(.4,.4)]],dtype=np.float32)
        result=extract(points,{**PARAMS,'envelope':False,'interpolate':True,'maxgap':1,'maxheight':.2,'maxslope':20})
        centers,reasons,counts,_,grid,meta=result
        self.assertEqual(meta['observed_cells'],4)
        self.assertEqual(meta['interpolated_cells'],21)
        self.assertEqual(meta['valid_cells'],25)
        self.assertEqual(int((reasons==5).sum()),21)
        self.assertTrue((counts[reasons==5]==0).all())
        self.assertEqual(int(np.isfinite(grid).sum()),25)

    def test_wall_and_ceiling_separation(self):
        points=[]
        for x in [.05,.15,.25]:
            for y in [.05,.15,.25]:
                for dx,dy in [(-.01,-.01),(-.01,.01),(.01,-.01),(.01,.01)]:
                    points.extend([[x+dx,y+dy,0],[x+dx,y+dy,2.8]])
        points.extend([[.15,.15,float(z)] for z in np.linspace(.1,1,25)])
        r=extract(np.array(points,dtype=np.float32),{**PARAMS,'envelope':False,'wallfilter':True})
        centers,reasons,_,_,grid,meta=r
        wall=np.isclose(centers[:,0],.15)&np.isclose(centers[:,1],.15)
        self.assertTrue((reasons[wall]==6).all())
        self.assertTrue(np.isnan(grid[1,1]))
        self.assertEqual(meta['observed_cells'],8)  # ceiling does not erase floor
        self.assertEqual(meta['obstacle_cells'],1)

    def test_recover_low_count_floor_but_not_wall(self):
        points=[]
        for x,y in [(x,y) for x in range(7) for y in range(7) if x in (0,6) or y in (0,6)]:
            points.extend([[x*.1+.05,y*.1+.05,0]]*3)
        points.append([.35,.35,.02])
        p={**PARAMS,'envelope':False,'minpoints':3,'wallfilter':True,'interpolate':True,'maxgap':1.2,'maxheight':.2,'recover':True}
        r=extract(np.array(points,dtype=np.float32),p)
        self.assertEqual(r[-1]['recovered_cells'],1)
        self.assertAlmostEqual(float(r[4][3,3]),.02,places=6)
        self.assertEqual(int((r[1]==7).sum()),1)
        without=extract(np.array(points,dtype=np.float32),{**p,'recover':False})
        self.assertTrue(np.isnan(without[4][3,3]))
        # Replace that weak floor return with a vertical obstacle. Never recover.
        wallpoints=points[:-1]+[[.35,.35,float(z)] for z in np.linspace(0,1,25)]
        wall=extract(np.array(wallpoints,dtype=np.float32),p)
        self.assertEqual(wall[-1]['recovered_cells'],0)
        self.assertTrue(np.isnan(wall[4][3,3]))
        self.assertTrue((wall[1][np.isclose(wall[0][:,0],.35)&np.isclose(wall[0][:,1],.35)]==6).all())

    def test_http_exports_match_current_result(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.state=State(None)
        points=[]
        for x,y in [(x,y) for x in range(7) for y in range(7) if x in (0,6) or y in (0,6)]:
            points.extend([[x*.1+.05,y*.1+.05,0]]*3)
        points.append([.35,.35,.02])
        points.extend([[.85,.85,float(z)] for z in np.linspace(0,1,25)])
        server.state.points=np.array(points,dtype=np.float32);server.state.name='fixture.pcd';server.state.revision=1
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base=f'http://127.0.0.1:{server.server_port}'
        def post(path,data):
            with urlopen(Request(base+path,json.dumps(data).encode(),{'Content-Type':'application/json'})) as r:return r.read()
        try:
            result=json.loads(post('/api/process',{'revision':1,'parameters':{**PARAMS,'envelope':False,'interpolate':True,'maxgap':1.2,'minpoints':3,'wallfilter':True,'recover':True}}))
            for kind in ['npz','csv','pcd','json']:
                blob=post('/api/export',{'token':result['token'],'format':kind})
                if kind=='npz':
                    with np.load(io.BytesIO(blob)) as data:
                        self.assertEqual(int(data['valid'].sum()),49)
                        self.assertEqual(int(data['observed'].sum()),24)
                        self.assertEqual(int(data['interpolated'].sum()),24)
                        self.assertEqual(int(data['recovered'].sum()),1)
                        self.assertEqual(int(data['obstacle'].sum()),1)
                        self.assertFalse((data['valid']&data['obstacle']).any())
                        self.assertFalse((data['observed']&data['interpolated']).any())
                if kind=='pcd': self.assertEqual(len(read_pcd(blob)[0]),49)
                if kind=='pcd': self.assertIn(b'FIELDS x y z interpolated recovered',blob)
                if kind=='csv': self.assertEqual(len(blob.decode().splitlines()),50)
                if kind=='csv': self.assertEqual(sum(line.split(',')[3]=='1' for line in blob.decode().splitlines()[1:]),24)
                if kind=='csv': self.assertEqual(sum(line.split(',')[4]=='1' for line in blob.decode().splitlines()[1:]),1)
                if kind=='json': self.assertEqual(json.loads(blob)['recovered_cells'],1)
        finally:server.shutdown();server.server_close();thread.join()

if __name__=='__main__':unittest.main()
