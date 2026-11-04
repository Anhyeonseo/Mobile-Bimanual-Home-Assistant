import base64,json,sys
from pathlib import Path
import cv2,numpy as np
R0=Path('/home/yeoeun/Bimanual-Household-Manipulation'); sys.path.insert(0,str(R0))
from home_perception.io.snapshot import load_snapshot
from home_perception.geometry.plane import backproject,fit_plane
from home_perception.geometry.workspace import workspace_mask
SP=Path(sys.argv[1]); G=R0/'tmp/tabletop-grasps-r3'; P=R0/'tmp/tabletop-perception-gpu-r2'
gr=json.load(open(G/'report.json')); per=json.load(open(P/'report.json'))
by={}
for o in gr['objects']: by.setdefault(o['capture'],[]).append(o)
NAMES={'01':'섞인 물체 1','02':'섞인 물체 2','03':'비슷한 물체 1','04':'비슷한 물체 2','05':'리모컨만 1','06':'리모컨만 2','07':'검은 컵 2개','08':'컵이 컵을 가림','09':'여러 물체 1','10':'여러 물체 2'}
scenes=[]; rng=np.random.default_rng(0)
for c in per['captures']:
    cap=c['capture']; s=load_snapshot(R0/'data/tabletop_rgbd_20261101'/cap); d=s.depth_m(); I=s.intrinsics; H,W=d.shape
    plane=fit_plane(backproject(d,I,d>0))
    masks=[cv2.imread(str(P/cap/f'mask_{i:03}.png'),0)>0 for i in range(len(c['observations']))]
    ws=workspace_mask(d,I,plane,masks)&(d>0)
    idx=np.full((H,W),-1,np.int8)
    for i,m in enumerate(masks): idx[m]=i
    keep=(ws&(idx>=0))|(ws&(rng.random((H,W))<.13))
    pts=backproject(d,I,keep); col=cv2.cvtColor(s.bgr,cv2.COLOR_BGR2RGB)[keep]; oid=idx[keep]
    # table frame: origin on the table, +Y up, so the view stays level
    up=plane.normal; ex=np.array([1.,0,0]); ex-=up*(ex@up); ex/=np.linalg.norm(ex); ez=np.cross(ex,up); T=np.vstack([ex,up,ez])
    # keep the camera on the +Z side so three.js looks in a natural direction
    origin=np.median(pts[oid<0],axis=0); origin=origin-up*(plane.height(origin[None])[0])
    if (T@(-origin))[2]<0: ez=-ez; ex=-ex; T=np.vstack([ex,up,ez])
    tp=(pts-origin)@T.T; cam=(T@(-origin)).tolist()
    xyz=np.int16(np.round(tp*10000))
    objs=[]
    for g,o in zip(by[cap],c['observations']):
        cand=[{'r':(T@np.array(x['rotation_matrix'])).round(5).tolist(),'t':(T@(np.array(x['translation_m'])-origin)).round(5).tolist(),'w':round(x['width_m'],4),'d':round(x['depth_m'],4),'s':round(x['score'],3),'src':x['source']} for x in g['refined_grasps']]
        objs.append({'label':o['label'],'status':o['status'],'reason':o['reason'],'exec':g['executable'],'cands':cand})
    scenes.append({'id':cap[:2],'name':NAMES[cap[:2]],'cam':cam,'n':len(tp),'xyz':base64.b64encode(xyz.tobytes()).decode(),'rgb':base64.b64encode(col.tobytes()).decode(),'obj':base64.b64encode(oid.tobytes()).decode(),'objects':objs})
    print(cap,len(tp),[round(v,3) for v in cam])
page=(SP/'template3d.html').read_text(encoding='utf-8').replace('/*DATA*/null',json.dumps(scenes,ensure_ascii=False,separators=(',',':')))
(SP/'page3d/grasp_3d_viewer.html').write_text(page,encoding='utf-8'); print(len(page)/1e6,'MB')
