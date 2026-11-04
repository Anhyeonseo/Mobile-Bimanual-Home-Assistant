import base64,html,io,json,sys
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path('/home/yeoeun/Bimanual-Household-Manipulation'); sys.path.insert(0,str(ROOT))
from home_perception.io.snapshot import load_snapshot
from home_perception.geometry.plane import backproject,fit_plane
from home_perception.geometry.mask_cloud import clean_cloud,SUPPORT_MARGIN_M
SP=Path(sys.argv[1]); RUN=ROOT/'tmp/tabletop-perception-gpu-r2'; DATA=ROOT/'data/tabletop_rgbd_20261101'
FONT=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',17)
GREEN=(34,197,94); AMBER=(245,158,11); RED=(239,68,68); BLUE=(59,130,246)
rep=json.load(open(RUN/'report.json')); caps={c['capture']:c for c in rep['captures']}
def rgb(cap,hide=True):
    a=cv2.cvtColor(load_snapshot(DATA/cap).bgr,cv2.COLOR_BGR2RGB)
    if cap.startswith('01') and hide:  # person in the background
        a[:62,465:]=cv2.GaussianBlur(a[:62,465:],(0,0),18)
    return a
def b64(arr):
    b=io.BytesIO(); Image.fromarray(arr).save(b,'JPEG',quality=82); return 'data:image/jpeg;base64,'+base64.b64encode(b.getvalue()).decode()
def label(d,x,y,text,color):
    x=max(2,min(x,560)); y=max(2,y)
    box=d.textbbox((x,y),text,font=FONT); d.rectangle((box[0]-4,box[1]-3,box[2]+4,box[3]+3),fill=color); d.text((x,y),text,font=FONT,fill=(15,23,42))
def boxes(a,items):
    im=Image.fromarray(a.copy()); d=ImageDraw.Draw(im)
    for t,b,c in items:
        d.rectangle(b,outline=c,width=3); label(d,int(b[0]),int(b[1])-24,t,c)
    return np.asarray(im)
def overlay(cap):
    a=rgb(cap).copy(); obs=caps[cap]['observations']
    for i,o in enumerate(obs):
        m=cv2.imread(str(RUN/cap/f'mask_{i:03}.png'),0)>0; c=np.array(GREEN if o['status']=='accepted' else AMBER)
        a[m]=(.45*a[m]+.55*c).astype(np.uint8)
    im=Image.fromarray(a); d=ImageDraw.Draw(im)
    for o in obs:
        c=GREEN if o['status']=='accepted' else AMBER
        label(d,int(o['box_xyxy'][0]),int(o['box_xyxy'][1])-24,o['label'],c)
    return np.asarray(im)
def depthmap(cap):
    s=load_snapshot(DATA/cap); z=s.depth_raw.astype(float)
    n=np.clip((z-280)/(560-280),0,1); c=cv2.applyColorMap(np.uint8(255*(1-n)),cv2.COLORMAP_TURBO)[:,:,::-1].copy(); c[z==0]=(20,20,24); return c
def holes(cap):
    s=load_snapshot(DATA/cap); a=rgb(cap).copy(); obs=caps[cap]['observations']
    for i,o in enumerate(obs):
        m=cv2.imread(str(RUN/cap/f'mask_{i:03}.png'),0)>0; bad=m&(s.depth_raw==0); good=m&(s.depth_raw>0)
        a[good]=(.5*a[good]+.5*np.array(GREEN)).astype(np.uint8); a[bad]=RED
    return a
def pieces(cap,i):
    s=load_snapshot(DATA/cap); d=s.depth_m(); p=fit_plane(backproject(d,s.intrinsics,d>0)); m=cv2.imread(str(RUN/cap/f'mask_{i:03}.png'),0)>0
    pts=backproject(d,s.intrinsics,m&(d>0)); pts=pts[p.height(pts)>SUPPORT_MARGIN_M]; kept,_=clean_cloud(pts); I=s.intrinsics
    u=np.int32(kept[:,0]*I['fx']/kept[:,2]+I['ppx']); v=np.int32(kept[:,1]*I['fy']/kept[:,2]+I['ppy'])
    k=np.zeros(m.shape,np.uint8); k[v,u]=1; k=cv2.dilate(k,np.ones((7,7),np.uint8)).astype(bool)&m
    a=rgb(cap).copy(); a[m]=(.4*a[m]+.6*np.array(RED)).astype(np.uint8); a[k]=(.35*a[k]+.65*np.array(GREEN)).astype(np.uint8)
    x1,y1,x2,y2=[int(v) for v in caps[cap]['observations'][i]['box_xyxy']]; pad=30
    crop=a[max(0,y1-pad):y2+pad,max(0,x1-pad):x2+pad]; return cv2.resize(crop,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC)
def perturbed(cap,kind):
    a=cv2.cvtColor(cv2.imread(str(RUN/cap/kind/'color.png')),cv2.COLOR_BGR2RGB); det=json.load(open(RUN/cap/kind/'qwen/detections.json'))['detections']
    return boxes(a,[(d['label'],d['bbox_xyxy'],GREEN) for d in det])
ex='10_misc_objects'; E=caps[ex]['observations']
steps=[('사진을 찍는다','로봇 카메라가 일반 사진과 거리 정보를 함께 찍습니다.',rgb(ex)),
 ('거리를 잰다','색이 곧 거리입니다. 빨강에 가까울수록 카메라와 가깝고, 파랑에 가까울수록 멉니다. 검은 곳은 거리를 재지 못한 자리입니다.',depthmap(ex)),
 ('물체를 찾는다','AI에게 "컵, 리모컨…을 찾아줘"라고 한국어로 시키면 네모 칸으로 위치를 표시합니다.',boxes(rgb(ex),[(o['label'],o['box_xyxy'],BLUE) for o in E])),
 ('모양을 따고 판정한다','물체 모양대로 색을 칠하고 거리 정보를 합쳐 실제 크기와 위치를 계산합니다. 정보가 충분하면 초록, 부족하면 노랑입니다.',overlay(ex))]
step_html=''.join(f'<figure class="step"><img src="{b64(i)}" alt="{html.escape(t)}"><figcaption><strong>{n}. {t}</strong><span>{d}</span></figcaption></figure>' for n,(t,d,i) in enumerate(steps,1))
NAMES={'01':'섞인 물체 1','02':'섞인 물체 2','03':'비슷한 물체 1','04':'비슷한 물체 2','05':'리모컨만 1','06':'리모컨만 2','07':'검은 컵 2개','08':'컵이 컵을 가림','09':'여러 물체 1','10':'여러 물체 2'}
why={'partial_mask_depth':'거리 정보 부족','fragmented_mask_cloud':'물체가 조각남'}
cards=''
for c,v in caps.items():
    chips=''
    for o in v['observations']:
        cm=max(o['obb']['extent_m'])*100
        if o['status']=='accepted': chips+=f'<li class="ok"><b>{o["label"]}</b><span>통과 · 길이 약 {cm:.0f}cm</span></li>'
        else: chips+=f'<li class="hold"><b>{o["label"]}</b><span>보류 · {why[o["reason"]]}</span></li>'
    cards+=f'<article class="card"><img src="{b64(overlay(c))}" alt="{NAMES[c[:2]]} 결과" loading="lazy"><h3>{c[:2]} {NAMES[c[:2]]}</h3><ul>{chips}</ul></article>'
ab={a['capture']:a for a in rep['absent_target']}
imgs=dict(cup_photo=b64(rgb('08_same_objects')),cup_hole=b64(holes('08_same_objects')),m_bad=b64(pieces('03_similar_objects',1)),m_good=b64(pieces('04_similar_objects',1)),
 false7=b64(boxes(rgb('07_same_objects'),[('"리모컨"이라고 답함',b,RED) for b in ab['07_same_objects']['boxes']])),false8=b64(boxes(rgb('08_same_objects'),[('"리모컨"이라고 답함',b,RED) for b in ab['08_same_objects']['boxes']])))
rob=''.join(f'<figure><img src="{b64(perturbed("09_misc_objects",k))}" alt="{t}" loading="lazy"><figcaption>{t}</figcaption></figure>' for k,t in (('gamma_dark','어둡게'),('gamma_bright','밝게'),('blur','흐리게'),('noise','지글거리게')))
page=(SP/'template.html').read_text(encoding='utf-8')
for k,v in dict(STEPS=step_html,CARDS=cards,ROB=rob,**{k.upper():v for k,v in imgs.items()}).items(): page=page.replace('{{'+k+'}}',v)
(SP/'page/tabletop_results.html').write_text(page,encoding='utf-8'); print(len(page)/1e6,'MB')
Image.fromarray(np.vstack([np.hstack([cv2.resize(pieces('03_similar_objects',1),(320,480)),cv2.resize(pieces('04_similar_objects',1),(320,480)),holes('08_same_objects')])])).save(SP/'check.jpg')
