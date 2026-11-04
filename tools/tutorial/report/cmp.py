import json,sys
from pathlib import Path
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
R=Path('/home/yeoeun/Bimanual-Household-Manipulation'); SP=Path(sys.argv[1])/'cmp'
F=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',18); T=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',24)
q=json.load(open(R/'tmp/presence-eval/report.json'))['rows']; d=json.load(open(R/'tmp/presence-eval-dino/report.json'))['rows']
GREEN=(34,197,94);RED=(239,68,68);ORANGE=(245,158,11)
K={'cup':'컵','remote control':'리모컨','calculator':'계산기','marker pen':'마커펜','utility knife':'커터칼'}
def img(c):
    p=next((R/'data/tabletop_rgbd_20261101').glob(c+'_*'));a=cv2.cvtColor(cv2.imread(str(p/'color.png')),cv2.COLOR_BGR2RGB)
    if c=='01': a[:62,465:]=cv2.GaussianBlur(a[:62,465:],(0,0),18)
    if c=='02': a[:62,300:]=cv2.GaussianBlur(a[:62,300:],(0,0),18)
    return a
def row(rows,c,o): return next(x for x in rows if x['capture'].startswith(c) and x['object']==o)
def tag(dr,x,y,t,col):
    x=max(2,min(x,520));y=max(2,y);b=dr.textbbox((x,y),t,font=F);dr.rectangle((b[0]-4,b[1]-3,b[2]+4,b[3]+3),fill=col);dr.text((x,y),t,font=F,fill=(15,23,42))
def panel(rows,c,o,title):
    im=Image.fromarray(img(c).copy());dr=ImageDraw.Draw(im);x=row(rows,c,o)
    if not x['answers']: tag(dr,12,12,'박스 없음',(148,163,184))
    for a in x['answers']:
        col=GREEN if a['yes'] else RED
        dr.rectangle(a['box'],outline=col,width=3);tag(dr,int(a['box'][0]),int(a['box'][1])-24,'재확인: '+('예' if a['yes'] else '아니오'),col)
    return head(np.asarray(im),title)
def head(a,t):
    h=Image.new('RGB',(a.shape[1],40),(255,255,255));ImageDraw.Draw(h).text((10,7),t,font=T,fill=(23,35,44));return np.vstack([np.asarray(h),a])
def pair(ims):
    g=np.full((ims[0].shape[0],12,3),255,np.uint8);o=[]
    for i in ims:o+= [i,g]
    return np.hstack(o[:-1])
def save(a,n): Image.fromarray(a).save(SP/(n+'.jpg'),quality=85)
def both(c,o,n): save(pair([panel(q,c,o,f'Qwen · {c}번 · {K[o]} 찾기'),panel(d,c,o,f'DINO · {c}번 · {K[o]} 찾기')]),n)
both('05','cup','absent_cup')       # no cup in the photo
both('06','marker pen','absent_marker')
both('02','remote control','wrong_pass')
both('01','remote control','dino_miss')
both('03','utility knife','knife_miss')
both('10','utility knife','knife_10')
# found boxes per object on the busiest capture
def all10(rows,t):
    im=Image.fromarray(img('10').copy());dr=ImageDraw.Draw(im)
    for o,n in K.items():
        for a in row(rows,'10',o)['answers']:
            dr.rectangle(a['box'],outline=(59,130,246),width=3);tag(dr,int(a['box'][0]),int(a['box'][1])-24,n,(147,197,253))
    return head(np.asarray(im),t)
save(pair([all10(q,'Qwen · 10번 · 5종 박스'),all10(d,'DINO · 10번 · 5종 박스')]),'all10')
print([ (o,[ (round(v) for v in []) ]) for o in []])
for n,r in (('q',q),('d',d)):
    print(n,[(x['capture'][:2],x['object'],[([round(v) for v in a['box']],a['answer'][:6]) for a in x['answers']]) for x in r if (x['capture'][:2],x['object']) in (('02','remote control'),('01','remote control'),('10','remote control'),('10','utility knife'),('03','utility knife'))])
