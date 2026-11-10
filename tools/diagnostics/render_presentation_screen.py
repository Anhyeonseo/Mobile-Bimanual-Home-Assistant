#!/usr/bin/env python3
"""Encode a recording-friendly perception display from verified RGB-D results.

The animation is a presentation of recorded analysis, not a live navigation or
robot execution trace. No detections, grasp values or execution success invented.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import evidence_summary,interpret,write_json

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
BG='#0b111b';CARD='#121d2b';LINE='#243449';WHITE='#eaf2fa';MUTED='#93a8be';GREEN='#5ae3bd'
W,H=960,1080


class Display:
    def __init__(self,args):
        self.args=args
        self.detections=evidence_summary(args.detections,args.snapshot)
        self.grasps=evidence_summary(args.grasps,args.snapshot)
        self.prepared=json.loads(args.grasps.read_text())
        self.raw_det=json.loads(args.detections.read_text())
        index=self.prepared['group']['observation_index']
        observation=self.prepared['observations'][index]
        selected=[d for d in self.raw_det['detections'] if d.get('segmentation',{}).get('sha256')==observation['mask_sha256']]
        if len(selected)!=1:raise ValueError('파지 대상과 검출 마스크가 일치해야 합니다.')
        self.target=selected[0]
        self.mask=np.array(Image.open(args.detections.parent/self.target['segmentation']['mask_file']))>0
        self.rgb=np.array(Image.open(args.snapshot/'color.png').convert('RGB'))
        metadata=json.loads((args.snapshot/'metadata.json').read_text())
        self.intr=metadata['aligned_depth_intrinsics']
        self.depth=np.array(Image.open(args.snapshot/'depth_aligned.png'),dtype=np.uint16)*metadata['depth_scale_m']
        if self.rgb.shape[:2]!=self.depth.shape or self.mask.shape!=self.depth.shape:raise ValueError('촬영 크기 불일치')
        self.candidates=self.prepared['group']['refined_grasps']
        if not 0<=args.candidate<len(self.candidates):raise ValueError('없는 파지 후보 번호')
        self.candidate=self.candidates[args.candidate]
        self.meaning=interpret(args.request,'presentation-screen')['intent']
        if self.meaning['issues']:raise ValueError('요청 해석 필요: '+str(self.meaning['issues']))
        self.fonts={}
        valid=self.depth>0
        lo,hi=np.percentile(self.depth[valid],(5,95))
        heat=cv2.applyColorMap(np.uint8(np.clip((self.depth-lo)/max(hi-lo,.01),0,1)*255),cv2.COLORMAP_TURBO)
        heat[~valid]=0
        self.depth_image=Image.fromarray(cv2.cvtColor(heat,cv2.COLOR_BGR2RGB))
        y,x=np.nonzero(valid & self.mask)
        ids=np.linspace(0,len(x)-1,min(8000,len(x))).astype(int)
        x,y=x[ids],y[ids];z=self.depth[y,x];i=self.intr
        self.points=np.c_[(x-i['ppx'])*z/i['fx'],(y-i['ppy'])*z/i['fy'],z]
        self.colors=self.rgb[y,x]
        self.center=self.points.mean(axis=0)
        self.extent=max(np.ptp(self.points,axis=0).max(),.01)
        self.cloud_cache={}

    def font(self,size,bold=False):
        key=(size,bold)
        if key not in self.fonts:self.fonts[key]=ImageFont.truetype(BOLD if bold else FONT,size)
        return self.fonts[key]

    def text(self,draw,xy,text,size=22,color=WHITE,bold=False,width=None):
        while width and draw.textlength(text,font=self.font(size,bold))>width and size>12:size-=1
        draw.text(xy,text,font=self.font(size,bold),fill=color)

    def card(self,draw,box):
        draw.rounded_rectangle(box,radius=14,fill=CARD,outline=LINE,width=1)

    def rgb_panel(self,t):
        a=self.rgb.copy()
        if t>=8:
            a[self.mask]=(a[self.mask]*.60+np.array([50,215,176])*.40).astype(np.uint8)
            contours,_=cv2.findContours(self.mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(a,contours,-1,(90,255,203),2)
        image=Image.fromarray(a);draw=ImageDraw.Draw(image)
        if t>=18:
            i=self.intr
            def pixel(p):return (p[0]/p[2]*i['fx']+i['ppx'],p[1]/p[2]*i['fy']+i['ppy'])
            # GraspNet convention: column 0 approach, column 1 closing.
            for n,c in enumerate(self.candidates):
                if n%4 and n!=self.args.candidate:continue
                r=np.asarray(c['rotation_matrix']);center=np.asarray(c['translation_m'])+r[:,0]*c['depth_m']
                left=center-r[:,1]*c['width_m']/2;right=center+r[:,1]*c['width_m']/2
                path=[left-r[:,0]*.018,left,right,right-r[:,0]*.018]
                if min(p[2] for p in path)<=0:continue
                color='#ffd778' if n==self.args.candidate else '#668fa8'
                draw.line([pixel(p) for p in path],fill=color,width=4 if n==self.args.candidate else 1)
        return image

    def cloud(self,t):
        angle=int(np.clip((t-14)*3,-10,35))
        if angle in self.cloud_cache:return self.cloud_cache[angle]
        q=self.points-self.center;a=np.deg2rad(angle)
        rotated=q@np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]]).T
        xy=rotated[:,:2]/self.extent*235+[208,148]
        order=np.argsort(rotated[:,2])[::-1];x,y=np.rint(xy[order]).astype(int).T
        colors=self.colors[order];canvas=np.full((300,416,3),[12,22,34],np.uint8)
        for dx,dy in ((0,0),(1,0),(0,1),(1,1)):
            inside=(x+dx>=0)&(x+dx<416)&(y+dy>=0)&(y+dy<300)
            canvas[y[inside]+dy,x[inside]+dx]=colors[inside]
        image=Image.fromarray(canvas);d=ImageDraw.Draw(image)
        self.text(d,(12,267),'관측된 표면 점만 표시',15,MUTED)
        self.cloud_cache[angle]=image
        return image

    def frame(self,t):
        image=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(image)
        self.text(d,(32,24),'HOME ASSISTANT',33,bold=True)
        self.text(d,(34,68),'ALOHA Mini 1  /  COMMAND · PERCEPTION · GRASP',16,MUTED)
        self.text(d,(746,38),'화면 미리보기' if self.args.preview else '녹화 분석 재생',18,GREEN)
        self.card(d,(32,108,928,220))
        self.text(d,(52,121),'USER REQUEST',14,GREEN,bold=True)
        typed=self.args.request[:max(0,int((t-.5)*12))]
        self.text(d,(52,146),typed+(' ▏' if t<4 else ''),26,width=852)
        if t>=4:
            m=self.meaning
            self.text(d,(52,187),f"대상  리모컨     출발  {m['source_phrase']}     도착  {m['destination_phrase']}",18,MUTED)
        self.text(d,(34,237),'TASK PLAN',14,MUTED)
        for n,(label,mode) in enumerate([('명령 해석','로컬 분석'),('책상으로 이동','텔레옵'),('집기','관측 기반'),('침대로 이동','텔레옵'),('놓기','촬영 단계')]):
            x=32+n*181
            self.card(d,(x,264,x+172,326))
            self.text(d,(x+12,272),label,18, GREEN if n==0 and t>=4 else WHITE,width=148)
            self.text(d,(x+12,299),mode,13,MUTED)
        self.card(d,(32,344,474,712));self.card(d,(486,344,928,712))
        self.text(d,(46,356),'RGB  /  '+('SAM MASK + GRASP' if t>=18 else 'SAM MASK' if t>=8 else 'CAMERA'),17,GREEN)
        self.text(d,(500,356),'DEPTH  /  '+('3D POINT CLOUD' if t>=14 else 'ALIGNED DEPTH'),17,GREEN)
        image.paste(self.rgb_panel(t).resize((416,300)),(45,394))
        image.paste(self.cloud(t) if t>=14 else self.depth_image.resize((416,300)),(499,394))
        xyz=self.target.get('surface_point',{}).get('xyz_m')
        metrics=[('표면 XYZ · 카메라 기준',' / '.join(f'{v:+.3f}' for v in xyz) if xyz and t>=11 else '분석 대기','m'),
                 ('대상 인식',f"{self.target['score']:.2f}" if t>=8 else '—','검출 신뢰도'),
                 ('파지 후보',str(len(self.candidates)) if t>=18 else '—','개 · 후보 평가 결과')]
        for n,(label,value,unit) in enumerate(metrics):
            x=32+n*302;self.card(d,(x,730,x+292,834))
            self.text(d,(x+15,740),label,15,MUTED)
            self.text(d,(x+15,764),value,25,GREEN,width=260)
            self.text(d,(x+15,802),unit,13,MUTED)
        self.card(d,(32,852,928,1018));self.text(d,(49,862),'PROCESS LOG',14,GREEN)
        log=[(4,'[request]  책상 → 리모컨 → 침대 · 작업 요청 해석'),
             (6,'[rgbd]     컬러·깊이와 카메라 보정 정보 연결'),
             (8,'[vision]   Grounding DINO 검출 → SAM 마스킹'),
             (11,'[geometry] 깊이 역투영 → 카메라 기준 3D 표면점'),
             (14,f'[cloud]    실제 관측점 {len(self.points):,}개 표시'),
             (18,f'[grasp]    GraspNet 기반 후보 {len(self.candidates)}개'),
             (22,f"[grasp]    후보 #{self.args.candidate} · 관측 폭 {self.candidate['width_m']*1000:.1f} mm"),
             (26,'[task]     이동·집기·놓기 진행은 촬영자 확인과 연결')]
        for n,(_,line) in enumerate([row for row in log if row[0]<=t][-5:]):
            self.text(d,(50,890+n*22),line,17,WHITE,width=858)
        self.text(d,(34,1037),'화면 구성 샘플 · 기존 실제 RGB-D 사용' if self.args.preview else 'RGB-D 기록 재분석 · 이동은 텔레옵 · 파지 후보와 실물 성공은 별도',15,MUTED)
        self.text(d,(824,1037),f'{int(t)//60:02}:{int(t)%60:02}',16,MUTED)
        return image


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot','detections','grasps','output','ffmpeg'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--request',default='책상에 있는 리모컨 침대로 가져다줘')
    p.add_argument('--candidate',type=int,default=0)
    p.add_argument('--duration',type=int,default=32)
    p.add_argument('--fps',type=int,default=24)
    p.add_argument('--preview',action='store_true')
    args=p.parse_args()
    if not 27<=args.duration<=180 or args.fps not in (12,24,30):p.error('duration 27..180; fps 12/24/30')
    if args.output.exists():p.error('기존 영상을 덮어쓰지 않습니다.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    display=Display(args)
    tmp=args.output.with_name(args.output.stem+'.partial.mp4')
    command=[str(args.ffmpeg),'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24',
        '-video_size',f'{W}x{H}','-framerate',str(args.fps),'-i','pipe:0','-an','-c:v','libx264',
        '-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp)]
    try:
        with (args.output.with_suffix('.encode.log')).open('w') as log:
            process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
            try:
                for n in range(args.duration*args.fps):
                    frame=display.frame(n/args.fps)
                    if n in (2*args.fps,12*args.fps,24*args.fps):frame.save(args.output.with_name(f'{args.output.stem}-{n//args.fps}s.png'))
                    process.stdin.write(frame.tobytes())
                process.stdin.close()
                if process.wait(timeout=60):raise RuntimeError('영상 인코딩 실패')
            finally:
                if process.poll() is None:process.kill();process.wait()
        os.replace(tmp,args.output)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    write_json(args.output.with_suffix('.json'),dict(kind='recorded_analysis_presentation',preview=args.preview,
        timing='editorial_reveal_not_inference_duration',robot_commands=0,physical_task_completed=False,
        source_sha256=display.prepared['source_sha256'],
        inputs={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.detections,args.grasps)},
        request=display.meaning,selected_candidate=args.candidate,size=[W,H],fps=args.fps,duration_s=args.duration))
    print(args.output)


if __name__=='__main__':main()
