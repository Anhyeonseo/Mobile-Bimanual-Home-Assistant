#!/usr/bin/env python3
"""Live filming window. Local operator markers only; no robot/device connection."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2_ws/src/home_robot_tasks')]
from tools.lib.presentation_session import FilmSession, LABELS, STAGES, write_json


def show(folder):
    import tkinter as tk
    session = FilmSession(folder)
    inbox = folder / 'inbox'
    inbox.mkdir()
    root = tk.Tk()
    root.title('HOME ASSISTANT — Live task monitor')
    root.geometry('1040x820+20+20')
    root.resizable(False, False)
    bg, panel, text, muted, green = '#0c1423', '#131f32', '#edf4ff', '#92a6c4', '#55d9b4'
    root.configure(bg=bg)

    def label(parent, content, size=14, color=text, **kwargs):
        return tk.Label(parent, text=content, font=('Noto Sans CJK KR', size),
                        bg=kwargs.pop('bg', bg), fg=color, anchor='w', **kwargs)

    label(root, 'HOME ASSISTANT', 26).pack(anchor='w', padx=36, pady=(26, 0))
    label(root, 'ALOHA Mini1  /  작업 진행 모니터', 12, muted).pack(anchor='w', padx=38)
    box = tk.Frame(root, bg=panel, padx=18, pady=14)
    box.pack(fill='x', padx=36, pady=(24, 10))
    label(box, '자연어 요청', 12, muted, bg=panel).pack(anchor='w')
    entry = tk.Entry(box, font=('Noto Sans CJK KR', 17), bg=panel, fg=text,
                     insertbackground=green, relief='flat', highlightthickness=0)
    entry.pack(fill='x', pady=8)
    entry.focus_set()
    hint = label(root, '요청을 입력하고 Enter를 누르세요.', 13, muted)
    hint.pack(anchor='w', padx=38, pady=(0, 12))
    summary = label(root, '대상  —      출발 장소  —      목적 장소  —', 15, green)
    summary.pack(anchor='w', padx=38, pady=(0, 16))
    stage_labels = []
    for index, stage in enumerate(STAGES, 1):
        item = label(root, f'{index:02d}    {LABELS[stage]}', 16)
        item.pack(anchor='w', padx=42, pady=5)
        stage_labels.append(item)
    label(root, 'PROCESS LOG', 12, muted).pack(anchor='w', padx=38, pady=(20, 6))
    logs = tk.Text(root, height=4, font=('Noto Sans Mono CJK KR', 12), bg=panel,
                   fg=text, relief='flat', padx=14, pady=10, wrap='word', state='disabled')
    logs.pack(fill='both', expand=True, padx=36)
    label(root, '요청 해석: 로컬 파서  ·  이동·동작 단계: 운영자 확인', 10, muted).pack(
        anchor='w', padx=38, pady=(10, 16))

    def refresh():
        state = session.state
        if state['request'] and not state['request']['intent']['issues']:
            intent = state['request']['intent']
            obj = '리모컨' if intent['category'] == 'remote_control' else intent['category']
            summary.configure(text=f"대상  {obj}      출발 장소  {intent['source_phrase']}      목적 장소  {intent['destination_phrase']}")
        words = dict(waiting='대기', active='진행 중', done='완료', failed='중단',
                     interrupted='중단', not_filmed='별도 촬영')
        for index, stage in enumerate(STAGES):
            status = state['stages'][stage]
            stage_labels[index].configure(text=f'{index+1:02d}    {LABELS[stage]:12}   {words[status]}',
                fg=green if status in ('done', 'active') else muted)
        lines = []
        for event in state['events'][-8:]:
            stamp = datetime.fromisoformat(event['at_utc']).astimezone(ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')
            lines.append(f"{stamp}  {event['message']}")
        logs.configure(state='normal')
        logs.delete('1.0', 'end')
        logs.insert('end', '\n'.join(lines))
        logs.configure(state='disabled')

    def dispatch(action, value=''):
        if action == 'request':
            result = session.request(value)
            if result['type'] == 'request_parsed':
                entry.delete(0, 'end')
                entry.insert(0, value)
                entry.configure(state='readonly', readonlybackground=panel)
                hint.configure(text='요청 해석 완료 · 책상 이동 시작 대기', fg=green)
            else:
                hint.configure(text=result['message'], fg='#ffbf6b')
        elif action == 'start':
            session.start()
            hint.configure(text=LABELS[session.state['active']] + ' 진행 중', fg=green)
        elif action == 'done':
            stage = session.state['active']
            session.finish()
            hint.configure(text=LABELS[stage] + ' 완료 확인', fg=green)
        elif action == 'mark':
            session.event('marker', value, 'operator')
        else:
            raise ValueError('지원하지 않는 동작')
        refresh()

    def submit(event=None):
        try:
            dispatch('request', entry.get())
        except ValueError as exc:
            hint.configure(text=str(exc), fg='#ffbf6b')

    def poll():
        for path in sorted(inbox.glob('*.json')):
            command = json.loads(path.read_text())
            try:
                dispatch(command['action'], command.get('value', ''))
                response = dict(ok=True)
            except (ValueError, KeyError) as exc:
                response = dict(ok=False, error=str(exc))
            write_json(folder / ('ack-' + path.name), response)
            path.unlink()
        root.after(150, poll)

    def close():
        session.close()
        root.destroy()

    entry.bind('<Return>', submit)
    root.protocol('WM_DELETE_WINDOW', close)
    refresh()
    root.update()
    write_json(folder / 'window.json', dict(window_id=root.winfo_id(), display=os.environ.get('DISPLAY'),
        width=root.winfo_width(), height=root.winfo_height(), pid=os.getpid()))
    print('Live window ready:', folder, flush=True)
    poll()
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['show', 'record', 'request', 'start', 'done', 'mark'])
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--value', default='')
    args = parser.parse_args()
    if args.action == 'show':
        show(args.session)
    elif args.action == 'record':
        window = json.loads((args.session / 'window.json').read_text())
        os.kill(window['pid'], 0)
        encoder = ROOT / 'output/presentation-runtime/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
        target = args.session / 'screen.mp4'
        partial = args.session / 'screen.partial.mp4'
        if target.exists() or partial.exists():
            parser.error('녹화 파일이 이미 있습니다. 새 테이크를 사용하세요.')
        write_json(args.session / 'recording.json', dict(status='recording',
            started_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            source='actual_x11_application_window', window=window))
        result = subprocess.run([str(encoder), '-hide_banner', '-loglevel', 'warning',
            '-f', 'x11grab', '-window_id', str(window['window_id']),
            '-video_size', f"{window['width']}x{window['height']}", '-framerate', '24',
            '-draw_mouse', '0', '-i', window['display'], '-c:v', 'libx264',
            '-preset', 'ultrafast', '-crf', '20', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(partial)])
        metadata = json.loads((args.session / 'recording.json').read_text())
        metadata.update(status='saved' if result.returncode == 0 else 'failed',
            ended_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(), returncode=result.returncode)
        if result.returncode == 0:
            partial.replace(target)
        write_json(args.session / 'recording.json', metadata)
        print(metadata['status'], target)
    else:
        inbox = args.session / 'inbox'
        if not inbox.is_dir():
            parser.error('먼저 show로 촬영 창을 여세요.')
        command_id = uuid.uuid4().hex
        write_json(inbox / (command_id + '.json'), dict(action=args.action, value=args.value))
        print(command_id)


if __name__ == '__main__':
    main()
