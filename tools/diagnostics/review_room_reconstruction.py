#!/usr/bin/env python3
"""Make portable, offline review artifacts from real reconstructed fragments."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '4')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/home-assistant-matplotlib')


def review(dataset, reconstruction):
    import cv2
    import numpy as np
    import open3d as o3d
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import plotly.graph_objects as go
    data = json.loads((dataset / 'dataset.json').read_text())
    report = json.loads((reconstruction / 'reconstruction.json').read_text())
    rows = [r for r in data['frames'] if 'quality' in r]
    # Time-sampled RGB contact sheet exposes what was actually observed.
    tiles = []
    for seconds in np.linspace(0, data['duration_seconds'] - .5, 15):
        row = min(rows, key=lambda r: abs(r['seconds'] - seconds))
        rgb = cv2.imread(str(dataset / row['color']))
        tile = np.full((210, 320, 3), 245, np.uint8)
        tile[:180] = cv2.resize(rgb, (320, 180))
        caption = f"{row['seconds']:.1f}s | useful depth {row['quality']['useful_depth_fraction']:.0%}"
        cv2.putText(tile, caption, (7, 201), cv2.FONT_HERSHEY_SIMPLEX, .48, (30, 30, 30), 1, cv2.LINE_AA)
        tiles.append(tile)
    contact = np.vstack([np.hstack(tiles[i:i+3]) for i in range(0, 15, 3)])
    cv2.imwrite(str(reconstruction / 'capture-contact-sheet.jpg'), contact)
    fig, axes = plt.subplots(2, 1, figsize=(12, 5.2), sharex=True, constrained_layout=True)
    times = [r['seconds'] for r in rows]
    axes[0].plot(times, [100 * r['quality']['useful_depth_fraction'] for r in rows], color='#007f91')
    axes[0].axhline(35, color='#c78926', linestyle='--', label='Capture warning threshold')
    axes[0].set(ylabel='Useful depth (%)', ylim=(0, 105), title='Recorded scene quality and local tracking')
    axes[0].legend(loc='lower center', fontsize=8)
    for s in report['segments']:
        axes[1].plot([s['start_seconds'], s['end_seconds']], [s['id'], s['id']], linewidth=6)
    axes[1].set(xlabel='Recording time (seconds)', ylabel='Independent fragment', xlim=(0, data['duration_seconds']))
    for ax in axes:
        ax.grid(alpha=.2)
    fig.savefig(reconstruction / 'quality-timeline.png', dpi=130)
    plt.close(fig)

    ranked = sorted(report['segments'], key=lambda s: s['frames'], reverse=True)
    selected = [s for s in ranked if s['frames'] >= 5 and s['points'] >= 1000][:8]
    figure = go.Figure()
    clouds = []
    for n, s in enumerate(selected):
        cloud = o3d.io.read_point_cloud(str(reconstruction / s['pointcloud']))
        points, colors = np.asarray(cloud.points), np.asarray(cloud.colors)
        step = max(1, len(points) // 65000)
        # Display y-up only; saved metric PLYs remain in their recorded camera frame.
        xyz = points[::step] * [1, -1, -1]
        colors = np.rint(colors[::step] * 255).astype(np.uint8)
        clouds.append((s, xyz, colors / 255))
        figure.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], mode='markers',
                                     marker=dict(size=1.5, color=[f'rgb({r},{g},{b})' for r, g, b in colors]), visible=(n == 0), name='관측한 표면',
                                     hoverinfo='skip'))
        trajectory = [t['camera_to_fragment'] for t in report['trajectory'] if t['fragment'] == s['id']]
        trajectory = np.array(trajectory)[:, :3, 3] * [1, -1, -1]
        figure.add_trace(go.Scatter3d(x=trajectory[:, 0], y=trajectory[:, 1], z=trajectory[:, 2], mode='lines',
                                     line=dict(width=5, color='#ffb547'), visible=(n == 0), name='추정 카메라 경로'))
    buttons = []
    for n, s in enumerate(selected):
        visible = [False] * (len(selected) * 2)
        visible[2*n:2*n+2] = [True, True]
        label = f"조각 {s['id']} · {s['start_seconds']:.1f}–{s['end_seconds']:.1f}초"
        buttons.append(dict(label=label, method='update', args=[{'visible': visible}, {'title': label}]))
    figure.update_layout(title='실제 RGB-D 복원 · 가장 긴 연결 구간', template='plotly_dark', height=750,
                         scene=dict(aspectmode='data', xaxis_title='X (m)', yaxis_title='Y 표시축 (m)', zaxis_title='Z 표시축 (m)',
                                    camera=dict(up=dict(x=0, y=1, z=0), eye=dict(x=1.2, y=.5, z=1.5))),
                         margin=dict(l=0, r=0, b=0, t=80), updatemenus=[dict(buttons=buttons, x=0, y=1.10)],
                         legend=dict(x=0, y=0))
    plot_html = figure.to_html(full_html=False, include_plotlyjs=True, config={'displayModeBar': False, 'displaylogo': False, 'responsive': True})
    summary = (f"촬영 {data['duration_seconds']:.1f}초 · 독립된 복원 조각 {len(report['segments'])}개. "
               "드래그로 회전, 스크롤로 확대하세요. 위 메뉴에서 다른 조각을 볼 수 있습니다.")
    (reconstruction / 'review.html').write_text(
        "<!doctype html><html lang='ko'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>방 스캔 복원 검토</title><style>body{background:#111827;color:#e5e7eb;font:17px sans-serif;margin:24px}"
        "p{line-height:1.7;max-width:1000px}img{max-width:100%;border-radius:8px}a{color:#7dd3fc}</style>"
        "<h1>방 스캔 · 3D 복원 검토</h1><p>" + summary + "</p>"
        "<p><b>부분 복원 결과입니다.</b> 조각끼리는 서로 다른 좌표계를 사용합니다. "
        "집 전체 지도·로봇 이동 지도로 사용할 수 없으며, 누적 오차와 가려진 영역은 남아 있습니다.</p>" + plot_html +
        "<details><summary>3D 화면이 보이지 않을 때: 이미지로 보기</summary><img src='reconstruction-preview.png'></details>"
        "<h2>깊이 품질과 연결 구간</h2><img src='quality-timeline.png'>"
        "<h2>실제 촬영 장면</h2><img src='capture-contact-sheet.jpg'>"
        "<p><a href='reconstruction.json'>복원 수치와 경로 기록</a></p></html>")
    if clouds:
        fig = plt.figure(figsize=(13, 5.8), facecolor='#f5f7fa')
        for i, (s, xyz, colors) in enumerate(clouds[:2]):
            ax = fig.add_subplot(1, min(2, len(clouds)), i+1, projection='3d')
            take = max(1, len(xyz) // 25000)
            ax.scatter(xyz[::take, 0], xyz[::take, 2], xyz[::take, 1], c=colors[::take], s=.5, depthshade=False)
            ranges = np.ptp(xyz, axis=0)
            ax.set_box_aspect(np.maximum(ranges[[0, 2, 1]], .05))
            ax.view_init(elev=15, azim=-65)
            ax.set(xlabel='X (m)', ylabel='Z display (m)', zlabel='Y display (m)',
                   title=f"Fragment {s['id']} | {s['start_seconds']:.1f}–{s['end_seconds']:.1f}s\n{s['frames']} frames | independent local coordinates")
        fig.suptitle('Actual RGB-D reconstruction — partial surfaces, not a validated room map', fontsize=13)
        fig.tight_layout()
        fig.savefig(reconstruction / 'reconstruction-preview.png', dpi=160)
        plt.close(fig)
    print(reconstruction / 'review.html')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset', type=Path)
    p.add_argument('reconstruction', type=Path)
    a = p.parse_args()
    review(a.dataset, a.reconstruction)
