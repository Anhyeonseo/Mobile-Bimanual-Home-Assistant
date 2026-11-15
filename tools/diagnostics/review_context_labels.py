#!/usr/bin/env python3
"""Build, search, and rename an offline household vocabulary from image review.

No model endpoint, robot connection, or motor command. Images remain linked to
the immutable recording; the output is a separate, portable review directory.
"""
import argparse
import hashlib
import html
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2_ws/src/home_robot_tasks'),
               str(ROOT/'ros2_ws/src/so101_arm_bridge')]
from home_robot_tasks.assistant.context_labels import (
    build_context_labels, rename_context_label, resolve_context_label)
from home_robot_tasks.assistant.world import WorldStore


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def persist(output, bundle):
    # WorldStore is the existing persistent memory implementation. Keep this
    # historical label layer separate from live, actionable assistant_entities.
    store = WorldStore(output/'household-memory.sqlite')
    try:
        store.save_record('household_context_labels', bundle['review_id'], bundle)
        if store.record('household_context_labels', bundle['review_id']) != bundle:
            raise ValueError('persistent labels readback mismatch')
    finally:
        store.close()
    write(output/'household-labels.json', bundle)


def render(output, bundle, occupancy):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.colors import ListedColormap
    font = next((f.name for f in font_manager.fontManager.ttflist
                 if f.name.startswith('Noto Sans CJK')), None)
    if font:
        plt.rcParams['font.family'] = font
    plt.rcParams['axes.unicode_minus'] = False
    geometry = read(output/'review-geometry.json')
    grid = np.array(occupancy['data']).reshape(occupancy['height'], occupancy['width'])
    x, y = occupancy['origin']; res = occupancy['resolution']
    shown = [i for i in bundle['entities'] if i['show_on_map']]
    colors = plt.get_cmap('tab20')
    fig, (ax, legend) = plt.subplots(1, 2, figsize=(12, 10), gridspec_kw={'width_ratios': [1.8, 1]})
    ax.imshow(np.where(grid < 0, 0, np.where(grid >= 65, 2, 1)), origin='lower',
              extent=[x, x+grid.shape[1]*res, y, y+grid.shape[0]*res],
              cmap=ListedColormap(['#e2e6ea', '#ffffff', '#34404d']), vmin=0, vmax=2,
              interpolation='nearest')
    marker_positions = []
    for n, item in enumerate(shown, 1):
        points = np.array(geometry[item['id']])
        color = colors((n-1) % 20)
        ax.scatter(points[:, 0], points[:, 1], color=color, s=4, alpha=.4, linewidths=0)
        center = np.array(item['observed_anchor_xyz_m'][:2])
        marker = center.copy()
        while any(np.linalg.norm(marker-p) < .38 for p in marker_positions):
            marker += [.45, 0]
        marker_positions.append(marker)
        if not np.array_equal(marker, center):
            ax.plot([center[0], marker[0]], [center[1], marker[1]], color=color, linewidth=1)
        ax.text(*marker, str(n), ha='center', va='center', fontsize=10,
                bbox=dict(boxstyle='circle,pad=.3', facecolor='white', edgecolor=color, linewidth=2))
        legend.text(.02, .96-(n-1)*.077, f'{n:02}  {item["label"]}', fontsize=13,
                    va='top', color='#1c2936')
        legend.text(.11, .926-(n-1)*.077, item['function_label'], fontsize=9,
                    va='top', color='#5d6977')
    legend.axis('off')
    ax.set(xlabel='X (m)', ylabel='Y (m)', aspect='equal')
    fig.suptitle('우리 집 이름 지도 · 사진 맥락 검토', fontsize=20)
    ax.set_title('색 점: 기존 RGB-D 관측   숫자: 이름 연결 위치', fontsize=10)
    legend.set_title('가구 이름 제안', fontsize=14)
    fig.text(.5, .035, '방의 경계·안방 여부는 미확정 · 이름 위치는 이동 목표가 아님\n'
             '라이다와 장애물 형상은 유지 · 사진 속 관측 부분만 표시', ha='center', fontsize=10)
    fig.tight_layout(rect=[0, .09, 1, .95])
    fig.savefig(output/'named-map.png', dpi=150)
    plt.close(fig)
    cards = []
    for n, item in enumerate(bundle['entities'], 1):
        e = html.escape
        pictures = ''.join(f'<a href="evidence/view-{v:02}.jpg"><img loading="lazy" '
                           f'src="evidence/view-{v:02}.jpg" alt="관측 사진 {v}"></a>'
                           for v in item['display_views'])
        cards.append(f'<article><h2>{e(item["label"])}</h2><div class="photos">{pictures}</div>'
                     f'<p>{e(item["reason"])}</p><p class="meta">용도: {e(item["function_label"])}'
                     f' · 별칭: {e(", ".join(item["aliases"]))}</p>'
                     f'<p class="meta">{"사용자 이름 확인됨" if item["user_confirmed"] else "사진 기반 이름 제안"}'
                     f' · {len(item["member_instance_ids"])}개 관측 조각 연결</p></article>')
    zones = ''.join(f'<li><b>{html.escape(z["label"])}</b>: {html.escape(z["reason"])}</li>'
                    for z in bundle['zones'])
    questions = ''.join(f'<li>{html.escape(q)}</li>' for q in bundle['questions'])
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>우리 집 이름 지도</title>
<style>body{font-family:system-ui,sans-serif;background:#f3f5f7;color:#172432;margin:0}
main{max-width:1120px;margin:auto;padding:32px}h1{font-size:32px}p,li{line-height:1.7}
.map{width:100%;background:white;border-radius:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:18px}
article,section{background:white;border-radius:14px;padding:22px;margin-bottom:18px}
article h2{margin-top:0}.photos{display:flex;gap:8px}.photos a{width:50%}.photos img{width:100%;border-radius:8px}
.meta{font-size:13px;color:#526274}input{padding:12px;border:1px solid #aab6c1;border-radius:8px;width:90%;margin-bottom:20px}
</style><main><h1>우리 집 이름 지도</h1>
<p>60개 시점의 사진과 기존 지도 위치를 비교해 가구 이름과 용도를 정리했습니다.
이름은 제안이며, 사진을 누르면 원본 RGB 화면을 볼 수 있습니다.</p>
<img class="map" src="named-map.png" alt="가구 이름 지도">
<section><h2>공간의 용도</h2><ul>''' + zones + '''</ul>
<p class="meta">실제 방을 나눈 경계가 아니라 사진에서 확인한 용도별 구역입니다.</p></section>
<section><h2>우리 집에서 부르는 이름</h2><ul>''' + questions + '''</ul>
<p>바꾸고 싶은 이름을 대화에서 알려 주세요. 확인된 이름은 별칭과 함께 저장합니다.</p></section>
<input id="search" aria-label="가구 이름 찾기" placeholder="화장대, 냉장고, 책상 등으로 찾아보기">
<div class="grid">''' + ''.join(cards) + r'''</div>
<p class="meta">원본 영상·기존 지도 유지 / 별도 모델 API 호출 없음 / 이번 대화에서 사진 검토 / 젯슨 미배포</p>
<script>document.querySelector('#search').addEventListener('input',e=>{const q=e.target.value.replace(/\s/g,'');
document.querySelectorAll('article').forEach(a=>a.hidden=!a.textContent.replace(/\s/g,'').includes(q));});</script></main></html>'''
    (output/'index.html').write_text(page)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    sub = parser.add_subparsers(dest='command', required=True)
    build = sub.add_parser('build')
    build.add_argument('--semantic', required=True, type=Path)
    build.add_argument('--review', required=True, type=Path)
    build.add_argument('--map', required=True, type=Path)
    query = sub.add_parser('query'); query.add_argument('phrase')
    rename = sub.add_parser('rename'); rename.add_argument('id'); rename.add_argument('label')
    rename.add_argument('--alias', action='append', default=[])
    rename.add_argument('--expected-revision', required=True, type=int)
    args = parser.parse_args(); output = args.output
    if args.command == 'build':
        if (output/'household-labels.json').exists():
            raise ValueError('use a new output directory; existing names are not overwritten')
        report, review = read(args.semantic), read(args.review)
        if sha(args.map) != report['map_sha256']:
            raise ValueError('source map hash mismatch')
        bundle = build_context_labels(report, review, report_sha256=sha(args.semantic))
        from PIL import Image
        (output/'evidence').mkdir(parents=True, exist_ok=True)
        evidence_hashes = {}
        for i, frame in enumerate(report['frames']):
            source = Path(frame['capture']['source_path'])
            if sha(source) != frame['capture']['sha256']:
                raise ValueError('source RGB-D hash mismatch')
            with np.load(source, allow_pickle=False) as raw:
                Image.fromarray(raw['color_bgr'][..., ::-1]).save(output/f'evidence/view-{i:02}.jpg', quality=95)
            evidence_hashes[f'view-{i:02}.jpg'] = sha(output/f'evidence/view-{i:02}.jpg')
        instances = {i['instance_id']: i for i in report['instances']}
        geometry = {}
        for item in bundle['entities']:
            points = np.concatenate([(np.array(instances[k]['observed_voxels'])+.5)*instances[k]['voxel_size_m']
                                     for k in item['member_instance_ids']])
            geometry[item['id']] = np.unique(points, axis=0).tolist()
        write(output/'review-geometry.json', geometry)
        write(output/'source-map.json', read(args.map))
        write(output/'evidence-hashes.json', evidence_hashes)
        write(output/'source-review.json', review)
        persist(output, bundle)
    else:
        bundle = read(output/'household-labels.json')
        if args.command == 'query':
            print(json.dumps(resolve_context_label(bundle, args.phrase), ensure_ascii=False, indent=2))
            return
        bundle = rename_context_label(bundle, args.id, args.label, args.alias,
                                       expected_revision=args.expected_revision)
        persist(output, bundle)
    render(output, bundle, read(output/'source-map.json'))
    print(json.dumps(dict(status='LOCAL_CONTEXT_NAMES_READY', entities=len(bundle['entities']),
                          map_labels=sum(e['show_on_map'] for e in bundle['entities']),
                          zones=len(bundle['zones']), revision=bundle['revision'],
                          motion_authorized=False, deployed_to_jetson=False)))


if __name__ == '__main__':
    main()
