#!/usr/bin/env python3
"""Write the fixed Korean request test set."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
cases = []


def add(group, text, action=None, target=None, source=None, destination=None, location=None, excluded=(), supported=True):
    cases.append({'group': group, 'text': text, 'action': action, 'target': target, 'source': source,
                  'destination': destination, 'location': location, 'excluded': list(excluded), 'supported': supported})


for text in ('리모컨 집어줘', '리모컨 좀 집어줄래', '리모컨을 집어', '저 리모컨 잡아줘', '리모컨 들어줘'):
    add('pick', text, 'pick', '리모컨')
for text in ('컵 집어줘', '검은 컵 집어줘', '마커펜 잡아줘', '계산기 들어올려줘', '커터칼 집어'):
    add('pick', text, 'pick', text.split(' 집')[0].split(' 잡')[0].split(' 들')[0])
for text, target in (('리모컨 가져다줘', '리모컨'), ('리모컨 좀 갖다줘', '리모컨'), ('나한테 컵 가져와', '컵'), ('마커펜 가져와줘', '마커펜'),
                     ('계산기 좀 가져다 줄래', '계산기'), ('검은 컵 갖다줘', '검은 컵'), ('내 리모컨 가져와', '내 리모컨'), ('커터칼 가져와', '커터칼')):
    add('fetch', text, 'fetch', target)
for text, target, source in (('책상에 있는 리모컨 가져다줘', '리모컨', '책상'), ('소파 위 리모컨 가져와', '리모컨', '소파'), ('탁자에 있는 컵 가져와줘', '컵', '탁자'),
                             ('협탁 위에 있는 계산기 갖다줘', '계산기', '협탁'), ('거실 탁자 위의 마커펜 가져와', '마커펜', '거실 탁자')):
    add('fetch_source', text, 'fetch', target, source=source)
for text, target, source, destination in (
        ('거실 탁자에 있는 리모컨을 침실 협탁에 놓아줘', '리모컨', '거실 탁자', '침실 협탁'), ('리모컨을 협탁에 놓아줘', '리모컨', None, '협탁'),
        ('컵을 책상 위에 올려줘', '컵', None, '책상'), ('계산기를 소파로 옮겨줘', '계산기', None, '소파'), ('마커펜을 탁자에 갖다 놔', '마커펜', None, '탁자'),
        ('소파에 있는 리모컨을 책상으로 옮겨', '리모컨', '소파', '책상'), ('책상 위 컵을 협탁으로 옮겨줘', '컵', '책상', '협탁'),
        ('리모컨 침대 위에 놔줘', '리모컨', None, '침대'), ('커터칼을 서랍장 위에 둬', '커터칼', None, '서랍장'), ('탁자의 계산기를 책상에 놓아', '계산기', '탁자', '책상')):
    add('move', text, 'move', target, source=source, destination=destination)
for text, target in (('리모컨 어디 있어', '리모컨'), ('리모컨 찾아줘', '리모컨'), ('컵 찾아봐', '컵'), ('계산기 어디 뒀지', '계산기'), ('마커펜 좀 찾아줄래', '마커펜')):
    add('find', text, 'find', target)
for text, location in (('왼쪽 컵 집어줘', 'left'), ('오른쪽 컵 집어줘', 'right'), ('앞에 있는 컵 집어줘', 'front'), ('뒤에 있는 컵 집어줘', 'back'), ('왼쪽에 있는 컵 잡아', 'left'),
                       ('오른쪽 컵 가져다줘', 'right'), ('앞쪽 컵 가져와', 'front'), ('뒤쪽 컵 찾아줘', 'back')):
    add('location', text, 'find' if '찾' in text else 'fetch' if '가져' in text else 'pick', '컵', location=location)
for text, target, excluded in (('컵 말고 계산기 집어줘', '계산기', '컵'), ('리모컨 말고 마커펜 집어', '마커펜', '리모컨'), ('계산기 말고 컵 잡아줘', '컵', '계산기'), ('마커펜 말고 커터칼 집어줘', '커터칼', '마커펜'),
                               ('컵 말고 리모컨 가져와', '리모컨', '컵'), ('커터칼 말고 계산기 가져다줘', '계산기', '커터칼'), ('리모컨 말고 컵 찾아줘', '컵', '리모컨'), ('컵 아니고 리모컨 집어줘', '리모컨', '컵'),
                               ('계산기 아니라 마커펜 가져와', '마커펜', '계산기'), ('빨간 컵 말고 검은 컵 집어줘', '검은 컵', '빨간 컵')):
    add('negation', text, 'find' if '찾' in text else 'fetch' if '가져' in text else 'pick', target, excluded=[excluded])
for text in ('컵 집고 나서 리모컨도 가져와', '리모컨 가져온 다음에 컵도 가져와', '컵 옮기고 나서 계산기 찾아줘', '리모컨 찾은 뒤 책상에 놔줘', '컵 들어준 다음 소파로 와',
             '리모컨 가져오고 그리고 컵도 치워', '계산기 집고 나서 탁자에 놓아줘', '컵 가져와 그리고 리모컨도', '마커펜 찾은 다음에 가져와', '리모컨 집은 뒤 협탁에 놔줘'):
    add('multiple', text, supported=False)
for text in ('컵에 물 따라줘', '라면 끓여줘', '방 청소해줘', '창문 열어줘', '서랍 열어줘', '문 닫아줘', '불 켜줘', '설거지 해줘', '빨래 개줘', '커피 데워줘',
             '쓰레기 버려줘', '리모컨 내 손에 건네줘', '2층으로 올라가', '노래 불러줘', '바닥 닦아줘', '계란 볶아줘', '컵 던져줘', '춤 춰봐', '뜨거운 냄비 옮겨줘', '물병 따라줘'):
    add('unsupported', text, supported=False)
for text in ('그거 집어줘', '저거 가져와', '이것 좀', '가져다줘', '옮겨줘', '집어줘', '찾아줘'):
    add('missing_target', text, 'pick' if '집' in text else 'fetch' if '가져' in text else 'move' if '옮' in text else 'find' if '찾' in text else None)
(ROOT / 'config/perception_request_cases.json').write_text(json.dumps(cases, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
print(len(cases), {group: sum(c['group'] == group for c in cases) for group in dict.fromkeys(c['group'] for c in cases)})
