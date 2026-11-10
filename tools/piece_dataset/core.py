"""정답 카탈로그, 원본 프레임 회전 추출, 촬영 그룹 단위 데이터셋 내보내기."""
from __future__ import annotations

import hashlib
import json
import math
import random
import shutil
import sqlite3
import uuid
from datetime import datetime
from pathlib import Path

from PIL import Image
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"janggi"))
from piece_detection.crops import crop_geometry, prepare_image

KINDS = [('king', '궁'), ('cha', '차'), ('po', '포'), ('horse', '마'), ('sang', '상'), ('sa', '사'), ('soldier', '졸')]
CLASSES = {f'{side}_{kind}': f'{name} {"병" if side == "red" and kind == "soldier" else ko}'
           for side, name in [('blue', '초'), ('red', '한')] for kind, ko in KINDS}
SPLITS = ('train', 'val', 'test')


def timestamp():
    return datetime.now().astimezone().isoformat()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


class Cancelled(Exception):
    pass


class Catalog:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.root/'catalog.sqlite', timeout=30)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.executescript('''
          CREATE TABLE IF NOT EXISTS sources (
            id TEXT PRIMARY KEY, path TEXT NOT NULL, group_name TEXT NOT NULL,
            split TEXT NOT NULL CHECK(split IN ('train','val','test')), metadata TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS samples (
            id TEXT PRIMARY KEY, source_id TEXT NOT NULL REFERENCES sources(id), frame_index INTEGER,
            time_s REAL, box TEXT, crop_box TEXT, confidence REAL, frame_path TEXT, crop_path TEXT,
            crop_hash TEXT, blur REAL, clipped INTEGER, settings TEXT,
            status TEXT NOT NULL DEFAULT 'pending', label TEXT);
          CREATE INDEX IF NOT EXISTS samples_source ON samples(source_id, frame_index);
          CREATE TABLE IF NOT EXISTS label_history (
            id INTEGER PRIMARY KEY, sample_id TEXT REFERENCES samples(id), old_status TEXT, old_label TEXT,
            new_status TEXT, new_label TEXT, updated_at TEXT, undone INTEGER DEFAULT 0);
        ''')

    def close(self):
        self.db.close()

    def register_source(self, path, group, split, digest, metadata):
        if split not in SPLITS or not group.strip():
            raise ValueError('촬영 그룹과 train/val/test를 지정하세요.')
        group = group.strip()
        existing = self.db.execute('SELECT * FROM sources WHERE id=?', (digest,)).fetchone()
        if existing:
            # 같은 영상은 이름/경로가 바뀌어도 기존 분리와 정답을 유지한다.
            self.db.execute('UPDATE sources SET path=? WHERE id=?', (str(Path(path).resolve()), digest))
            self.db.commit()
            return dict(existing)
        self.db.execute('INSERT INTO sources VALUES (?,?,?,?,?)',
                        (digest, str(Path(path).resolve()), group.strip(), split, json.dumps(metadata)))
        self.db.commit()
        return dict(self.db.execute('SELECT * FROM sources WHERE id=?', (digest,)).fetchone())

    def sources(self):
        return [dict(r) for r in self.db.execute('''SELECT s.*, count(p.id) AS samples
          FROM sources s LEFT JOIN samples p ON p.source_id=s.id GROUP BY s.id ORDER BY s.rowid''')]

    def assign_source(self, source_id, group, split):
        if split not in SPLITS or not group.strip():
            raise ValueError('촬영 그룹과 split을 확인하세요.')
        old = self.db.execute('SELECT group_name FROM sources WHERE id=?', (source_id,)).fetchone()
        if old is None:
            raise ValueError('원본이 없습니다.')
        group = group.strip()
        with self.db:
            if group == old[0]:
                self.db.execute('UPDATE sources SET split=? WHERE group_name=?', (split, group))
            else:
                if self.db.execute('SELECT 1 FROM sources WHERE group_name=? AND split!=?', (group, split)).fetchone():
                    raise ValueError('합칠 그룹의 split과 다릅니다. 그룹 전체의 split을 먼저 변경하세요.')
                self.db.execute('UPDATE sources SET group_name=?, split=? WHERE id=?', (group, split, source_id))

    def rename_group(self, source_id, group):
        if not group.strip():
            raise ValueError('촬영 그룹 이름을 입력하세요.')
        with self.db:
            self.db.execute('UPDATE sources SET group_name=? WHERE id=?', (group.strip(), source_id))

    def auto_split(self, seed=42):
        groups = sorted({r['group_name'] for r in self.sources()})
        if len(groups) < 3:
            raise ValueError('독립 촬영 그룹이 최소 3개 필요합니다. 같은 촬영을 나눈 영상은 같은 그룹으로 묶으세요.')
        random.Random(seed).shuffle(groups)
        n = len(groups)
        nv = max(1, round(n*.15))
        nt = max(1, round(n*.15))
        assignments = {g: ('val' if i < nv else 'test' if i < nv+nt else 'train') for i, g in enumerate(groups)}
        with self.db:
            for group, split in assignments.items():
                self.db.execute('UPDATE sources SET split=? WHERE group_name=?', (split, group))
        return assignments

    def samples(self, status='all', source_id=None):
        clauses, params = [], []
        if status != 'all':
            clauses.append('p.status=?')
            params.append(status)
        if source_id:
            clauses.append('p.source_id=?')
            params.append(source_id)
        where = ' WHERE '+' AND '.join(clauses) if clauses else ''
        return [dict(r) for r in self.db.execute('''SELECT p.*, s.group_name, s.split, s.path AS source_path
            FROM samples p JOIN sources s ON s.id=p.source_id'''+where+' ORDER BY s.rowid, p.frame_index, p.rowid', params)]

    def add_sample(self, values):
        keys = list(values)
        with self.db:
            result = self.db.execute(f'INSERT OR IGNORE INTO samples ({",".join(keys)}) VALUES ({",".join("?" for _ in keys)})',
                                     [values[k] for k in keys])
        return result.rowcount == 1

    def label(self, sample_id, label=None, status='confirmed'):
        if status not in ('pending', 'confirmed', 'hold', 'rejected') or (status == 'confirmed' and label not in CLASSES):
            raise ValueError('정답 라벨 또는 검수 상태가 잘못되었습니다.')
        if status != 'confirmed':
            label = None
        old = self.db.execute('SELECT status,label FROM samples WHERE id=?', (sample_id,)).fetchone()
        if old is None:
            raise ValueError('샘플이 없습니다.')
        with self.db:
            self.db.execute('INSERT INTO label_history(sample_id,old_status,old_label,new_status,new_label,updated_at) VALUES (?,?,?,?,?,?)',
                            (sample_id, old[0], old[1], status, label, timestamp()))
            self.db.execute('UPDATE samples SET status=?,label=? WHERE id=?', (status, label, sample_id))

    def undo(self):
        event = self.db.execute('SELECT * FROM label_history WHERE undone=0 ORDER BY id DESC LIMIT 1').fetchone()
        if event is None:
            return None
        with self.db:
            self.db.execute('UPDATE samples SET status=?, label=? WHERE id=?',
                            (event['old_status'], event['old_label'], event['sample_id']))
            self.db.execute('UPDATE label_history SET undone=1 WHERE id=?', (event['id'],))
        return event['sample_id']


def rotation_steps(step=.1):
    if not math.isfinite(step) or not .001 <= step <= 180:
        raise ValueError('회전 각도 간격은 0.001~180°로 지정하세요.')
    count = round(360/step)
    if not math.isclose(count*step, 360, abs_tol=1e-6):
        raise ValueError('회전 각도 간격은 360°를 나눌 수 있어야 합니다. 예: 1, 0.1, 0.01')
    return count


def augment_image(image, rng, size=224, *, box=None, angle_step=.1, angle=None):
    steps = rotation_steps(angle_step)
    params = dict(angle=round(rng.randrange(1, steps)*angle_step, 9) if angle is None else angle,
                  brightness=rng.uniform(.9, 1.1), contrast=rng.uniform(.9, 1.1))
    return prepare_image(image, size, box=box, **params), params


def sample_angles(image, box, rng, count, angle_step=.1, cancel=None):
    remaining = rotation_steps(angle_step)-1
    if count > remaining:
        raise ValueError('추가 증강 개수가 사용 가능한 서로 다른 회전 각도보다 많습니다.')
    angles, swapped = [], {}
    # 부분 Fisher-Yates: 세밀한 각도 간격이어도 전체 각도 목록을 만들 필요가 없다.
    while remaining and len(angles) < count:
        if cancel is not None and cancel.is_set():
            raise Cancelled('내보내기 취소됨')
        index = rng.randrange(remaining)
        candidate = swapped.get(index, index)+1
        remaining -= 1
        swapped[index] = swapped.get(remaining, remaining)
        angle = round(candidate*angle_step, 9)
        try:
            crop_geometry(image, box, angle)
        except ValueError:
            continue
        angles.append(angle)
    if len(angles) < count:
        raise ValueError('원본 경계 안에서 가능한 회전 각도가 부족합니다. 증강 개수를 줄이거나 보류하세요.')
    return angles


def validate_ratios(ratios):
    values = tuple(float(v) for v in ratios)
    if len(values) != 3 or any(not math.isfinite(v) or v < 0 for v in values) or not math.isclose(sum(values), 100, abs_tol=1e-6):
        raise ValueError('Train/Val/Test 비율은 0 이상의 값이며 합계가 100%여야 합니다.')
    if values[0] <= 0:
        raise ValueError('Train 비율은 0보다 커야 합니다.')
    return values


def validate_minimums(min_per_class, ratios):
    values = tuple(float(v) for v in min_per_class)
    if len(values) != 3 or any(not math.isfinite(v) or v < 0 or not v.is_integer() for v in values):
        raise ValueError('클래스당 최소 수는 Train/Val/Test 각각 0 이상의 정수로 지정하세요.')
    return tuple(int(v) if ratio else 0 for v, ratio in zip(values, ratios))


def split_counts(total, ratios, minimums=(0, 0, 0)):
    if total < sum(minimums):
        raise ValueError(f'최소 합계 {sum(minimums)}개에 비해 원본 단위가 {total}개뿐입니다.')
    quotas = [total*v/100 for v in ratios]
    counts = [math.floor(v) for v in quotas]
    for i in sorted(range(3), key=lambda i: (-(quotas[i]-counts[i]), i))[:total-sum(counts)]:
        counts[i] += 1
    if total and counts[0] == 0:
        donor = max(range(3), key=lambda j: counts[j])
        counts[donor] -= 1
        counts[0] += 1
    # 충분한 원본이 있으면 요청한 분할마다 최소 한 개를 확보한다.
    if total >= sum(v > 0 for v in ratios):
        for i, ratio in enumerate(ratios):
            if ratio and counts[i] == 0:
                donor = max((j for j in range(3) if counts[j] > 1), key=lambda j: counts[j]-quotas[j])
                counts[donor] -= 1
                counts[i] += 1
    for i, minimum in enumerate(minimums):
        while counts[i] < minimum:
            donor = max((j for j in range(3) if counts[j] > minimums[j]), key=lambda j: counts[j]-quotas[j])
            counts[donor] -= 1
            counts[i] += 1
    return counts


def plan_export(rows, ratios=(70, 15, 15), mode='random', unit='sample', seed=42, min_per_class=(1, 1, 1)):
    """원본 정답을 복사해 내보내기별 분할을 정한다. 카탈로그 배정은 바꾸지 않는다."""
    ratios = validate_ratios(ratios)
    minimums = validate_minimums(min_per_class, ratios)
    if mode not in ('random', 'ordered') or unit not in ('sample', 'group'):
        raise ValueError('분배 방식(random/ordered)과 단위(sample/group)를 확인하세요.')
    rows = [dict(row) for row in rows]
    if not rows:
        raise ValueError('정답이 확정된 이미지가 없습니다.')
    # 완전히 같은 Crop은 여러 원본에 있어도 하나의 단위로 묶어 평가 누수를 막는다.
    parents = list(range(len(rows)))
    def find(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    hashes, groups = {}, {}
    for i, row in enumerate(rows):
        digest = row['crop_hash']
        if digest in hashes:
            previous = hashes[digest]
            if rows[previous]['label'] != row['label']:
                raise ValueError('동일 Crop에 서로 다른 정답이 있습니다. 라벨을 검수하세요.')
            parents[find(i)] = find(previous)
        else:
            hashes[digest] = i
        if unit == 'group':
            group = row['group_name']
            if group in groups:
                parents[find(i)] = find(groups[group])
            else:
                groups[group] = i
    bundles = {}
    for i, row in enumerate(rows):
        bundles.setdefault(find(i), []).append(row)
    buckets = {}
    for bundle in bundles.values():
        key = bundle[0]['label'] if unit == 'sample' else 'all'
        buckets.setdefault(key, []).append(bundle)
    if unit == 'sample':
        shortages = [f'{CLASSES[key]}: 원본 {len(units)}단위 / 필요 {sum(minimums)}단위, {sum(minimums)-len(units)}단위 부족'
                     for key, units in buckets.items() if len(units) < sum(minimums)]
        if shortages:
            raise ValueError('클래스별 최소 수를 채울 원본이 부족합니다. 동일 Crop은 한 단위로 셉니다.\n'+'\n'.join(shortages))
    for key, units in buckets.items():
        if mode == 'random':
            random.Random(f'{seed}:split:{key}').shuffle(units)
        counts = split_counts(len(units), ratios, minimums if unit == 'sample' else (0, 0, 0))
        offset = 0
        for split, count in zip(SPLITS, counts):
            for bundle in units[offset:offset+count]:
                for row in bundle:
                    row['split'] = split
            offset += count
    classes = sorted({r['label'] for r in rows})
    counts = {split: {label: 0 for label in classes} for split in SPLITS}
    for row in rows:
        counts[row['split']][row['label']] += 1
    distinct_counts = {split: {label: len({r['crop_hash'] for r in rows if r['split'] == split and r['label'] == label})
                              for label in classes} for split in SPLITS}
    shortages = [f'{split} · {CLASSES[label]}: 고유 원본 {distinct_counts[split][label]} / 최소 {minimum}, {minimum-distinct_counts[split][label]}개 부족'
                 for split, minimum in zip(SPLITS, minimums) for label in classes if distinct_counts[split][label] < minimum]
    if shortages:
        raise ValueError('현재 촬영 그룹 분배가 클래스별 최소 수를 충족하지 못합니다.\n'+'\n'.join(shortages)+
                         '\n그룹을 유지한 채 다른 seed/비율을 시도하거나 사진별 분배를 선택하세요.')
    warnings = []
    for split, ratio in zip(SPLITS, ratios):
        missing = [CLASSES[label] for label, count in counts[split].items() if not count]
        if ratio and missing:
            warnings.append(f'{split}에 없는 클래스: {", ".join(missing)} (원본 수 또는 촬영 그룹 부족)')
    if unit == 'sample':
        warnings.append('사진별 분배는 같은 촬영의 유사 이미지가 여러 분할에 포함될 수 있습니다. 독립 평가에는 촬영 그룹별 분배를 사용하세요.')
    return rows, dict(ratios_percent=dict(zip(SPLITS, ratios)), mode=mode, unit=unit, seed=seed,
                      counts_original=counts, minimum_per_class=dict(zip(SPLITS, minimums)),
                      minimum_unit='distinct_crop', counts_distinct=distinct_counts, warnings=warnings)


def export_dataset(project, size=224, augment=5, seed=42, cancel=None, progress=None, angle_step=.1,
                   ratios=(70, 15, 15), split_mode='random', split_unit='sample', min_per_class=(1, 1, 1)):
    if not 32 <= size <= 1024 or not 0 <= augment <= 50:
        raise ValueError('이미지 크기는 32~1024, 증강 개수는 0~50으로 지정하세요.')
    steps = rotation_steps(angle_step)
    if augment >= steps:
        raise ValueError('추가 증강 개수가 사용 가능한 서로 다른 회전 각도보다 많습니다.')
    catalog = Catalog(project)
    try:
        rows = catalog.samples('confirmed')
    finally:
        catalog.close()
    rows, split_info = plan_export(rows, ratios, split_mode, split_unit, seed, min_per_class)
    classes = sorted({r['label'] for r in rows})
    counts = split_info['counts_original']
    source_ids = {}
    for r in rows:
        source_ids[r['source_id']] = dict(path=r['source_path'], group=r['group_name'])
    root = Path(project).resolve()
    output = root/'exports'/('dataset_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    temporary = output.with_name('.'+output.name+'.'+uuid.uuid4().hex)
    temporary.mkdir(parents=True)
    manifest = []
    try:
        for split in SPLITS:
            for label in classes:
                (temporary/split/label).mkdir(parents=True)
        total = len(rows)
        cached_path, frame = None, None
        for i, r in enumerate(rows):
            if cancel is not None and cancel.is_set():
                raise Cancelled('내보내기 취소됨')
            frame_path = root/r['frame_path']
            if frame_path != cached_path:
                if not frame_path.is_file():
                    raise ValueError(f'원본 프레임이 없습니다: {frame_path}. 회전 추출에는 원본 프레임이 필요합니다.')
                with Image.open(frame_path) as original:
                    frame = original.convert('RGB')
                cached_path = frame_path
            box = json.loads(r['box'])
            rng = random.Random(f'{seed}:{r["id"]}')
            # 경계 밖을 단색으로 채우지 않는다. 불가능한 샘플은 정답을 유지한 채 내보내기만 중단한다.
            try:
                crop_geometry(frame, box)
                count = augment if r['split'] == 'train' else 0
                angles = sample_angles(frame, box, rng, count, angle_step, cancel)
            except ValueError as exc:
                raise ValueError(f'샘플 {r["id"]} (프레임 {r["frame_index"]}): {exc}') from exc
            for variant, angle in enumerate([0., *angles]):
                if cancel is not None and cancel.is_set():
                    raise Cancelled('내보내기 취소됨')
                if variant:
                    prepared, params = augment_image(frame, rng, size, box=box, angle_step=angle_step, angle=angle)
                else:
                    prepared, params = prepare_image(frame, size, box=box), dict(angle=0., brightness=1., contrast=1.)
                relative = Path(r['split'])/r['label']/f'{r["id"]}_{variant:02}.png'
                prepared.save(temporary/relative)
                manifest.append(dict(file=str(relative), sample_id=r['id'], label=r['label'], split=r['split'],
                                     source_id=r['source_id'], group=r['group_name'], frame=r['frame_index'],
                                     time_s=r['time_s'], box=box, raw_crop=r['crop_path'],
                                     original_pixels=json.loads(r['crop_box']), frame_path=r['frame_path'],
                                     sampling_corners=crop_geometry(frame, box, angle)[-1],
                                     augmented=bool(variant), transform=params))
            if progress:
                progress(i+1, total)
        info = dict(schema_version=3, created_at=timestamp(), image_size=size, seed=seed,
                    augmentation_per_train_image=augment, angle_step_degrees=angle_step,
                    classes=classes, class_names={c: CLASSES[c] for c in classes},
                    missing_classes=sorted(set(CLASSES)-set(classes)), counts_original=counts,
                    sources=source_ids, split=split_info, preprocessing='tight square centered on detection box; rotated sampling from original frame; RGB; no padding; no flips')
        (temporary/'dataset.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
        with (temporary/'manifest.jsonl').open('w', encoding='utf-8') as f:
            for row in manifest:
                f.write(json.dumps(row, ensure_ascii=False)+'\n')
        temporary.rename(output)
        return output
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
