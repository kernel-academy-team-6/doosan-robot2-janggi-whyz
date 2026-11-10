import hashlib
import json
import random
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piece_dataset.core import Catalog, Cancelled, export_dataset, prepare_image, sample_angles, crop_geometry, plan_export
from piece_dataset.extraction import extract_video, ROOT


def add_sample(catalog, source, name, color, label=None):
    relative = Path('crops/raw')/(name+'.png')
    path = catalog.root/relative
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new('RGB', (20, 30), color).save(path)
    frame_path = Path('frames')/(name+'.png')
    (catalog.root/frame_path).parent.mkdir(parents=True, exist_ok=True)
    frame = Image.new('RGB', (100, 100), (30, 110, 60))
    frame.paste(Image.new('RGB', (20, 30), color), (40, 35))
    frame.save(catalog.root/frame_path)
    catalog.add_sample(dict(id=name, source_id=source, frame_index=0, time_s=0., box='[40,35,60,65]',
                           crop_box='[0,0,20,30]', confidence=.9, frame_path=str(frame_path), crop_path=str(relative),
                           crop_hash=hashlib.sha256(path.read_bytes()).hexdigest(), blur=10., clipped=0, settings='{}'))
    if label:
        catalog.label(name, label)


def make_project(path):
    catalog = Catalog(path)
    for i, split in enumerate(('train', 'val', 'test')):
        source = catalog.register_source(f'/example/{split}.mp4', split, split, f'source{i}', {})
        for j, label in enumerate(('blue_cha', 'blue_sang')):
            add_sample(catalog, source['id'], f'{split}{j}', (40+i*30, 20+j*90, 170), label)
    return catalog


def test_label_persistence_undo_and_group_integrity(tmp_path):
    catalog = Catalog(tmp_path)
    catalog.register_source('/a.mp4', ' session ', 'train', 'a', {})
    catalog.register_source('/b.mp4', 'session', 'test', 'b', {})
    add_sample(catalog, 'a', 'sample', 'red')
    catalog.label('sample', 'blue_cha')
    catalog.close()
    catalog = Catalog(tmp_path)
    assert catalog.samples()[0]['label'] == 'blue_cha'
    assert catalog.undo() == 'sample'
    assert catalog.samples()[0]['status'] == 'pending'
    catalog.assign_source('a', 'session', 'val')
    assert {s['split'] for s in catalog.sources()} == {'val'}
    existing = catalog.register_source('/renamed.mp4', 'other', 'test', 'a', {})
    assert existing['group_name'] == 'session' and existing['split'] == 'val'
    with pytest.raises(ValueError, match='최소 3개'):
        catalog.auto_split()
    catalog.close()


def test_rotated_crop_uses_original_background_without_padding():
    frame = Image.new('RGB', (200, 200), (30, 140, 70))
    frame.paste(Image.new('RGB', (40, 40), (255, 0, 0)), (80, 80))
    box = [80, 80, 120, 120]
    assert np.array_equal(np.asarray(prepare_image(frame, 40, box=box)), np.asarray(frame.crop(box)))
    rotated = np.asarray(prepare_image(frame, 40, 45, box=box))
    # 旧Crop에 없던 실제 초록 배경이 모서리를 채운다. 회색 패딩이 아니다.
    assert np.allclose(rotated[0, 0], (30, 140, 70), atol=2)
    assert np.allclose(rotated[20, 20], (255, 0, 0), atol=2)
    assert not ((rotated == 127).all(axis=2)).any()


def test_rotated_crop_geometry_and_fine_angles():
    frame = Image.new('RGB', (200, 200), 'white')
    frame.paste(Image.new('RGB', (4, 4), 'red'), (110, 98))
    box = [80, 80, 120, 120]
    rotated = np.asarray(prepare_image(frame, 40, 90, box=box))
    red = (rotated[:, :, 0] > 240) & (rotated[:, :, 1] < 10)
    yy, xx = np.nonzero(red)
    assert yy.mean() < 10 and 17 < xx.mean() < 22  # 오른쪽 표식이 반시계 회전 후 위로 간다.
    angles = sample_angles(frame, box, random.Random(42), 50, .001)
    assert len(set(angles)) == 50 and all(0 < a < 360 for a in angles)
    assert all(abs(a/.001-round(a/.001)) < 1e-6 for a in angles)
    assert any(abs(a-round(a)) > .01 for a in angles)
    with pytest.raises(ValueError, match='원본 프레임 밖'):
        prepare_image(frame, 64, 45, box=[0, 0, 40, 40])
    edge_angles = sample_angles(frame, [0, 0, 40, 40], random.Random(1), 3, 90)
    assert set(edge_angles) == {90, 180, 270}
    with pytest.raises(ValueError, match='360°'):
        sample_angles(frame, box, random.Random(1), 2, .7)


def test_export_augments_only_train_and_preserves_ground_truth(tmp_path):
    catalog = make_project(tmp_path)
    catalog.close()
    output = export_dataset(tmp_path, size=64, augment=2, seed=7)
    assert len(list((output/'train').rglob('*.png'))) == 6
    assert len(list((output/'val').rglob('*.png'))) == 2
    assert len(list((output/'test').rglob('*.png'))) == 2
    manifest = [json.loads(line) for line in (output/'manifest.jsonl').read_text().splitlines()]
    assert all(r['split'] == 'train' for r in manifest if r['augmented'])
    assert all(Path(r['file']).parent.name == r['label'] for r in manifest)
    assert all(0 <= r['transform']['angle'] <= 360 for r in manifest)
    info = json.loads((output/'dataset.json').read_text())
    assert info['classes'] == ['blue_cha', 'blue_sang']
    assert info['schema_version'] == 3 and info['angle_step_degrees'] == .1
    assert all(len(r['sampling_corners']) == 4 and r['frame_path'] for r in manifest)
    assert info['counts_original']['train'] == {'blue_cha': 1, 'blue_sang': 1}
    second = export_dataset(tmp_path, size=64, augment=2, seed=7)
    for path in output.rglob('*.png'):
        assert path.read_bytes() == (second/path.relative_to(output)).read_bytes()
    event = threading.Event()
    event.set()
    with pytest.raises(Cancelled):
        export_dataset(tmp_path, cancel=event)
    assert not list((tmp_path/'exports').glob('.*'))


def test_export_bundles_duplicates_and_ignores_legacy_splits(tmp_path):
    catalog = make_project(tmp_path)
    catalog.db.execute("UPDATE samples SET crop_hash='duplicate' WHERE id IN ('train0','val0')")
    catalog.db.commit()
    before = catalog.samples()
    output = export_dataset(tmp_path, augment=0, min_per_class=(0, 0, 0))
    manifest = [json.loads(line) for line in (output/'manifest.jsonl').read_text().splitlines()]
    assert len({r['split'] for r in manifest if r['sample_id'] in ('train0', 'val0')}) == 1
    assert catalog.samples() == before
    catalog.label('val0', 'blue_sang')
    with pytest.raises(ValueError, match='서로 다른 정답'):
        export_dataset(tmp_path)
    catalog.close()


def test_video_extraction_resume_preserves_labels_and_original_coordinates(tmp_path):
    sys.path.insert(0, str(ROOT/'janggi'))
    from piece_detection.detector import Detection

    class Detector:
        def detect(self, frame, **kwargs):
            return [Detection(0, 'janggi_dol', 'unknown', 'piece', .9, (8, 5, 24, 25)),
                    Detection(0, 'janggi_dol', 'unknown', 'piece', .8, (9, 6, 25, 26)),
                    Detection(0, 'janggi_dol', 'unknown', 'piece', .85, (0, 0, 7, 7))]

    video, model = tmp_path/'input.mp4', tmp_path/'model.pt'
    model.write_bytes(b'test model')
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 2, (32, 32))
    for i in range(4):
        writer.write(np.full((32, 32, 3), i*40+30, np.uint8))
    writer.release()
    project = tmp_path/'project'
    result = extract_video(project, video, interval=.5, margin=.12, model=model, detector=Detector(), limit=1)
    assert result['added'] == 2
    catalog = Catalog(project)
    rows = catalog.samples()
    assert len(rows) == 2 and rows[0]['clipped'] == 1
    catalog.label(rows[0]['id'], 'blue_cha')
    catalog.close()
    assert extract_video(project, video, interval=.5, margin=.12, model=model, detector=Detector(), limit=1)['added'] == 0
    catalog = Catalog(project)
    assert catalog.samples()[0]['label'] == 'blue_cha'
    assert json.loads(catalog.samples()[1]['box']) == [8, 5, 24, 25]
    assert (project/catalog.samples()[0]['frame_path']).is_file()
    catalog.close()


def test_export_requires_full_frame_and_cleans_partial_output(tmp_path):
    catalog = make_project(tmp_path)
    frame_path = tmp_path/catalog.samples()[0]['frame_path']
    catalog.close()
    frame_path.unlink()
    with pytest.raises(ValueError, match='원본 프레임이 없습니다'):
        export_dataset(tmp_path)
    assert not list((tmp_path/'exports').iterdir())


def split_rows(n=20):
    return [dict(id=f'{label}_{i}', label=label, crop_hash=f'{label}_{i}', group_name=f'group{i//2}',
                 source_id='one-video', frame_index=i) for i in range(n) for label in ('blue_cha', 'blue_sang')]


def test_export_split_ratios_order_random_and_no_mutation():
    rows = split_rows()
    before = [dict(row) for row in rows]
    ordered, info = plan_export(rows, (70, 15, 15), 'ordered')
    assert rows == before
    for label in ('blue_cha', 'blue_sang'):
        selected = [r for r in ordered if r['label'] == label]
        assert [r['split'] for r in selected] == ['train']*14+['val']*3+['test']*3
    random_rows, _ = plan_export(rows, seed=5)
    assert random_rows == plan_export(rows, seed=5)[0]
    assert random_rows != plan_export(rows, seed=6)[0]
    assert random_rows != ordered
    assert info['counts_original'] == {'train': {'blue_cha': 14, 'blue_sang': 14},
                                       'val': {'blue_cha': 3, 'blue_sang': 3},
                                       'test': {'blue_cha': 3, 'blue_sang': 3}}
    for ratios in ((80, 10, 20), (0, 50, 50), (-10, 50, 60), (float('nan'), 0, 0)):
        with pytest.raises(ValueError):
            plan_export(rows, ratios)


def test_export_group_partition_and_small_class_warnings():
    rows = split_rows()
    # 동일 Crop이 서로 다른 그룹에 있으면 그 그룹들도 함께 배정한다.
    rows[-1]['crop_hash'] = rows[1]['crop_hash']
    planned, _ = plan_export(rows, (60, 20, 20), unit='group')
    groups = {}
    for row in planned:
        groups.setdefault(row['group_name'], set()).add(row['split'])
    assert all(len(v) == 1 for v in groups.values())
    assert planned[-1]['split'] == planned[1]['split']
    with pytest.raises(ValueError, match='최소 수'):
        plan_export(split_rows(1))
    small, info = plan_export(split_rows(1), min_per_class=(0, 0, 0))
    assert all(row['split'] == 'train' for row in small)
    assert any('val에 없는 클래스' in warning for warning in info['warnings'])
    all_train, _ = plan_export(rows, (100, 0, 0))
    assert {row['split'] for row in all_train} == {'train'}


def test_export_single_pool_new_ratios_then_train_augmentation(tmp_path):
    catalog = make_project(tmp_path)
    for source in catalog.sources():
        catalog.assign_source(source['id'], source['group_name'], 'train')
    before = catalog.samples()
    output = export_dataset(tmp_path, size=32, augment=1, ratios=(100, 0, 0), split_mode='ordered')
    assert len(list((output/'train').rglob('*.png'))) == 12
    assert not list((output/'val').rglob('*.png'))
    assert catalog.samples() == before
    info = json.loads((output/'dataset.json').read_text())
    assert info['split']['ratios_percent'] == {'train': 100, 'val': 0, 'test': 0}
    output2 = export_dataset(tmp_path, size=32, augment=1, ratios=(34, 33, 33), split_mode='ordered')
    assert len(list((output2/'train').rglob('*.png'))) == 4
    assert len(list((output2/'val').rglob('*.png'))) == 2
    assert len(list((output2/'test').rglob('*.png'))) == 2
    assert catalog.samples() == before
    catalog.close()


def test_class_minimums_override_ratios_before_augmentation():
    rows, info = plan_export(split_rows(20), ratios=(90, 5, 5), min_per_class=(5, 4, 3), seed=4)
    for label in ('blue_cha', 'blue_sang'):
        assert [info['counts_original'][split][label] for split in ('train', 'val', 'test')] == [13, 4, 3]
    assert info['minimum_per_class'] == dict(train=5, val=4, test=3)
    assert rows == plan_export(split_rows(20), ratios=(90, 5, 5), min_per_class=(5, 4, 3), seed=4)[0]
    for minimums in ((-1, 1, 1), (1.5, 1, 1), (1, 2)):
        with pytest.raises(ValueError):
            plan_export(split_rows(), min_per_class=minimums)
    with pytest.raises(ValueError, match='초 차.*부족'):
        plan_export(split_rows(2), min_per_class=(1, 1, 1))
    # 동일 Crop 복제본은 최소 수를 채울 서로 다른 원본으로 세지 않는다.
    duplicates = split_rows(5)
    for row in duplicates:
        row['crop_hash'] = row['label']
    with pytest.raises(ValueError, match='동일 Crop'):
        plan_export(duplicates)
    all_train, _ = plan_export(split_rows(2), ratios=(100, 0, 0), min_per_class=(2, 999, 999))
    assert {r['split'] for r in all_train} == {'train'}


def test_group_minimums_fail_without_splitting_group():
    rows = split_rows(6)
    for row in rows:
        row['group_name'] = 'single-session'
    with pytest.raises(ValueError, match='촬영 그룹 분배'):
        plan_export(rows, unit='group')
    planned, _ = plan_export(rows, unit='group', min_per_class=(1, 0, 0))
    assert {r['split'] for r in planned} == {'train'}
