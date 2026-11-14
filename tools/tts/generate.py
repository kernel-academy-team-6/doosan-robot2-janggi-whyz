#!/usr/bin/env python3
"""Download the pinned archive model, pre-render finite game messages, then exit."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import time
import wave

WS=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(WS/'janggi/realtime_ui'))
from speech_catalog import DEFAULT_AUDIO_DIR, MESSAGES, MODEL_REPO, MODEL_REVISION, clip_name


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def wav_info(path):
    with wave.open(str(path)) as stream:
        if stream.getnchannels()!=1 or stream.getsampwidth()!=2 or not stream.getnframes():
            raise ValueError(f'invalid PCM WAV: {path}')
        return {'sample_rate':stream.getframerate(),'frames':stream.getnframes(),
                'duration_sec':stream.getnframes()/stream.getframerate()}


def generate(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    target=output/'manifest.json'
    config={'model_repo':MODEL_REPO,'model_revision':MODEL_REVISION,
            'package': 'supertonic==1.3.1','lang':'ko','voice':'F1',
            'steps':args.steps,'speed':args.speed,'pcm':'PCM_16','seed':42}
    try:old=json.loads(target.read_text())
    except (OSError,ValueError):old={}
    clips=old.get('clips',{}) if old.get('generation')==config else {}
    missing=[]
    for text in MESSAGES.values():
        entry=clips.get(text);path=output/clip_name(text)
        try:
            if entry and entry.get('file')==path.name and digest(path)==entry['sha256']:
                wav_info(path);continue
        except (OSError,KeyError,ValueError,wave.Error):pass
        missing.append(text)
    if not missing:
        print(f'Cached: {len(clips)} clips; no download, model load or synthesis required.')
        return
    # Heavy packages exist only in the dedicated generation environment.
    if importlib.metadata.version('supertonic')!='1.3.1':
        raise RuntimeError('Use tools/tts/setup.sh: this generator requires supertonic==1.3.1.')
    os.environ.setdefault('HF_HOME',str(args.model_dir.resolve()/'.hf'))
    from huggingface_hub import snapshot_download
    import numpy as np
    import onnxruntime as ort
    ort.disable_telemetry_events()
    from supertonic import TTS
    model=args.model_dir.resolve()
    if not args.offline:
        snapshot_download(MODEL_REPO,revision=MODEL_REVISION,local_dir=model,
                          allow_patterns=['onnx/*','voice_styles/F1.json','LICENSE','README.md'])
    tts=TTS(model='supertonic-3',model_dir=model,auto_download=False,
            intra_op_num_threads=args.threads,inter_op_num_threads=1)
    style=tts.get_voice_style(voice_name='F1')
    started=time.monotonic()
    for index,text in enumerate(missing,1):
        np.random.seed(42)
        audio,duration=tts.synthesize(text=text,voice_style=style,lang='ko',
                                      total_steps=args.steps,speed=args.speed,verbose=False)
        audio=np.asarray(audio).reshape(-1)
        if not np.isfinite(audio).all() or not len(audio) or np.max(np.abs(audio))<1.e-5:
            raise ValueError('model produced silent/invalid audio')
        # Explicit PCM16 allows stdlib-only software volume during game playback.
        import soundfile as sf
        path=output/clip_name(text);tmp=path.with_suffix('.tmp.wav')
        sf.write(tmp,audio,tts.sample_rate,subtype='PCM_16');tmp.replace(path)
        clips[text]={'file':path.name,'sha256':digest(path),**wav_info(path)}
        manifest={'schema':1,'generation':config,'clips':clips}
        tmp_manifest=target.with_suffix('.tmp')
        tmp_manifest.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        tmp_manifest.replace(target)
        print(f'{index}/{len(missing)} {text} ({clips[text]["duration_sec"]:.2f}s)',flush=True)
    print(f'Generated {len(missing)} clips in {time.monotonic()-started:.1f}s; {output}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir',type=Path,default=WS/'tools/tts/model')
    parser.add_argument('--output',type=Path,default=DEFAULT_AUDIO_DIR)
    parser.add_argument('--offline',action='store_true',help='use already downloaded model only')
    parser.add_argument('--steps',type=int,default=10)
    parser.add_argument('--speed',type=float,default=1.05)
    parser.add_argument('--threads',type=int,default=4)
    args=parser.parse_args()
    if not 1<=args.steps<=100 or not .7<=args.speed<=2 or args.threads<1:parser.error('invalid synthesis parameters')
    generate(args)

if __name__=='__main__':main()
