#!/usr/bin/env python3
"""Play actual cached F1 WAVs through the game's async player; no model/network."""
import argparse
import json
from pathlib import Path
import sys
WS=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(WS/'janggi/realtime_ui'))
from speech import Speech
from speech_catalog import MESSAGES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--volume',type=float,default=60.)
    parser.add_argument('--message',choices=list(MESSAGES),action='append')
    args=parser.parse_args()
    errors=[]
    service=Speech(volume=args.volume,on_error=errors.append)
    try:
        if not service.set_enabled(True):raise RuntimeError(service.unavailable_reason)
        for key in args.message or ['voice_test','초_check','한_victory']:
            text=MESSAGES[key]
            if not service.say(text) or not service.wait_idle(20.):raise RuntimeError('playback timed out')
            if errors:raise RuntimeError('; '.join(errors))
            print(json.dumps({'event':key,'text':text,'playback':'completed','volume':args.volume},ensure_ascii=False),flush=True)
    finally:service.close()

if __name__=='__main__':main()
