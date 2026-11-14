"""Finite Korean event vocabulary; shared by offline generation and game playback."""
import hashlib
import re
from pathlib import Path

DEFAULT_AUDIO_DIR = Path(__file__).resolve().parent / 'assets' / 'speech' / 'ko_F1'
MODEL_REPO = 'supertone-oss-archive/supertonic-3'
MODEL_REVISION = 'aafc6e32416a594460b32413efc49d7fe4ce6d46'
RETRY_MESSAGES = {
    'vision': '영상 확인이 지연되어 홈으로 복귀한 뒤 말 위치를 다시 확인하겠습니다.',
    'move': '말이 이동하지 않아 다시 집어 옮기겠습니다.',
    'place': '말을 놓지 못한 것으로 보여 놓기를 다시 시도하겠습니다.',
    'capture': '포획할 말이 남아 있어 다시 집어 치우겠습니다.',
}
# Sentences are clips, so composed notices reuse the same WAV instead of making
# every combination of start/move/check/victory into a separate synthesis job.
MESSAGES = {
    'voice_on': '음성 안내를 켰습니다.',
    'voice_test': '장기 음성 안내 테스트입니다.',
    'move_recognized': '착수를 인식했습니다.',
    'capture_recognized': '포획을 인식했습니다.',
    'move_confirmed': '착수를 확인했습니다.',
    'capture_confirmed': '포획을 확인했습니다.',
    'board_confirmed': '현재 판을 확인했습니다.',
    'game_start': '대국을 시작합니다.',
    'human_turn': '사람 차례입니다.',
    'thinking': '로봇이 다음 수를 생각합니다.',
    'robot_move': '로봇이 말을 옮깁니다.',
    'robot_capture': '로봇이 상대 말을 잡습니다.',
    'robot_complete': '로봇 동작이 끝났습니다.',
    'robot_preview': '로봇의 수를 계산했습니다.',
    'robot_no_move': '로봇이 둘 수 있는 수가 없습니다.',
    'robot_pass': '로봇이 한 수 쉽니다.',
    'home_start': '호밍을 시작합니다.',
    'home_preview': '호밍 목표를 표시합니다.',
    'robot_stop': '로봇 정지 요청을 보냈습니다.',
    'turn_recheck': '착수 상태를 다시 확인해 주세요.',
    'robot_error': '로봇 동작 중 오류가 발생했습니다.',
    'check_screen': '화면을 확인해 주세요.',
    'capture_pending': '궁 포획이 완료되지 않았습니다.',
    'check_board': '판을 확인해 주세요.',
    'king_buried': '상대 궁을 포획해 묘지로 옮겼습니다.',
    'new_game': '새 대국을 시작해 주세요.',
    'auto_on': '자동 착수 인식을 켰습니다.',
    'auto_off': '자동 착수 인식을 껐습니다.',
    'countdown_restart': '카운트를 초기화하고 다시 기다립니다.',
    'video_delay': '영상이 지연되어 카운트를 초기화합니다.',
    'obstructed': '방해가 감지되어 카운트를 초기화합니다.',
    'move_cancelled': '착수 인식이 취소되었습니다.',
    'check_test': '장군입니다.',
    **{f'retry_{kind}': text for kind,text in RETRY_MESSAGES.items()},
}
for side in ('초', '한'):
    MESSAGES.update({
        f'{side}_check': f'{side} 진영 장군입니다.',
        f'{side}_mate': f'{side} 진영 외통입니다.',
        f'{side}_king_missing': f'{side} 궁이 인식되지 않습니다.',
        f'{side}_victory': f'{side} 승리입니다.',
        f'{side}_victory_preview': f'{side}의 궁 포획 승리 수를 계산했습니다.',
        f'{side}_game_over': f'{side} 승리로 대국이 끝났습니다.',
    })


def normalize(text):
    return ' '.join(text.split())


def sentences(text):
    return [normalize(t) for t in re.split(r'(?<=[.!?])\s+', normalize(text)) if t]


def clip_name(text):
    return hashlib.sha256(normalize(text).encode('utf-8')).hexdigest()[:24]+'.wav'
