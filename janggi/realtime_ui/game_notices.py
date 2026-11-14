"""Short Korean event descriptions, independent of audio and robot hardware."""
from rules import Game

SIDES={'blue':'초','red':'한'}


def position_key(game):
    return (id(game),game.turn,game.top_side,tuple(sorted(game.board.items())))


def position_notice(game):
    board=dict(game.board)
    key=(id(game),game.turn,game.top_side,tuple(sorted(board.items())))
    status=Game(board,top_side=game.top_side,turn=game.turn).status()
    side=SIDES[game.turn]
    text={'장군':f'{side} 진영 장군입니다.',
          '외통':f'{side} 진영 외통입니다.',
          '궁 없음':f'{side} 궁이 인식되지 않습니다.'}.get(status,'')
    return key,text


def automatic_notice(result, before, after):
    if result.remaining is None or result.fire or result.move is None:
        return None,None
    dst=result.move[1]
    capture=dst in before and before[dst][0] != after[dst][0]
    key=(result.move,result.deadline)
    text='포획을 인식했습니다.' if capture else '착수를 인식했습니다.'
    return key,text
