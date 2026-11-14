"""Observation-only human move countdown. No robot calls, threads, or Tk state."""
from dataclasses import dataclass


def observed_move(before, after, side):
    """Infer one physical move from occupancy, retaining known piece identity.

    Stationary CNN kind changes are not moves. Captures need replacement by
    the human side at one previously enemy-occupied destination, not just a
    missing piece. Unrelated vacancies/additions remain ambiguous.
    """
    removed=set(before)-set(after)
    added=set(after)-set(before)
    if len(removed)!=1:
        return None
    src=next(iter(removed))
    if before[src][0]!=side:
        return None
    if len(added)==1:
        dst=next(iter(added))
        # One departure and one arrival identify the same physical piece;
        # retain its prior side as well as its kind if CNN reclassifies it.
        # A separate enemy-to-human replacement would be a second change.
        if any(before[c][0] not in (side,'unknown') and after[c][0]==side
               for c in set(before)&set(after)):
            return None
    elif not added:
        targets=[c for c in set(before)&set(after)
                 if before[c][0] not in (side,'unknown') and after[c][0]==side]
        if len(targets)!=1:
            return None
        dst=targets[0]
    else:
        return None
    return src,dst


@dataclass(frozen=True)
class AutoTurnState:
    text: str
    remaining: float | None = None
    move: tuple | None = None
    fire: bool = False
    deadline: float | None = None
    board: dict | None = None


class AutomaticTurn:
    def __init__(self, delay_sec=3., baseline_settle_sec=.3):
        self.delay_sec = delay_sec
        self.baseline_settle_sec = baseline_settle_sec
        self.reset()

    def reset(self):
        self.baseline = None
        self._baseline_candidate = self._baseline_since = None
        self._candidate = self._since = None
        self._epoch = self._stamp = None
        self._fired = False

    def cancel_countdown(self):
        self._candidate = self._since = None

    def set_baseline(self, board):
        self.reset()
        if board:
            self.baseline = dict(board)

    def moved_board(self, move):
        board=dict(self.baseline)
        src,dst=move
        board[dst]=board.pop(src)
        return {c:p for c,p in board.items() if p[0] in ('blue','red') and p[1]!='piece'}

    def update(self, board, side, *, now, stamp, clear, epoch):
        if self._fired:
            return AutoTurnState('로봇 착수 대기')
        fresh = self._stamp is None or stamp > self._stamp
        interrupted = self._epoch is not None and epoch != self._epoch
        self._epoch = epoch
        if fresh:
            self._stamp = stamp
        if not clear or now-stamp > 1.5 or now < stamp:
            self.cancel_countdown()
            self._baseline_candidate = self._baseline_since = None
            return AutoTurnState('영상 지연 · 새 영상 대기' if now-stamp > 1.5 or now < stamp else
                                 '방해 해제·새 영상 대기')
        if interrupted:
            self.cancel_countdown()
            self._baseline_candidate = self._baseline_since = None
        if self.baseline is None:
            # Establish the actual post-home board from distinct, clear frames.
            # Keep this separate from the engine's predicted robot destination.
            if not board:
                return AutoTurnState('기준 판 인식 대기')
            if self._baseline_candidate != board:
                self._baseline_candidate = dict(board)
                self._baseline_since = now
            elif fresh and now-self._baseline_since >= self.baseline_settle_sec:
                self.baseline = dict(board)
            return AutoTurnState('사람 이동·포획 대기' if self.baseline is not None else '기준 판 확인 중')
        # Resolve previously unknown occupants without rewriting already known
        # identities or absorbing an actual movement into the reference board.
        for cell in set(self.baseline)&set(board):
            if (self.baseline[cell][0]=='unknown' or self.baseline[cell][1]=='piece') and board[cell][0] in ('blue','red'):
                self.baseline[cell]=board[cell]
        move = observed_move(self.baseline, board, side)
        if move is None:
            self.cancel_countdown()
            return AutoTurnState('사람 이동·포획 대기')
        if self._candidate != move:
            self._candidate, self._since = move, now
        elif self._since is None:
            self._since = now
        remaining = max(0., self._since+self.delay_sec-now)
        if fresh and remaining == 0.:
            self._fired = True
            return AutoTurnState('자동 착수 실행', 0., move, True, board=self.moved_board(move))
        text='포획 감지' if move[1] in self.baseline else '이동 감지'
        return AutoTurnState(text, remaining, move, deadline=self._since+self.delay_sec,
                             board=self.moved_board(move))
