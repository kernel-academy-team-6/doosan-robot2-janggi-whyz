"""Small delayed interpolation buffer; it never extrapolates or commands a pose."""
from collections import deque
import math
import threading


class HeldStreamSampler:
    """Feed only real updates of a fast stream that repeats slow, held values.

    The virtual Doosan driver republishes the DRCF monitoring pose at 100 Hz, but
    the pose itself changes only every ~0.1 s (measured 2026-11-14) and its
    velocity field is always zero. Buffering every repeat produced ~90 ms holds
    followed by ~10 ms jumps: a 10 Hz velocity reference swinging 0 ↔ 300+ °/s.

    - Repeats are dropped while moving; a change is stamped where the update
      period predicts it, clipped to the receipt-time quantisation window.
    - The first change after rest gets a hold anchor one period earlier, so the
      start is a normal segment instead of a jump from a stale sample.
    - At rest, repeats are kept at one per period so the stationary tail can be
      realigned exactly as before.
    - `delay_sec` covers two update periods: a Hermite segment needs the sample
      after its end point, otherwise its end slope changes on arrival.
    Velocities are never taken from the stream (always zero in virtual mode).
    """
    def __init__(self, period=.1, *, min_period=.01, max_period=.25, lead=.03, tolerance=1e-9):
        self.period, self.min_period, self.max_period = period, min_period, max_period
        self.lead, self.tolerance = lead, tolerance
        self._last = self._last_receipt = None
        self._last_change = None
        self._moving = False
        self._periods = deque(maxlen=9)
        self._receipts = deque(maxlen=31)

    @property
    def delay_sec(self):
        return 2*self.period+self.lead

    def _quantum(self):
        values = sorted(self._receipts)
        return values[len(values)//2] if values else .01

    def add(self, buffer, positions, stamp):
        sample = dict(positions)
        if not sample or not math.isfinite(stamp) or not all(math.isfinite(v) for v in sample.values()):
            return False
        if self._last_receipt is not None:
            if stamp <= self._last_receipt:
                return False
            self._receipts.append(stamp-self._last_receipt)
        self._last_receipt = stamp
        last = buffer.last_stamp()
        if self._last is None or last is None:
            self._push(buffer, stamp, sample)
            self._last, self._last_change = sample, stamp
            return True
        changed = (sample.keys() != self._last.keys()
                   or any(abs(v-self._last[n]) > self.tolerance for n, v in sample.items()))
        if not changed:
            if self._moving and stamp-self._last_change > 1.6*self.period:
                self._moving = False   # motion ended at the last change
                self._push(buffer, stamp, self._last)
            elif not self._moving and stamp-last >= self.period:
                self._push(buffer, stamp, self._last)
            return False
        if self._moving:
            at = min(stamp, max(stamp-self._quantum(), last+self.period))
            self._periods.append(at-last)
            ordered = sorted(self._periods)
            self.period = min(self.max_period, max(self.min_period, ordered[len(ordered)//2]))
        else:
            # Start from rest: the first segment is one full period long. Rest
            # repeats newer than the anchor are not played yet (playback lags by
            # >= 2 periods) and would shorten the first segment into a jump.
            anchor = stamp-self.period
            buffer.drop_held_tail_after(anchor, self._last, self.tolerance)
            last = buffer.last_stamp()
            if anchor > last:
                self._push(buffer, anchor, self._last)
            at = max(stamp, last+self.period)
        self._push(buffer, at, sample)
        self._last, self._last_change, self._moving = sample, stamp, True
        return True

    @staticmethod
    def _push(buffer, stamp, sample):
        last = buffer.last_stamp()
        if last is not None and stamp <= last:
            stamp = last+1e-6
        buffer.add(sample, stamp)


class JointTargetBuffer:
    def __init__(self, delay_sec=.025, *, cubic=False, continuous=False, max_samples=128):
        if not math.isfinite(delay_sec) or delay_sec < 0:
            raise ValueError('Interpolation delay must be finite and nonnegative')
        self.delay_sec = delay_sec
        self.cubic = cubic
        self.continuous = continuous
        self._last_query = self._playback = None
        self._samples = deque(maxlen=max_samples)
        self._lock = threading.Lock()

    def add(self, positions, stamp, velocities=None):
        sample = dict(positions)
        if not math.isfinite(stamp) or not sample or not all(math.isfinite(v) for v in sample.values()):
            return
        with self._lock:
            if self._samples and stamp <= self._samples[-1][0]:
                return
            speeds={n:v for n,v in (velocities or {}).items() if n in sample and math.isfinite(v)}
            self._samples.append((stamp, sample, speeds))

    def last_stamp(self):
        with self._lock:
            return self._samples[-1][0] if self._samples else None

    def drop_held_tail_after(self, stamp, positions, tolerance=1e-9):
        """Remove trailing repeats of `positions` newer than `stamp` (keeps one sample)."""
        with self._lock:
            while (len(self._samples)>1 and self._samples[-1][0]>stamp
                   and self._samples[-1][1].keys()>=positions.keys()
                   and all(abs(self._samples[-1][1][n]-v)<=tolerance for n,v in positions.items())):
                self._samples.pop()

    def sample(self, now):
        return self.sample_state(now)[0]

    def align_stationary_tail(self, now, names):
        """Drop only repeated rest targets, never an unplayed motion segment.

        Low-RTF playback can otherwise accumulate wall-time idle frames between
        UI operations even after the physical arm has reached its endpoint.
        Caller must first confirm that the actual arm is stationary there.
        """
        with self._lock:
            if len(self._samples)<2:
                return False
            latest=self._samples[-1][1]
            if not all(n in latest for n in names):
                return False
            if any(any(n not in q or abs(q[n]-latest[n])>1e-6 for n in names)
                   for _,q,_ in self._samples):
                return False
            self._samples.clear()
            self._samples.append((now,dict(latest),dict.fromkeys(latest,0.)))
            self._last_query=self._playback=None
            return True

    def sample_state(self, now):
        """Return coherent position/velocity references; stale data means rest."""
        at = now-self.delay_sec
        with self._lock:
            if not self._samples:
                return {}, {}
            rate=1.
            if self.continuous:
                # After a feedback gap, resume the recorded path instead of
                # jumping to "now". Catch up gently, at most 5% faster.
                # A changed delay (adaptive source period) is absorbed by a
                # 0.95-1.05x playback rate; playback never moves backwards.
                at=min(at,self._samples[-1][0])
                if self._last_query is not None and now>=self._last_query:
                    dt=now-self._last_query
                    at=min(max(at,self._playback+.95*dt),self._playback+1.05*dt,
                           max(self._playback,self._samples[-1][0]))
                    rate=(at-self._playback)/dt if dt>0 else 0.
                self._last_query,self._playback=now,at
            while len(self._samples)>(3 if self.cubic else 2) and self._samples[2 if self.cubic else 1][0]<=at:
                self._samples.popleft()
            index=0
            if self.cubic and len(self._samples)>2 and self._samples[1][0]<=at:
                index=1
            t0, q0, v0 = self._samples[index]
            # Exactly on a knot is the segment start (Hermite slope), not "before".
            if len(self._samples)==1 or at<t0:
                return dict(q0), dict.fromkeys(q0, 0.)
            t1, q1, v1 = self._samples[index+1]
            if at>=t1:
                return dict(q1), dict.fromkeys(q1, 0.)  # stale: hold, never extrapolate
            ratio = (at-t0)/(t1-t0)
            position={name:q0[name]+ratio*(value-q0[name]) if name in q0 else value
                      for name,value in q1.items()}
            velocity={name:(value-q0[name])/(t1-t0) if name in q0 else 0.
                      for name,value in q1.items()}
            if self.cubic:
                dt=t1-t0;u=ratio
                def slope(neighbor,name,current):
                    if neighbor<0 or neighbor>=len(self._samples):return 0.
                    tn,qn,_=self._samples[neighbor]
                    if name not in qn:return 0.
                    other=(q0[name]-qn[name])/(t0-tn) if neighbor<index else (qn[name]-q1[name])/(tn-t1)
                    # Harmonic slopes preserve monotonicity and prevent overshoot.
                    return 2*other*current/(other+current) if other*current>0 else 0.
                for name in q1:
                    if name not in q0:continue
                    d=velocity[name]
                    m0=v0.get(name,slope(index-1,name,d));m1=v1.get(name,slope(index+2,name,d))
                    if m0*d<=0:m0=0.
                    if m1*d<=0:m1=0.
                    # Clamp unequal-interval slopes to the monotone Hermite region.
                    if d:
                        length=(m0/d)**2+(m1/d)**2
                        if length>9:
                            factor=3/math.sqrt(length);m0*=factor;m1*=factor
                    position[name]=(2*u**3-3*u*u+1)*q0[name]+(u**3-2*u*u+u)*dt*m0+(-2*u**3+3*u*u)*q1[name]+(u**3-u*u)*dt*m1
                    velocity[name]=((6*u*u-6*u)*q0[name]+(3*u*u-4*u+1)*dt*m0+(-6*u*u+6*u)*q1[name]+(3*u*u-2*u)*dt*m1)/dt
            return position, {n:v*rate for n,v in velocity.items()}
