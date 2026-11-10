import itertools
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assignment import assign_candidates


def test_global_assignment_preserves_two_tracks_when_greedy_would_lose_one():
    candidates = [(2., 'a', 0), (1.8, 'a', 1), (1.9, 'b', 0)]
    result = assign_candidates(candidates)
    assert {(pid, index) for _, pid, index in result} == {('a', 1), ('b', 0)}
    assert assign_candidates([]) == []


def test_assignment_matches_exhaustive_optimum_with_optional_unmatched_rows():
    rng = random.Random(42)
    for n, m in itertools.product(range(1, 5), repeat=2):
        values = {(str(i), j): rng.uniform(-1.2, 2) for i in range(n) for j in range(m) if rng.random() > .3}
        candidates = [(score, pid, index) for (pid, index), score in values.items()]
        actual = assign_candidates(candidates)
        best = 0.
        for choices in itertools.product([None]+list(range(m)), repeat=n):
            used = [j for j in choices if j is not None]
            if len(set(used)) != len(used):
                continue
            if any((str(i), j) not in values for i, j in enumerate(choices) if j is not None):
                continue
            best = max(best, sum(values[str(i), j]+1.25 for i, j in enumerate(choices) if j is not None))
        assert abs(sum(score+1.25 for score, _, _ in actual)-best) < 1e-8
