"""패키지 추가 없이 작은 직사각 비용 행렬의 최소 비용 일대일 배정."""
import math
from collections import Counter


def assign_candidates(candidates):
    """(score, object_id, detection_index). 더미 열은 연결 보류를 허용한다."""
    if not candidates:
        return []
    rows_count = Counter(pid for _, pid, _ in candidates)
    cols_count = Counter(index for _, _, index in candidates)
    independent = [(score, pid, index) for score, pid, index in candidates
                   if rows_count[pid] == 1 and cols_count[index] == 1]
    if independent:
        resolved = {pid for _, pid, _ in independent}
        return ([item for item in independent if item[0] > -1.25]
                + assign_candidates([item for item in candidates if item[1] not in resolved]))
    ids = list(dict.fromkeys(pid for _, pid, _ in candidates))
    indices = sorted({index for _, _, index in candidates})
    rows, cols = {pid: i for i, pid in enumerate(ids)}, {d: i for i, d in enumerate(indices)}
    costs = [[1e6]*len(indices)+[0.]*len(ids) for _ in ids]
    scores = {}
    for score, pid, index in candidates:
        # 유효 연결의 이익을 최대화한다. 새 ID 생성을 강제하지 않는다.
        costs[rows[pid]][cols[index]] = -(score+1.25)
        scores[pid, index] = score
    n, m = len(ids), len(costs[0])
    u, v, p, way = [0.]*(n+1), [0.]*(m+1), [0]*(m+1), [0]*(m+1)
    for i in range(1, n+1):
        p[0], j0 = i, 0
        minimum, used = [math.inf]*(m+1), [False]*(m+1)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], math.inf, 0
            for j in range(1, m+1):
                if used[j]:
                    continue
                cur = costs[i0-1][j-1]-u[i0]-v[j]
                if cur < minimum[j]:
                    minimum[j], way[j] = cur, j0
                if minimum[j] < delta:
                    delta, j1 = minimum[j], j
            for j in range(m+1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minimum[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0], j0 = p[j1], j1
    result = []
    for j in range(1, len(indices)+1):
        if p[j]:
            pid, index = ids[p[j]-1], indices[j-1]
            score = scores.get((pid, index))
            if score is not None and score > -1.25:
                result.append((score, pid, index))
    return result
