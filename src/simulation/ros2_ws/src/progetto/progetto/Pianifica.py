import heapq
from math import sqrt

SQRT2 = sqrt(2.0)

def astar_8conn(map_data, width, height, start, goal, bbox=None, allow_unknown=False):
    """
    start, goal: (gx, gy)
    bbox: (minx, maxx, miny, maxy) oppure None
    allow_unknown: se True tratta -1 come libero
    ritorna: lista di (x,y) oppure None
    """

    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def in_bbox(x, y):
        if bbox is None:
            return True
        minx, maxx, miny, maxy = bbox
        return (minx <= x <= maxx) and (miny <= y <= maxy)

    def is_free(x, y):
        v = map_data[y * width + x]  # -1 unknown, 0 free, 100 occupied (tipico)
        if v == 0:
            return True
        if allow_unknown and v == -1:
            return True
        return False

    def h(a, b):  # euristica octile (adatta a 8-conn con costi 1 e sqrt(2))
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

    if not in_bounds(*start) or not in_bounds(*goal):
        return None
    if not in_bbox(*start) or not in_bbox(*goal):
        return None
    if not is_free(*start) or not is_free(*goal):
        return None

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQRT2), (1, -1, SQRT2), (-1,  1, SQRT2), (-1, -1, SQRT2)]
    open_heap = []
    heapq.heappush(open_heap, (h(start, goal), 0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:  # reconstruct
            path = [cur]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path
        closed.add(cur)
        cx, cy = cur
        for dx, dy, cost in moves:
            nx, ny = cx + dx, cy + dy
            if not in_bounds(nx, ny) or not in_bbox(nx, ny):
                continue
            if not is_free(nx, ny):
                continue
            # evita "taglio dell'angolo" sui diagonali:
            if dx != 0 and dy != 0:
                if not (is_free(cx + dx, cy) and is_free(cx, cy + dy)):
                    continue
            nb = (nx, ny)
            tentative_g = g + cost
            if tentative_g < g_score.get(nb, float("inf")):
                came_from[nb] = cur
                g_score[nb] = tentative_g
                heapq.heappush(open_heap, (tentative_g + h(nb, goal), tentative_g, nb))
    return None
