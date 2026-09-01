"""
경로 생성 모듈
===============
두 지점 사이의 이동 경로를 만든다. 두 가지 방식을 지원.

  1) 도로망 기반 (road)  - OpenStreetMap 보행로를 따라 실제 최단경로
  2) 직선 기반 (line)    - 직선을 살짝 휜 곡선 (인터넷 불필요, 빠름)

도로망은 최초 1회만 다운로드하고 pkl로 캐시하므로,
이후에는 인터넷 없이도 동작한다.

경유지(waypoint)와 함께 쓰면 "이 길목을 지나서 저기로" 같은
구체적인 경로를 지정할 수 있다.
"""

import math
import os
import pickle
import sys


# ══════════════════════════════════════════════
# 도로망 로드 / 캐시
# ══════════════════════════════════════════════

_graph_cache = {}   # 프로세스 내 메모리 캐시


def load_road_graph(center_lat, center_lon, radius_m=3000,
                    cache_path="road_graph.pkl", verbose=True):
    """
    보행 도로망을 가져온다.

    최초 실행 시 OpenStreetMap에서 다운로드하고 pkl로 저장한다.
    이후에는 캐시 파일을 읽으므로 인터넷이 필요 없다.

    radius_m: 생활반경. 모든 장소와 배회 지점을 포함할 만큼 넉넉히.
    """
    key = (round(center_lat, 4), round(center_lon, 4), radius_m)
    if key in _graph_cache:
        return _graph_cache[key]

    if os.path.exists(cache_path):
        if verbose:
            print(f"  도로망 캐시 로드: {cache_path}")
        with open(cache_path, "rb") as f:
            G = pickle.load(f)
        _graph_cache[key] = G
        return G

    try:
        import osmnx as ox
    except ImportError:
        print("  osmnx가 설치되지 않았습니다:  pip install osmnx networkx")
        return None

    if verbose:
        print(f"  OpenStreetMap에서 도로망 다운로드 중 (반경 {radius_m}m)...")
        print("  최초 1회만 걸리며, 이후에는 캐시를 사용합니다.")

    try:
        G = ox.graph_from_point(
            (center_lat, center_lon),
            dist=radius_m,
            network_type="walk",     # 차도가 아닌 보행로 기준
            simplify=True,
        )
    except Exception as e:
        print(f"  도로망 다운로드 실패: {e}")
        print("  인터넷 연결을 확인하거나 --route line 으로 실행하세요.")
        return None

    with open(cache_path, "wb") as f:
        pickle.dump(G, f)
    if verbose:
        print(f"  저장 완료: {cache_path} "
              f"(노드 {len(G.nodes):,} / 엣지 {len(G.edges):,})")

    _graph_cache[key] = G
    return G


# ══════════════════════════════════════════════
# 도로망 최단경로
# ══════════════════════════════════════════════

def road_path(G, lat1, lon1, lat2, lon2):
    """
    두 지점 사이의 보행 최단경로를 좌표 리스트로 반환.
    실패하면 None (호출측에서 직선으로 폴백).

    반환: [(lat, lon), ...]
    """
    try:
        import osmnx as ox
        import networkx as nx
    except ImportError:
        return None

    try:
        n1 = ox.distance.nearest_nodes(G, X=lon1, Y=lat1)
        n2 = ox.distance.nearest_nodes(G, X=lon2, Y=lat2)

        if n1 == n2:
            return [(lat1, lon1), (lat2, lon2)]

        route = nx.shortest_path(G, n1, n2, weight="length")
        coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route]

        # 출발/도착 실제 좌표를 앞뒤에 붙인다
        # (도로 노드는 실제 위치와 수십 m 떨어져 있을 수 있음)
        return [(lat1, lon1)] + coords + [(lat2, lon2)]

    except Exception:
        return None


# ══════════════════════════════════════════════
# 경로를 등간격 좌표로 리샘플링
# ══════════════════════════════════════════════

def resample_path(coords, plane, speed, interval, noise,
                  rng, jitter_ratio=0.0):
    """
    꺾인 경로(폴리라인)를 일정 시간 간격의 GPS 좌표로 변환한다.

    coords       : [(lat, lon), ...] 경로 꼭짓점
    speed        : m/s
    interval     : 초 단위 샘플링 주기
    noise        : GPS 노이즈 표준편차 (m)
    jitter_ratio : 경로에서 벗어나는 정도 (0이면 경로 그대로)

    반환: [(lat, lon), ...] 등간격 좌표
    """
    if len(coords) < 2:
        return []

    # 미터 평면으로 변환
    pts = [plane.to_xy(la, lo) for la, lo in coords]

    # 각 구간 길이와 누적 거리
    seg_len = []
    for a, b in zip(pts, pts[1:]):
        seg_len.append(math.hypot(b[0]-a[0], b[1]-a[1]))
    total = sum(seg_len)

    if total < 1:
        return []

    step_dist = speed * interval          # 한 샘플당 이동 거리
    n_steps   = max(2, int(total / step_dist))

    out = []
    for i in range(n_steps + 1):
        target = (i / n_steps) * total

        # target 거리에 해당하는 구간 찾기
        acc = 0.0
        for k, L in enumerate(seg_len):
            if acc + L >= target or k == len(seg_len) - 1:
                t = (target - acc) / L if L > 0 else 0.0
                t = max(0.0, min(1.0, t))
                ax, ay = pts[k]
                bx, by = pts[k+1]
                x = ax + (bx - ax) * t
                y = ay + (by - ay) * t

                # 경로 이탈 지터 (0이면 경로 정확히 따라감)
                if jitter_ratio > 0 and L > 0:
                    # 진행 방향의 수직 방향으로 오프셋
                    dx, dy = (bx-ax)/L, (by-ay)/L
                    off = rng.gauss(0, total * jitter_ratio * 0.1)
                    x += -dy * off
                    y +=  dx * off

                # GPS 측정 노이즈
                x += rng.gauss(0, noise)
                y += rng.gauss(0, noise)

                out.append(plane.to_latlon(x, y))
                break
            acc += L

    return out


# ══════════════════════════════════════════════
# 직선(곡선) 경로 - 폴백용
# ══════════════════════════════════════════════

def line_path(lat1, lon1, lat2, lon2, plane, bend=0.05, arc_points=12):
    """
    직선을 살짝 휜 곡선 경로의 꼭짓점을 반환.
    도로망을 못 쓸 때 폴백으로 사용.
    """
    x1, y1 = plane.to_xy(lat1, lon1)
    x2, y2 = plane.to_xy(lat2, lon2)
    dist = math.hypot(x2-x1, y2-y1)
    if dist < 1:
        return [(lat1, lon1), (lat2, lon2)]

    coords = []
    for i in range(arc_points + 1):
        t = i / arc_points
        off = math.sin(t * math.pi) * dist * bend
        x = x1 + (x2-x1)*t - (y2-y1)/dist*off
        y = y1 + (y2-y1)*t + (x2-x1)/dist*off
        coords.append(plane.to_latlon(x, y))
    return coords


# ══════════════════════════════════════════════
# 통합 인터페이스
# ══════════════════════════════════════════════

class RouteBuilder:
    """
    경로 생성기.

    mode="road" 면 도로망 최단경로, "line" 이면 직선 곡선.
    도로망 조회에 실패하면 자동으로 직선으로 폴백한다.
    """

    def __init__(self, plane, mode="road", graph=None,
                 bend=0.05, verbose=True):
        self.plane   = plane
        self.mode    = mode
        self.G       = graph
        self.bend    = bend
        self.verbose = verbose
        self.fallback_count = 0
        self.road_count     = 0

    def build(self, lat1, lon1, lat2, lon2):
        """두 지점 사이 경로 꼭짓점 반환: [(lat, lon), ...]"""
        if self.mode == "road" and self.G is not None:
            coords = road_path(self.G, lat1, lon1, lat2, lon2)
            if coords and len(coords) >= 2:
                self.road_count += 1
                return coords
            self.fallback_count += 1

        return line_path(lat1, lon1, lat2, lon2, self.plane, self.bend)

    def report(self):
        if self.mode == "road":
            total = self.road_count + self.fallback_count
            if total:
                print(f"  경로 생성: 도로망 {self.road_count}건 / "
                      f"직선 폴백 {self.fallback_count}건")
