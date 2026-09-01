"""
핵심 분석 모듈
  - GPS: 이상치 제거 → 칼만 필터 → DBSCAN 안전구역 → 위험도 스코어
  - 낙상: 서버 측 보조 검증 + 이력 저장
"""

import math
import numpy as np
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from sklearn.cluster import DBSCAN


# ════════════════════════════════════════════
# 1. 좌표 유틸
# ════════════════════════════════════════════

EARTH_R = 6371000.0

def haversine(lat1, lon1, lat2, lon2) -> float:
    """두 좌표 사이 거리 (m)"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * EARTH_R * math.asin(math.sqrt(max(0, a)))


class LocalPlane:
    """위경도 ↔ 미터 평면 변환 (수 km 범위면 이 근사로 충분)"""
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.mx = 111320.0 * math.cos(math.radians(lat0))
        self.my = 110540.0

    def to_xy(self, lat, lon):
        return (lon - self.lon0) * self.mx, (lat - self.lat0) * self.my

    def to_latlon(self, x, y):
        return self.lat0 + y / self.my, self.lon0 + x / self.mx


# ════════════════════════════════════════════
# 2. GPS 이상치 제거
# ════════════════════════════════════════════

class OutlierGate:
    """
    두 가지 필터:
      A) 속도 필터  - 물리적으로 불가능한 순간이동 차단
      B) HDOP 필터  - GPS 정밀도가 나쁜 좌표 차단
    """
    MAX_WALK_SPEED = 3.0   # m/s 도보 기준
    SPEED_MARGIN   = 3.0   # 여유 배수 (GPS 오차 허용)
    HDOP_LIMIT     = 8.0   # HDOP 이 이상이면 신뢰 불가

    # 속도 초과 횟수 카운터 (차량 이동 판별용)
    VEHICLE_COUNT_THRESH = 3    # 연속 N회 고속 → 차량 모드 진입
    VEHICLE_EXIT_THRESH  = 4    # 연속 N회 저속 → 차량 모드 해제 (히스테리시스)

    def __init__(self):
        self.prev = None             # (lat, lon, ts)
        self._high_speed_count = 0   # 연속 고속 횟수
        self._low_speed_count  = 0   # 연속 저속 횟수 (차량 해제용)
        self.move_mode = "walk"      # "walk" | "vehicle"

    def check(self, lat, lon, ts: datetime, hdop: float):
        """
        반환: (accept: bool, reason: str, move_mode: str)
        ★ 차량 이동은 폐기하지 않고 move_mode="vehicle"로 플래그
        """
        # (0, 0) 좌표 명시적 차단
        if lat == 0.0 and lon == 0.0:
            return False, "zero_coord", self.move_mode

        # HDOP 필터
        if hdop is not None and hdop > self.HDOP_LIMIT:
            return False, f"hdop_too_high_{hdop:.1f}", self.move_mode

        if self.prev is not None:
            plat, plon, pts = self.prev
            dt = (ts - pts).total_seconds()
            if dt <= 0:
                return False, "non_monotonic_time", self.move_mode

            speed = haversine(plat, plon, lat, lon) / dt

            if speed > self.MAX_WALK_SPEED * self.SPEED_MARGIN:
                self._high_speed_count += 1
                self._low_speed_count = 0

                # ★ 중요: 폐기하더라도 prev는 갱신해야 한다.
                #   갱신하지 않으면 다음 비교가 '더 오래된 기준점'과 이뤄져
                #   시간 간격이 커지고 계산 속도가 낮아진다.
                #   그 결과 차량으로 계속 달리는데도 카운터가 3에 도달하지 못해
                #   차량 이동을 영원히 놓치게 된다.
                self.prev = (lat, lon, ts)

                # 이미 차량 모드면 고속이 정상이므로 그대로 통과
                if self.move_mode == "vehicle":
                    return True, "vehicle_mode", "vehicle"

                if self._high_speed_count >= self.VEHICLE_COUNT_THRESH:
                    # 연속 N회 고속 → 차량 이동으로 재분류 (폐기하지 않음)
                    self.move_mode = "vehicle"
                    return True, "vehicle_mode", "vehicle"

                # 1~2회: 일단 GPS 튐으로 보고 폐기
                return False, f"speed_outlier_{speed:.1f}mps", self.move_mode

            else:
                self._high_speed_count = 0

                # ★ 히스테리시스: 차량 모드는 한 번 저속이 나왔다고 바로 풀지 않는다.
                #   신호등·정체로 잠깐 느려질 때마다 모드가 튀면
                #   다음 고속 좌표가 또 폐기되어 추적이 끊긴다.
                if self.move_mode == "vehicle":
                    self._low_speed_count += 1
                    if self._low_speed_count >= self.VEHICLE_EXIT_THRESH:
                        self.move_mode = "walk"
                        self._low_speed_count = 0
                else:
                    self._low_speed_count = 0

        self.prev = (lat, lon, ts)
        return True, "ok", self.move_mode


# ════════════════════════════════════════════
# 3. 칼만 필터 (등속 모델)
# ════════════════════════════════════════════

class KalmanCV:
    """
    상태 벡터: [x, y, vx, vy]  (미터 평면)
    ★ HDOP으로 측정 노이즈 R을 동적 조정 → 불량 좌표는 덜 믿음
    """
    def __init__(self, x0, y0, q=0.3, base_r=25.0):
        self.x = np.array([x0, y0, 0.0, 0.0])
        self.P = np.eye(4) * 100.0
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        self.q      = q        # 프로세스 노이즈 (움직임 불확실성)
        self.base_r = base_r   # 기본 측정 노이즈 (GPS 오차 ~5m → 5²)

    def _F(self, dt):
        return np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], dtype=float)

    def _Q(self, dt):
        g = np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]], dtype=float)
        return g @ g.T * self.q

    def predict(self, dt):
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

    def update(self, zx, zy, hdop=1.0):
        # HDOP이 나쁠수록 R이 커져서 측정을 덜 반영
        r = self.base_r * max(1.0, hdop) ** 2
        R = np.eye(2) * r
        z = np.array([zx, zy])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos(self):
        return self.x[0], self.x[1]

    @property
    def speed_mps(self):
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def uncertainty_m(self):
        return float(math.sqrt(max(self.P[0,0], 0) + max(self.P[1,1], 0)))


# ════════════════════════════════════════════
# 4. 안전구역 학습 (DBSCAN)
# ════════════════════════════════════════════

@dataclass
class SafeZone:
    places: list = field(default_factory=list)  # [(cx, cy, radius, visits)]
    routes: list = field(default_factory=list)  # [(x1,y1,x2,y2)]
    route_buffer: float = 30.0                  # 경로 폭 허용 (m, 중심선 기준 좌우)

    @property
    def is_trained(self) -> bool:
        """안전구역이 학습됐는가. 학습 전에는 이탈 판단을 하면 안 된다."""
        return len(self.places) > 0 or len(self.routes) > 0

    def dist_outside(self, x, y) -> float:
        """
        안전구역 경계로부터의 이탈 거리(m). 안에 있으면 0.
        ★ 학습 전(콜드 스타트)에는 0을 반환해 판단을 보류한다.
          안 그러면 모든 위치가 '이탈'로 잡혀 알림 폭탄이 된다.
        """
        if not self.is_trained:
            return 0.0

        best = float("inf")

        for cx, cy, r, _ in self.places:
            d = math.hypot(x-cx, y-cy) - r
            best = min(best, d)
            if best <= 0:
                return 0.0

        for x1, y1, x2, y2 in self.routes:
            d = _seg_dist(x, y, x1, y1, x2, y2) - self.route_buffer
            best = min(best, d)
            if best <= 0:
                return 0.0

        return max(0.0, best if best != float("inf") else 9999.0)


def _seg_dist(px, py, x1, y1, x2, y2) -> float:
    dx, dy = x2-x1, y2-y1
    L2 = dx*dx + dy*dy
    if L2 == 0:
        return math.hypot(px-x1, py-y1)
    t = max(0.0, min(1.0, ((px-x1)*dx + (py-y1)*dy) / L2))
    return math.hypot(px-(x1+t*dx), py-(y1+t*dy))


def learn_safezone(points, plane: LocalPlane,
                   stay_radius=40.0, stay_minutes=8,
                   eps=60.0, min_samples=3) -> SafeZone:
    """
    points: [(lat, lon, datetime), ...]  - 정상 구간(risk<30) 로그
    반환: SafeZone

    ★ 학습은 매일 새벽 배치로만 실행. 실시간 요청마다 돌리지 말 것.
    """
    xy = [(plane.to_xy(lat, lon), ts) for lat, lon, ts in points]

    # ── Stay-point 추출 ───────────────────
    stays = []
    n, i = len(xy), 0
    while i < n:
        j = i + 1
        while j < n:
            d = math.hypot(xy[j][0][0]-xy[i][0][0], xy[j][0][1]-xy[i][0][1])
            if d > stay_radius:
                break
            j += 1
        dur = (xy[j-1][1] - xy[i][1]).total_seconds() / 60.0
        if dur >= stay_minutes:
            cx = float(np.mean([xy[k][0][0] for k in range(i, j)]))
            cy = float(np.mean([xy[k][0][1] for k in range(i, j)]))
            stays.append((cx, cy))
            i = j
        else:
            i += 1

    # ── DBSCAN 클러스터링 ─────────────────
    places = []
    if len(stays) >= min_samples:
        arr = np.array(stays)
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(arr)
        for lb in set(labels):
            if lb == -1:
                continue
            grp = arr[labels == lb]
            cx, cy = grp[:,0].mean(), grp[:,1].mean()
            spread = float(np.max(np.hypot(grp[:,0]-cx, grp[:,1]-cy)))
            places.append((float(cx), float(cy), max(70.0, spread+40.0), len(grp)))

    # ── 경로 선분 추출 ───────────────────
    routes = []
    for (ax, ay), (bx, by) in zip(
        [p[0] for p in xy], [p[0] for p in xy[1:]]
    ):
        d = math.hypot(bx-ax, by-ay)
        dt = (xy[xy.index(next(p for p in xy if p[0] == (bx,by)))][1]
              - xy[xy.index(next(p for p in xy if p[0] == (ax,ay)))][1]
              ).total_seconds() if False else 30  # 단순화
        if 5.0 < d < 400.0:
            routes.append((float(ax), float(ay), float(bx), float(by)))

    return SafeZone(places=places, routes=routes)


def learn_safezone_simple(points, plane: LocalPlane,
                          stay_radius=40.0, stay_minutes=8,
                          eps=60.0, min_samples=3) -> SafeZone:
    """
    learn_safezone의 단순화 버전 (타임스탬프 인덱스 문제 없음)
    실제 서버에서는 이 버전을 사용.
    """
    xys = []
    tss = []
    for lat, lon, ts in points:
        x, y = plane.to_xy(lat, lon)
        xys.append((x, y))
        tss.append(ts)

    # Stay-point 추출
    stays = []
    n, i = len(xys), 0
    while i < n:
        j = i + 1
        while j < n:
            d = math.hypot(xys[j][0]-xys[i][0], xys[j][1]-xys[i][1])
            if d > stay_radius:
                break
            j += 1
        dur = (tss[min(j-1, n-1)] - tss[i]).total_seconds() / 60.0
        if dur >= stay_minutes:
            cx = float(np.mean([xys[k][0] for k in range(i, j)]))
            cy = float(np.mean([xys[k][1] for k in range(i, j)]))
            stays.append((cx, cy))
            i = j
        else:
            i += 1

    # DBSCAN
    places = []
    if len(stays) >= min_samples:
        arr = np.array(stays)
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(arr)
        for lb in set(labels):
            if lb == -1:
                continue
            grp = arr[labels == lb]
            cx, cy = float(grp[:,0].mean()), float(grp[:,1].mean())
            spread = float(np.max(np.hypot(grp[:,0]-cx, grp[:,1]-cy)))
            places.append((cx, cy, max(70.0, spread+40.0), int(len(grp))))

    # 경로
    routes = []
    for i in range(len(xys)-1):
        ax, ay = xys[i]
        bx, by = xys[i+1]
        d = math.hypot(bx-ax, by-ay)
        if 5.0 < d < 400.0:
            routes.append((ax, ay, bx, by))

    return SafeZone(places=places, routes=routes)


# ════════════════════════════════════════════
# 5. 위험도 스코어
# ════════════════════════════════════════════

def _sigmoid(v, center, scale):
    return 1.0 / (1.0 + math.exp(-(v - center) / scale))


class RiskScorer:
    """
    위치 기반 위험도 0~100점
    공간(70%) + 방향(30%) — 시간 요소 없음
    위치에서 얼마나 멀어졌는가, 지금도 멀어지는 중인가만 본다
    """
    W_SPATIAL   = 0.7   # 안전구역에서 얼마나 멀어졌는가
    W_DIRECTION = 0.3   # 지금 방향이 안으로 향하는가 밖으로 향하는가
    HIST_LEN    = 8     # 방향 판단에 쓸 최근 거리 샘플 수
    CONFIRM_N   = 3     # 시간 요소가 없어서 GPS 튐 오탐 방지를 위해 3회로 올림
    ALERT_COOLDOWN_MIN = 10

    def __init__(self):
        self.dist_hist   = []
        self._pending    = None
        self.level       = "정상"
        self._last_alert = {}

    def score(self, dist_out: float, ts: datetime,
              night=False, vehicle=False, uncertainty=0.0):
        # 안전구역 안이면 즉시 초기화
        if dist_out <= 0:
            self.dist_hist.clear()
            self._pending = None
            self.level    = "정상"
            return 0.0, {"spatial": 0, "direction": 0}

        self.dist_hist.append(dist_out)
        self.dist_hist = self.dist_hist[-self.HIST_LEN:]

        # (a) 공간 점수: 이탈 거리 기반 시그모이드
        #     100m 부근부터 올라가기 시작, 200m 넘으면 거의 만점
        spatial = _sigmoid(dist_out, 150.0, 50.0)

        # (b) 방향 점수: 최근 거리 변화 추세
        #     계속 멀어지는 중 → 1.0 / 불명확 → 0.5 / 돌아오는 중 → 0.15
        if len(self.dist_hist) < 3:
            direction = 0.5
        else:
            slope = float(np.polyfit(range(len(self.dist_hist)),
                                     self.dist_hist, 1)[0])
            direction = 1.0 if slope > 1 else (0.15 if slope < -1 else 0.5)

        risk = 100 * (self.W_SPATIAL * spatial + self.W_DIRECTION * direction)

        # 맥락 보정
        if night:            risk *= 1.3   # 야간 배회는 더 위험
        if vehicle:          risk *= 1.5   # 차량 이동은 수색 범위 폭증
        if uncertainty > 80: risk *= 0.7   # GPS 불확실하면 판단 보류

        risk = float(min(100.0, max(0.0, risk)))

        # 등급 확정: 3회 연속 같은 등급이어야 확정 (순간 GPS 튐 방지)
        raw_level = _risk_level(risk)
        if self._pending and self._pending[0] == raw_level:
            self._pending = (raw_level, self._pending[1] + 1)
        else:
            self._pending = (raw_level, 1)
        if self._pending[1] >= self.CONFIRM_N:
            self.level = raw_level

        # 알림 쿨다운
        alert = False
        if self.level in ("주의", "경고"):
            last = self._last_alert.get(self.level)
            if last is None or (ts - last).total_seconds() / 60 >= self.ALERT_COOLDOWN_MIN:
                self._last_alert[self.level] = ts
                alert = True

        return risk, {
            "spatial":   round(spatial, 2),
            "direction": round(direction, 2),
            "alert":     alert,
        }


def _risk_level(risk: float) -> str:
    if risk >= 85: return "경고"
    if risk >= 60: return "주의"
    if risk >= 30: return "관심"
    return "정상"


def next_interval_sec(level: str) -> int:
    """위험도에 따른 다음 전송 주기 (적응형 전력 관리)"""
    return {"경고":5, "주의":15, "관심":30, "정상":60}.get(level, 60)


# ════════════════════════════════════════════
# 6. 기기 1대분 파이프라인
# ════════════════════════════════════════════

class VestTracker:
    """
    GPS 좌표 1개가 들어올 때마다 feed()를 호출.
    칼만 필터 상태를 유지하므로 반드시 기기 하나에 인스턴스 하나.
    """
    def __init__(self, device_id: str, home_lat: float, home_lon: float,
                 safezone: SafeZone = None):
        self.device_id = device_id
        self.plane     = LocalPlane(home_lat, home_lon)
        self.gate      = OutlierGate()
        self.kf        = None
        self.safezone  = safezone or SafeZone()
        self.scorer    = RiskScorer()
        self.last_ts   = None
        self.trail     = []  # (lat, lon, ts, risk)
        self._speed_hist = []

        # 정지 하트비트용
        self._last_send_ts = datetime.now()
        self.HEARTBEAT_SEC = 120  # 2분마다 강제 전송

    def set_safezone(self, sz: SafeZone):
        self.safezone = sz

    def feed(self, lat: float, lon: float, ts: datetime,
             hdop: float = 1.0, speed_kmh: float = None):
        """
        반환: dict
          status: "ok" | "rejected" | "heartbeat"
          ...위험도, 등급, 다음 전송 주기 등
        """
        now = ts
        ok, reason, move_mode = self.gate.check(lat, lon, ts, hdop)

        if not ok:
            # ★ 정지 하트비트 - 이동이 없어도 2분마다 위치 전송
            if (now - self._last_send_ts).total_seconds() >= self.HEARTBEAT_SEC:
                self._last_send_ts = now
                return {
                    "status": "heartbeat",
                    "reason": reason,
                    "lat": lat, "lon": lon,
                    "risk": 0.0, "level": self.scorer.level,
                    "is_stationary": True,
                    "next_interval_sec": next_interval_sec(self.scorer.level),
                }
            return {"status": "rejected", "reason": reason}

        # 미터 평면으로 변환
        x, y = self.plane.to_xy(lat, lon)

        # 칼만 필터
        if self.kf is None:
            self.kf = KalmanCV(x, y)
        else:
            dt = max(1.0, (ts - self.last_ts).total_seconds())
            self.kf.predict(dt)
            self.kf.update(x, y, hdop)
        self.last_ts = ts

        fx, fy = self.kf.pos
        flat, flon = self.plane.to_latlon(fx, fy)

        # 차량 판별 (순간 속도 튐에 흔들리지 않도록 중앙값 사용)
        self._speed_hist = (self._speed_hist + [self.kf.speed_mps])[-5:]
        vehicle = (len(self._speed_hist) >= 3
                   and float(np.median(self._speed_hist)) > 6.0)
        if move_mode == "vehicle":
            vehicle = True

        # 안전구역 이탈 거리
        dist_out = self.safezone.dist_outside(fx, fy)

        # 야간 여부
        night = ts.hour >= 22 or ts.hour < 6

        # 위험도
        risk, parts = self.scorer.score(
            dist_out, ts, night, vehicle, self.kf.uncertainty_m)

        self.trail.append((flat, flon, ts, risk))
        self._last_send_ts = now

        result = {
            "status":        "ok",
            "lat":           round(flat, 6),
            "lon":           round(flon, 6),
            "raw_lat":       lat,
            "raw_lon":       lon,
            "speed_mps":     round(self.kf.speed_mps, 2),
            "uncertainty_m": round(self.kf.uncertainty_m, 1),
            "dist_outside_m":round(dist_out, 1),
            "risk":          round(risk, 1),
            "level":         self.scorer.level,
            "breakdown":     parts,
            "night":         night,
            "vehicle":       vehicle,
            "move_mode":     move_mode,
            "safezone_ready": self.safezone.is_trained,   # False면 아직 학습 중
            "next_interval_sec": next_interval_sec(self.scorer.level),
        }
        return result