"""
Firebase 연동 브리지 (Realtime Database)
=========================================
친구 앱이 지도에서 찍은 안전구역을 읽어 경로를 만들고,
좌표가 들어오면 이탈 여부를 판단해 다시 Firebase에 기록한다.

Firebase 구조
-------------
  safe_zones/                    ← 친구 앱이 쓰는 곳 (읽기)
    zone_1787210846296/
      id        : "zone_1787210846296"
      latitude  : 37.34241389880134
      longitude : 126.72939039766788
      radius    : 460

  sensor_data/test/              ← 좌표 수신 + 결과 기록
    latitude, longitude          (읽음)
    is_out_of_path               (씀) ★ 이탈 여부
    risk, dist_out_m             (씀)

  safezone/                      ← 지도에 그리라고 올려주는 것 (씀)
    zones/    원 그리기용
    links/    경로 선 그리기용 (좌표 배열)
    buffer    경로 폭

  status/                        ← 서버 상태 (씀)
    safezone_ready, zone_count, link_count, updated_at

안전구역 3겹
-----------
  1) 등록 구역  — 친구 앱이 찍은 원
  2) 연결 경로  — 구역 사이를 잇는 경로 (폭 120m)
                  OSM 보행 도로망 최단경로, 실패 시 직선
  3) 학습 궤적  — 실제 지나간 길 (폭 60m)

  셋 중 하나라도 안에 들어가면 정상으로 판단한다.

명령어
------
  --check-zones   친구가 찍은 안전구역을 제대로 읽는지 확인 ★
  --status        전체 상태 확인
  --build-links   구역 사이 연결 경로 생성 (OSM 도로망)
  --publish       안전구역을 지도용으로 Firebase에 올림
  --collect       좌표 수집만 (판단 안 함)
  --train         수집분으로 자동 학습
  --test          단발 테스트
  (인자 없음)      실시간 감시

설치
----
  pip install firebase-admin numpy scikit-learn osmnx networkx
"""

import argparse
import json
import math
import os
import sqlite3
import sys
from contextlib import contextmanager
from datetime import datetime, timedelta

import firebase_admin
from firebase_admin import credentials, db

from core import (VestTracker, LocalPlane, SafeZone,
                  learn_safezone_simple)


# ══════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════

SERVICE_ACCOUNT = "smartvest-3069b-firebase-adminsdk-fbsvc-8063b56e86.json"
DATABASE_URL    = "https://smartvest-3069b-default-rtdb.firebaseio.com"

# ★ 친구 앱이 안전구역을 저장하는 경로
ZONES_PATH = "safe_zones"

SENSOR_PATH       = "sensor_data/test"   # 좌표 수신 + 결과 기록
STATUS_PATH       = "status"             # 서버 상태
SAFEZONE_PUB_PATH = "safezone"           # 지도용 업로드

LAT_FIELD = "latitude"
LON_FIELD = "longitude"
OUT_FIELD = "is_out_of_path"

OUT_LEVELS = ("주의", "경고")

DEFAULT_ZONE_RADIUS  = 80.0    # radius 없을 때 기본값
# ── 안전구역 버퍼 ─────────────────────────────
# 중심선에서 좌우 각 N미터. 총 폭은 2N이 된다.
#
# 근거:
#   칼만 필터 통과 후 GPS 오차   ~5m   (원본 60m대 → 필터 후 6m대)
#   도로 중심선 ~ 실제 인도      ~10m  (왕복 4차선 기준)
#   여유                        ~15m
#   ─────────────────────────────────
#                               30m   (총 폭 60m)
#
# ★ OSM이 실제 다니는 길과 다른 길을 고른 경우는
#   버퍼를 넓혀서 덮으려 하지 말 것. 그러면 안전구역만 커지고
#   정작 배회를 못 잡는다. 자동 학습(--train)이 실제 궤적을
#   쌓아 해결하는 것이 맞다.
DEFAULT_ROUTE_BUFFER = 30.0    # 학습 궤적 폭 (실제 지나간 길)
LINK_BUFFER_M        = 30.0    # 연결 경로 폭 (OSM 최단경로)

ROAD_RADIUS_M = 3000           # 도로망 다운로드 반경

DEVICE_ID    = "VEST-001"
DB_PATH      = "vest_local.db"
LEARNED_PATH = "learned_zone.json"
LINK_PATH    = "linked_routes.json"
GRAPH_CACHE  = "road_graph.pkl"

# 안전구역이 하나도 없을 때만 쓰는 최후 기준점
FALLBACK_HOME = (37.3459, 126.7367)


# ══════════════════════════════════════════════
# Firebase 초기화
# ══════════════════════════════════════════════

def init_firebase():
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT)
        firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
    except FileNotFoundError:
        print(f"[오류] 키 파일 없음: {SERVICE_ACCOUNT}")
        sys.exit(1)
    except ValueError:
        pass


# ══════════════════════════════════════════════
# 안전구역 읽기 (친구 앱이 쓴 값)
# ══════════════════════════════════════════════

def _short_name(key):
    """zone_1787210846296 → zone_…846296 처럼 짧게"""
    k = str(key)
    return k if len(k) <= 16 else f"{k[:5]}…{k[-6:]}"


def read_zones():
    """
    safe_zones 를 읽어 정규화한다.

    ★ 필드명이 latitude/longitude 인 점에 주의.
      혹시 lat/lon 으로 저장하는 버전이 섞여 있어도 읽히도록
      두 이름을 모두 허용한다.

    반환: {"zones": [{key, name, lat, lon, radius}, ...],
           "route_buffer": 60.0,
           "errors": [문제가 있던 항목 설명]}
    """
    raw = db.reference(ZONES_PATH).get()
    out = {"zones": [], "route_buffer": DEFAULT_ROUTE_BUFFER, "errors": []}

    if raw is None:
        return out

    if isinstance(raw, dict):
        items = list(raw.items())
    elif isinstance(raw, list):
        items = [(str(i), z) for i, z in enumerate(raw) if z]
    else:
        out["errors"].append(f"{ZONES_PATH} 형식이 올바르지 않습니다")
        return out

    for key, z in items:
        if not isinstance(z, dict):
            out["errors"].append(f"{key}: 객체가 아님")
            continue

        lat = z.get("latitude", z.get("lat"))
        lon = z.get("longitude", z.get("lon"))

        if lat is None or lon is None:
            have = ", ".join(z.keys())
            out["errors"].append(
                f"{key}: latitude/longitude 없음 (가진 필드: {have})")
            continue

        try:
            lat_f, lon_f = float(lat), float(lon)
        except (TypeError, ValueError):
            out["errors"].append(f"{key}: 좌표가 숫자가 아님 ({lat}, {lon})")
            continue

        if not (-90 <= lat_f <= 90) or not (-180 <= lon_f <= 180):
            out["errors"].append(
                f"{key}: 좌표 범위를 벗어남 ({lat_f}, {lon_f})")
            continue

        try:
            radius = float(z.get("radius", DEFAULT_ZONE_RADIUS))
        except (TypeError, ValueError):
            radius = DEFAULT_ZONE_RADIUS
            out["errors"].append(f"{key}: radius 형식 오류 → 기본값 사용")

        out["zones"].append({
            "key":    z.get("id", key),
            "name":   z.get("name") or _short_name(z.get("id", key)),
            "lat":    lat_f,
            "lon":    lon_f,
            "radius": radius,
        })

    # 순서를 안정시킨다 (경로 생성 결과가 매번 같게)
    out["zones"].sort(key=lambda z: (z["lat"], z["lon"]))
    return out


def origin_of(cfg):
    """평면 좌표계의 원점. 안전구역들의 중심을 쓴다."""
    zs = cfg.get("zones", [])
    if not zs:
        return FALLBACK_HOME
    return (sum(z["lat"] for z in zs) / len(zs),
            sum(z["lon"] for z in zs) / len(zs))


# ══════════════════════════════════════════════
# 로컬 DB — 좌표 수집
# ══════════════════════════════════════════════

@contextmanager
def get_db():
    con = sqlite3.connect(DB_PATH)
    con.row_factory = sqlite3.Row
    try:
        yield con
        con.commit()
    finally:
        con.close()


def init_db():
    with get_db() as con:
        con.execute("""
            CREATE TABLE IF NOT EXISTS points (
                id   INTEGER PRIMARY KEY AUTOINCREMENT,
                ts   TEXT, lat REAL, lon REAL, hdop REAL
            )""")
        con.execute("CREATE INDEX IF NOT EXISTS ix_ts ON points(ts)")


def save_point(lat, lon, hdop, ts=None):
    """직전 좌표와 5m 이내면 저장하지 않는다"""
    ts = ts or datetime.now()
    with get_db() as con:
        last = con.execute(
            "SELECT lat, lon FROM points ORDER BY id DESC LIMIT 1").fetchone()
        if last:
            d = math.hypot((lat - last["lat"]) * 110540,
                           (lon - last["lon"]) * 88000)
            if d < 5:
                return False
        con.execute("INSERT INTO points(ts,lat,lon,hdop) VALUES(?,?,?,?)",
                    (ts.isoformat(), lat, lon, hdop))
    return True


def load_points(days=30):
    since = (datetime.now() - timedelta(days=days)).isoformat()
    with get_db() as con:
        rows = con.execute(
            "SELECT lat,lon,ts FROM points WHERE ts>? ORDER BY ts",
            (since,)).fetchall()
    return [(r["lat"], r["lon"], datetime.fromisoformat(r["ts"]))
            for r in rows]


def count_points():
    with get_db() as con:
        return con.execute("SELECT COUNT(*) c FROM points").fetchone()["c"]


# ══════════════════════════════════════════════
# 자동 학습 결과 저장 / 로드
# ══════════════════════════════════════════════

def save_learned(sz, plane):
    data = {
        "trained_at": datetime.now().isoformat(),
        "places": [
            {"lat": plane.to_latlon(cx, cy)[0],
             "lon": plane.to_latlon(cx, cy)[1],
             "radius": r, "visits": v}
            for cx, cy, r, v in sz.places
        ],
        "routes": [
            [*plane.to_latlon(x1, y1), *plane.to_latlon(x2, y2)]
            for x1, y1, x2, y2 in sz.routes
        ],
    }
    with open(LEARNED_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)


def load_learned():
    if not os.path.exists(LEARNED_PATH):
        return None
    with open(LEARNED_PATH, encoding="utf-8") as f:
        return json.load(f)


# ══════════════════════════════════════════════
# 안전구역 사이 연결 경로
# ══════════════════════════════════════════════

def _polyline_len(coords):
    total = 0.0
    for a, b in zip(coords, coords[1:]):
        total += math.hypot((b[0] - a[0]) * 110540,
                            (b[1] - a[1]) * 88000)
    return total


def _zones_hash(cfg):
    """구역 좌표 지문. 바뀌면 경로를 다시 뽑는다."""
    return json.dumps([[round(z["lat"], 6), round(z["lon"], 6),
                        round(z["radius"], 1)]
                       for z in cfg.get("zones", [])], sort_keys=True)


def build_links(cfg, verbose=True):
    """
    안전구역들을 서로 잇는 경로를 만든다.

    ★ 왜 필요한가:
      구역만 있으면 그 사이를 이동할 때 전부 이탈로 잡힌다.
      학습 데이터가 쌓이기 전 공백을 메우는 경로다.

    ★ 우선순위:
      1) OSM 보행 도로망 최단경로 — 실제 걸을 수 있는 길
      2) 직선 — OSM 조회 실패 시 폴백
    """
    zones = cfg.get("zones", [])
    if len(zones) < 2:
        if verbose:
            print("  안전구역이 2곳 미만이라 연결할 경로가 없습니다.")
        data = {
            "built_at": datetime.now().isoformat(),
            "mode": "none",
            "buffer": LINK_BUFFER_M,
            "links": [],
            "zones_hash": _zones_hash(cfg),
        }
        with open(LINK_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
        return data

    clat, clon = origin_of(cfg)

    G = None
    mode = "line"
    try:
        from route_builder import load_road_graph
        if verbose:
            print(f"  도로망 준비 중 (중심 {clat:.5f}, {clon:.5f} / "
                  f"반경 {ROAD_RADIUS_M}m)...")
        G = load_road_graph(clat, clon, radius_m=ROAD_RADIUS_M,
                            cache_path=GRAPH_CACHE, verbose=verbose)
        if G is not None:
            mode = "road"
    except ImportError:
        if verbose:
            print("  osmnx 미설치 — 직선으로 연결합니다.")
            print("  설치하려면:  pip install osmnx networkx")

    links = []
    road_ok = line_fallback = 0

    for i in range(len(zones)):
        for j in range(i + 1, len(zones)):
            a, b = zones[i], zones[j]
            coords = None

            if G is not None:
                try:
                    from route_builder import road_path
                    coords = road_path(G, a["lat"], a["lon"],
                                       b["lat"], b["lon"])
                except Exception:
                    coords = None

            if coords and len(coords) >= 2:
                road_ok += 1
                src = "road"
            else:
                coords = [(a["lat"], a["lon"]), (b["lat"], b["lon"])]
                line_fallback += 1
                src = "line"

            segs = [[coords[k][0], coords[k][1],
                     coords[k+1][0], coords[k+1][1]]
                    for k in range(len(coords) - 1)]

            links.append({
                "from": a["name"], "to": b["name"],
                "from_key": a["key"], "to_key": b["key"],
                "source": src,
                "segments": segs,
                "path": [[c[0], c[1]] for c in coords],   # 지도 그리기용
                "length_m": round(_polyline_len(coords)),
            })

    data = {
        "built_at": datetime.now().isoformat(),
        "mode": mode,
        "buffer": LINK_BUFFER_M,
        "links": links,
        "zones_hash": _zones_hash(cfg),
    }
    with open(LINK_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)

    if verbose:
        print(f"\n  연결 경로 {len(links)}개 생성")
        for lk in links:
            tag = "도로망" if lk["source"] == "road" else "직선"
            print(f"    {lk['from']} ↔ {lk['to']}  "
                  f"{lk['length_m']:,}m  [{tag}]")
        if line_fallback and road_ok:
            print(f"\n  도로망 {road_ok}개 / 직선 폴백 {line_fallback}개")
        elif line_fallback:
            print(f"\n  전부 직선으로 연결됐습니다.")
            print(f"  인터넷 연결과 osmnx 설치를 확인하세요.")

    return data


def load_links():
    if not os.path.exists(LINK_PATH):
        return None
    with open(LINK_PATH, encoding="utf-8") as f:
        return json.load(f)


# ══════════════════════════════════════════════
# 안전구역 3겹 병합
# ══════════════════════════════════════════════

class LayeredSafeZone(SafeZone):
    """
    안전구역을 세 겹으로 관리한다.

      places  등록 구역 + 학습 장소  (원)
      routes  학습된 실제 궤적       (폭 60m 띠)
      links   구역 간 연결 경로       (폭 120m 띠)

    ★ links를 routes와 분리하는 이유:
      학습 궤적은 실제 지나간 길이라 60m면 충분하다.
      연결 경로는 OSM 최단경로라 실제 다니는 길과 다를 수 있어
      더 넉넉한 폭이 필요하다.
    """

    def __init__(self):
        super().__init__()
        self.links = []
        self.link_buffer = LINK_BUFFER_M

    @property
    def is_trained(self):
        return bool(self.places or self.routes or self.links)

    def dist_outside(self, x, y):
        best = super().dist_outside(x, y)
        if best <= 0:
            return 0.0

        for x1, y1, x2, y2 in self.links:
            from core import _seg_dist
            d = _seg_dist(x, y, x1, y1, x2, y2) - self.link_buffer
            if d <= 0:
                return 0.0
            best = min(best, d)

        return max(0.0, best if best != float("inf") else 9999.0)


def build_safezone(cfg, plane):
    """
    세 겹을 합쳐 하나의 안전구역을 만든다.
    반환: (safezone, 등록구역수, 학습장소수, 연결구간수)
    """
    sz = LayeredSafeZone()
    sz.route_buffer = cfg.get("route_buffer", DEFAULT_ROUTE_BUFFER)

    registered = learned = linked = 0

    # ── 1) 친구 앱이 찍은 안전구역 ──
    for z in cfg.get("zones", []):
        cx, cy = plane.to_xy(z["lat"], z["lon"])
        sz.places.append((cx, cy, z["radius"], 999))   # 999 = 등록 표시
        registered += 1

    # ── 2) 구역 간 연결 경로 ──
    lk = load_links()
    if lk:
        sz.link_buffer = lk.get("buffer", LINK_BUFFER_M)
        for link in lk.get("links", []):
            for s in link.get("segments", []):
                sz.links.append((*plane.to_xy(s[0], s[1]),
                                 *plane.to_xy(s[2], s[3])))
        linked = len(lk.get("links", []))

    # ── 3) 자동 학습 결과 ──
    lz = load_learned()
    if lz:
        for p in lz.get("places", []):
            cx, cy = plane.to_xy(p["lat"], p["lon"])
            if any(math.hypot(cx - ox, cy - oy) < r
                   for ox, oy, r, _ in sz.places):
                continue          # 등록 구역과 겹치면 건너뜀
            sz.places.append((cx, cy, p["radius"], p.get("visits", 0)))
            learned += 1

        for r in lz.get("routes", []):
            sz.routes.append((*plane.to_xy(r[0], r[1]),
                              *plane.to_xy(r[2], r[3])))

    return sz, registered, learned, linked


# ══════════════════════════════════════════════
# 트래커 (구역 변경 시 자동 재생성)
# ══════════════════════════════════════════════

_state = {"tracker": None, "hash": None,
          "registered": 0, "learned": 0, "linked": 0}


def get_tracker(force_reload=False, rebuild_links=True):
    """
    안전구역이 바뀌면 트래커를 새로 만든다.
    친구가 앱에서 구역을 추가하면 재시작 없이 반영된다.
    """
    cfg = read_zones()
    h = _zones_hash(cfg)

    if _state["tracker"] is None or h != _state["hash"] or force_reload:
        # 구역이 바뀌었으면 연결 경로 재생성
        if rebuild_links and len(cfg["zones"]) >= 2:
            lk = load_links()
            if (lk or {}).get("zones_hash") != h:
                print("  안전구역이 변경되어 연결 경로를 다시 계산합니다...")
                build_links(cfg, verbose=True)

        origin = origin_of(cfg)
        plane = LocalPlane(*origin)
        sz, reg, lrn, lnk = build_safezone(cfg, plane)

        _state["tracker"]    = VestTracker(DEVICE_ID, *origin, safezone=sz)
        _state["hash"]       = h
        _state["registered"] = reg
        _state["learned"]    = lrn
        _state["linked"]     = lnk

        if not cfg["zones"]:
            print("  [경고] safe_zones 가 비어 있습니다. 판단이 보류됩니다.")
            print("         친구 앱에서 안전구역을 찍으면 즉시 반영됩니다.")

    return _state["tracker"], cfg


def publish_status():
    tr, cfg = get_tracker()
    try:
        db.reference(STATUS_PATH).update({
            "safezone_ready": bool(tr.safezone and tr.safezone.is_trained),
            "zone_count":     _state["registered"],
            "learned_count":  _state["learned"],
            "link_count":     _state["linked"],
            "updated_at":     datetime.now().isoformat(),
        })
    except Exception:
        pass


# ══════════════════════════════════════════════
# 지도용 업로드
# ══════════════════════════════════════════════

def publish_safezone(verbose=True):
    """
    안전구역과 경로를 친구 앱이 지도에 그릴 수 있게 올린다.

    safezone/
      zones/  [{name, lat, lon, radius, source}]
      links/  [{from, to, path: [[lat,lon], ...], length_m, source}]
      buffer  경로 폭
    """
    cfg = read_zones()
    origin = origin_of(cfg)
    plane = LocalPlane(*origin)

    zones_out = [
        {"key": z["key"], "name": z["name"],
         "lat": z["lat"], "lon": z["lon"],
         "radius": z["radius"], "source": "registered"}
        for z in cfg["zones"]
    ]

    # 자동 학습으로 발견된 장소도 함께 올린다
    lz = load_learned()
    if lz:
        for i, p in enumerate(lz.get("places", []), 1):
            cx, cy = plane.to_xy(p["lat"], p["lon"])
            if any(math.hypot(cx - plane.to_xy(z["lat"], z["lon"])[0],
                              cy - plane.to_xy(z["lat"], z["lon"])[1])
                   < z["radius"] for z in cfg["zones"]):
                continue
            zones_out.append({
                "key": f"learned_{i}", "name": f"학습장소{i}",
                "lat": p["lat"], "lon": p["lon"],
                "radius": p["radius"], "source": "learned",
            })

    lk = load_links() or {}
    links_out = [
        {"from": l["from"], "to": l["to"],
         "path": l.get("path", []),
         "length_m": l.get("length_m", 0),
         "source": l.get("source", "line")}
        for l in lk.get("links", [])
    ]

    payload = {
        "zones": zones_out,
        "links": links_out,
        "buffer": lk.get("buffer", LINK_BUFFER_M),
        "route_buffer": cfg.get("route_buffer", DEFAULT_ROUTE_BUFFER),
        "updated_at": datetime.now().isoformat(),
    }
    db.reference(SAFEZONE_PUB_PATH).set(payload)

    if verbose:
        print(f"  {SAFEZONE_PUB_PATH} 에 업로드 완료")
        print(f"    구역 {len(zones_out)}개 / 경로 {len(links_out)}개")

    return payload


# ══════════════════════════════════════════════
# 좌표 읽기 / 처리
# ══════════════════════════════════════════════

def read_sensor():
    snap = db.reference(SENSOR_PATH).get()
    if not isinstance(snap, dict):
        return None
    if LAT_FIELD not in snap or LON_FIELD not in snap:
        return None
    try:
        return (float(snap[LAT_FIELD]), float(snap[LON_FIELD]),
                float(snap.get("hdop", 1.5)))
    except (TypeError, ValueError):
        return None


def process(lat, lon, hdop):
    tr, cfg = get_tracker()
    res = tr.feed(lat, lon, datetime.now(), hdop)

    level  = res.get("level", "정상")
    is_out = level in OUT_LEVELS

    db.reference(f"{SENSOR_PATH}/{OUT_FIELD}").set(is_out)
    db.reference(f"{SENSOR_PATH}/risk").set(round(res.get("risk", 0), 1))
    db.reference(f"{SENSOR_PATH}/dist_out_m").set(
        round(res.get("dist_outside_m", 0), 1))

    return {
        "lat": lat, "lon": lon, "level": level,
        "risk": res.get("risk", 0),
        "dist": res.get("dist_outside_m", 0),
        "is_out": is_out,
        "status": res.get("status", "ok"),
    }


# ══════════════════════════════════════════════
# 명령: 안전구역 읽기 검증 ★
# ══════════════════════════════════════════════

def cmd_check_zones():
    """친구가 찍은 안전구역을 제대로 읽는지 확인한다"""
    print("=" * 62)
    print("안전구역 읽기 검증")
    print("=" * 62)
    print(f"\n  경로: {ZONES_PATH}")

    raw = db.reference(ZONES_PATH).get()
    if raw is None:
        print(f"\n  ★ {ZONES_PATH} 가 비어 있습니다.")
        print(f"    친구 앱에서 지도를 찍으면 여기에 생성됩니다.")
        return

    n_raw = len(raw) if isinstance(raw, (dict, list)) else 0
    print(f"  원본 항목 수: {n_raw}개")

    cfg = read_zones()
    zones = cfg["zones"]

    print(f"\n  읽어들인 구역: {len(zones)}곳")
    if zones:
        print(f"  {'이름':<18} {'위도':>12} {'경도':>13} {'반경':>7}")
        print("  " + "-" * 54)
        for z in zones:
            print(f"  {z['name']:<18} {z['lat']:12.6f} "
                  f"{z['lon']:13.6f} {z['radius']:6.0f}m")

    if cfg["errors"]:
        print(f"\n  ★ 읽지 못한 항목 {len(cfg['errors'])}개")
        for e in cfg["errors"]:
            print(f"    - {e}")

    # 구역 간 거리 (경로 생성이 의미 있는지 판단용)
    if len(zones) >= 2:
        print(f"\n  구역 간 거리")
        for i in range(len(zones)):
            for j in range(i + 1, len(zones)):
                a, b = zones[i], zones[j]
                d = math.hypot((b["lat"] - a["lat"]) * 110540,
                               (b["lon"] - a["lon"]) * 88000)
                gap = d - a["radius"] - b["radius"]
                note = "  (원이 겹침)" if gap < 0 else ""
                print(f"    {a['name']} ↔ {b['name']}  "
                      f"{d:,.0f}m{note}")

    # 반경이 과한지 알려준다
    big = [z for z in zones if z["radius"] > 300]
    if big:
        print(f"\n  참고: 반경이 300m를 넘는 구역이 {len(big)}개 있습니다.")
        for z in big:
            print(f"    - {z['name']}  {z['radius']:.0f}m")
        print(f"    반경이 크면 그만큼 이탈 감지가 둔해집니다.")
        print(f"    집은 80~150m, 넓은 공원은 200m 정도가 적당합니다.")

    print(f"\n" + "=" * 62)
    if zones and not cfg["errors"]:
        print("정상적으로 읽었습니다.")
        if len(zones) >= 2:
            print("\n다음 단계:")
            print("  python firebase_bridge.py --build-links")
    elif not zones:
        print("읽어들인 구역이 없습니다. 필드명을 확인하세요.")
        print(f"기대하는 필드: {LAT_FIELD}, {LON_FIELD}, radius")
    print("=" * 62)


# ══════════════════════════════════════════════
# 명령: 전체 상태
# ══════════════════════════════════════════════

def cmd_status():
    print("=" * 62)
    print("전체 상태")
    print("=" * 62)

    cfg = read_zones()

    print(f"\n[안전구역]  경로: {ZONES_PATH}")
    if cfg["zones"]:
        for z in cfg["zones"]:
            print(f"  - {z['name']} ({z['lat']:.5f}, {z['lon']:.5f}) "
                  f"반경 {z['radius']:.0f}m")
    else:
        print(f"  없음 — 친구 앱에서 지도를 찍으면 생성됩니다.")
    if cfg["errors"]:
        print(f"  ★ 읽지 못한 항목 {len(cfg['errors'])}개 "
              f"(--check-zones 로 확인)")

    print(f"\n[연결 경로]")
    lk = load_links()
    if lk and lk.get("links"):
        mode = "OSM 도로망" if lk.get("mode") == "road" else "직선"
        print(f"  생성 시각: {lk.get('built_at','?')[:19]}  방식: {mode}")
        print(f"  폭: {lk.get('buffer', LINK_BUFFER_M):.0f}m")
        for l in lk["links"]:
            tag = "도로망" if l["source"] == "road" else "직선"
            print(f"    {l['from']} ↔ {l['to']}  "
                  f"{l['length_m']:,}m  [{tag}]")
        if lk.get("zones_hash") != _zones_hash(cfg):
            print(f"  ★ 구역이 변경됐습니다. --build-links 로 갱신하세요.")
    else:
        print(f"  없음 — 구역 2곳 이상이면 --build-links 로 생성")

    print(f"\n[자동 학습]")
    lz = load_learned()
    if lz:
        print(f"  학습 시각: {lz.get('trained_at','?')[:19]}")
        print(f"  학습된 장소: {len(lz.get('places',[]))}곳")
        print(f"  경로 선분: {len(lz.get('routes',[])):,}개")
    else:
        print(f"  아직 학습 안 됨")

    n = count_points()
    print(f"\n[수집된 좌표] {n:,}개")
    if n < 50:
        print(f"  자동 학습에는 50개 이상 필요 ({max(0,50-n)}개 더)")
    else:
        print(f"  학습 가능:  python firebase_bridge.py --train")

    origin = origin_of(cfg)
    plane = LocalPlane(*origin)
    sz, reg, lrn, lnk = build_safezone(cfg, plane)

    print(f"\n[최종 안전구역]")
    print(f"  구역  : 등록 {reg}곳 + 학습 {lrn}곳 = 총 {len(sz.places)}곳")
    print(f"  연결선: {len(sz.links):,}개 선분  (폭 {sz.link_buffer:.0f}m)")
    print(f"  학습선: {len(sz.routes):,}개 선분  (폭 {sz.route_buffer:.0f}m)")
    print(f"  → {'판단 가능' if sz.is_trained else '판단 보류'}")


# ══════════════════════════════════════════════
# 명령: 연결 경로 생성
# ══════════════════════════════════════════════

def cmd_build_links():
    print("=" * 62)
    print("연결 경로 생성")
    print("=" * 62)

    cfg = read_zones()
    zones = cfg["zones"]

    if len(zones) < 2:
        print(f"\n  안전구역이 {len(zones)}곳입니다. 2곳 이상 필요합니다.")
        print(f"  친구 앱에서 지도를 더 찍어주세요.")
        return

    print(f"\n  안전구역 {len(zones)}곳:")
    for z in zones:
        print(f"    - {z['name']} ({z['lat']:.5f}, {z['lon']:.5f}) "
              f"반경 {z['radius']:.0f}m")
    print()

    build_links(cfg, verbose=True)
    print(f"\n  저장: {LINK_PATH}")

    # 지도용으로도 올린다
    print()
    publish_safezone()

    print(f"\n감시를 시작하세요:")
    print(f"  python firebase_bridge.py")


# ══════════════════════════════════════════════
# 명령: 지도용 업로드
# ══════════════════════════════════════════════

def cmd_publish():
    print("=" * 62)
    print("안전구역 지도용 업로드")
    print("=" * 62)
    print()
    payload = publish_safezone()
    print(f"\n친구 앱에서 이렇게 읽으면 됩니다:")
    print(f"""
  // 원 그리기
  safezone.zones.forEach(z => new google.maps.Circle({{
    center: {{lat: z.lat, lng: z.lon}},
    radius: z.radius, map: map
  }}));

  // 경로 선 그리기
  safezone.links.forEach(l => new google.maps.Polyline({{
    path: l.path.map(p => ({{lat: p[0], lng: p[1]}})),
    map: map
  }}));
""")


# ══════════════════════════════════════════════
# 명령: 수집 / 학습
# ══════════════════════════════════════════════

def cmd_collect():
    print("=" * 62)
    print("좌표 수집 모드 (판단하지 않음)")
    print("=" * 62)
    print(f"  감시 경로: {SENSOR_PATH}")
    print(f"  현재 보유: {count_points():,}개")
    print(f"  (Ctrl+C 로 종료)\n")

    def on_change(event):
        s = read_sensor()
        if s is None:
            return
        lat, lon, hdop = s
        if save_point(lat, lon, hdop):
            print(f"{datetime.now():%H:%M:%S} 수집 "
                  f"({lat:.5f}, {lon:.5f})  누적 {count_points():,}개")

    db.reference(SENSOR_PATH).listen(on_change)


def cmd_train(days=30):
    print("=" * 62)
    print("자동 학습 (수집된 좌표 기반)")
    print("=" * 62)

    points = load_points(days)
    print(f"\n  대상 좌표: {len(points):,}개 (최근 {days}일)")

    if len(points) < 50:
        print(f"\n  좌표가 부족합니다. 최소 50개 필요.")
        print(f"  --collect 로 더 모으세요.")
        print(f"  그동안에도 등록 구역으로는 판단이 됩니다.")
        return

    cfg = read_zones()
    plane = LocalPlane(*origin_of(cfg))
    sz = learn_safezone_simple(points, plane)

    print(f"\n[학습 결과]")
    print(f"  자동 발견한 장소: {len(sz.places)}곳")
    for i, (cx, cy, r, v) in enumerate(sz.places, 1):
        la, lo = plane.to_latlon(cx, cy)
        print(f"    #{i} ({la:.5f}, {lo:.5f})  반경 {r:.0f}m  방문 {v}회")
    print(f"  경로 선분: {len(sz.routes):,}개")

    save_learned(sz, plane)
    print(f"\n  저장: {LEARNED_PATH}")

    merged, reg, lrn, lnk = build_safezone(cfg, plane)
    print(f"\n[최종 안전구역]")
    print(f"  등록 {reg}곳 + 학습 {lrn}곳 = 총 {len(merged.places)}곳")

    publish_safezone()
    print(f"\n감시를 시작하세요:")
    print(f"  python firebase_bridge.py")


# ══════════════════════════════════════════════
# 명령: 실시간 감시
# ══════════════════════════════════════════════

def watch_mode():
    tr, cfg = get_tracker()
    publish_status()

    print("=" * 62)
    print("Firebase 실시간 감시")
    print("=" * 62)
    print(f"  안전구역  : {ZONES_PATH}")
    print(f"  좌표 수신 : {SENSOR_PATH}")
    print(f"  결과 기록 : {OUT_FIELD}, risk, dist_out_m")
    print(f"  구역      : 등록 {_state['registered']}곳 + "
          f"학습 {_state['learned']}곳")
    if tr.safezone and getattr(tr.safezone, "links", None):
        print(f"  연결 경로 : {_state['linked']}개 구간 "
              f"({len(tr.safezone.links):,} 선분, 폭 "
              f"{tr.safezone.link_buffer:.0f}m)")

    if not (tr.safezone and tr.safezone.is_trained):
        print(f"  ★ 안전구역이 없어 판단이 보류됩니다.")
        print(f"    친구 앱에서 지도를 찍으면 즉시 반영됩니다.")

    print(f"  (Ctrl+C 로 종료)\n")

    def on_sensor(event):
        s = read_sensor()
        if s is None:
            return
        lat, lon, hdop = s
        try:
            r = process(lat, lon, hdop)
            save_point(lat, lon, hdop)
            icon = "[이탈]" if r["is_out"] else "[정상]"
            print(f"{datetime.now():%H:%M:%S} {icon} "
                  f"({r['lat']:.5f}, {r['lon']:.5f}) | "
                  f"{r['level']} 위험도 {r['risk']:.0f} | "
                  f"이탈 {r['dist']:.0f}m → {OUT_FIELD}={r['is_out']}")
        except Exception as e:
            print(f"처리 오류: {e}")

    def on_zones(event):
        before = _state["hash"]
        get_tracker()
        if _state["hash"] != before:
            print(f"{datetime.now():%H:%M:%S} [안전구역 변경] "
                  f"등록 {_state['registered']}곳 + "
                  f"학습 {_state['learned']}곳 + "
                  f"연결 {_state['linked']}구간")
            publish_status()
            publish_safezone(verbose=False)

    db.reference(ZONES_PATH).listen(on_zones)
    db.reference(SENSOR_PATH).listen(on_sensor)


# ══════════════════════════════════════════════
# 명령: 단발 테스트
# ══════════════════════════════════════════════

def test_mode():
    print("=" * 62)
    print("연동 테스트")
    print("=" * 62)

    tr, cfg = get_tracker()

    print(f"\n[1] 안전구역 읽기")
    print(f"    {len(cfg['zones'])}곳 등록됨")
    for z in cfg["zones"][:5]:
        print(f"      {z['name']} ({z['lat']:.5f}, {z['lon']:.5f}) "
              f"반경 {z['radius']:.0f}m")
    if len(cfg["zones"]) > 5:
        print(f"      … 외 {len(cfg['zones'])-5}곳")

    print(f"\n[2] 좌표 읽기")
    s = read_sensor()
    if s is None:
        print(f"    좌표 없음. 구역 중심을 넣습니다.")
        o = origin_of(cfg)
        db.reference(SENSOR_PATH).update({
            LAT_FIELD: o[0], LON_FIELD: o[1], "hdop": 1.5})
        s = (o[0], o[1], 1.5)
    lat, lon, hdop = s
    print(f"    ({lat:.6f}, {lon:.6f})")

    print(f"\n[3] 판단")
    r = process(lat, lon, hdop)
    print(f"    등급  : {r['level']}")
    print(f"    위험도: {r['risk']:.1f}")
    print(f"    이탈  : {r['dist']:.0f}m")

    print(f"\n[4] 기록 확인")
    print(f"    {OUT_FIELD} = "
          f"{db.reference(f'{SENSOR_PATH}/{OUT_FIELD}').get()}")

    publish_status()

    print(f"\n" + "=" * 62)
    if not (tr.safezone and tr.safezone.is_trained):
        print("안전구역이 없어 판단이 보류됩니다.")
        print("친구 앱에서 지도를 찍어주세요.")
    else:
        print("테스트 완료")
    print("=" * 62)


# ══════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Firebase 연동 브리지",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
실행 순서
  1. --check-zones    친구가 찍은 안전구역을 읽는지 확인
  2. --build-links    구역 사이 연결 경로 생성 (OSM 도로망)
  3. (인자 없음)       실시간 감시 시작

기타
  --status     전체 상태 확인
  --publish    안전구역을 지도용으로 Firebase에 올림
  --collect    좌표 수집만 (판단 안 함)
  --train      수집분으로 자동 학습
  --test       단발 테스트
""")
    ap.add_argument("--check-zones", action="store_true",
                    help="안전구역 읽기 검증")
    ap.add_argument("--status",      action="store_true")
    ap.add_argument("--build-links", action="store_true",
                    help="구역 간 연결 경로 생성")
    ap.add_argument("--publish",     action="store_true",
                    help="지도용 업로드")
    ap.add_argument("--collect",     action="store_true")
    ap.add_argument("--train",       action="store_true")
    ap.add_argument("--test",        action="store_true")
    ap.add_argument("--days", type=int, default=30)
    args = ap.parse_args()

    init_db()
    init_firebase()
    print("Firebase 연결 성공\n")

    if args.check_zones:
        cmd_check_zones()
    elif args.status:
        cmd_status()
    elif args.build_links:
        cmd_build_links()
    elif args.publish:
        cmd_publish()
    elif args.collect:
        cmd_collect()
    elif args.train:
        cmd_train(days=args.days)
    elif args.test:
        test_mode()
    else:
        watch_mode()


if __name__ == "__main__":
    main()