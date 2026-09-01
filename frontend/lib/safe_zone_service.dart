import 'dart:math';
import 'package:flutter/material.dart';
import 'package:firebase_database/firebase_database.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';

class SafeZone {
  final String id;
  final LatLng center;
  final double radius;

  SafeZone({
    required this.id,
    required this.center,
    required this.radius,
  });

  factory SafeZone.fromMap(String id, Map<dynamic, dynamic> map) {
    return SafeZone(
      id: id,
      center: LatLng(
        ((map['latitude'] ?? map['lat']) as num).toDouble(),
        ((map['longitude'] ?? map['lng']) as num).toDouble(),
      ),
      radius: (map['radius'] as num).toDouble(),
    );
  }
}

class SafeZoneService {
  static final DatabaseReference _db = FirebaseDatabase.instance.ref();

  /// 1. 원본 안심 구역(safe_zones) 실시간 감시
  static Stream<List<SafeZone>> getSafeZonesStream() {
    return _db.child('safe_zones').onValue.map((event) {
      final data = event.snapshot.value;
      if (data == null || data is! Map) return [];

      List<SafeZone> zones = [];
      data.forEach((key, value) {
        if (value is Map) {
          zones.add(SafeZone.fromMap(key.toString(), value));
        }
      });
      return zones;
    });
  }

  /// 2. AI가 생성한 경로 실시간 감시
  static Stream<List<List<LatLng>>> getAIPathStream() {
    return _db.child('safezone/links').onValue.asyncMap((event) async {
      final data = event.snapshot.value;

      if (data == null) return <List<LatLng>>[];

      final safeZoneSnapshot = await _db.child('safe_zones').get();
      if (!safeZoneSnapshot.exists || safeZoneSnapshot.value == null) {
        return <List<LatLng>>[];
      }

      List<List<LatLng>> paths = [];

      Iterable linksIterable = [];
      if (data is List) {
        linksIterable = data;
      } else if (data is Map) {
        linksIterable = data.values;
      }

      for (var link in linksIterable) {
        if (link != null && link is Map && link.containsKey('path')) {
          final pathData = link['path'];
          List<LatLng> segment = [];

          Iterable pointsIterable = [];
          if (pathData is List) {
            pointsIterable = pathData;
          } else if (pathData is Map) {
            pointsIterable = pathData.values;
          }

          for (var pt in pointsIterable) {
            if (pt is Map) {
              num? lat = pt['lat'] ?? pt['latitude'];
              num? lng = pt['lng'] ?? pt['longitude'];
              if (lat != null && lng != null) {
                segment.add(LatLng(lat.toDouble(), lng.toDouble()));
              }
            } else if (pt is List && pt.length >= 2) {
              num? lat = pt[0] as num?;
              num? lng = pt[1] as num?;
              if (lat != null && lng != null) {
                segment.add(LatLng(lat.toDouble(), lng.toDouble()));
              }
            }
          }

          if (segment.isNotEmpty) {
            paths.add(segment);
          }
        }
      }

      return paths;
    });
  }

  /// 3. AI 경로(links)를 지도 표시용 Polyline 객체로 변환
  static Set<Polyline> convertToPolylines(
      List<List<LatLng>> aiPathPoints, {
        Color pathColor = Colors.orange,
      }) {
    Set<Polyline> polylines = {};
    for (int i = 0; i < aiPathPoints.length; i++) {
      polylines.add(
        Polyline(
          polylineId: PolylineId('ai_path_$i'),
          points: aiPathPoints[i],
          color: pathColor,
          width: 5,
        ),
      );
    }
    return polylines;
  }

  /// 4. 안심 구역 추가
  static Future<void> addSafeZone(LatLng center, double radius) async {
    final newRef = _db.child('safe_zones').push();
    await newRef.set({
      'latitude': center.latitude,
      'longitude': center.longitude,
      'radius': radius,
      'timestamp': ServerValue.timestamp,
    });
  }

  /// 5. 단일 안심 구역 삭제
  static Future<void> deleteSafeZone(String id) async {
    await _db.child('safe_zones').child(id).remove();
    await _db.child('safezone/links').remove();
  }

  /// 6. 전체 안심 구역 삭제
  static Future<void> clearAllSafeZones() async {
    await _db.child('safe_zones').remove();
    await _db.child('safezone').remove();
  }

  /// 7. 두 좌표 간 실제 거리 계산 (Haversine 공식, m)
  static double getDistanceMeter(LatLng p1, LatLng p2) {
    const double r = 6371000;
    double dLat = (p2.latitude - p1.latitude) * pi / 180;
    double dLng = (p2.longitude - p1.longitude) * pi / 180;

    double a = sin(dLat / 2) * sin(dLat / 2) +
        cos(p1.latitude * pi / 180) *
            cos(p2.latitude * pi / 180) *
            sin(dLng / 2) *
            sin(dLng / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return r * c;
  }

  /// 💡 [핵심 해결] 점(P)과 선분(AB) 사이의 수직 최단 거리(m) 계산
  static double _distanceToSegmentMeter(LatLng p, LatLng a, LatLng b) {
    double x = p.longitude;
    double y = p.latitude;
    double x1 = a.longitude;
    double y1 = a.latitude;
    double x2 = b.longitude;
    double y2 = b.latitude;

    double dx = x2 - x1;
    double dy = y2 - y1;

    if (dx == 0 && dy == 0) {
      return getDistanceMeter(p, a);
    }

    // 선분 상에서 점 P와 가장 가까운 투영점 위치 비율 t (0~1)
    double t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy);

    if (t < 0) {
      return getDistanceMeter(p, a);
    } else if (t > 1) {
      return getDistanceMeter(p, b);
    }

    LatLng projection = LatLng(y1 + t * dy, x1 + t * dx);
    return getDistanceMeter(p, projection);
  }

  /// 8. 안심구역 내부 OR AI 경로(선분) 위 중 하나에 속하는지 판정
  static bool isSafePosition({
    required LatLng currentPos,
    required List<SafeZone> safeZones,
    required List<List<LatLng>> aiPathPoints,
    double pathToleranceMeter = 40.0, // 💡 GPS 오차 보정을 고려하여 40m 설정
  }) {
    // 1) 안심 구역(원) 내부 검사
    for (var zone in safeZones) {
      if (getDistanceMeter(currentPos, zone.center) <= zone.radius) {
        return true;
      }
    }

    // 2) AI 경로(선분)와의 최단 거리 검사
    for (var segment in aiPathPoints) {
      if (segment.length < 2) {
        for (var point in segment) {
          if (getDistanceMeter(currentPos, point) <= pathToleranceMeter) {
            return true;
          }
        }
      } else {
        // 점과 점 사이의 '선분' 전체를 대상으로 거리 측정
        for (int i = 0; i < segment.length - 1; i++) {
          double dist = _distanceToSegmentMeter(
            currentPos,
            segment[i],
            segment[i + 1],
          );
          if (dist <= pathToleranceMeter) {
            return true;
          }
        }
      }
    }

    return false;
  }
}