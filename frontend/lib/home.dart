import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:firebase_database/firebase_database.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:flutter_background_service/flutter_background_service.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'safe_zone_service.dart';
import 'fall_detector.dart';
import 'notification_helper.dart';
import 'login.dart';

class MainScreen extends StatefulWidget {
  final String guardianName;
  final String patientName;
  final String patientAge;

  const MainScreen({
    super.key,
    required this.guardianName,
    required this.patientName,
    required this.patientAge,
  });

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  final DatabaseReference dbRef =
  FirebaseDatabase.instance.ref("sensor_data/test");

  double latitude = 36.68425, longitude = 126.78352;
  String healthStatus = "정상";
  bool isOutOfPath = false;
  bool isNotifiedFall = false;
  bool isNotifiedPath = false;
  bool isFallenLocked = false;

  List<SafeZone> safeZones = [];
  List<List<LatLng>> aiPathPoints = [];

  StreamSubscription? _safeZoneSub;
  StreamSubscription? _aiPathSub;

  double newZoneRadius = 100.0;
  GoogleMapController? mapController;

  // 💡 지도 자동 원복용 타이머 및 상태 플래그
  Timer? _cameraIdleTimer;
  bool _isUserInteracting = false;

  @override
  void initState() {
    super.initState();
    readSensorData();
    listenFirebaseData();
  }

  @override
  void dispose() {
    _cameraIdleTimer?.cancel();
    _safeZoneSub?.cancel();
    _aiPathSub?.cancel();
    super.dispose();
  }

  /// 💡 카메라 원복 타이머 리셋/시작 함수 (10초 대기)
  void _resetCameraIdleTimer() {
    _cameraIdleTimer?.cancel();
    _cameraIdleTimer = Timer(const Duration(seconds: 10), () {
      if (mounted) {
        setState(() {
          _isUserInteracting = false;
        });
        _moveCamera(latitude, longitude); // 10초 후 어르신 GPS 위치로 복귀
      }
    });
  }

  /// 안심구역/경로 및 이탈 여부 통합 재판정
  void _reevaluateSafety() {
    LatLng currentPos = LatLng(latitude, longitude);

    bool isSafe = SafeZoneService.isSafePosition(
      currentPos: currentPos,
      safeZones: safeZones,
      aiPathPoints: aiPathPoints,
      pathToleranceMeter: 25.0,
    );

    bool hasConstraints = safeZones.isNotEmpty || aiPathPoints.isNotEmpty;
    bool newOutOfPathState = hasConstraints && !isSafe;

    setState(() {
      isOutOfPath = newOutOfPathState;
    });

    dbRef.child("is_out_of_path").set(isOutOfPath);

    if (isOutOfPath) {
      if (!isNotifiedPath) {
        triggerNotification(
          2,
          "⚠️ 경로 및 안심 구역 이탈 경고!",
          "${widget.patientName} 어르신이 지정된 안심 구역과 AI 이동 경로를 벗어났습니다.",
        );
        isNotifiedPath = true;
      }
    } else {
      isNotifiedPath = false;
    }
  }

  void listenFirebaseData() {
    _safeZoneSub = SafeZoneService.getSafeZonesStream().listen((zones) {
      if (mounted) {
        setState(() {
          safeZones = zones;
        });
        _reevaluateSafety();
      }
    });

    _aiPathSub = SafeZoneService.getAIPathStream().listen((path) {
      if (mounted) {
        setState(() {
          aiPathPoints = path;
        });
        _reevaluateSafety();
      }
    });
  }

  Future<void> _logout() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.clear();

    if (mounted) {
      Navigator.pushAndRemoveUntil(
        context,
        MaterialPageRoute(builder: (context) => const LoginScreen()),
            (route) => false,
      );
    }
  }

  void _moveCamera(double lat, double lng) {
    if (mapController != null) {
      mapController!.animateCamera(
        CameraUpdate.newLatLngZoom(LatLng(lat, lng), 15.0),
      );
    }
  }

  void readSensorData() {
    dbRef.onValue.listen((event) {
      final value = event.snapshot.value;
      if (value != null && value is Map) {
        final data = Map<dynamic, dynamic>.from(value);

        double rawLat = FallDetector.parseToDouble(data["latitude"], 36.68425);
        double rawLng = FallDetector.parseToDouble(data["longitude"], 126.78352);

        if (rawLat > 100 && rawLng < 50) {
          double temp = rawLat;
          rawLat = rawLng;
          rawLng = temp;
        }

        setState(() {
          latitude = rawLat;
          longitude = rawLng;

          bool isFallen = FallDetector.checkFall(data);

          if (isFallen) {
            healthStatus = "넘어짐";
            isFallenLocked = true;

            if (!isNotifiedFall) {
              triggerNotification(
                1,
                "🚨 낙상 발생 비상 알림!",
                "${widget.patientName} 어르신이 넘어지셨습니다. 위치를 즉시 확인해 주세요!",
              );
              isNotifiedFall = true;
            }
          } else {
            if (!isFallenLocked) {
              healthStatus = "정상";
            }
          }
        });

        _reevaluateSafety();

        // 💡 사용자가 지도를 조작 중이 아닐 때만 센서 데이터 수신 시 카메라 추적
        if (!_isUserInteracting) {
          _moveCamera(latitude, longitude);
        }
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    bool isDanger = healthStatus == "넘어짐";

    final Set<Marker> currentMarkers = {
      Marker(
        markerId: const MarkerId('patient_location'),
        position: LatLng(latitude, longitude),
        infoWindow: InfoWindow(
          title: '${widget.patientName} 어르신 위치',
          snippet: isOutOfPath
              ? '🚨 안심 구역/경로 이탈!'
              : (healthStatus == "넘어짐" ? '🚨 비상 상황 발생!' : '안전 이동 중'),
        ),
      ),
    };

    final Set<Circle> safeZoneCircles = {};

    for (int i = 0; i < safeZones.length; i++) {
      var zone = safeZones[i];

      currentMarkers.add(
        Marker(
          markerId: MarkerId('safe_zone_${zone.id}'),
          position: zone.center,
          icon: BitmapDescriptor.defaultMarkerWithHue(BitmapDescriptor.hueBlue),
          infoWindow:
          InfoWindow(title: '🛡️ 안심 구역 ${i + 1} (${zone.radius.toInt()}m)'),
        ),
      );

      safeZoneCircles.add(
        Circle(
          circleId: CircleId('circle_${zone.id}'),
          center: zone.center,
          radius: zone.radius,
          fillColor: Colors.blue.withValues(alpha: 0.15),
          strokeColor: Colors.blueAccent,
          strokeWidth: 2,
        ),
      );
    }

    final Set<Polyline> aiPolylines = SafeZoneService.convertToPolylines(
      aiPathPoints,
      pathColor: Colors.orange,
    );

    return Scaffold(
      appBar: AppBar(
        title: const Text("스마트 조끼 모니터링"),
        backgroundColor: Colors.blueAccent,
        foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: const Icon(Icons.logout),
            tooltip: '로그아웃',
            onPressed: _logout,
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          Padding(
            padding: const EdgeInsets.only(left: 8, bottom: 8),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  "${widget.patientName} (${widget.patientAge}세)",
                  style: const TextStyle(
                      fontSize: 22, fontWeight: FontWeight.bold),
                ),
                const SizedBox(height: 4),
                Text(
                  "담당 보호자: ${widget.guardianName}",
                  style: const TextStyle(fontSize: 14, color: Colors.grey),
                ),
              ],
            ),
          ),
          Card(
            color: isDanger ? Colors.redAccent : Colors.blueAccent,
            child: Padding(
              padding: const EdgeInsets.all(20),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Text(
                    "상태: $healthStatus",
                    style: const TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                        fontWeight: FontWeight.bold),
                  ),
                ],
              ),
            ),
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                child: Text(
                  "📍 안심 구역 (${safeZones.length}개) / AI 경로",
                  style: const TextStyle(
                      fontSize: 18, fontWeight: FontWeight.bold),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
              const SizedBox(width: 8),
              Text(
                "(${latitude.toStringAsFixed(4)}, ${longitude.toStringAsFixed(4)})",
                style: const TextStyle(fontSize: 12, color: Colors.grey),
              ),
            ],
          ),
          Container(
            height: 280,
            margin: const EdgeInsets.symmetric(vertical: 10),
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: Colors.grey.shade300),
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(12),
              child: GoogleMap(
                initialCameraPosition: CameraPosition(
                  target: LatLng(latitude, longitude),
                  zoom: 15.0,
                ),
                markers: currentMarkers,
                circles: safeZoneCircles,
                polylines: aiPolylines,
                onMapCreated: (GoogleMapController controller) {
                  mapController = controller;
                  _moveCamera(latitude, longitude);
                },
                // 💡 사용자가 지도를 조작하기 시작할 때
                onCameraMoveStarted: () {
                  _isUserInteracting = true;
                  _cameraIdleTimer?.cancel();
                },
                // 💡 사용자가 지도를 조작하다 손을 뗐을 때 (움직임 정지) -> 10초 타이머 동작
                onCameraIdle: () {
                  if (_isUserInteracting) {
                    _resetCameraIdleTimer();
                  }
                },
                onTap: (LatLng point) async {
                  final messenger = ScaffoldMessenger.of(context);
                  await SafeZoneService.addSafeZone(point, newZoneRadius);

                  if (!mounted) return;

                  messenger.showSnackBar(
                    SnackBar(
                      content: Text(
                          "🛡️ 새 안심 구역(${newZoneRadius.toInt()}m)이 추가되었습니다."),
                      duration: const Duration(seconds: 1),
                    ),
                  );
                },
                scrollGesturesEnabled: true,
                zoomGesturesEnabled: true,
                rotateGesturesEnabled: true,
                tiltGesturesEnabled: true,
                gestureRecognizers: <Factory<OneSequenceGestureRecognizer>>{
                  Factory<OneSequenceGestureRecognizer>(
                        () => EagerGestureRecognizer(),
                  ),
                },
                myLocationEnabled: false,
                zoomControlsEnabled: true,
              ),
            ),
          ),
          Card(
            elevation: 2,
            child: Padding(
              padding: const EdgeInsets.all(14.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      const Expanded(
                        child: Text(
                          "🛡️ 추가할 구역 반경 (10m 단위)",
                          style: TextStyle(
                              fontSize: 15, fontWeight: FontWeight.bold),
                          overflow: TextOverflow.ellipsis,
                        ),
                      ),
                      Text(
                        "${newZoneRadius.toInt()}m",
                        style: const TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                            color: Colors.blueAccent),
                      ),
                    ],
                  ),
                  const SizedBox(height: 4),
                  const Text(
                    "💡 지도의 원하는 위치를 누르면 아래 설정한 반경의 안심 구역이 추가됩니다.",
                    style: TextStyle(fontSize: 12, color: Colors.grey),
                  ),
                  Slider(
                    value: newZoneRadius,
                    min: 10,
                    max: 150,
                    divisions: 14,
                    label: "${newZoneRadius.toInt()}m",
                    activeColor: Colors.blueAccent,
                    onChanged: (double value) {
                      setState(() {
                        newZoneRadius = value;
                      });
                    },
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      OutlinedButton.icon(
                        onPressed: () async {
                          await SafeZoneService.addSafeZone(
                              LatLng(latitude, longitude), newZoneRadius);
                        },
                        icon: const Icon(Icons.my_location, size: 16),
                        label: const Text("현재위치 구역추가"),
                      ),
                      TextButton.icon(
                        onPressed: () async {
                          await SafeZoneService.clearAllSafeZones();
                        },
                        icon: const Icon(Icons.delete_forever,
                            color: Colors.red, size: 18),
                        label: const Text("구역 전체 삭제",
                            style: TextStyle(color: Colors.red)),
                      ),
                    ],
                  ),
                  if (safeZones.isNotEmpty) ...[
                    const Divider(height: 20),
                    const Text("등록된 구역 목록",
                        style: TextStyle(
                            fontSize: 13, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 6),
                    Column(
                      children: safeZones.asMap().entries.map((entry) {
                        int idx = entry.key;
                        SafeZone zone = entry.value;
                        return Container(
                          margin: const EdgeInsets.only(bottom: 4),
                          padding: const EdgeInsets.symmetric(
                              horizontal: 10, vertical: 6),
                          decoration: BoxDecoration(
                            color: Colors.blue.shade50,
                            borderRadius: BorderRadius.circular(8),
                          ),
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.spaceBetween,
                            children: [
                              Text("구역 ${idx + 1}: 반경 ${zone.radius.toInt()}m",
                                  style: const TextStyle(fontSize: 13)),
                              IconButton(
                                constraints: const BoxConstraints(),
                                padding: EdgeInsets.zero,
                                icon: const Icon(Icons.close,
                                    size: 18, color: Colors.grey),
                                onPressed: () async {
                                  await SafeZoneService.deleteSafeZone(zone.id);
                                },
                              )
                            ],
                          ),
                        );
                      }).toList(),
                    ),
                  ]
                ],
              ),
            ),
          ),
          if (isOutOfPath) ...[
            const SizedBox(height: 10),
            Card(
              color: Colors.orange.shade100,
              child: Padding(
                padding: const EdgeInsets.all(16.0),
                child: Row(
                  children: [
                    const Icon(Icons.add_location_alt,
                        color: Colors.deepOrange, size: 36),
                    const SizedBox(width: 16),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            "⚠️ 경로 및 안심 구역 이탈 감지!",
                            style: TextStyle(
                              fontSize: 16,
                              fontWeight: FontWeight.bold,
                              color: Colors.orange.shade900,
                            ),
                          ),
                          const SizedBox(height: 4),
                          Text(
                            "어르신이 안심 구역과 AI 이동 경로를 모두 벗어났습니다.",
                            style: TextStyle(
                              fontSize: 13,
                              color: Colors.orange.shade800,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
          const SizedBox(height: 16),
          const Text("🚨 낙상 감지 모니터링",
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
          Card(
            color: isDanger ? Colors.red.shade50 : Colors.green.shade50,
            child: Padding(
              padding: const EdgeInsets.all(16.0),
              child: Row(
                children: [
                  Icon(
                    isDanger
                        ? Icons.warning_amber_rounded
                        : Icons.check_circle_outline,
                    color: isDanger ? Colors.red : Colors.green,
                    size: 36,
                  ),
                  const SizedBox(width: 16),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          isDanger ? "어르신이 넘어지셨습니다!" : "현재 안전한 상태입니다.",
                          style: TextStyle(
                            fontSize: 16,
                            fontWeight: FontWeight.bold,
                            color: isDanger
                                ? Colors.red.shade900
                                : Colors.green.shade900,
                          ),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          "판정 결과: $healthStatus",
                          style: TextStyle(
                            fontSize: 14,
                            color: isDanger
                                ? Colors.red.shade700
                                : Colors.green.shade700,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
          if (isFallenLocked) ...[
            const SizedBox(height: 10),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton.icon(
                onPressed: () {
                  setState(() {
                    isFallenLocked = false;
                    healthStatus = "정상";
                    isNotifiedFall = false;
                  });
                  FlutterBackgroundService().invoke('resetFallNotification');
                },
                icon: const Icon(Icons.check_circle, color: Colors.white),
                label: const Text(
                  "보호자 상태 확인 완료 (정상 복구)",
                  style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold),
                ),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blueAccent,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(vertical: 14),
                ),
              ),
            ),
          ],
        ],
      ),
    );
  }
}