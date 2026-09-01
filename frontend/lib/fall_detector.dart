import 'dart:math';

class FallDetector {
  static double parseToDouble(dynamic val, double fallback) {
    if (val == null) return fallback;
    if (val is num) return val.toDouble();
    if (val is String) return double.tryParse(val) ?? fallback;
    return fallback;
  }

  // 가속도/자이로 센서 데이터 기반 낙상 여부 판정
  static bool checkFall(Map<dynamic, dynamic> data) {
    double accelX = parseToDouble(data["accel_x"], 0);
    double accelY = parseToDouble(data["accel_y"], 0);
    double accelZ = parseToDouble(data["accel_z"], 0);

    double gyroX = parseToDouble(data["gyro_x"], 0);
    double gyroY = parseToDouble(data["gyro_y"], 0);
    double gyroZ = parseToDouble(data["gyro_z"], 0);

    double totalAcceleration =
    sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    double totalGyroscope =
    sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

    bool isImpactWithRotation =
        (totalAcceleration > 12.0) && (totalGyroscope > 180.0);
    bool isExtremeImpact = totalAcceleration > 18.0;
    bool isFallenPose =
        (accelZ < 3.0) && (totalGyroscope > 100.0 || totalAcceleration > 12.0);
    bool isHighGyroscope = totalGyroscope > 250.0;

    return isImpactWithRotation ||
        isExtremeImpact ||
        isFallenPose ||
        isHighGyroscope;
  }
}