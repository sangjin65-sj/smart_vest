#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <cmath>

// =========================================================
// Network Config
// =========================================================
namespace NetworkConfig {

#define WIFI_SSID "Scm"
#define WIFI_PASSWORD "ss3184521"

#define DATABASE_URL \
    "https://smartvest-3069b-default-rtdb.firebaseio.com"

#define DATABASE_SECRET \
    "k4G5HSOa0fxaWqOocHCrJE6djgF6LWoqyxFKRvTF"

}

// =========================================================
// Pin Config
// =========================================================
namespace PinConfig {

constexpr uint8_t MPU_SDA_PIN = 21;
constexpr uint8_t MPU_SCL_PIN = 22;

constexpr int8_t GPS_RX_PIN = 16;
constexpr int8_t GPS_TX_PIN = 17;

}

// =========================================================
// MPU Config
// =========================================================
namespace MpuConfig {

// ---------------------------------------------------------
// ★ MPU 측정 주기
//
// 20ms = 50Hz
// ---------------------------------------------------------
constexpr uint32_t SAMPLE_INTERVAL_MS = 20;

// MPU6050 설정
constexpr uint32_t I2C_CLOCK_HZ = 100000;

constexpr uint8_t DEFAULT_ADDRESS = 0x68;
constexpr uint8_t SECONDARY_ADDRESS = 0x69;

// MPU6050 Register
constexpr uint8_t REG_POWER_MANAGEMENT = 0x6B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_DATA_START = 0x3B;

// ±16g
constexpr uint8_t ACCEL_CONFIG_16G = 0x18;

// ±500 deg/s
constexpr uint8_t GYRO_CONFIG_500_DEG = 0x08;

// RAW 변환 값
constexpr float ACCEL_SCALE_16G = 2048.0F;
constexpr float GYRO_SCALE_500_DEG = 65.5F;

// 중력 가속도
constexpr float GRAVITY_MS2 = 9.80665F;

// ---------------------------------------------------------
// ★ Gyro Bias Calibration
// ---------------------------------------------------------
constexpr size_t GYRO_CALIBRATION_SAMPLE_COUNT = 200;

constexpr uint32_t GYRO_CALIBRATION_DELAY_MS = 5;

// ---------------------------------------------------------
// ★ EMA Filter
//
// RAW 충격 판정에는 사용하지 않음.
// 자세/무동작 판단용.
// ---------------------------------------------------------
constexpr float EMA_ALPHA = 0.25F;

// ---------------------------------------------------------
// 낙상 후보 초기 Threshold
// ---------------------------------------------------------
constexpr float FALL_ACCEL_THRESHOLD_G = 2.8F;

constexpr float FALL_GYRO_THRESHOLD_DEG_S = 150.0F;

}

// =========================================================
// GPS Config
// =========================================================
namespace GpsConfig {

constexpr uint32_t GPS_SERIAL_BAUD = 9600;

// 기존 코드와 동일
constexpr uint32_t MAX_LOCATION_AGE_MS = 5000;

}

// =========================================================
// Transmission Config
// =========================================================
namespace TransmissionConfig {

// Firebase 전송은 그대로 1초
constexpr uint32_t DATA_SEND_INTERVAL_MS = 1000;

}

// =========================================================
// MPU Mode
// =========================================================
enum class MpuMode {

    UNAVAILABLE,

    ADAFRUIT_LIBRARY,

    RAW_COMPATIBLE

};

// =========================================================
// MPU Data
// =========================================================
struct MotionData {

    float accelX = 0.0F;
    float accelY = 0.0F;
    float accelZ = 0.0F;

    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    float accelMagnitude = 0.0F;

    float gyroMagnitude = 0.0F;
};

// =========================================================
// Gyroscope Bias
// =========================================================
struct GyroBias {

    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

// =========================================================
// Hardware Objects
// =========================================================
Adafruit_MPU6050 mpu;

HardwareSerial gpsSerial(2);

TinyGPSPlus gps;

FirebaseData fbdo;

FirebaseAuth auth;

FirebaseConfig config;

// =========================================================
// Runtime State
// =========================================================
MpuMode mpuMode =
    MpuMode::UNAVAILABLE;

uint8_t activeAddress =
    MpuConfig::DEFAULT_ADDRESS;

// Firebase에 전달할 최신 RAW 값
MotionData latestRawMotion;

// 내부 판단에 사용할 Filtered 값
MotionData filteredMotion;

// Gyro Offset
GyroBias gyroBias;

// 필터 최초 초기화 여부
bool filterInitialized = false;

// ---------------------------------------------------------
// ★ 1초 동안 최대 충격 보존
//
// 순간 낙상을 놓치지 않도록 함.
// ---------------------------------------------------------
float peakAccelerationG = 0.0F;

float peakGyroDegS = 0.0F;

uint32_t lastMpuSampleMillis = 0;

uint32_t lastSendMillis = 0;

// GPS
double currentLat = 0.0;

double currentLng = 0.0;

bool isGpsFixed = false;

// =========================================================
// I2C Register Write
// =========================================================
bool writeRegister(
    const uint8_t devAddr,
    const uint8_t regAddr,
    const uint8_t value
) {

    Wire.beginTransmission(
        devAddr
    );

    Wire.write(
        regAddr
    );

    Wire.write(
        value
    );

    return Wire.endTransmission() == 0;
}

// =========================================================
// I2C Register Read
// =========================================================
bool readRegisters(
    const uint8_t devAddr,
    const uint8_t startReg,
    uint8_t* buffer,
    const size_t length
) {

    Wire.beginTransmission(
        devAddr
    );

    Wire.write(
        startReg
    );

    if (
        Wire.endTransmission(false) != 0
    ) {

        return false;
    }

    const size_t receivedLength =
        Wire.requestFrom(
            devAddr,
            static_cast<uint8_t>(length)
        );

    if (
        receivedLength != length
    ) {

        return false;
    }

    for (
        size_t index = 0;
        index < length;
        ++index
    ) {

        buffer[index] =
            static_cast<uint8_t>(
                Wire.read()
            );
    }

    return true;
}

// =========================================================
// I2C Scanner
// =========================================================
uint8_t scanI2cAddress() {

    Serial.println(
        "I2C 장치 스캔 중..."
    );

    uint8_t foundAddress = 0;

    for (
        uint8_t address = 1;
        address < 127;
        ++address
    ) {

        Wire.beginTransmission(
            address
        );

        if (
            Wire.endTransmission() != 0
        ) {

            continue;
        }

        Serial.printf(
            "I2C 장치 발견! 주소: 0x%02X\n",
            address
        );

        if (
            address == MpuConfig::DEFAULT_ADDRESS ||
            address == MpuConfig::SECONDARY_ADDRESS
        ) {

            foundAddress =
                address;
        }
    }

    return foundAddress;
}

// =========================================================
// MPU Setup
// =========================================================
bool setupMpu() {

    uint8_t detectedAddress =
        scanI2cAddress();

    if (
        detectedAddress == 0
    ) {

        detectedAddress =
            MpuConfig::DEFAULT_ADDRESS;
    }

    // =====================================================
    // Adafruit Mode
    // =====================================================
    if (
        mpu.begin(
            detectedAddress,
            &Wire
        )
    ) {

        mpu.setAccelerometerRange(
            MPU6050_RANGE_16_G
        );

        mpu.setGyroRange(
            MPU6050_RANGE_500_DEG
        );

        // MPU 내부 DLPF
        mpu.setFilterBandwidth(
            MPU6050_BAND_21_HZ
        );

        activeAddress =
            detectedAddress;

        mpuMode =
            MpuMode::ADAFRUIT_LIBRARY;

        Serial.printf(
            "MPU6050 Adafruit 모드 연결 성공 (0x%02X)\n",
            detectedAddress
        );

        return true;
    }

    // =====================================================
    // RAW Compatible Mode
    // =====================================================
    Serial.println(
        "Adafruit 초기화 실패 -> RAW 모드 시도"
    );

    if (
        !writeRegister(
            detectedAddress,
            MpuConfig::REG_POWER_MANAGEMENT,
            0x00
        )
    ) {

        return false;
    }

    delay(50);

    if (
        !writeRegister(
            detectedAddress,
            MpuConfig::REG_ACCEL_CONFIG,
            MpuConfig::ACCEL_CONFIG_16G
        )
    ) {

        return false;
    }

    if (
        !writeRegister(
            detectedAddress,
            MpuConfig::REG_GYRO_CONFIG,
            MpuConfig::GYRO_CONFIG_500_DEG
        )
    ) {

        return false;
    }

    uint8_t testBuffer[14] = {};

    if (
        !readRegisters(
            detectedAddress,
            MpuConfig::REG_ACCEL_DATA_START,
            testBuffer,
            sizeof(testBuffer)
        )
    ) {

        return false;
    }

    activeAddress =
        detectedAddress;

    mpuMode =
        MpuMode::RAW_COMPATIBLE;

    Serial.printf(
        "MPU6050 RAW 호환 모드 연결 성공 (0x%02X)\n",
        detectedAddress
    );

    return true;
}

// =========================================================
// Adafruit MPU Read
// =========================================================
bool readMpuAdafruit(
    MotionData& result
) {

    sensors_event_t accelerationEvent;
    sensors_event_t gyroEvent;
    sensors_event_t temperatureEvent;

    if (
        !mpu.getEvent(
            &accelerationEvent,
            &gyroEvent,
            &temperatureEvent
        )
    ) {

        return false;
    }

    // m/s² → g
    result.accelX =
        accelerationEvent.acceleration.x /
        MpuConfig::GRAVITY_MS2;

    result.accelY =
        accelerationEvent.acceleration.y /
        MpuConfig::GRAVITY_MS2;

    result.accelZ =
        accelerationEvent.acceleration.z /
        MpuConfig::GRAVITY_MS2;

    // rad/s → deg/s
    result.gyroX =
        gyroEvent.gyro.x *
        RAD_TO_DEG;

    result.gyroY =
        gyroEvent.gyro.y *
        RAD_TO_DEG;

    result.gyroZ =
        gyroEvent.gyro.z *
        RAD_TO_DEG;

    return true;
}

// =========================================================
// RAW MPU Read
// =========================================================
bool readMpuRaw(
    MotionData& result
) {

    uint8_t rawBuffer[14] = {};

    if (
        !readRegisters(
            activeAddress,
            MpuConfig::REG_ACCEL_DATA_START,
            rawBuffer,
            sizeof(rawBuffer)
        )
    ) {

        return false;
    }

    const int16_t rawAX =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[0]
                ) << 8
            ) |
            rawBuffer[1]
        );

    const int16_t rawAY =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[2]
                ) << 8
            ) |
            rawBuffer[3]
        );

    const int16_t rawAZ =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[4]
                ) << 8
            ) |
            rawBuffer[5]
        );

    const int16_t rawGX =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[8]
                ) << 8
            ) |
            rawBuffer[9]
        );

    const int16_t rawGY =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[10]
                ) << 8
            ) |
            rawBuffer[11]
        );

    const int16_t rawGZ =
        static_cast<int16_t>(
            (
                static_cast<uint16_t>(
                    rawBuffer[12]
                ) << 8
            ) |
            rawBuffer[13]
        );

    result.accelX =
        static_cast<float>(rawAX) /
        MpuConfig::ACCEL_SCALE_16G;

    result.accelY =
        static_cast<float>(rawAY) /
        MpuConfig::ACCEL_SCALE_16G;

    result.accelZ =
        static_cast<float>(rawAZ) /
        MpuConfig::ACCEL_SCALE_16G;

    result.gyroX =
        static_cast<float>(rawGX) /
        MpuConfig::GYRO_SCALE_500_DEG;

    result.gyroY =
        static_cast<float>(rawGY) /
        MpuConfig::GYRO_SCALE_500_DEG;

    result.gyroZ =
        static_cast<float>(rawGZ) /
        MpuConfig::GYRO_SCALE_500_DEG;

    return true;
}

// =========================================================
// Unified MPU Read
// =========================================================
bool readMpu(
    MotionData& result
) {

    switch (
        mpuMode
    ) {

        case MpuMode::ADAFRUIT_LIBRARY:

            return readMpuAdafruit(
                result
            );

        case MpuMode::RAW_COMPATIBLE:

            return readMpuRaw(
                result
            );

        case MpuMode::UNAVAILABLE:

        default:

            return false;
    }
}

// =========================================================
// Magnitude
// =========================================================
float calculateMagnitude(
    const float x,
    const float y,
    const float z
) {

    return sqrtf(
        x * x +
        y * y +
        z * z
    );
}

// =========================================================
// EMA
// =========================================================
float applyEma(
    const float previous,
    const float current
) {

    return
        (
            MpuConfig::EMA_ALPHA *
            current
        ) +
        (
            (
                1.0F -
                MpuConfig::EMA_ALPHA
            ) *
            previous
        );
}

// =========================================================
// ★ Gyroscope Bias Calibration
// =========================================================
void calibrateGyroscope() {

    if (
        mpuMode ==
        MpuMode::UNAVAILABLE
    ) {

        return;
    }

    Serial.println();

    Serial.println(
        "Gyro Bias 보정 시작"
    );

    Serial.println(
        "MPU6050을 약 1초간 움직이지 마세요."
    );

    float sumX = 0.0F;
    float sumY = 0.0F;
    float sumZ = 0.0F;

    size_t validCount = 0;

    for (
        size_t index = 0;
        index <
            MpuConfig::
                GYRO_CALIBRATION_SAMPLE_COUNT;
        ++index
    ) {

        MotionData sample;

        if (
            readMpu(
                sample
            )
        ) {

            sumX +=
                sample.gyroX;

            sumY +=
                sample.gyroY;

            sumZ +=
                sample.gyroZ;

            ++validCount;
        }

        delay(
            MpuConfig::
                GYRO_CALIBRATION_DELAY_MS
        );
    }

    if (
        validCount == 0
    ) {

        Serial.println(
            "Gyro Bias 보정 실패"
        );

        return;
    }

    const float count =
        static_cast<float>(
            validCount
        );

    gyroBias.x =
        sumX / count;

    gyroBias.y =
        sumY / count;

    gyroBias.z =
        sumZ / count;

    Serial.printf(
        "Gyro Bias 완료 | X: %.3f | Y: %.3f | Z: %.3f deg/s\n",
        gyroBias.x,
        gyroBias.y,
        gyroBias.z
    );

    Serial.println();
}

// =========================================================
// ★ MPU Preprocessing
// =========================================================
void preprocessMotion(
    const MotionData& raw
) {

    MotionData corrected =
        raw;

    // -----------------------------------------------------
    // Gyro Bias 제거
    // -----------------------------------------------------
    corrected.gyroX -=
        gyroBias.x;

    corrected.gyroY -=
        gyroBias.y;

    corrected.gyroZ -=
        gyroBias.z;

    // -----------------------------------------------------
    // 최초 데이터
    // -----------------------------------------------------
    if (
        !filterInitialized
    ) {

        filteredMotion =
            corrected;

        filterInitialized =
            true;

    } else {

        filteredMotion.accelX =
            applyEma(
                filteredMotion.accelX,
                corrected.accelX
            );

        filteredMotion.accelY =
            applyEma(
                filteredMotion.accelY,
                corrected.accelY
            );

        filteredMotion.accelZ =
            applyEma(
                filteredMotion.accelZ,
                corrected.accelZ
            );

        filteredMotion.gyroX =
            applyEma(
                filteredMotion.gyroX,
                corrected.gyroX
            );

        filteredMotion.gyroY =
            applyEma(
                filteredMotion.gyroY,
                corrected.gyroY
            );

        filteredMotion.gyroZ =
            applyEma(
                filteredMotion.gyroZ,
                corrected.gyroZ
            );
    }

    filteredMotion.accelMagnitude =
        calculateMagnitude(
            filteredMotion.accelX,
            filteredMotion.accelY,
            filteredMotion.accelZ
        );

    filteredMotion.gyroMagnitude =
        calculateMagnitude(
            filteredMotion.gyroX,
            filteredMotion.gyroY,
            filteredMotion.gyroZ
        );
}

// =========================================================
// ★ MPU 50Hz Sampling
// =========================================================
void updateMpuSampling(
    const uint32_t currentMillis
) {

    if (
        currentMillis -
            lastMpuSampleMillis <
        MpuConfig::SAMPLE_INTERVAL_MS
    ) {

        return;
    }

    lastMpuSampleMillis =
        currentMillis;

    MotionData raw;

    if (
        !readMpu(
            raw
        )
    ) {

        return;
    }

    // -----------------------------------------------------
    // RAW 합성값
    //
    // 순간 충격은 필터하지 않은 값을 사용
    // -----------------------------------------------------
    raw.accelMagnitude =
        calculateMagnitude(
            raw.accelX,
            raw.accelY,
            raw.accelZ
        );

    raw.gyroMagnitude =
        calculateMagnitude(
            raw.gyroX,
            raw.gyroY,
            raw.gyroZ
        );

    // -----------------------------------------------------
    // Firebase에 보낼 최신 센서 값
    //
    // 기존 방식과 동일하게 RAW 사용
    // -----------------------------------------------------
    latestRawMotion =
        raw;

    // -----------------------------------------------------
    // 1초 동안 Peak 보존
    // -----------------------------------------------------
    if (
        raw.accelMagnitude >
        peakAccelerationG
    ) {

        peakAccelerationG =
            raw.accelMagnitude;
    }

    if (
        raw.gyroMagnitude >
        peakGyroDegS
    ) {

        peakGyroDegS =
            raw.gyroMagnitude;
    }

    // -----------------------------------------------------
    // 별도의 전처리 데이터 생성
    // -----------------------------------------------------
    preprocessMotion(
        raw
    );
}

// =========================================================
// GPS
//
// ★ 기존 코드 거의 그대로
// =========================================================
void readGps() {

    while (
        gpsSerial.available() > 0
    ) {

        const char character =
            static_cast<char>(
                gpsSerial.read()
            );

        gps.encode(
            character
        );
    }

    if (
        gps.location.isValid() &&
        gps.location.age() <
            GpsConfig::MAX_LOCATION_AGE_MS
    ) {

        currentLat =
            gps.location.lat();

        currentLng =
            gps.location.lng();

        isGpsFixed =
            true;

    } else {

        isGpsFixed =
            false;
    }
}

// =========================================================
// Setup
// =========================================================
void setup() {

    Serial.begin(
        115200
    );

    delay(
        1000
    );

    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "ESP32 MPU6050 + GPS 통합 전송 시스템"
    );

    Serial.println(
        "=========================================="
    );

    // =====================================================
    // 1. GPS
    // =====================================================
    gpsSerial.begin(
        GpsConfig::GPS_SERIAL_BAUD,
        SERIAL_8N1,
        PinConfig::GPS_RX_PIN,
        PinConfig::GPS_TX_PIN
    );

    Serial.println(
        "GPS UART 연결 완료"
    );

    // =====================================================
    // 2. Wi-Fi
    //
    // 기존 방식 유지
    // =====================================================
    WiFi.mode(
        WIFI_STA
    );

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print(
        "Wi-Fi 연결 중"
    );

    int retry = 0;

    while (
        WiFi.status() !=
            WL_CONNECTED &&
        retry < 30
    ) {

        delay(
            500
        );

        Serial.print(
            "."
        );

        ++retry;
    }

    if (
        WiFi.status() ==
        WL_CONNECTED
    ) {

        Serial.println();

        Serial.print(
            "Wi-Fi 연결 성공! IP: "
        );

        Serial.println(
            WiFi.localIP()
        );

    } else {

        Serial.println();

        Serial.println(
            "Wi-Fi 연결 실패 "
            "(오프라인 모드로 계속 진행)"
        );
    }

    // =====================================================
    // 3. Firebase
    //
    // ★ 기존 코드 그대로
    // =====================================================
    config.host =
        DATABASE_URL;

    config.signer.tokens.legacy_token =
        DATABASE_SECRET;

    Firebase.begin(
        &config,
        &auth
    );

    Firebase.reconnectWiFi(
        true
    );

    // =====================================================
    // 4. I2C
    // =====================================================
    Wire.begin(
        PinConfig::MPU_SDA_PIN,
        PinConfig::MPU_SCL_PIN
    );

    Wire.setClock(
        MpuConfig::I2C_CLOCK_HZ
    );

    // =====================================================
    // 5. MPU6050
    // =====================================================
    if (
        !setupMpu()
    ) {

        Serial.println(
            "MPU6050 초기화 실패 "
            "(배선 확인 필요)"
        );

        mpuMode =
            MpuMode::UNAVAILABLE;

    } else {

        // ★ 추가된 전처리
        calibrateGyroscope();
    }

    const uint32_t currentMillis =
        millis();

    lastMpuSampleMillis =
        currentMillis;

    lastSendMillis =
        currentMillis;

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "시스템 시작"
    );

    Serial.println();
}

// =========================================================
// Main Loop
// =========================================================
void loop() {

    const uint32_t currentMillis =
        millis();

    // =====================================================
    // GPS는 기존처럼 계속 읽기
    // =====================================================
    readGps();

    // =====================================================
    // ★ MPU는 50Hz로 계속 읽기
    // =====================================================
    updateMpuSampling(
        currentMillis
    );

    // =====================================================
    // Firebase는 기존처럼 1초마다
    // =====================================================
    if (
        currentMillis -
            lastSendMillis <
        TransmissionConfig::
            DATA_SEND_INTERVAL_MS
    ) {

        return;
    }

    lastSendMillis =
        currentMillis;

    // =====================================================
    // 기존 변수와 동일한 의미
    // =====================================================
    const float ax =
        latestRawMotion.accelX;

    const float ay =
        latestRawMotion.accelY;

    const float az =
        latestRawMotion.accelZ;

    const float gx =
        latestRawMotion.gyroX;

    const float gy =
        latestRawMotion.gyroY;

    const float gz =
        latestRawMotion.gyroZ;

    // =====================================================
    // ★ 낙상 "후보" 판단
    //
    // 1초 중 최대값을 사용
    // =====================================================
    const bool isFallDetected =
        (
            peakAccelerationG >=
            MpuConfig::
                FALL_ACCEL_THRESHOLD_G
        ) ||
        (
            peakGyroDegS >=
            MpuConfig::
                FALL_GYRO_THRESHOLD_DEG_S
        );

    // =====================================================
    // Serial Debug
    // =====================================================
    Serial.println(
        "------------------------------------------"
    );

    Serial.printf(
        "가속도(G) | X:%6.3f | Y:%6.3f | Z:%6.3f\n",
        ax,
        ay,
        az
    );

    Serial.printf(
        "각속도(deg/s) | X:%6.2f | Y:%6.2f | Z:%6.2f\n",
        gx,
        gy,
        gz
    );

    Serial.printf(
        "Peak | Acc: %.2fG | Gyro: %.2f deg/s\n",
        peakAccelerationG,
        peakGyroDegS
    );

    Serial.printf(
        "Filtered | AccMag: %.2fG | GyroMag: %.2f deg/s\n",
        filteredMotion.accelMagnitude,
        filteredMotion.gyroMagnitude
    );

    Serial.printf(
        "낙상 후보: %s\n",
        isFallDetected
            ? "YES"
            : "NO"
    );

    if (
        isGpsFixed
    ) {

        Serial.printf(
            "위치(GPS) | Lat: %.6f | Lng: %.6f | 위성수: %lu\n",
            currentLat,
            currentLng,
            static_cast<unsigned long>(
                gps.satellites.isValid()
                    ? gps.satellites.value()
                    : 0
            )
        );

    } else {

        Serial.printf(
            "위치(GPS) | 위성 탐색 중... | 위성수: %lu\n",
            static_cast<unsigned long>(
                gps.satellites.isValid()
                    ? gps.satellites.value()
                    : 0
            )
        );
    }

    // =====================================================
    // Firebase
    //
    // ★ 기존 구조 그대로
    // =====================================================
    if (
        WiFi.status() ==
            WL_CONNECTED &&
        Firebase.ready()
    ) {

        FirebaseJson json;

        json.set(
            "accel_x",
            ax
        );

        json.set(
            "accel_y",
            ay
        );

        json.set(
            "accel_z",
            az
        );

        json.set(
            "gyro_x",
            gx
        );

        json.set(
            "gyro_y",
            gy
        );

        json.set(
            "gyro_z",
            gz
        );

        json.set(
            "latitude",
            currentLat
        );

        json.set(
            "longitude",
            currentLng
        );

        json.set(
            "is_fall",
            isFallDetected
        );

        if (
            Firebase.RTDB.updateNode(
                &fbdo,
                "/sensor_data/test",
                &json
            )
        ) {

            Serial.println(
                "[Firebase] 센서 데이터 및 is_fall 상태 전송 성공!"
            );

        } else {

            Serial.printf(
                "[Firebase] 전송 실패: %s\n",
                fbdo.errorReason().c_str()
            );
        }
    }

    // =====================================================
    // ★ 다음 1초 측정을 위해 Peak 초기화
    // =====================================================
    peakAccelerationG = 0.0F;

    peakGyroDegS = 0.0F;
}