// ============================================================
//  RC PLANE FLIGHT CONTROLLER
//  Hardware : Arduino Nano v3 (CH340)
//  IMU      : MPU6050 (I2C – SDA=A4, SCL=A5)
//  Motor    : Brushless A2212 + ESC 30A  (PWM – D9)
//  Servo    : Aileron SG90 (PWM – D3) | Elevator SG90 (PWM – D5)
//  Radio    : Bluetooth HC-05/HC-06 (Serial RX/TX)
//  Library  : MPU6050_tockn  (install via Library Manager)
//             Wire (built-in)
//             Servo (built-in)
// ============================================================

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

// ------------------------------------------------------------
// PIN CONFIGURATION
// ------------------------------------------------------------
#define PIN_ESC      9   // ESC / Motor Brushless  (PWM)
#define PIN_AILERON  3   // Servo Aileron          (PWM)
#define PIN_ELEVATOR 5   // Servo Elevator         (PWM)

// ------------------------------------------------------------
// BLUETOOTH COMMAND MAPPING
// Ganti karakter di sini untuk menyesuaikan aplikasi BT kamu
// ------------------------------------------------------------
#define CMD_THROTTLE_UP    'W'
#define CMD_THROTTLE_DOWN  'S'
#define CMD_ROLL_LEFT      'A'
#define CMD_ROLL_RIGHT     'D'
#define CMD_PITCH_UP       'U'
#define CMD_PITCH_DOWN     'J'
#define CMD_ARM            'X'   // Arm / disarm motor
#define CMD_PID_RESET      'R'   // Reset integral PID (berguna saat tuning)

// ------------------------------------------------------------
// SERVO & THROTTLE LIMITS
// Nilai dalam mikro-detik (us) untuk servo / ESC
// ------------------------------------------------------------
#define SERVO_CENTER   1500   // Posisi netral servo (us)
#define SERVO_MIN      1000   // Defleksi maksimum ke satu arah
#define SERVO_MAX      2000   // Defleksi maksimum ke arah lain
#define SERVO_RANGE     400   // ±400 us dari center → sudut penuh

#define THROTTLE_MIN   1000   // ESC: sinyal minimum (motor mati)
#define THROTTLE_MAX   1900   // ESC: sinyal maksimum (full power)
#define THROTTLE_ARM   1050   // Sinyal arming ESC
#define THROTTLE_STEP    20   // Kenaikan throttle per tombol

// ------------------------------------------------------------
// PID PARAMETERS  – TUNING UTAMA ADA DI SINI
//
//  Kp : Proportional – respons langsung terhadap error sudut.
//       Terlalu besar → servo jitter/osilasi.
//       Terlalu kecil → pesawat lambat koreksi.
//
//  Ki : Integral – koreksi bias/drift jangka panjang.
//       Terlalu besar → overshot & osilasi lambat.
//       Mulai dari 0, naikkan perlahan setelah Kp oke.
//
//  Kd : Derivative – redaman, mencegah overshoot.
//       Terlalu besar → servo bereaksi terhadap noise sensor.
//       Set sekitar 1/10 dari Kp sebagai titik awal.
// ------------------------------------------------------------
struct PIDGains {
  float kp, ki, kd;
};

PIDGains gainRoll     = { 2.5f, 0.02f, 0.8f };
PIDGains gainPitch    = { 2.5f, 0.02f, 0.8f };

// Target stabilisasi (setpoint) – 0 = level flight
float setpointRoll  = 0.0f;
float setpointPitch = 0.0f;

// ------------------------------------------------------------
// OBJEK GLOBAL
// ------------------------------------------------------------
MPU6050 mpu(Wire);
Servo escMotor;
Servo servoAileron;
Servo servoElevator;

// ------------------------------------------------------------
// STATE VARIABLES
// ------------------------------------------------------------
struct PIDState {
  float prevError;
  float integral;
  unsigned long lastTime;
};

PIDState statRoll  = { 0, 0, 0 };
PIDState statPitch = { 0, 0, 0 };

// Input manual dari Bluetooth (dalam us, relatif terhadap center)
int manualRoll     = 0;   // range: -SERVO_RANGE .. +SERVO_RANGE
int manualPitch    = 0;
int throttle       = THROTTLE_MIN;
bool isArmed       = false;

// Pembacaan sensor terkini
float currentRoll  = 0.0f;
float currentPitch = 0.0f;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);   // HC-05 default baud rate

  // --- Inisialisasi I2C & MPU6050 ---
  Wire.begin();
  mpu.begin();
  Serial.println(F("Kalibrasi MPU6050... jaga pesawat tetap datar!"));
  delay(500);
  mpu.calcGyroOffsets(true);   // Auto-kalibrasi gyro (±2 detik)
  Serial.println(F("Kalibrasi selesai."));

  // --- Inisialisasi Servo & ESC ---
  servoAileron.attach(PIN_AILERON);
  servoElevator.attach(PIN_ELEVATOR);
  escMotor.attach(PIN_ESC);

  // Kirim sinyal minimum ke ESC saat boot (prosedur arming standar)
  escMotor.writeMicroseconds(THROTTLE_MIN);
  servoAileron.writeMicroseconds(SERVO_CENTER);
  servoElevator.writeMicroseconds(SERVO_CENTER);

  // Inisialisasi timestamp PID
  unsigned long now = millis();
  statRoll.lastTime  = now;
  statPitch.lastTime = now;

  Serial.println(F("=== RC PLANE CONTROLLER READY ==="));
  Serial.println(F("Kirim 'X' untuk ARM motor."));
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readBluetooth();    // 1. Baca & proses perintah Bluetooth
  readSensor();       // 2. Baca sudut dari MPU6050
  computeAndWrite();  // 3. Hitung PID + mixing + output servo/ESC
}

// ============================================================
//  MODUL 1: readBluetooth()
//  Membaca karakter dari HC-05 dan memperbarui variabel kontrol.
// ============================================================
void readBluetooth() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();

    switch (cmd) {

      // -- Throttle --
      case CMD_THROTTLE_UP:
        if (isArmed) {
          throttle = constrain(throttle + THROTTLE_STEP, THROTTLE_ARM, THROTTLE_MAX);
        }
        break;

      case CMD_THROTTLE_DOWN:
        throttle = constrain(throttle - THROTTLE_STEP, THROTTLE_MIN, THROTTLE_MAX);
        break;

      // -- Roll (Aileron) --
      case CMD_ROLL_LEFT:
        manualRoll = constrain(manualRoll - 50, -SERVO_RANGE, SERVO_RANGE);
        break;

      case CMD_ROLL_RIGHT:
        manualRoll = constrain(manualRoll + 50, -SERVO_RANGE, SERVO_RANGE);
        break;

      // -- Pitch (Elevator) --
      case CMD_PITCH_UP:
        manualPitch = constrain(manualPitch + 50, -SERVO_RANGE, SERVO_RANGE);
        break;

      case CMD_PITCH_DOWN:
        manualPitch = constrain(manualPitch - 50, -SERVO_RANGE, SERVO_RANGE);
        break;

      // -- Arm / Disarm --
      case CMD_ARM:
        isArmed = !isArmed;
        if (!isArmed) {
          throttle = THROTTLE_MIN;
          escMotor.writeMicroseconds(THROTTLE_MIN);
        } else {
          escMotor.writeMicroseconds(THROTTLE_ARM);
          delay(200);
        }
        Serial.print(F("Motor: "));
        Serial.println(isArmed ? F("ARMED") : F("DISARMED"));
        break;

      // -- Reset integral PID (pakai saat tuning) --
      case CMD_PID_RESET:
        statRoll.integral  = 0;
        statPitch.integral = 0;
        Serial.println(F("PID integral reset."));
        break;

      // Karakter tidak dikenal: abaikan
      default:
        break;
    }
  }

  // Auto-return ke netral saat tidak ada input roll/pitch
  // (simulasi spring-back joystick – hapus blok ini jika tidak diperlukan)
  manualRoll  = (int)(manualRoll  * 0.90f);
  manualPitch = (int)(manualPitch * 0.90f);
}

// ============================================================
//  MODUL 2: readSensor()
//  Membaca sudut Roll dan Pitch dari MPU6050.
// ============================================================
void readSensor() {
  mpu.update();
  currentRoll  = mpu.getAngleX();   // sudut roll  (derajat)
  currentPitch = mpu.getAngleY();   // sudut pitch (derajat)
}

// ============================================================
//  HELPER: pidCompute()
//  Menghitung output PID untuk satu sumbu.
//
//  Parameter:
//    gains    – struct {kp, ki, kd}
//    state    – struct {prevError, integral, lastTime}
//    setpoint – target sudut (derajat)
//    measured – sudut terukur saat ini (derajat)
//
//  Return:
//    Koreksi dalam unit us servo (sudah di-scale & di-clamp)
// ============================================================
float pidCompute(const PIDGains& gains, PIDState& state,
                 float setpoint, float measured) {

  unsigned long now = millis();
  float dt = (now - state.lastTime) / 1000.0f;   // konversi ke detik

  // Hindari dt = 0 atau terlalu besar (misal setelah resume dari sleep)
  if (dt <= 0.0f || dt > 0.5f) {
    state.lastTime = now;
    return 0.0f;
  }

  float error = setpoint - measured;

  // Proportional
  float pTerm = gains.kp * error;

  // Integral (dengan anti-windup ±150 us)
  state.integral += error * dt;
  state.integral  = constrain(state.integral, -150.0f / gains.ki,
                                               150.0f / gains.ki);
  float iTerm = gains.ki * state.integral;

  // Derivative (berdasarkan perubahan error, bukan setpoint)
  float dTerm = gains.kd * (error - state.prevError) / dt;

  state.prevError = error;
  state.lastTime  = now;

  // Total output, clamp ke ±SERVO_RANGE
  float output = pTerm + iTerm + dTerm;
  return constrain(output, -(float)SERVO_RANGE, (float)SERVO_RANGE);
}

// ============================================================
//  MODUL 3: computeAndWrite()
//  Mixing PID + input manual, lalu kirim ke servo & ESC.
//
//  --- PENJELASAN MIXING LOGIC ---
//
//  Output akhir servo = Input Manual (pilot) + Koreksi PID (autopilot)
//
//  Analogi: Setpoint PID bergeser sesuai input pilot.
//  Ketika pilot menekan "roll kanan" (manualRoll > 0),
//  setpointRoll digeser ke +X derajat. PID kemudian bekerja
//  untuk membawa pesawat ke sudut itu. Ini berarti:
//
//   • Saat stick netral (manual=0) → setpoint=0 → PID menjaga
//     pesawat tetap level secara otomatis.
//   • Saat pilot memberi input → setpoint bergeser → pesawat
//     berbelok, tapi PID tetap mencegah overshoot/osilasi.
//
//  Keuntungan pendekatan ini vs additive mixing biasa:
//   - PID selalu aktif, tidak ada "gap" antara mode manual & auto.
//   - Pesawat terasa "dibantu" bukan "dilawan" oleh autopilot.
// ============================================================
void computeAndWrite() {

  // --- Hitung setpoint dinamis dari input manual ---
  // Manual input (±SERVO_RANGE) dikonversi ke derajat target
  // Skala: SERVO_RANGE us → MAX_ANGLE derajat
  const float MAX_ANGLE = 30.0f;   // batas bank angle (derajat)
  float dynamicSetpointRoll  = setpointRoll  + (manualRoll  / (float)SERVO_RANGE) * MAX_ANGLE;
  float dynamicSetpointPitch = setpointPitch + (manualPitch / (float)SERVO_RANGE) * MAX_ANGLE;

  // --- Hitung koreksi PID ---
  float pidRoll  = pidCompute(gainRoll,  statRoll,  dynamicSetpointRoll,  currentRoll);
  float pidPitch = pidCompute(gainPitch, statPitch, dynamicSetpointPitch, currentPitch);

  // --- Konversi output PID ke posisi servo (us) ---
  int aileronUs  = SERVO_CENTER + (int)pidRoll;
  int elevatorUs = SERVO_CENTER + (int)pidPitch;

  // Clamp ke batas servo yang aman
  aileronUs  = constrain(aileronUs,  SERVO_MIN, SERVO_MAX);
  elevatorUs = constrain(elevatorUs, SERVO_MIN, SERVO_MAX);

  // --- Tulis ke hardware ---
  servoAileron.writeMicroseconds(aileronUs);
  servoElevator.writeMicroseconds(elevatorUs);

  if (isArmed) {
    escMotor.writeMicroseconds(throttle);
  } else {
    escMotor.writeMicroseconds(THROTTLE_MIN);
  }

  // --- Debug output (hapus / comment untuk produksi) ---
  // Uncomment baris di bawah untuk memonitor via Serial Monitor
  /*
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print(F("R:")); Serial.print(currentRoll, 1);
    Serial.print(F(" P:")); Serial.print(currentPitch, 1);
    Serial.print(F(" pidR:")); Serial.print(pidRoll, 1);
    Serial.print(F(" pidP:")); Serial.print(pidPitch, 1);
    Serial.print(F(" Ail:")); Serial.print(aileronUs);
    Serial.print(F(" Ele:")); Serial.print(elevatorUs);
    Serial.print(F(" Thr:")); Serial.println(throttle);
    lastPrint = millis();
  }
  */
}
