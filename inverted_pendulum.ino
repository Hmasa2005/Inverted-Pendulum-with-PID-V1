#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <math.h> // 念のため追加（fmin使用時）
//タスク管理
const unsigned long task1Interval = 10;
const unsigned long task2Interval = 0;

// タスクの前回実行時間
unsigned long previousTask1 = 0;
unsigned long previousTask2 = 0;

/* === 定数設定 === */
#define BNO055_SAMPLERATE_DELAY_MS (50)  // 更新間隔 50ms（20Hz）

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* === PIDパラメータ === */
float Kp = 20;     // 比例ゲイン
float Ki = 10.0;    // 積分ゲイン
float Kd = 0;     // 微分ゲイン

float targetAngle =0;  // 目標角度（垂直位置）
float integral = 0;
float prevError = 0;
float pidValue = 0;

// ===== ピン定義 =====
const int DIR_R  = 18;
const int STEP_R = 19;
const int DIR_L  = 16;
const int STEP_L = 17;
const int EN     = 4;

// ===== モーター設定 =====
const int STEPS_PER_REV_FULL = 200;   // フルステップ数
const int MICROSTEP = 16;             // マイクロステップ設定
const int STEPS_PER_REV = STEPS_PER_REV_FULL * MICROSTEP; // 3200ステップ = 1回転
float pidOutput = 0;
int moter_status = 0;

/* === PID制御関数 === */
float computePID(float input) {
  float error = targetAngle - input;
  integral += error * (BNO055_SAMPLERATE_DELAY_MS / 1000.0);
  integral = constrain(integral, -50, 50);
  float derivative = (error - prevError) / (BNO055_SAMPLERATE_DELAY_MS / 1000.0);
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // 出力制限（±255相当）
  if (output > 255) output = 255;
  if (output < -255) output = -255;

  return output;
}

unsigned long lastStepMicros = 0;
float stepDelay = 500;  // 初期値

void stepMotor() {
  unsigned long now = micros();
  float absOutput = fabs(pidOutput);

  // デッドゾーン（ほぼ垂直なら停止）
  if (absOutput < 3) return;

  // PID出力から速度（間隔）を算出

  // float absOutput = fabs(pidOutput);  // PID出力の絶対値を取得

  // PID出力0〜255をステップ間隔500〜50usにマッピング
  float stepDelay = 500.0 - (fmin(absOutput, 255.0) / 255.0) * (500.0 - 30.0);
  stepDelay = constrain(stepDelay, 30.0, 500.0);

  // stepDelay μs経過したら1パルス出す
  if (now - lastStepMicros >= stepDelay) {
    lastStepMicros = now;

    // 方向設定
    if (pidOutput > 0) {
      digitalWrite(DIR_R, LOW);
      digitalWrite(DIR_L, LOW);
    } else {
      digitalWrite(DIR_R, HIGH);
      digitalWrite(DIR_L, HIGH);
    }

    // ステップトグル
    digitalWrite(STEP_R, !digitalRead(STEP_R));
    digitalWrite(STEP_L, !digitalRead(STEP_L));
  }
}




void task1(){
  sensors_event_t event;
  bno.getEvent(&event);

  float angleY = event.orientation.y;  // Y軸角度（-180〜180）
  pidOutput = computePID(angleY);
  
  // 結果を出力
  Serial.print("Y Angle: ");
  Serial.print(angleY, 2);
  Serial.print("\tTarget: ");
  Serial.print(targetAngle);
  Serial.print("\tPID Output: ");
  Serial.println(pidOutput, 2);

  // delay(BNO055_SAMPLERATE_DELAY_MS);
}

void task2(){
  stepMotor();
}
/* === 初期化 === */
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("BNO055 Y-Axis PID Controller (No Motor Output)");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected. Check wiring!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized.");

  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(EN, LOW);      // ドライバ有効
  digitalWrite(DIR_R, LOW);  // 右モータ正転
  digitalWrite(DIR_L, LOW);   // 左モータ逆転
}


/* === メインループ === */
void loop() {
  unsigned long currentMillis = millis();

// --- タスク1 ---
  if (currentMillis - previousTask1 >= task1Interval) {
    previousTask1 = currentMillis; // 前回実行時間を更新
    task1();
  }
// --- タスク2 ---
  if (currentMillis - previousTask2 >= task2Interval) {
    previousTask2 = currentMillis;
    task2();
  }
  
}
