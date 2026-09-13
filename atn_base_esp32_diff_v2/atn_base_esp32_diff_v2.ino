// ==========================================================
//  Auto-Trash Navigator — base controller (ESP32) / 差動2輪版
//  3S LiPo / メカナム Ø80 / MD10C ×2 (左右ペア並列) / エンコーダ ×4
//
//  ★メカナムを左右ペアで駆動する場合、旋回の実効モーメントアームは
//    物理トレッドの半分 (0.180) ではなく lx+ly = 0.300。
//    45°ローラーが横方向の滑りを許すため。
//    diff_drive_controller の wheel_separation = 0.600 と一致させること。
//
//  修正: 開ループ指令(l/r)が PID ループに上書きされる不具合を修正
// ==========================================================

#include <driver/pcnt.h>

#define TEST_MODE 1

// ---------- 車体パラメータ ----------
const float WHEEL_R  = 0.040f;   // m
const float HALF_SEP = 0.300f;   // ★lx+ly。0.180 ではない
const float CPR      = 755.0f;   // ★実測値(7545 / 10回転)。再測定したら更新

// ---------- 電源保護 ----------
// モーター定格 9V、3S満充電 12.6V → duty上限 71%
const int DUTY_MAX = 182;
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// ---------- ピン ----------
// 左 = MD10C ① (FL + RL 並列) / 右 = MD10C ② (FR + RR 並列)
const int PWM_PIN[2]  = {13, 14};
const int DIR_PIN[2]  = {27, 32};
const int DIR_SIGN[2] = {+1, -1};   // ★実機で確認。右は鏡像なので反転のはず

// エンコーダは4輪ぶん読む（スリップ検出のため）
const int ENC_A[4] = {16, 18, 21, 23};   // FL FR RL RR
const int ENC_B[4] = {17, 19, 22,  5};
const int ENC_SIGN[4] = {+1, -1, +1, -1};

const int RELAY_SENSE = 34;

const pcnt_unit_t UNIT[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};

long    encTotal[4] = {0,0,0,0};
int16_t encLast[4]  = {0,0,0,0};
float   wheelVel[4] = {0,0,0,0};    // rad/s 各輪
float   sideVel[2]  = {0,0};        // rad/s 左右（ペア平均）
float   target[2]   = {0,0};
float   integ[2]    = {0,0};

bool openLoop = false;              // ★true の間は PID を止める

// PID（初期値。実機で追い込む）
const float KP = 6.0f, KI = 40.0f, KFF = 12.0f;

// オドメトリ
float odomX = 0, odomY = 0, odomTh = 0;

unsigned long lastCtrl = 0;
const int CTRL_MS = 10;   // 100 Hz

// ---------------------------------------------------------------
void setupPCNT(int i) {
  pcnt_config_t c = {};
  c.pulse_gpio_num = ENC_A[i];
  c.ctrl_gpio_num  = ENC_B[i];
  c.channel        = PCNT_CHANNEL_0;
  c.unit           = UNIT[i];
  c.pos_mode       = PCNT_COUNT_INC;
  c.neg_mode       = PCNT_COUNT_DEC;
  c.lctrl_mode     = PCNT_MODE_REVERSE;
  c.hctrl_mode     = PCNT_MODE_KEEP;
  c.counter_h_lim  =  20000;
  c.counter_l_lim  = -20000;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(UNIT[i], 100);
  pcnt_filter_enable(UNIT[i]);
  pcnt_counter_pause(UNIT[i]);
  pcnt_counter_clear(UNIT[i]);
  pcnt_counter_resume(UNIT[i]);
}

void readEncoders(float dt) {
  for (int i = 0; i < 4; i++) {
    int16_t c;
    pcnt_get_counter_value(UNIT[i], &c);
    int16_t d = c - encLast[i];
    if (d >  15000) d -= 32768;
    if (d < -15000) d += 32768;
    encLast[i]   = c;
    encTotal[i] += (long)d * ENC_SIGN[i];
    wheelVel[i]  = ((float)d * ENC_SIGN[i] * 2.0f * PI / CPR) / dt;
  }
  sideVel[0] = (wheelVel[0] + wheelVel[2]) * 0.5f;   // 左 = FL + RL
  sideVel[1] = (wheelVel[1] + wheelVel[3]) * 0.5f;   // 右 = FR + RR
}

// ペア内で回転差が大きければスリップ
bool slipDetected(int side) {
  int a = (side == 0) ? 0 : 1;
  int b = (side == 0) ? 2 : 3;
  float m = fabs(sideVel[side]);
  if (m < 0.5f) return false;
  return fabs(wheelVel[a] - wheelVel[b]) > 0.30f * m;
}

void driveRaw(int s, int duty) {
  duty *= DIR_SIGN[s];
  if (duty >  DUTY_MAX) duty =  DUTY_MAX;
  if (duty < -DUTY_MAX) duty = -DUTY_MAX;
  digitalWrite(DIR_PIN[s], duty >= 0 ? HIGH : LOW);
  ledcWrite(s, abs(duty));
}

// 差動逆運動学: vx[m/s], wz[rad/s] → 左右 rad/s
void diffIK(float vx, float wz, float out[2]) {
  out[0] = (vx - HALF_SEP * wz) / WHEEL_R;   // 左
  out[1] = (vx + HALF_SEP * wz) / WHEEL_R;   // 右
}

// 順運動学（オドメトリ）
void updateOdom(float dt) {
  float vL = sideVel[0] * WHEEL_R;
  float vR = sideVel[1] * WHEEL_R;
  float vx = (vL + vR) * 0.5f;
  float wz = (vR - vL) / (2.0f * HALF_SEP);
  odomTh += wz * dt;
  odomX  += vx * cosf(odomTh) * dt;
  odomY  += vx * sinf(odomTh) * dt;
}

void controlStep(float dt) {
  if (openLoop) return;              // ★開ループ中は PID を止める
  for (int s = 0; s < 2; s++) {
    float err = target[s] - sideVel[s];
    integ[s] += err * dt;
    if (integ[s] >  5.0f) integ[s] =  5.0f;
    if (integ[s] < -5.0f) integ[s] = -5.0f;
    float u = KFF * target[s] + KP * err + KI * integ[s];
    if (target[s] == 0.0f && fabs(sideVel[s]) < 0.3f) { u = 0; integ[s] = 0; }
    driveRaw(s, (int)u);
  }
}

void stopAll() {
  openLoop = false;
  for (int s = 0; s < 2; s++) { target[s] = 0; integ[s] = 0; driveRaw(s, 0); }
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  for (int s = 0; s < 2; s++) {
    pinMode(DIR_PIN[s], OUTPUT);
    digitalWrite(DIR_PIN[s], LOW);
    ledcSetup(s, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PWM_PIN[s], s);
    ledcWrite(s, 0);
  }
  for (int i = 0; i < 4; i++) setupPCNT(i);
  pinMode(RELAY_SENSE, INPUT);
  lastCtrl = millis();
  Serial.println();
  Serial.println("=== ATN base (differential) ready ===");
#if TEST_MODE
  Serial.println("commands:");
  Serial.println("  l <duty>      左ペアを開ループで回す  例: l 100");
  Serial.println("  r <duty>      右ペアを開ループで回す");
  Serial.println("  v <vx> <wz>   速度指令 (m/s, rad/s)  例: v 0.2 0");
  Serial.println("  s             停止（開ループ解除）");
  Serial.println("  e             エンコーダ積算と回転数");
  Serial.println("  z             エンコーダとオドメトリをクリア");
  Serial.println("  o             オドメトリ表示");
  Serial.println("  p             現在の設定を表示");
  Serial.printf("  DUTY_MAX=%d  CPR=%.0f  HALF_SEP=%.3f  max wz=%.2f rad/s\n",
                DUTY_MAX, CPR, HALF_SEP, 0.45f / HALF_SEP);
#endif
}

// ---------------------------------------------------------------
void loop() {
  unsigned long now = millis();
  if (now - lastCtrl >= CTRL_MS) {
    float dt = (now - lastCtrl) / 1000.0f;
    lastCtrl = now;
    readEncoders(dt);
    controlStep(dt);
    updateOdom(dt);
  }

#if TEST_MODE
  static unsigned long lastRep = 0;

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      char c = line.charAt(0);

      if (c == 's') {
        stopAll();
        Serial.println("stop");

      } else if (c == 'z') {
        for (int i = 0; i < 4; i++) {
          encTotal[i] = 0;
          pcnt_counter_clear(UNIT[i]);
          encLast[i] = 0;
        }
        odomX = odomY = odomTh = 0;
        Serial.println("cleared");

      } else if (c == 'e') {
        Serial.printf("enc FL=%ld FR=%ld RL=%ld RR=%ld\n",
                      encTotal[0], encTotal[1], encTotal[2], encTotal[3]);
        Serial.printf("rev FL=%.3f FR=%.3f RL=%.3f RR=%.3f  (CPR=%.0f)\n",
                      encTotal[0]/CPR, encTotal[1]/CPR,
                      encTotal[2]/CPR, encTotal[3]/CPR, CPR);

      } else if (c == 'o') {
        Serial.printf("odom x=%.3f y=%.3f th=%.3f rad (%.1f deg)\n",
                      odomX, odomY, odomTh, odomTh * 180.0f / PI);

      } else if (c == 'p') {
        Serial.printf("openLoop=%d  target L=%.2f R=%.2f\n",
                      openLoop, target[0], target[1]);
        Serial.printf("DIR_SIGN %d %d   ENC_SIGN %d %d %d %d\n",
                      DIR_SIGN[0], DIR_SIGN[1],
                      ENC_SIGN[0], ENC_SIGN[1], ENC_SIGN[2], ENC_SIGN[3]);

      } else if (c == 'v') {
        float vx, wz;
        if (sscanf(line.c_str() + 1, "%f %f", &vx, &wz) == 2) {
          openLoop = false;              // ★PID に戻す
          integ[0] = integ[1] = 0;
          diffIK(vx, wz, target);
          Serial.printf("target L=%.2f R=%.2f rad/s\n", target[0], target[1]);
        } else {
          Serial.println("usage: v <vx> <wz>");
        }

      } else if (c == 'l' || c == 'r') {
        int s = (c == 'l') ? 0 : 1;
        int duty = line.substring(1).toInt();
        openLoop = true;                 // ★PID を止める
        target[0] = target[1] = 0;
        integ[0] = integ[1] = 0;
        driveRaw(s, duty);
        Serial.printf("side %d duty %d (open loop)\n", s, duty);

      } else {
        Serial.println("unknown command");
      }
    }
  }

  if (now - lastRep > 500) {
    lastRep = now;
    Serial.printf("%s L=%.2f R=%.2f rad/s | wheels %.2f %.2f %.2f %.2f",
                  openLoop ? "[OPEN]" : "[PID ]",
                  sideVel[0], sideVel[1],
                  wheelVel[0], wheelVel[1], wheelVel[2], wheelVel[3]);
    if (slipDetected(0)) Serial.print("  [SLIP-L]");
    if (slipDetected(1)) Serial.print("  [SLIP-R]");
    Serial.println();
  }
#endif
}
