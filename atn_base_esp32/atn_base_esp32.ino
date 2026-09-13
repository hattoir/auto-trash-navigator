// Auto-Trash Navigator — base controller (ESP32)
// 3S LiPo / メカナム Ø80 / MDD10A ×2 / ホールエンコーダ ×4
//
// TEST_MODE = 1 : シリアル操作の単体試験（micro-ROS 不要）
// TEST_MODE = 0 : PID速度制御ループのみ（micro-ROS は別途組み込む）

#include <driver/pcnt.h>

#define TEST_MODE 1

// ---------- 車体パラメータ ----------
const float WHEEL_R   = 0.040f;   // m
const float LX        = 0.120f;   // ホイールベース/2
const float LY        = 0.180f;   // トレッド/2
const float CPR       = 3300.0f;  // ★仮定値。実測して置き換えること

// ---------- 電源保護 ----------
// モーター定格 9V、3S満充電 12.6V → duty上限 71%
const int DUTY_MAX = 182;         // 255 * 9 / 12.6
const int PWM_FREQ = 20000;       // 可聴域を避ける
const int PWM_BITS = 8;

// ---------- ピン ----------
const int PWM_PIN[4] = {13, 14, 25, 26};   // FL FR RL RR
const int DIR_PIN[4] = {27, 32, 33, 15};
const int ENC_A[4]   = {16, 18, 21, 23};
const int ENC_B[4]   = {17, 19, 22,  5};
const int RELAY_SENSE = 34;

// ★実機で確認して調整する。回転方向・エンコーダ符号の反転フラグ
const int DIR_SIGN[4] = {+1, -1, +1, -1};  // 右側は鏡像なので反転が要るはず
const int ENC_SIGN[4] = {+1, -1, +1, -1};

const pcnt_unit_t UNIT[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};

long   encTotal[4] = {0, 0, 0, 0};
int16_t encLast[4] = {0, 0, 0, 0};
float  wheelVel[4] = {0, 0, 0, 0};   // rad/s
float  target[4]   = {0, 0, 0, 0};   // rad/s
float  integ[4]    = {0, 0, 0, 0};

// PID（初期値。実機で追い込む）
const float KP = 6.0f, KI = 40.0f, KFF = 12.0f;

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
  pcnt_set_filter_value(UNIT[i], 100);   // ノイズ除去
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
    if (d >  15000) d -= 32768;      // オーバーフロー補正
    if (d < -15000) d += 32768;
    encLast[i]  = c;
    encTotal[i] += (long)d * ENC_SIGN[i];
    float rad = (float)d * ENC_SIGN[i] * 2.0f * PI / CPR;
    wheelVel[i] = rad / dt;
  }
}

void driveRaw(int i, int duty) {
  duty *= DIR_SIGN[i];
  if (duty >  DUTY_MAX) duty =  DUTY_MAX;
  if (duty < -DUTY_MAX) duty = -DUTY_MAX;
  digitalWrite(DIR_PIN[i], duty >= 0 ? HIGH : LOW);
  ledcWrite(i, abs(duty));
}

// メカナム逆運動学: vx[m/s], vy[m/s], wz[rad/s] → 各輪 rad/s
void mecanumIK(float vx, float vy, float wz, float out[4]) {
  const float k = LX + LY;
  out[0] = (vx - vy - k * wz) / WHEEL_R;   // FL
  out[1] = (vx + vy + k * wz) / WHEEL_R;   // FR
  out[2] = (vx + vy - k * wz) / WHEEL_R;   // RL
  out[3] = (vx - vy + k * wz) / WHEEL_R;   // RR
}

void controlStep(float dt) {
  for (int i = 0; i < 4; i++) {
    float err = target[i] - wheelVel[i];
    integ[i] += err * dt;
    if (integ[i] >  5.0f) integ[i] =  5.0f;   // アンチワインドアップ
    if (integ[i] < -5.0f) integ[i] = -5.0f;
    float u = KFF * target[i] + KP * err + KI * integ[i];
    if (target[i] == 0.0f && fabs(wheelVel[i]) < 0.3f) { u = 0; integ[i] = 0; }
    driveRaw(i, (int)u);
  }
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  for (int i = 0; i < 4; i++) {
    pinMode(DIR_PIN[i], OUTPUT);
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PWM_PIN[i], i);
    ledcWrite(i, 0);
    setupPCNT(i);
  }
  pinMode(RELAY_SENSE, INPUT);
  lastCtrl = millis();
  Serial.println();
  Serial.println("ATN base ready");
#if TEST_MODE
  Serial.println("commands:");
  Serial.println("  0-3 <duty>  1輪を直接回す (例: 0 80)");
  Serial.println("  v <vx> <vy> <wz>   速度指令");
  Serial.println("  s           停止");
  Serial.println("  e           エンコーダ積算を表示");
  Serial.println("  z           エンコーダをゼロクリア");
  Serial.printf("  DUTY_MAX=%d  CPR=%.0f\n", DUTY_MAX, CPR);
#endif
}

void loop() {
  unsigned long now = millis();
  if (now - lastCtrl >= CTRL_MS) {
    float dt = (now - lastCtrl) / 1000.0f;
    lastCtrl = now;
    readEncoders(dt);
    controlStep(dt);
  }

#if TEST_MODE
  static unsigned long lastRep = 0;
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    char c = line.charAt(0);
    if (c == 's') {
      for (int i = 0; i < 4; i++) { target[i] = 0; integ[i] = 0; driveRaw(i, 0); }
      Serial.println("stop");
    } else if (c == 'z') {
      for (int i = 0; i < 4; i++) { encTotal[i] = 0; pcnt_counter_clear(UNIT[i]); encLast[i] = 0; }
      Serial.println("encoders cleared");
    } else if (c == 'e') {
      Serial.printf("enc  FL=%ld  FR=%ld  RL=%ld  RR=%ld\n",
                    encTotal[0], encTotal[1], encTotal[2], encTotal[3]);
      Serial.printf("rev  FL=%.3f FR=%.3f RL=%.3f RR=%.3f\n",
                    encTotal[0]/CPR, encTotal[1]/CPR, encTotal[2]/CPR, encTotal[3]/CPR);
    } else if (c == 'v') {
      float vx, vy, wz;
      if (sscanf(line.c_str() + 1, "%f %f %f", &vx, &vy, &wz) == 3) {
        mecanumIK(vx, vy, wz, target);
        Serial.printf("target rad/s: %.2f %.2f %.2f %.2f\n",
                      target[0], target[1], target[2], target[3]);
      }
    } else if (c >= '0' && c <= '3') {
      int idx = c - '0';
      int duty = line.substring(1).toInt();
      for (int i = 0; i < 4; i++) target[i] = 0;
      driveRaw(idx, duty);
      Serial.printf("wheel %d duty %d (open loop)\n", idx, duty);
    }
  }
  if (now - lastRep > 500) {
    lastRep = now;
    Serial.printf("vel %.2f %.2f %.2f %.2f rad/s | enc %ld %ld %ld %ld\n",
                  wheelVel[0], wheelVel[1], wheelVel[2], wheelVel[3],
                  encTotal[0], encTotal[1], encTotal[2], encTotal[3]);
  }
#endif
}
