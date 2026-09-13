// ==========================================================
//  Auto-Trash Navigator — base controller (ESP32)  本番用
//  ROS 2 シリアルブリッジ対応版
//
//  構成:
//    駆動   差動2輪（メカナムを左右ペアで並列駆動）
//    電源   LiPo 3S / MD10C ×2
//    エンコーダ FL と FR の2個
//
//  ★ wheel_separation = 0.600 （HALF_SEP = 0.300）
//    メカナムを左右ペアで駆動すると、旋回の実効モーメントアームは
//    物理トレッドの半分 0.180 ではなく lx+ly = 0.300 になる。
//
//  配線:
//    MD10C ①(左 FL+RL)  PWM=GPIO13  DIR=GPIO27  GND=ESP32 GND
//    MD10C ②(右 FR+RR)  PWM=GPIO14  DIR=GPIO32  GND=ESP32 GND
//    ENC FL  S1=GPIO16  S2=GPIO17   白=3V3  黄=GND
//    ENC FR  S1=GPIO18  S2=GPIO19   白=3V3  黄=GND
//
//  ── シリアルプロトコル ─────────────────────────
//  【手動用（小文字）】人が打つ。応答あり
//    l <duty> / r <duty>   開ループ
//    v <vx> <wz>           速度指令
//    s / e / o / p         停止・エンコーダ・オドメトリ・設定
//    c <cpr>               CPR変更
//    d <0|1> / n <0|1>     符号反転
//    g <kp> <ki> <kff>     PID変更
//
//  【ROS用（大文字）】ノードが送る。1行1応答
//    V <vx> <wz>   → OK
//    O             → O <x> <y> <th> <vx> <wz> <encL> <encR>
//    S             → OK
//    Z             → OK
//  ─────────────────────────────────────────
// ==========================================================

#include <driver/pcnt.h>

// ---------- 車体パラメータ ----------
const float WHEEL_R  = 0.040f;
const float HALF_SEP = 0.300f;   // ★lx+ly
float       CPR      = 755.0f;   // ★実測値

// ---------- 電源保護 ----------
const int DUTY_MAX = 182;        // 255 * 9 / 12.6
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// ---------- ピン ----------
const int PWM_PIN[2]  = {13, 14};
const int DIR_PIN[2]  = {27, 32};
int       DIR_SIGN[2] = {+1, -1};

const int ENC_A[2] = {16, 18};
const int ENC_B[2] = {17, 19};
int       ENC_SIGN[2] = {+1, -1};

const int RELAY_SENSE = 34;

const pcnt_unit_t UNIT[2] = {PCNT_UNIT_0, PCNT_UNIT_1};

// ---------- 状態 ----------
long    encTotal[2] = {0, 0};
int16_t encLast[2]  = {0, 0};
float   sideVel[2]  = {0, 0};
float   target[2]   = {0, 0};
float   integ[2]    = {0, 0};
bool    openLoop    = false;
bool    rosMode     = false;      // 大文字コマンドを受けたら true（定期表示を止める）

float odomX = 0, odomY = 0, odomTh = 0;
float lastVx = 0, lastWz = 0;

float KP = 6.0f, KI = 40.0f, KFF = 12.0f;

unsigned long lastCtrl = 0;
unsigned long lastCmd  = 0;
const int CTRL_MS    = 10;     // 100 Hz
const int TIMEOUT_MS = 500;    // 指令が途切れたら停止

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
  for (int i = 0; i < 2; i++) {
    int16_t c;
    pcnt_get_counter_value(UNIT[i], &c);
    int16_t d = c - encLast[i];
    if (d >  15000) d -= 32768;
    if (d < -15000) d += 32768;
    encLast[i]   = c;
    encTotal[i] += (long)d * ENC_SIGN[i];
    sideVel[i]   = ((float)d * ENC_SIGN[i] * 2.0f * PI / CPR) / dt;
  }
}

void driveRaw(int s, int duty) {
  duty *= DIR_SIGN[s];
  if (duty >  DUTY_MAX) duty =  DUTY_MAX;
  if (duty < -DUTY_MAX) duty = -DUTY_MAX;
  digitalWrite(DIR_PIN[s], duty >= 0 ? HIGH : LOW);
  ledcWrite(s, abs(duty));
}

void diffIK(float vx, float wz, float out[2]) {
  out[0] = (vx - HALF_SEP * wz) / WHEEL_R;
  out[1] = (vx + HALF_SEP * wz) / WHEEL_R;
}

void updateOdom(float dt) {
  float vL = sideVel[0] * WHEEL_R;
  float vR = sideVel[1] * WHEEL_R;
  lastVx = (vL + vR) * 0.5f;
  lastWz = (vR - vL) / (2.0f * HALF_SEP);
  odomTh += lastWz * dt;
  if (odomTh >  PI) odomTh -= 2.0f * PI;
  if (odomTh < -PI) odomTh += 2.0f * PI;
  odomX += lastVx * cosf(odomTh) * dt;
  odomY += lastVx * sinf(odomTh) * dt;
}

void controlStep(float dt) {
  if (openLoop) return;
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

void clearAll() {
  for (int i = 0; i < 2; i++) {
    encTotal[i] = 0;
    pcnt_counter_clear(UNIT[i]);
    encLast[i] = 0;
  }
  odomX = odomY = odomTh = 0;
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
    setupPCNT(s);
  }
  pinMode(RELAY_SENSE, INPUT);
  lastCtrl = millis();
  lastCmd  = millis();

  Serial.println();
  Serial.println("=== ATN base ready (diff / enc x2 / ROS bridge) ===");
  Serial.println("  manual : l r v s e o p c d n g");
  Serial.println("  ros    : V <vx> <wz> | O | S | Z");
  Serial.printf("  DUTY_MAX=%d CPR=%.0f HALF_SEP=%.3f maxWz=%.2f\n",
                DUTY_MAX, CPR, HALF_SEP, 0.45f / HALF_SEP);
}

// ---------------------------------------------------------------
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);
  const char *p = line.c_str() + 1;
  lastCmd = millis();

  // ===== ROS 用（大文字）=====
  if (c == 'V') {
    float vx, wz;
    if (sscanf(p, "%f %f", &vx, &wz) == 2) {
      rosMode = true;
      openLoop = false;
      diffIK(vx, wz, target);
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
    return;
  }
  if (c == 'O') {
    rosMode = true;
    Serial.printf("O %.4f %.4f %.4f %.4f %.4f %ld %ld\n",
                  odomX, odomY, odomTh, lastVx, lastWz,
                  encTotal[0], encTotal[1]);
    return;
  }
  if (c == 'S') { rosMode = true; stopAll(); Serial.println("OK"); return; }
  if (c == 'Z') { rosMode = true; clearAll(); Serial.println("OK"); return; }

  // ===== 手動用（小文字）=====
  if (c == 's') {
    stopAll();
    Serial.println("stop");

  } else if (c == 'z') {
    clearAll();
    Serial.println("cleared");

  } else if (c == 'e') {
    Serial.printf("enc FL=%ld FR=%ld   rev %.3f / %.3f  (CPR=%.0f)\n",
                  encTotal[0], encTotal[1],
                  encTotal[0]/CPR, encTotal[1]/CPR, CPR);

  } else if (c == 'o') {
    Serial.printf("odom x=%.3f y=%.3f th=%.3f rad (%.1f deg)\n",
                  odomX, odomY, odomTh, odomTh * 180.0f / PI);

  } else if (c == 'p') {
    Serial.printf("openLoop=%d rosMode=%d  target L=%.2f R=%.2f\n",
                  openLoop, rosMode, target[0], target[1]);
    Serial.printf("DIR_SIGN %+d %+d   ENC_SIGN %+d %+d\n",
                  DIR_SIGN[0], DIR_SIGN[1], ENC_SIGN[0], ENC_SIGN[1]);
    Serial.printf("CPR=%.1f  KP=%.1f KI=%.1f KFF=%.1f\n", CPR, KP, KI, KFF);

  } else if (c == 'c') {
    float v = atof(p);
    if (v > 10.0f) { CPR = v; Serial.printf("CPR = %.1f\n", CPR); }
    else Serial.println("usage: c <cpr>");

  } else if (c == 'd') {
    int i = atoi(p);
    if (i == 0 || i == 1) {
      DIR_SIGN[i] = -DIR_SIGN[i];
      Serial.printf("DIR_SIGN[%d] = %+d\n", i, DIR_SIGN[i]);
    }

  } else if (c == 'n') {
    int i = atoi(p);
    if (i == 0 || i == 1) {
      ENC_SIGN[i] = -ENC_SIGN[i];
      Serial.printf("ENC_SIGN[%d] = %+d\n", i, ENC_SIGN[i]);
    }

  } else if (c == 'g') {
    float a, b, d2;
    if (sscanf(p, "%f %f %f", &a, &b, &d2) == 3) {
      KP = a; KI = b; KFF = d2;
      Serial.printf("KP=%.1f KI=%.1f KFF=%.1f\n", KP, KI, KFF);
    } else Serial.println("usage: g <kp> <ki> <kff>");

  } else if (c == 'v') {
    float vx, wz;
    if (sscanf(p, "%f %f", &vx, &wz) == 2) {
      openLoop = false;
      integ[0] = integ[1] = 0;
      diffIK(vx, wz, target);
      Serial.printf("target L=%.2f R=%.2f rad/s\n", target[0], target[1]);
    } else Serial.println("usage: v <vx> <wz>");

  } else if (c == 'l' || c == 'r') {
    int s = (c == 'l') ? 0 : 1;
    int duty = atoi(p);
    openLoop = true;
    target[0] = target[1] = 0;
    integ[0] = integ[1] = 0;
    driveRaw(s, duty);
    Serial.printf("side %d duty %d (open loop)\n", s, duty);

  } else {
    Serial.println("unknown command");
  }
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

  // 指令が途切れたら安全停止
  if (!openLoop && (target[0] != 0.0f || target[1] != 0.0f)) {
    if (now - lastCmd > TIMEOUT_MS) {
      stopAll();
      if (!rosMode) Serial.println("timeout -> stop");
    }
  }

  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // ROS モード中は定期表示を出さない（応答と混ざるため）
  if (!rosMode) {
    static unsigned long lastRep = 0;
    if (now - lastRep > 500) {
      lastRep = now;
      Serial.printf("%s L=%+.2f R=%+.2f rad/s | enc %ld %ld | odom %.2f %.2f %.1fdeg\n",
                    openLoop ? "[OPEN]" : "[PID ]",
                    sideVel[0], sideVel[1],
                    encTotal[0], encTotal[1],
                    odomX, odomY, odomTh * 180.0f / PI);
    }
  }
}
