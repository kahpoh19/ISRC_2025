#include <Arduino.h>
#include <Servo.h>

// 左电机
const int LEFT_PWM = 5;
const int LEFT_DIR = 7;

// 右电机
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 4;

// 速度参数
const int BASE_SPEED = 110;   // 巡线时基础速度
const int MIN_SPEED = 60;     // 最低速度，避免停转
const float KP = 0.18f;       // 先加到 0.16，若抖再回 0.14
const int TURN_STRONG = 80;   // V 形急弯时给的强转差速
const float TURN_SLOW = 0.6f; // 急弯/宽线时整体降速比例

// 巡线传感器
const int irPinL = A0;
const int irPinR = A1;

// 超声波传感器
const int trigPin = A2;
const int echoPin = A3;

// 舵机
Servo myServo;
const int servoPin = 3;
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 30;
const int SERVO_RIGHT = 150;

const int OBST = 20; // 障碍阈值（cm）
const unsigned long PING_INTERVAL = 80; // 前向测距间隔（ms）
const int OBST_CLEAR = 30;  // 清障判断：大于此距离认为前方无障碍
const int SERVO_LEFT45 = 45; // 舵机左偏 45 度角
const unsigned long BYPASS_FORWARD_MS = 1100; // 约等于前进 ~22cm 的时间，需实测调整
const unsigned long BYPASS_TURN90_MS = 476;   // 左转 90 度所需时间，需实测调整
const unsigned long BYPASS_PIVOT_MS = 300;    // 避障初始右转的最小时间
const unsigned long SEARCH_LINE_TIMEOUT = 2000; // 最多找线时间（毫秒）
const unsigned long FORWARD_SEARCH_MS = 3000;   // 左转后直行找线的最长时间
const unsigned long STEP_PAUSE_MS = 120;        // 步骤间停顿，避免动作连在一起

// 小车状态
enum Mode { NORMAL, AVOID };
Mode mode = NORMAL;

unsigned long lastPingMs = 0;

// 记忆上次看到线的方向：-1 左、0 双线/未知、1 右
int lastDir = 0;
// 进度计数
int obstaclesSeen = 0;
int gapsSeen = 0;
int padsSeen = 0;      // 大块黑区计数
int vTurnsSeen = 0;
bool finished = false; // 达到终点后停止
bool originCleared = false; // 已离开起点黑块后才允许终点判定
bool bridgeGaps = false; // 进入跨gap直行模式
int gapsBridged = 0;     // 已跨过的 gap 数

// gap 与 pad 判定
bool inGap = false, gapCounted = false;
unsigned long gapStart = 0;
bool inPad = false, padCounted = false;
unsigned long padStart = 0;

// V 形弯判定
bool vActive = false;
unsigned long vStart = 0;

// 参数：检测窗口
const unsigned long GAP_DETECT_MS = 80;   // 白底持续视为 gap
const unsigned long PAD_DETECT_MS = 150;  // 双黑持续视为大黑块
const unsigned long V_WINDOW_MS = 400;    // 单侧压线持续视为一次 V 转


void setForwardSpeeds(int leftPwm, int rightPwm) {
  // 前进方向保持 HIGH，仅调节左右 PWM 差速
  analogWrite(LEFT_PWM, constrain(leftPwm, 0, 255));
  analogWrite(RIGHT_PWM, constrain(rightPwm, 0, 255));
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, HIGH);
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms 超时
  if (duration == 0) return 999; // 超时当作很远
  float distance = duration * 0.034f / 2.0f;

  //Serial.print("Distance: ");
  //Serial.println(distance);
  return (long)distance;
}

void forward() {
  setForwardSpeeds(BASE_SPEED, BASE_SPEED);
}

void stop() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, HIGH);
}

void back() {
  analogWrite(LEFT_PWM, BASE_SPEED);
  analogWrite(RIGHT_PWM, BASE_SPEED);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
}

void turnLeftBrief() {
  analogWrite(LEFT_PWM, BASE_SPEED);
  analogWrite(RIGHT_PWM, BASE_SPEED);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, HIGH);
  delay(250);
  stop();
}

void turnRightBrief() {
  analogWrite(LEFT_PWM, BASE_SPEED);
  analogWrite(RIGHT_PWM, BASE_SPEED);
  digitalWrite(LEFT_DIR, HIGH);
  digitalWrite(RIGHT_DIR, LOW);
  delay(250);
  stop();
}

void lineFollow() {
  // 数字巡线头：典型为黑线 LOW、白底 HIGH，取反后黑线为 1、白为 0
  int valL = digitalRead(irPinL) == LOW ? 0 : 1;
  int valR = digitalRead(irPinR) == LOW ? 0 : 1;

  // 更新行进进度（gap、pad、V 转统计）
  unsigned long now = millis();

  // gap 检测：双白持续一段时间
  if (valL == 0 && valR == 0) {
    if (!inGap) {
      inGap = true;
      gapCounted = false;
      gapStart = now;
    } else if (!gapCounted && (now - gapStart) >= GAP_DETECT_MS) {
      gapsSeen++;
      gapCounted = true;
    }
  } else {
    inGap = false;
    gapCounted = false;
  }

  // pad 检测：双黑持续一段时间
  if (valL == 1 && valR == 1) {
    if (!inPad) {
      inPad = true;
      padCounted = false;
      padStart = now;
    } else if (!padCounted && (now - padStart) >= PAD_DETECT_MS) {
      padsSeen++;
      padCounted = true;
    }
  } else {
    inPad = false;
    padCounted = false;
    // 只要离开过第一块黑区，允许后续黑块作为终点判断
    if (padsSeen > 0) originCleared = true;
  }

  // 跨 gap 模式：在第二个障碍之后，连续直行跨过 2 个 gap，直到再次看到黑线
  if (bridgeGaps) {
    if (valL == 0 && valR == 0) {
      // 仍在 gap 上，保持直行
      setForwardSpeeds(BASE_SPEED, BASE_SPEED);
      return;
    } else {
      // 从 gap 回到线，计数一次
      gapsBridged++;
      if (gapsBridged >= 2) {
        bridgeGaps = false; // 完成两次 gap 跨越，恢复正常巡线
      }
      // 看到线就继续后面的正常巡线逻辑
    }
  }

  // V 弯检测：gap>=2 后，遇到单侧压线算一次 V 转
  if (gapsSeen >= 2 && vTurnsSeen < 2) {
    if ((valL ^ valR) == 1) { // 单侧为 1
      if (!vActive) {
        vActive = true;
        vStart = now;
      }
    } else {
      if (vActive && (now - vStart) <= V_WINDOW_MS) {
        vTurnsSeen++;
      }
      vActive = false;
    }
  } else {
    vActive = false;
  }

  // 如果已离开起点且再次踩到大黑块，则认为到达终点
  if (originCleared && padsSeen >= 2) {
    stop();
    finished = true;
    return;
  }

  // 起点大黑块上优先直行驶出，不做减速或转向判断
  if (!originCleared) {
    setForwardSpeeds(BASE_SPEED, BASE_SPEED);
    return;
  }

  //Serial.print("IR L = "); //Serial.print(valL);
  //Serial.print("  IR R = "); //Serial.println(valR);

  // V 形急弯强化：单侧压线时强差速并整体降速；双黑时慢速直行
  if (valL == 1 && valR == 1) {
    int slow = (int)(BASE_SPEED * TURN_SLOW);
    setForwardSpeeds(constrain(slow, MIN_SPEED, 255), constrain(slow, MIN_SPEED, 255));
    lastDir = 0;
    return;
  }

  if (valL == 1 && valR == 0) {
    // 线在左侧：左轮停，右轮前进实现原地右转
    setForwardSpeeds(0, MIN_SPEED + TURN_STRONG);
    lastDir = -1;
    return;
  }

  if (valL == 0 && valR == 1) {
    // 线在右侧：右轮停，左轮前进实现原地左转
    setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);
    lastDir = 1;
    return;
  }

  // 走到白底（丢线）：按上次方向继续找线
  const int SEARCH_TURN = 70;
  if (lastDir <= 0) {
    // 上次在左或未知：左轮停右轮转，继续向左找
    setForwardSpeeds(0, MIN_SPEED + SEARCH_TURN);
    delay(10); // 短暂停留，给转向一点实际执行时间
  } else {
    // 上次在右：右轮停左轮转，向右找
    setForwardSpeeds(MIN_SPEED + SEARCH_TURN, 0);
    delay(10); // 短暂停留，给转向一点实际执行时间
  }
}

void avoidObstacle() {
  // 新策略：偏左绕障
  stop();

  // 将超声转到左 45 度，原地右转直到视野中无障碍或超时，且至少转一小段
  myServo.write(SERVO_LEFT45);
  delay(80);

  unsigned long start = millis();
  while (millis() - start < 1500) { // 最长 1.5 秒寻找空隙
    long d = getDistance();
    // 强制至少转 BYPASS_PIVOT_MS；之后若视野>OBST_CLEAR 就退出
    if ((millis() - start) > BYPASS_PIVOT_MS && d > OBST_CLEAR) break;
    // 右轮停、左轮转，持续右转
    setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);
    delay(20);
  }

  // 舵机回中
  myServo.write(SERVO_CENTER);
  delay(80);
  stop();
  delay(STEP_PAUSE_MS);

  // 前进约 22cm（时间估计，需实测）
  setForwardSpeeds(BASE_SPEED, BASE_SPEED);
  delay(BYPASS_FORWARD_MS);
  stop();
  delay(STEP_PAUSE_MS);

  // 左转约 90 度（左轮停，右轮前进，小半径左转）
  setForwardSpeeds(0, MIN_SPEED + TURN_STRONG);
  delay(BYPASS_TURN90_MS);
  stop();
  delay(STEP_PAUSE_MS);

  // 左转后直行找线：持续前进，直到看到黑线或超时
  bool foundLine = false;
  unsigned long forwardSearchStart = millis();
  while (millis() - forwardSearchStart < FORWARD_SEARCH_MS) {
    int valL = digitalRead(irPinL) == LOW ? 0 : 1;
    int valR = digitalRead(irPinR) == LOW ? 0 : 1;
    if (valL == 1 || valR == 1) {
      foundLine = true;
      break;
    }
    setForwardSpeeds(BASE_SPEED, BASE_SPEED);
    delay(10);
  }
  stop();
  delay(STEP_PAUSE_MS);

  // 若仍未找到线，先右转找线；再反向左转兜底
  if (!foundLine) {
    unsigned long searchStart = millis();
    while (millis() - searchStart < SEARCH_LINE_TIMEOUT) {
      int valL = digitalRead(irPinL) == LOW ? 0 : 1;
      int valR = digitalRead(irPinR) == LOW ? 0 : 1;
      if (valL == 1 || valR == 1) {
        foundLine = true;
        break;
      }
      // 右轮停、左轮转，小幅右转找线
      setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);
      delay(30);
    }
    stop();
    delay(STEP_PAUSE_MS);
  }

  if (!foundLine) {
    // 右转失败后，反向左转兜底
    unsigned long searchStart = millis();
    while (millis() - searchStart < SEARCH_LINE_TIMEOUT) {
      int valL = digitalRead(irPinL) == LOW ? 0 : 1;
      int valR = digitalRead(irPinR) == LOW ? 0 : 1;
      if (valL == 1 || valR == 1) {
        foundLine = true;
        break;
      }
      // 左轮停、右轮转，小幅左转找线
      setForwardSpeeds(0, MIN_SPEED + TURN_STRONG);
      delay(30);
    }
    stop();
    delay(STEP_PAUSE_MS);
  }

  // 完成本轮绕障后停一下，回到正常巡线
  stop();
  delay(STEP_PAUSE_MS);

  // 如果已完成第二个障碍，开启跨 gap 直行模式
  if (obstaclesSeen >= 2) {
    bridgeGaps = true;
    gapsBridged = 0;
  }
}

void setup() {
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(irPinL, INPUT);
  pinMode(irPinR, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  myServo.write(SERVO_CENTER); // 舵机回正

  Serial.begin(9600);
}

void loop() {
  if (finished) {
    stop();
    return;
  }

  if (mode == NORMAL) {
    lineFollow();

    unsigned long now = millis();
    if (now - lastPingMs >= PING_INTERVAL) {
      lastPingMs = now;

      myServo.write(SERVO_CENTER);
      long front = getDistance();
      if (front < OBST) {
        obstaclesSeen++;
        mode = AVOID;
      }
    }
  }

  if (mode == AVOID) {
    avoidObstacle();
    mode = NORMAL;
  }
}