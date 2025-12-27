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
// const float KP = 0.18f;       // 先加到 0.16，若抖再回 0.14
const int TURN_STRONG = 80;   // V 形急弯时给的强转差速
// const float TURN_SLOW = 0.6f; // 急弯/宽线时整体降速比例

// 巡线传感器
const int irPinL = A0;
const int irPinR = A1;
int valL = digitalRead(irPinL) == LOW ? 0 : 1;
int valR = digitalRead(irPinR) == LOW ? 0 : 1;

// 超声波传感器
const int trigPin = A4;
const int echoPin = A3;

const int debugLEDYellow = 11;
const int debugLEDGreen = 10;
const int debugLEDRed = 9;

// 舵机
Servo myServo;
const int servoPin = A5;
const int SERVO_CENTER = 90;
const int SERVO_LEFT = 30;
// const int SERVO_RIGHT = 150;

const int OBST = 25; // 障碍阈值（cm）
const unsigned long PING_INTERVAL = 80; // 前向测距间隔（ms）
// const int OBST_CLEAR = 33;  // 清障判断：大于此距离认为前方无障碍
const int SERVO_LEFT20 = 160; // 舵机左偏角，加大初始避障右转幅度
// const unsigned long BYPASS_FORWARD_MS = 1300; // 避障直行距离（约 22~26cm，需实测）
// const unsigned long BYPASS_TURN90_MS = 476;   // 左转 90 度所需时间，需实测调整
// const unsigned long BYPASS_PIVOT_MS = 400;    // 避障初始右转的最小时间（加长）
// const unsigned long SEARCH_LINE_TIMEOUT = 2000; // 最多找线时间（毫秒）
// const unsigned long FORWARD_SEARCH_MS = 3000;   // 左转后直行找线的最长时间
const unsigned long STEP_PAUSE_MS = 120;        // 步骤间停顿，避免动作连在一起
const unsigned long AVOID_COOLDOWN_MS = 3000;   // 避障结束后忽略障碍检测的冷却
const unsigned long GAP_HOLD_MS = 80;   // 双白保持多久才算 gap
static unsigned long gapStartMs = 0;
static bool gapStable = false;

// 小车状态
enum Mode { NORMAL, AVOID };
Mode mode = NORMAL;

unsigned long lastPingMs = 0;
unsigned long avoidCooldownUntil = 0;

// 记忆上次看到线的方向：-1 左、0 双线/未知、1 右
int lastDir = 0;
// 进度计数
int obstaclesSeen = 1;  //记得归0
int gapsSeen = 0;
int padsSeen = 0;      // 大块黑区计数
int vTurnsSeen = 0;
bool finished = false; // 达到终点后停止
bool originCleared = false; // 已离开起点黑块后才允许终点判定
bool bridgeGaps = false; // 进入跨gap直行模式
int gapsBridged = 0;     // 已跨过的 gap 数
int correctionGap = 0;
unsigned long bridgeGapsEnableAt = 0; // 第二个障碍结束后延迟开启跨 gap 的时间点

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

unsigned long now = millis();

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
  valL = digitalRead(irPinL) == LOW ? 0 : 1;
  valR = digitalRead(irPinR) == LOW ? 0 : 1;

  // 更新行进进度（gap、pad、V 转统计）
  now = millis();

  // // pad 检测：双黑持续一段时间
  // if (valL == 1 && valR == 1) {
  //   if (!inPad) {
  //     inPad = true;
  //     padCounted = false;
  //     padStart = now;
  //   } else if (!padCounted && (now - padStart) >= PAD_DETECT_MS) {
  //     padsSeen++;
  //     padCounted = true;
  //   }
  // } else {
  //   inPad = false;
  //   padCounted = false;
  //   // 只要离开过第一块黑区，允许后续黑块作为终点判断
  //   if (padsSeen > 0) originCleared = true;
  // }

  // 跨 gap 模式：在第二个障碍之后，连续直行跨过 2 个 gap，直到再次看到黑线
  // if (bridgeGaps) {
  //   bool bothWhite = (valL == 1 && valR == 1); // 你说白是 1
  //   bool gap = false;
  //   if (bothWhite) {
  //     // myServo.write(60);
  //     // delay(80);
  //     // myServo.write(120);
  //     // if (gapStartMs == 0) gapStartMs = now;
  //     // if (!gapStable && now - gapStartMs >= GAP_HOLD_MS) {
  //     //   gapStable = true;
  //     //   gapsSeen++;
  //     // }
      
  //     // gap 检测：双白持续一段时间
  //     if (valL == 1 && valR == 1) {
  //       if (!inGap) {
  //         inGap = true;
  //         gapCounted = false;
  //         gapStart = now;

  //         stop();
  //         //测试
  //         myServo.write(60);
  //         delay(80);
  //         myServo.write(120);
  //         delay(80);
  //         const int GAP_BIAS = 25;
  //         if (correctionGap == 0) {
  //           if (lastDir == 1) {
  //             //测试
  //             myServo.write(60);
  //             delay(80);
  //             myServo.write(120);
  //             delay(80);
  //             // 往右微调：左轮快一点
  //             setForwardSpeeds(BASE_SPEED + GAP_BIAS, BASE_SPEED - GAP_BIAS);
  //             delay(100);
  //           } else if (lastDir == -1) {
  //             //测试
  //             myServo.write(60);
  //             delay(80);
  //             myServo.write(120);
  //             delay(80);
  //             // 往左微调：右轮快一点
  //             setForwardSpeeds(BASE_SPEED - GAP_BIAS, BASE_SPEED + GAP_BIAS);
  //             delay(100);
  //           }
  //           correctionGap = 1;
  //         }
  //         //第一个gap直走
  //         setForwardSpeeds(BASE_SPEED, BASE_SPEED);
  //         delay(900);
  //       } else if (!gapCounted && (now - gapStart) >= GAP_DETECT_MS) {
  //         gapsSeen++;
  //         gapCounted = true;
  //       }
  //     } else {
  //       inGap = false;
  //       gapCounted = false;
  //     }
  //     // myServo.write(180);
  //     // 仍在 gap 上，保持直行
  //     // setForwardSpeeds(BASE_SPEED, BASE_SPEED);
  //     return;
  //   } else {
  //     // gapStartMs = 0;
  //     // gapStable = false;

  //     // 从 gap 回到线，计数一次
  //     gapsBridged++;
  //     correctionGap = 0;
  //     if (gapsBridged >= 2) {
  //       bridgeGaps = false; // 完成两次 gap 跨越，恢复正常巡线
  //     }
  //     // 看到线就继续后面的正常巡线逻辑
  //   }
  //   // if (gapStable) {
  //   //   setForwardSpeeds(BASE_SPEED, BASE_SPEED);
  //   //   gap = true;
  //   // } else {
  //   //   gap = false;
  //   // }
  //   // if (((valL == 0 && valR == 1) || (valL == 1 && valR == 0)) && gap == true){
  //   //   gapsBridged++;
  //   //   if (gapsBridged >= 2) {
  //   //     bridgeGaps = false; // 完成两次 gap 跨越，恢复正常巡线
  //   //   }
  //   //   gap = false;
  //   // }
  // }

  // // V 弯检测：gap>=2 后，遇到单侧压线算一次 V 转
  // if (gapsSeen >= 2 && vTurnsSeen < 2) {
  //   if ((valL ^ valR) == 1) { // 单侧为 1
  //     if (!vActive) {
  //       vActive = true;
  //       vStart = now;
  //     }
  //   } else {
  //     if (vActive && (now - vStart) <= V_WINDOW_MS) {
  //       vTurnsSeen++;
  //     }
  //     vActive = false;
  //   }
  // } else {
  //   vActive = false;
  // }

  // 如果已离开起点且再次踩到大黑块，则认为到达终点
  if (originCleared && padsSeen >= 2) {
    stop();
    finished = true;
    return;
  }

  // 起点为白底（黑框内），先直走，直到首次压到黑线/黑底才进入正常巡线
  if (!originCleared) {
    if (valL == 1 || valR == 1) {
      originCleared = true; // 看到黑后开始正常巡线
    } else {
      setForwardSpeeds(BASE_SPEED, BASE_SPEED);
      return;
    }
  }

  //Serial.print("IR L = "); //Serial.print(valL);
  //Serial.print("  IR R = "); //Serial.println(valR);


  // 线在左：左稍慢，右稍快，保持前进
  if (valL == 1 && valR == 0) {
    const int DIFF = 25; // 可调 30~50
    setForwardSpeeds(constrain(BASE_SPEED - DIFF, MIN_SPEED, 255),
                     constrain(BASE_SPEED + DIFF, MIN_SPEED, 255));
    lastDir = -1;
    return;
  }

  // 线在右：右稍慢，左稍快
  if (valL == 0 && valR == 1) {
    const int DIFF = 35;
    setForwardSpeeds(constrain(BASE_SPEED + DIFF, MIN_SPEED, 255),
                     constrain(BASE_SPEED - DIFF, MIN_SPEED, 255));
    lastDir = 1;
    return;
  }

  // 丢线搜索可稍微减小转向幅度
  const int SEARCH_TURN = 50; // 原 70，可再微调
  if (!bridgeGaps) {
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
}

void Gaps() {
  now = millis();

    valL = digitalRead(irPinL) == LOW ? 0 : 1;
    valR = digitalRead(irPinR) == LOW ? 0 : 1;
    now = millis();
    digitalWrite(debugLEDGreen, LOW);
    digitalWrite(debugLEDRed, HIGH);

    if (gapsSeen >= 2) {
      bridgeGaps = false;
      stop();
    }
    else if (valL == 0 && valR == 0) { // 白色
      if (gapStartMs == 0) gapStartMs = now;
      if (!gapStable && (now - gapStartMs >= 200)) {
        gapStable = true;
        gapsSeen++;
      }
      if (gapStable) {
        digitalWrite(debugLEDGreen, HIGH);
        digitalWrite(debugLEDYellow, HIGH);
        if (correctionGap == 0) {
          digitalWrite(debugLEDGreen, LOW);
          digitalWrite(debugLEDYellow, LOW);
          digitalWrite(debugLEDRed, LOW);
          if (lastDir == 1) {
            // 往右微调：左轮快一点
            setForwardSpeeds(135, 80);
            delay(75);
          } else if (lastDir == -1) {
            // 往左微调：右轮快一点
            setForwardSpeeds(80, 135);
            delay(75);
          }
          correctionGap = 1;
        }
        digitalWrite(debugLEDGreen, HIGH);
        digitalWrite(debugLEDYellow, HIGH);
        digitalWrite(debugLEDRed, HIGH);

        // setForwardSpeeds(BASE_SPEED, BASE_SPEED);
        // if (gapsSeen == 1) delay(350);
        // else delay(250);
        valL = digitalRead(irPinL) == LOW ? 0 : 1;
        valR = digitalRead(irPinR) == LOW ? 0 : 1;
        while (valL == 0 && valR == 0) {
          setForwardSpeeds(BASE_SPEED, BASE_SPEED);
          valL = digitalRead(irPinL) == LOW ? 0 : 1;
          valR = digitalRead(irPinR) == LOW ? 0 : 1;
        }
        stop();
      }
    } else {
      gapStartMs = 0;
    }

    if (gapsSeen!=0 && (valL == 1 || valR == 1)) {
      gapStable = false;
    }
    digitalWrite(debugLEDRed, LOW);
    digitalWrite(debugLEDGreen, HIGH);
}

void vTurn() {
  // V 形急弯强化：双黑时整体降速直行
  if (valL == 1 && valR == 1) {
    int slow = (int)(BASE_SPEED * 0.7f); // 原来 0.6，可稍快一点
    setForwardSpeeds(constrain(slow, MIN_SPEED, 255), constrain(slow, MIN_SPEED, 255));
    lastDir = 0;
    return;
  }

}

void avoidObstacle() {
  // 新策略：偏左绕障
  stop();
  delay(2 * STEP_PAUSE_MS);

  //右转
  setForwardSpeeds(190, 0);
  delay(250);
  stop();
  delay(STEP_PAUSE_MS);

  //直走
  setForwardSpeeds(BASE_SPEED, BASE_SPEED);
  delay(100);
  stop();
  delay(STEP_PAUSE_MS);

  // 将超声转到左 45 度，原地右转直到视野中无障碍或超时，且至少转一小段
  myServo.write(SERVO_LEFT20);
  delay(80);

  int AVOID_SPEED = 100;
  bool foundLine = false;
  while (!foundLine) {
    valL = digitalRead(irPinL) == LOW ? 0 : 1;
    valR = digitalRead(irPinR) == LOW ? 0 : 1;
    if (valL == 1 || valR == 1) {
      foundLine = false;
      break;
    }

    long d = getDistance();
    long d_OBST = 28;
    if (d < d_OBST) {
      setForwardSpeeds(AVOID_SPEED + 30, 0);
    } else if (d > d_OBST) {
      setForwardSpeeds(0, AVOID_SPEED + 30);
    } else {
      setForwardSpeeds(AVOID_SPEED + 30, AVOID_SPEED + 30);
    }
  }
  stop();

  valL = digitalRead(irPinL) == LOW ? 0 : 1;
  valR = digitalRead(irPinR) == LOW ? 0 : 1;

  if (valL == 1 && valR == 0) { //黑 白
    // setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
    // delay(350);
    // stop();
    // valL = digitalRead(irPinL) == LOW ? 0 : 1;
    // valR = digitalRead(irPinR) == LOW ? 0 : 1;
    // while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
    //   digitalWrite(debugLEDGreen,HIGH);
    //   setForwardSpeeds(MIN_SPEED + TURN_STRONG + 40, 0);
    //   valL = digitalRead(irPinL) == LOW ? 0 : 1;
    //   valR = digitalRead(irPinR) == LOW ? 0 : 1;
    // }



    // setForwardSpeeds(135, 75);
    // delay (750);
    // stop();


    while (valL == 1 && valR == 0) {
      setForwardSpeeds(BASE_SPEED, BASE_SPEED);
      valL = digitalRead(irPinL) == LOW ? 0 : 1;
      valR = digitalRead(irPinR) == LOW ? 0 : 1;
    }
    delay(300);
    stop();
  }
  else if (valL == 1 && valR == 0) { //白 黑
    //   setForwardSpeeds(BASE_SPEED, BASE_SPEED);
    //   delay(200);
    //   stop();
    //   goto label;
    setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
    delay(900);
    stop();
    valL = digitalRead(irPinL) == LOW ? 0 : 1;
    valR = digitalRead(irPinR) == LOW ? 0 : 1;
    while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
      digitalWrite(debugLEDGreen,HIGH);
      setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
      valL = digitalRead(irPinL) == LOW ? 0 : 1;
      valR = digitalRead(irPinR) == LOW ? 0 : 1;
    }
  }
  else {
    valL = digitalRead(irPinL) == LOW ? 0 : 1;
    valR = digitalRead(irPinR) == LOW ? 0 : 1;
    while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
      digitalWrite(debugLEDGreen,HIGH);
      setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
      valL = digitalRead(irPinL) == LOW ? 0 : 1;
      valR = digitalRead(irPinR) == LOW ? 0 : 1;
    }
  }

  // if (valL == 1 && valR == 0) { // 1 0
  //   setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
  //   delay(200);
  //   stop();
  //   valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //   valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
  //     digitalWrite(debugLEDGreen,HIGH);
  //     // analogWrite(LEFT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // analogWrite(RIGHT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // digitalWrite(LEFT_DIR, HIGH);
  //     // digitalWrite(RIGHT_DIR, LOW);
  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG + 40, 0);
  //     valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //     valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   }
  // } else if (valL == 0 && valR == 1) {  // 0 1
  //   setForwardSpeeds(MIN_SPEED + TURN_STRONG + 20, 0);
  //   delay(200);
  //   stop();
  //   valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //   valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
  //     digitalWrite(debugLEDGreen,HIGH);
  //     // analogWrite(LEFT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // analogWrite(RIGHT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // digitalWrite(LEFT_DIR, HIGH);
  //     // digitalWrite(RIGHT_DIR, LOW);
  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG + 40, 0);
  //     valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //     valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   }
  // } else if (valL == 1 && valR == 1) {  // 1 1
  //   valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //   valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
  //     digitalWrite(debugLEDGreen,HIGH);
  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG + 40, 0);
  //     valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //     valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   }
  // } else {  // 0 0
  //   valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //   valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   while ((valL == 0 && valR == 0) || (valL == 1 && valR == 1)) {
  //     digitalWrite(debugLEDGreen,HIGH);
  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG + 40, 0);
  //     valL = digitalRead(irPinL) == LOW ? 0 : 1;
  //     valR = digitalRead(irPinR) == LOW ? 0 : 1;
  //   }
  // }

  
  delay(150);
  stop();
  digitalWrite(debugLEDGreen, LOW);
  delay(STEP_PAUSE_MS);

  // if (valL == 0 || valR == 0) {
  //   //第二次避障有可能转不过来，需要调整
  //   if (obstaclesSeen == 1) {
  //     // setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);
  //     // delay(550);
  //     // analogWrite(LEFT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // analogWrite(RIGHT_PWM, constrain(MIN_SPEED + TURN_STRONG, 0, 255));
  //     // digitalWrite(LEFT_DIR, HIGH);
  //     // digitalWrite(RIGHT_DIR, LOW);

  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);

  //     digitalWrite(debugLEDGreen,HIGH);

  //     delay(550);

  //     digitalWrite(debugLEDGreen, LOW);
  //   }
  //   else {
  //     // setForwardSpeeds(AVOID_SPEED, AVOID_SPEED);
  //     // delay(50);
  //     setForwardSpeeds(MIN_SPEED + TURN_STRONG, 0);
  //     delay(350);
  //   }
  //   stop();
  //   delay(STEP_PAUSE_MS);
  // }
  
  myServo.write(SERVO_CENTER);
  delay(STEP_PAUSE_MS);

  obstaclesSeen++;

  // 第二个障碍结束，延迟 5 秒后才允许开启跨 gap 直行模式
  if (obstaclesSeen == 2) {
    bridgeGaps = false;
    bridgeGapsEnableAt = millis() + 2500;
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

  pinMode(debugLEDGreen, OUTPUT);
  pinMode(debugLEDYellow, OUTPUT);
  pinMode(debugLEDRed, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if (finished) {
    digitalWrite(debugLEDGreen, LOW);
    digitalWrite(debugLEDYellow, LOW);

    stop();
    return;
  }

  if (mode == NORMAL) {
    digitalWrite(debugLEDGreen, HIGH);
    digitalWrite(debugLEDYellow, LOW);

    unsigned long nowMs = millis();
    if (!bridgeGaps && bridgeGapsEnableAt != 0 && obstaclesSeen >= 2 && nowMs >= bridgeGapsEnableAt) {
      bridgeGaps = true;
      bridgeGapsEnableAt = 0;
      gapsBridged = 0;
    }

    if (bridgeGaps) Gaps();

    lineFollow();

    unsigned long now = millis();
    if (now - lastPingMs >= PING_INTERVAL) {
      lastPingMs = now;

      // 避障冷却期内不触发新的避障
      if (now >= avoidCooldownUntil) {
        myServo.write(SERVO_CENTER);
        long front = getDistance();
        if (front < OBST) {
          mode = AVOID;
        }
      }
    }
  }

  if (mode == AVOID) {
    digitalWrite(debugLEDGreen, LOW);
    digitalWrite(debugLEDYellow, HIGH);
    avoidObstacle();
    avoidCooldownUntil = millis() + AVOID_COOLDOWN_MS;
    mode = NORMAL;
  }
}