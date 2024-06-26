/*
 *    Arduino_Sample/STEP7/STEP7.ino
 *    
 *    Copyright(C) 2020 RT Corporation <support@rt-net.jp>
 *    Copyright(C) 2020 M.Hirai
 *      All rights reserved.
 *      
 *    License: Apache License, Version 2.0
 *     https://www.apache.org/licenses/LICENSE-2.0
 *     
 */

//ピンの設定
int DIR_R_Pin = D12;
int DIR_L_Pin = D0;
int PWM_R_Pin = D11;
int PWM_L_Pin = D10;

int SW1_Pin = D7;
int SW2_Pin = D8;

int LINE_L2_Pin = A5;
int LINE_L1_Pin = A4;
int LINE_R1_Pin = A3;
int LINE_R2_Pin = A2;

int LED_Pin = D13;
int BUZZER_Pin = D2;

// エンコーダーのピン
int leftEncoderAPin = D1; // LHallA
int leftEncoderBPin = D9; // LHallB
int rightEncoderAPin = A1; // RHallA
int rightEncoderBPin = A0; // RHallB 

// エンコーダーの値を保存するための変数
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// 変数宣言
long  Line_Controll;
int inside_offset = -34;
int outside_offset = -146;

int CW_R  = 0;
int CCW_R = 1;
int CW_L = 1;
int CCW_L = 0;
int PWM_R_Value = 0;
int PWM_L_Value = 0;

//ラインが白の場合:1、ラインが黒の場合-1
int Line_signed = -1;

int loop_count = 0;

// 1秒あたりの目標パルス数
int targetPulsePerSecond = 300;
// 目標値に対する差を計算
int leftError = 0;
int rightError = 0;

// 左エンコーダーの割り込み処理関数
void handleLeftEncoder() {
  if (digitalRead(leftEncoderBPin) == HIGH) {
    leftEncoderCount++;
    // Serial.print("\n Left +");
  } else {
    // leftEncoderCount--;
    // Serial.print("\n Left -");
  }
}

// 右エンコーダーの割り込み処理関数
void handleRightEncoder() {
  if (digitalRead(rightEncoderBPin) == HIGH) {
    rightEncoderCount++;
    // Serial.print("\n Right +");
  } else {
    // rightEncoderCount--;
    // Serial.print("\n Right -");
  }
}


void setup() {
  // put your setup code here, to run once:
  //IOポート設定
  pinMode(BUZZER_Pin, OUTPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(SW1_Pin, INPUT_PULLUP);
  pinMode(SW2_Pin, INPUT_PULLUP);
  pinMode(DIR_R_Pin, OUTPUT);
  pinMode(DIR_L_Pin, OUTPUT);
  pinMode(leftEncoderAPin, INPUT_PULLUP);
  pinMode(leftEncoderBPin, INPUT_PULLUP);
  pinMode(rightEncoderAPin, INPUT_PULLUP);
  pinMode(rightEncoderBPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), handleRightEncoder, CHANGE);

  digitalWrite(LED_Pin, HIGH);
  delay(100);
  digitalWrite(LED_Pin, LOW);
  delay(100);
  digitalWrite(LED_Pin, HIGH);
  delay(100);
  digitalWrite(LED_Pin, LOW);
  delay(100);

  Serial.begin(115200);

  while (1) {

    if (digitalRead(SW2_Pin) == LOW) {
      digitalWrite(BUZZER_Pin, HIGH);   // ブザー鳴らす
      delay(100);               // 指定時間待ち
      digitalWrite(BUZZER_Pin, LOW);    // ブザー止める
      delay(100);              // 指定時間待ち
      while (1) {
        Serial.printf("\n\r LL2=%d LL1=%d LR1=%d LR2=%d inside_Offset=%d,outside_offset=%d",
                      analogRead(LINE_L2_Pin), analogRead(LINE_L1_Pin), analogRead(LINE_R1_Pin), analogRead(LINE_R2_Pin),
                      (analogRead(LINE_L1_Pin) - analogRead(LINE_R1_Pin)), (analogRead(LINE_L2_Pin) - analogRead(LINE_R2_Pin)));
        delay(100);
        if (digitalRead(SW2_Pin) == LOW) {
          delay(200);
          break;
        }
      }
    }

    //左のスイッチを押したら、走行開始
    if (digitalRead(SW1_Pin) == LOW) {
      digitalWrite(LED_Pin, HIGH);
      digitalWrite(BUZZER_Pin, HIGH);   // ブザー鳴らす
      delay(70);               // 指定時間待ち
      digitalWrite(BUZZER_Pin, LOW);    // ブザー止める
      delay(70);              // 指定時間待ち
      digitalWrite(BUZZER_Pin, HIGH);   // ブザー鳴らす
      delay(70);               // 指定時間待ち
      digitalWrite(BUZZER_Pin, LOW);    // ブザー止める
      delay(70);              // 指定時間待ち
      PWM_L_Value = 255;
      PWM_R_Value = 255;
      break;
    }
  }
}
void loop() {


  // put your main code here, to run repeatedly:
  //ラインセンサの値から制御量を算出する

  Line_Controll = (analogRead(LINE_L1_Pin) - analogRead(LINE_R1_Pin) - inside_offset)
                  + 2 * (analogRead(LINE_L2_Pin) - analogRead(LINE_R2_Pin) - outside_offset);

  // PWM_L_Value = 80 - Line_signed * Line_Controll / 10;
  // PWM_R_Value = 80 + Line_signed * Line_Controll / 10;

  // 目標値に対する差を計算
  if (loop_count > 0){
    leftError = targetPulsePerSecond - leftEncoderCount/loop_count;
    rightError = targetPulsePerSecond - rightEncoderCount/loop_count;
  }

  PWM_L_Value += 255*leftError/targetPulsePerSecond;
  PWM_R_Value += 255*rightError/targetPulsePerSecond;

  //左モーターPWM出力
  if (PWM_L_Value < 0) {
    digitalWrite(DIR_L_Pin, CCW_L);//モーター後進設定
  } else {
    digitalWrite(DIR_L_Pin, CW_L);//モーター前進設定
  }
  PWM_L_Value = abs(PWM_L_Value);
  if (PWM_L_Value > 255) {
    PWM_L_Value = 255; //モーター制御値上下ガード処理
  }
  if (PWM_L_Value <= 0) {
    PWM_L_Value = 0; //モーター制御値上下ガード処理
  }
  analogWrite(PWM_L_Pin, PWM_L_Value);

  //右モーターPWM出力
  if (PWM_R_Value < 0) {
    digitalWrite(DIR_R_Pin, CCW_R);//モーター後進設定
  } else {
    digitalWrite(DIR_R_Pin, CW_R);//モーター前進設定
  }
  PWM_R_Value = abs(PWM_R_Value);
  if (PWM_R_Value > 255) {
    PWM_R_Value = 255; //モーター制御値上下ガード処理
  }
  if (PWM_R_Value <= 0) {
    PWM_R_Value = 0; //モーター制御値上下ガード処理
  }
  analogWrite(PWM_R_Pin, PWM_R_Value);

  delay(1);
  //PWM、コントロールの値確認用
  // Serial.printf("\n\r PWM_R=%d PWM_L=%d CONT=%d",PWM_R_Value,PWM_L_Value,Line_Controll);
  Serial.print("loop_count: ");
  Serial.print(loop_count);
  Serial.print(", Left Encoder: ");
  Serial.print(leftEncoderCount);
  Serial.print(", Right Encoder: ");
  Serial.println(rightEncoderCount);
  Serial.print(", Left Error: ");
  Serial.print(leftError);
  Serial.print(", Right Error: ");
  Serial.println(rightError);
  loop_count += 1;
  delay(1000);

}
