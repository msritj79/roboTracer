/*
 *    Arduino_Sample/STEP9_1/STEP9_1.ino
 *    
 *    Copyright(C) 2020 RT Corporation <support@rt-net.jp>
 *    Copyright(C) 2020 M.Hirai
 *      All rights reserved.
 *      
 *    License: Apache License, Version 2.0
 *     https://www.apache.org/licenses/LICENSE-2.0
 *     
 */
#define MAX_SECTIONS 100  // 最大区間数を定義

// setting parameters
float Kp = 0.2;
float Kd = 0.2;
float PWM_MAX = 140; //max:255
float PWM_MAX_FIRST = 80; //1回目走行でのPWM_MAX（コース形状計測用）
float PWM_INIT = 20;
float pwm_max = PWM_INIT;
float SLOW_TIME = 200;
// int PWM_LIMIT = 100;

const int ML_THRESHOLD = 100; // マーカーの閾値
long encoderSections[MAX_SECTIONS][2]; // エンコーダ値を保存する配列（最大10区間と仮定）
int sectionIndex = 0; // 現在の区間のインデックス
bool markerDetected = false; // 前回のマーカー検出状態


//ピンの設定
int DIR_R_PIN = D12;
int DIR_L_PIN = D0;
int PWM_R_PIN = D11;
int PWM_L_PIN = D10;

int SW1_PIN = D7;
int SW2_PIN = D8;

int POWER_PIN = A6;
int LINE_L2_PIN = A5;
int LINE_L1_PIN = A4;
int LINE_R1_PIN = A3;
int LINE_R2_PIN = A2;

int LED_PIN = D13;
int BUZZER_PIN = D2;


// エンコーダーのピン
int leftEncoderAPin = D1; // LHallA
int leftEncoderBPin = D9; // LHallB
int rightEncoderAPin = A1; // RHallA
int rightEncoderBPin = A0; // RHallB 


// 変数宣言
long line_control;
long line_control_before = 0;
long diff_control;
int cnt = 1;
int inside_offset = 0;
int outside_offset = 0;
int L2_Value = 0; //left outside
int L1_Value = 0; //left inside
int R1_Value = 0; //right inside
int R2_Value = 0; //right outside
int ML_Value = 0; //left marker
int MR_Value = 0; //right marker

int CW_R  = LOW;
int CCW_R = HIGH;
int CW_L = HIGH;
int CCW_L = LOW;
float PWM_R_Value = 80;
float PWM_L_Value = 80;

//ラインが白の場合:1、ラインが黒の場合-1
int LINE_COLOR = -1;

int course_out_count = 0;

//0:マーカー未検出、クロス未検出
//1:Startマーカーを検出
//3:マーカー未検出、クロス未検出（通常走行）
//4:クロスライン通過中
//5:マーカー未検出、クロス未検出
//6:ダミーマーカー通過中
//7:ゴールマーカー検出
int run_state = 3;

// エンコーダーの値を保存するための変数
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// 左エンコーダーの割り込み処理関数
void handleLeftEncoder() {
  if (digitalRead(leftEncoderBPin) == HIGH) {
    leftEncoderCount++;
    Serial.print("Left Encoder Count: ");
    Serial.println(leftEncoderCount);
  } else {
    // leftEncoderCount--;
    // Serial.print("\n Left -");
  }
}

// 右エンコーダーの割り込み処理関数
void handleRightEncoder() {
  if (digitalRead(rightEncoderBPin) == HIGH) {
    rightEncoderCount++;
    Serial.print("Right Encoder Count: ");
    Serial.println(rightEncoderCount);
  } else {
    // rightEncoderCount--;
    // Serial.print("\n Right -");
  }
}


void get_AD(void) {
  R2_Value = analogRead(LINE_R2_PIN);
  R1_Value = analogRead(LINE_R1_PIN);
  L1_Value = analogRead(LINE_L1_PIN);
  L2_Value = analogRead(LINE_L2_PIN);
  ML_Value = adc_read_value(PB_1, 10);
  MR_Value = adc_read_value(PB_0, 10);


  // point1: black
  // (L1,R1,L2,R2) = (131,133,121,45)

  // point2: white
  // (L1,R1,L2,R2) = (746,840,598,640)

  // L1-R1_diff = -0.1496 * L1	+ 17.597
  // L2-R2_diff = -0.2474 * L2	+ 105.93



  //床の反射率が一定ではないため、特定の箇所で調査し、平均化したオフセットを算出
  outside_offset = L2_Value * (-0.1579) + 113.47;
  inside_offset = L1_Value * (-0.1352) + 15.276;
}

void debug_AD() {
  Serial.printf("\n\r %d ML=%d,L2=%d L1=%d R1=%d,R2=%d,MR=%d",
                run_state, ML_Value, L2_Value, L1_Value, R1_Value, R2_Value, MR_Value);
}


int MarkerCheck(void) {
  //      debug_AD();
  //if ((MR_Value > (R2_Value*0.169+2.7488+30))  && (MR_Value > 90)) { //マーカーセンサアクティブ
  if (LINE_COLOR == 1 ) {
    if (MR_Value > 70){ //マーカーセンサアクティブ: start or goal
      if (run_state == 0) {
        run_state = 1;
        digitalWrite(LED_PIN,HIGH);
        return 1;
      // } else if (run_state == 1) {
      //   run_state = 2;
      //   return 2;
      } else if (run_state == 3) {
        run_state = 7;
        digitalWrite(LED_PIN,HIGH);
        return 7;
      } else if (run_state == 4) {
        run_state = 6;
        return 6;
      } else if (run_state == 5) {
        run_state = 6;
        return 6;
      } else if(run_state == 6){
        run_state = 6;
        return 6;      
      }
    }
    else{
      //マーカー未検出のとき
      if (run_state == 1) {
        run_state = 3;
        return 3;
      }

      if (run_state == 6) {
        run_state = 3;
        return 3;
      }
    }

    //クロスライン
    if (run_state == 3) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) > 1300) { //クロスライン
        run_state = 4; //クロスラインを検出
        digitalWrite(LED_PIN,HIGH);
        return 4;
      }
    }
    if (run_state == 4) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) < 1200) {
        run_state = 5;
        digitalWrite(LED_PIN,LOW);
        return 5;
      }
    }
  }


  if (LINE_COLOR == -1 ) {
    if (MR_Value < 30){ //マーカーセンサアクティブ: start or goal
      if (run_state == 0) {
        run_state = 1;
        digitalWrite(LED_PIN,HIGH);
        return 1;
      // } else if (run_state == 1) {
      //   run_state = 2;
      //   return 2;
      } else if (run_state == 3) {
        run_state = 7;
        digitalWrite(LED_PIN,HIGH);
        return 7;
      } else if (run_state == 4) {
        run_state = 6;
        return 6;
      } else if (run_state == 5) {
        run_state = 6;
        return 6;
      } else if(run_state == 6){
        run_state = 6;
        return 6;      
      }
    }
    else{
      //マーカー未検出のとき
      if (run_state == 1) {
        run_state = 3;
        return 3;
      }

      if (run_state == 6) {
        run_state = 3;
        return 3;
      }
    }

    //クロスライン
    if (run_state == 3) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) > 1300) { //クロスライン
        run_state = 4; //クロスラインを検出
        digitalWrite(LED_PIN,HIGH);
        return 4;
      }
    }
    if (run_state == 4) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) < 1200) {
        run_state = 5;
        digitalWrite(LED_PIN,LOW);
        return 5;
      }
    }
  }


  return run_state;
}

void setup() {
  // put your setup code here, to run once:
  //IOポート設定
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(DIR_R_PIN, OUTPUT);
  pinMode(DIR_L_PIN, OUTPUT);

  LED_DRIVE(2, 100, 100);
  Serial.begin(9600);

  // print_param();
  
  initialize_run_mode();
}

void print_param(){
  while (1) {
    if (digitalRead(SW2_PIN) == LOW) {
      BUZZER_DRIVE(1, 100, 100);
      while (1) {
        get_AD();
        Serial.printf("\n\r control = %d VDD=%dmV LL2=%d LL1=%d LR1=%d LR2=%d inside_offset=%d outside_offset=%d ML=%d MR=%d xline=%d  marker_check=%d CTRL=%d",
                      (L1_Value - R1_Value - inside_offset) + 2 * (L2_Value - R2_Value - outside_offset),
                      analogRead(POWER_PIN) * 9677 / 1000,
                      L2_Value, L1_Value, R1_Value, R2_Value,
                      (L1_Value - R1_Value), (L2_Value - R2_Value),
                      adc_read_value(PB_1, 10), adc_read_value(PB_0, 10),
                      L2_Value + R1_Value + L1_Value + R2_Value,
                      MarkerCheck(),
                      L1_Value - R1_Value - inside_offset + 2 * (L2_Value - R2_Value - outside_offset)
                     );
        delay(100);
        if (digitalRead(SW2_PIN) == LOW) {
          delay(200);
          break;
        }
      }
    }
  }


}

void initialize_run_mode() {
    while(1){
      //左のスイッチを押したら、走行開始（２回目）
      if (digitalRead(SW1_PIN) == LOW) {
        digitalWrite(LED_PIN, HIGH);
        get_AD();
        BUZZER_DRIVE(2, 70, 70);
        break;
      }

      //右のスイッチを押したら、計測モードで走行開始（1回目）
      if (digitalRead(SW2_PIN) == LOW) {
        PWM_MAX = PWM_MAX_FIRST; //ゆっくり走る
        digitalWrite(LED_PIN, HIGH);
        get_AD();
        BUZZER_DRIVE(4, 70, 70); //４回ブザーを鳴らす
        break;
      }

    }

}

void left_marker_check(){
  // マーカーが検出されていない状態から検出された状態に変わったとき
  if (ML_Value > ML_THRESHOLD && !markerDetected) {
    encoderSections[sectionIndex][0] = leftEncoderCount;
    encoderSections[sectionIndex][1] = rightEncoderCount;
    sectionIndex++;
    
    // エンコーダをリフレッシュ
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    
    markerDetected = true; // マーカーが検出されたことを記録
  } else if (ML_Value <= ML_THRESHOLD) {
    markerDetected = false; // マーカーが検出されていないことを記録
  }
}


void loop() {
  // put your main code here, to run repeatedly:

  attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), handleRightEncoder, CHANGE);


  get_AD();
  //マーカーを検出?
  MarkerCheck();
  left_marker_check();
  if (run_state == 1) {
    BUZZER_DRIVE(1, 50, 50);
  } else if (run_state == 7) {
    BUZZER_DRIVE(2, 50, 50);
    RUN_STOP();
    initialize_run_mode();
  }
  if (LINE_COLOR == 1) {
    // スタートでもゴールでもない、ラインから外れた時にストップさせる
    if (L1_Value + R1_Value < 300) {
      course_out_count++;
    }
  }else{
    // スタートでもゴールでもない、ラインから外れた時にストップさせる
    if (L1_Value + R1_Value > 1400) {
      course_out_count++;
    }
  }
  if (course_out_count > 180){
    RUN_STOP();
  }

  // 以下からはライントレース
  diff_control = line_control - line_control_before;
  line_control_before = line_control;
  //ラインセンサの値から制御量を算出する、80：ラインからのオフセット（マーカ検出の微調整のため）
  // line_control = (L1_Value - R1_Value - inside_offset + 80) + 2 * (L2_Value - R2_Value - outside_offset + 80);
  line_control = (L1_Value - R1_Value - inside_offset) + 2 * (L2_Value - R2_Value - outside_offset);


  // スタート時の速度を抑える（急加速により不安定になるのを防ぐため）
  // SLOW_TIME秒後に、pwm_max = PWM_MAXとなるように計算する
  if (cnt <= SLOW_TIME){
    pwm_max = PWM_INIT + (PWM_MAX - PWM_INIT) / SLOW_TIME * cnt;
    cnt++;
  }

  if (LINE_COLOR == 1){
    if (line_control > 0 ){
      PWM_L_Value = pwm_max - LINE_COLOR * (line_control * Kp - diff_control * Kd);
      PWM_R_Value = pwm_max;
    }else{
      PWM_L_Value = pwm_max;
      PWM_R_Value = pwm_max + LINE_COLOR * (line_control * Kp - diff_control * Kd);
    }
  }
  else if (LINE_COLOR == -1){
    if (line_control > 0 ){
      PWM_L_Value = pwm_max;
      PWM_R_Value = pwm_max + LINE_COLOR * (line_control * Kp - diff_control * Kd);
    }else{
      PWM_L_Value = pwm_max - LINE_COLOR * (line_control * Kp - diff_control * Kd);
      PWM_R_Value = pwm_max;
    }
  }

  // 【テスト用】直線状に走らせる
  // PWM_R_Value = 100*255/103;
  // PWM_L_Value = 103*255/103;
  

  PWM_L_Value = int(PWM_L_Value);
  PWM_R_Value = int(PWM_R_Value);


  // Serial.printf("\n\r PWM_L_VALUE=%d,PWM_R_VALUE=%d",
  //               int(PWM_L_Value), int(PWM_R_Value));
  
  digitalWrite(DIR_L_PIN, CW_L);//モーター前進設定
  digitalWrite(DIR_R_PIN, CW_R);//モーター前進設定

  // if (PWM_L_Value > PWM_LIMIT) {
  //   PWM_L_Value = PWM_LIMIT; //モーター制御値上下ガード処理
  // }
  if (PWM_L_Value <= 0) {
    PWM_L_Value = 0; //モーター制御値上下ガード処理
  }

  // if (PWM_R_Value > PWM_LIMIT) {
  //   PWM_R_Value = PWM_LIMIT; //モーター制御値上下ガード処理
  // }
  if (PWM_R_Value <= 0) {
    PWM_R_Value = 0; //モーター制御値上下ガード処理
  }

  analogWrite(PWM_L_PIN, PWM_L_Value);
  analogWrite(PWM_R_PIN, PWM_R_Value);

  delay(1);
}
