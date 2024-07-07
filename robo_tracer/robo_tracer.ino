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

// setting parameters 0.05/2.0/2.0
float Kp = 0.15; //0.15
float Kd = 2.0;  //2.0
int PWM_MAX = 70; //70 max:255
const int PWM_MIN = 10;
const int PWM_MAX_MIN = 30;
const int PWM_MAX_FIRST = 60; //60 1回目走行でのPWM_MAX（コース形状計測用）
const int PWM_MAX_SECOND = 80; //2回目走行でのPWM_MAX（コース形状計測用）
const int PWM_INIT = 20;
const int SLOW_TIME = 200;
float pwm_max = PWM_INIT;

//ラインが白の場合:1、ラインが黒の場合-1
const int LINE_COLOR = 1;

const int ML_THRESHOLD = 20; // section用 左マーカーの閾値 (白線,黒線)=(70,200)
const int MR_THRESHOLD = 70;  // run_state用 右マーカーの閾値 (白線,黒線)=(70,200)
const int CROSS_THRESHOLD = 1500; // クロスライン検出（R2+R1+L1+L2）の閾値 (白線,黒線)=(1500,1000)
const int COURSE_OUT_THRESHOLD = 350; // L1+R1 < thresholdのとき白ラインから外れてコースアウトと判断する
                                      // (白線,黒線)=(350,1400)

float k_reduce = -0.6; //0.5で左右比が2:1のとき減速比0.5
float l_reduce = 1.0;

int course_out_count = 0;
int course_out_reset_count = 0;
int CONTINUE_RUN_TIME = 100;
int continue_run_count = 0;

bool is_setting_mode = false;
bool is_first_run = true;
bool has_course_out = false;

//0:マーカー未検出、クロス未検出
//1:Startマーカーを検出
//3:マーカー未検出、クロス未検出（通常走行）
//4:クロスライン通過中
//5:マーカー未検出、クロス未検出
//6:ダミーマーカー通過中
//7:ゴールマーカー検出
int run_state = 0;

float PWM_R_Value = 80;
float PWM_L_Value = 80;

long encoderSections[MAX_SECTIONS][2]; // エンコーダ値を保存する配列（最大10区間と仮定）
int sectionIndex = 0; // 現在の区間のインデックス
bool markerDetected = false; // 前回のマーカー検出状態

float right_to_left_ratio_list[MAX_SECTIONS];
float reduction_ratio_list[MAX_SECTIONS];

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

// エンコーダーの値を保存するための変数
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;



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
  // Serial.begin(9600);

  // print_param();

    // 初期化
  for (int i=0; i<MAX_SECTIONS; i++){
    right_to_left_ratio_list[i] = 1.0;
    reduction_ratio_list[i] = 1.0;
  }

  initialize_run_mode();
}

void print_param(){
  // 右のスイッチを１回押すと各種パラメータをシリアルプリントし、２回押すとパラメータ確認を終えて走行モード選択へ移行する
  while (1) {
    if (digitalRead(SW2_PIN) == LOW) {
      BUZZER_DRIVE(1, 100, 100);
      while (1) {
        get_AD();
        int line_control_i = (L1_Value - R1_Value - inside_offset) + 2 * (L2_Value - R2_Value - outside_offset);
        right_marker_check();

        Serial.printf("\n\r line_control = %d L2=%d L1=%d R1=%d R2=%d ML=%d MR=%d \n\r L1-R1-inside_offset=%d L2-R2-outside_offset=%d \n\r L2+L1+R1+R2=%d L1+R1=%d \n\r run_state=%d",
                      line_control_i,
                      L2_Value, L1_Value, R1_Value, R2_Value,
                      ML_Value, MR_Value,
                      L1_Value - R1_Value - inside_offset,    //０になることが理想
                      L2_Value - R2_Value - outside_offset,   //０になることが理想
                      L2_Value + L1_Value + R1_Value + R2_Value,
                      L1_Value + R1_Value,
                      run_state
                      );

        Serial.printf("\n\r left_encorder_cnt=%d right_encorder_cnt=%d", leftEncoderCount, rightEncoderCount);
        Serial.printf("\n\r PWM_L_Value=%d PWM_R_Value=%d", PWM_L_Value, PWM_R_Value);

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
        is_first_run = false;
        if (has_course_out == false){
          PWM_MAX += 20;
        }else{
          PWM_MAX -= 30;
        }
        
        digitalWrite(LED_PIN, HIGH);
        get_AD();
        BUZZER_DRIVE(2, 70, 70);
        calc_ratio();
        break;
      }

      //右のスイッチを押したら、計測モードで走行開始（1回目）
      if (digitalRead(SW2_PIN) == LOW) {
        is_first_run = true;
        PWM_MAX = PWM_MAX_FIRST; //ゆっくり走る
        digitalWrite(LED_PIN, HIGH);
        get_AD();
        BUZZER_DRIVE(4, 70, 70); //４回ブザーを鳴らす
        break;
      }

    }

}

// void count_encoder() {
// // エンコーダーカウントの割り込み処理関数
//   if (digitalRead(leftEncoderAPin) == HIGH) {
//     leftEncoderCount++;
//     Serial.print("Left Encoder Count: ");
//     Serial.println(leftEncoderCount);
//   }
//   if (digitalRead(rightEncoderAPin) == HIGH) {
//     rightEncoderCount++;
//     Serial.print("Right Encoder Count: ");
//     Serial.println(rightEncoderCount);
//   }
// }

// 左エンコーダーの割り込み処理関数
void handleLeftEncoder() {
  if (digitalRead(leftEncoderBPin) == HIGH) {
    leftEncoderCount++;
  }
}

// 右エンコーダーの割り込み処理関数
void handleRightEncoder() {
  if (digitalRead(rightEncoderBPin) == HIGH) {
    rightEncoderCount++;
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

int right_marker_check(void) {
  if (LINE_COLOR == 1 ) {
    if (MR_Value > MR_THRESHOLD){ //マーカーセンサアクティブ: start or goal
      if (run_state == 0) {
        run_state = 1;
        digitalWrite(LED_PIN,HIGH);
        return 1;
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
      if ((R2_Value + R1_Value + L1_Value + L2_Value) > CROSS_THRESHOLD) { //クロスライン
        run_state = 4; //クロスラインを検出
        digitalWrite(LED_PIN,HIGH);
        return 4;
      }
    }
    if (run_state == 4) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) < CROSS_THRESHOLD) {
        run_state = 5;
        digitalWrite(LED_PIN,LOW);
        return 5;
      }
    }
  }


  if (LINE_COLOR == -1 ) {
    if (MR_Value < MR_THRESHOLD){ //マーカーセンサアクティブ: start or goal
      if (run_state == 0) {
        run_state = 1;
        digitalWrite(LED_PIN,HIGH);
        return 1;
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
      if ((R2_Value + R1_Value + L1_Value + L2_Value) < CROSS_THRESHOLD) { //クロスライン
        run_state = 4; //クロスラインを検出
        digitalWrite(LED_PIN,HIGH);
        return 4;
      }
    }
    if (run_state == 4) {
      if ((R2_Value + R1_Value + L1_Value + L2_Value) > CROSS_THRESHOLD) {
        run_state = 5;
        digitalWrite(LED_PIN,LOW);
        return 5;
      }
    }
  }


  return run_state;
}

void left_marker_check(){

  if (LINE_COLOR == 1){
    // マーカーが検出されていない状態から検出された状態に変わったとき
    if (ML_Value > ML_THRESHOLD && !markerDetected){
      if (is_first_run == true){
      encoderSections[sectionIndex][0] = leftEncoderCount;
      encoderSections[sectionIndex][1] = rightEncoderCount;
      // エンコーダをリフレッシュ
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      }

      sectionIndex++;
      markerDetected = true; // マーカーが検出されたことを記録
      // Serial.println("Left marker detected");
    } else if (ML_Value <= ML_THRESHOLD) {
      markerDetected = false; // マーカーが検出されていないことを記録
      // Serial.println("Left marker UNUNUNdetected");
    }
  }
  else{
    // マーカーが検出されていない状態から検出された状態に変わったとき
    if (ML_Value < ML_THRESHOLD && !markerDetected){
      encoderSections[sectionIndex][0] = leftEncoderCount;
      encoderSections[sectionIndex][1] = rightEncoderCount;
      sectionIndex++;
      
      // エンコーダをリフレッシュ
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      
      markerDetected = true; // マーカーが検出されたことを記録

    } else if (ML_Value >= ML_THRESHOLD) {
      markerDetected = false; // マーカーが検出されていないことを記録
    }
  }
}

void calc_ratio(){
  // ２回目以降の走行では前回走行で測定したエンコーダの値から左右のPWM速度比を設定しておく。そのためのパラメータをここでは計算しておき、trace_lineで使用する
  for (int section_i=0; section_i<MAX_SECTIONS; section_i++){
    if (encoderSections[section_i][0] != 0){
      right_to_left_ratio_list[section_i] = (float)encoderSections[section_i][1] / encoderSections[section_i][0];

      // カーブが急な時は最大速度を落とすための減速比を計算
      // 左右の速度比と減速比の対応関係を適当に一次関数で表現した
      if (right_to_left_ratio_list[section_i] < 1.0){
        reduction_ratio_list[section_i] = k_reduce * (1.0 - right_to_left_ratio_list[section_i]) + l_reduce;
      }else{
        reduction_ratio_list[section_i] = k_reduce * (right_to_left_ratio_list[section_i] - 1.0) + l_reduce;
      }
    }

    // if (section_i < 10){
    //   Serial.println(section_i);
    //   Serial.print("reduction_ratio");
    //   Serial.println(reduction_ratio_list[section_i]);
    //   Serial.print("right_to_left_ratio");
    //   Serial.println(right_to_left_ratio_list[section_i]);

    // }



  }

}

void detect_course_out(){
  if (LINE_COLOR == 1) {
    // スタートでもゴールでもない、ラインから外れた時にストップさせる
    if (L1_Value + R1_Value < COURSE_OUT_THRESHOLD) {
      course_out_count++;
    }else{
      course_out_reset_count++;
    }
  }else{
    // スタートでもゴールでもない、ラインから外れた時にストップさせる
    if (L1_Value + R1_Value > COURSE_OUT_THRESHOLD) {
      course_out_count++;
    }
  }
  if (course_out_count > 800){
    analogWrite(PWM_L_PIN, 0);
    analogWrite(PWM_R_PIN, 0);
    has_course_out = true;
    ready_to_rerun();
  }
  if (course_out_reset_count > 500){
    course_out_count = 0;
    course_out_reset_count = 0;
  }
}

void ready_to_rerun(){
  run_state=0;
  continue_run_count = 0;
  course_out_count = 0;
  continue_run_count = 0;
  cnt = 1;
  sectionIndex = 0;
  markerDetected = false;
  initialize_run_mode();
}

void trace_line(){
  float pwm_max_L=80;
  float pwm_max_R=80;

  diff_control = line_control - line_control_before;
  line_control_before = line_control;
  //ラインセンサの値から制御量を算出する、80：ラインから横方向へのオフセット（マーカ検出の微調整のため）
  line_control = (L1_Value - R1_Value - inside_offset - 180) + 2 * (L2_Value - R2_Value - outside_offset - 180);

  // スタート時の速度を抑える（急加速により不安定になるのを防ぐため）
  // SLOW_TIME秒後に、pwm_max = PWM_MAXとなるように計算する
  if (cnt <= SLOW_TIME){
    pwm_max = PWM_INIT + (float)(PWM_MAX - PWM_INIT) / SLOW_TIME * cnt;
    cnt++;
  }

  // カーブの曲率によって決まる減速比に応じて最大速度を変更
  if (sectionIndex > 0){
    pwm_max = PWM_MAX * reduction_ratio_list[sectionIndex];
  }

  if (pwm_max < PWM_MAX_MIN){
    pwm_max = PWM_MAX_MIN;
  }
  pwm_max_L = pwm_max;
  pwm_max_R = pwm_max;
  

  // // カーブの曲率によって決まる左右比に応じて、左右の最大速度を変更
  // if (right_to_left_ratio_list[sectionIndex] <= 1.0){
  //   pwm_max_R = pwm_max * right_to_left_ratio_list[sectionIndex];
  // }else{
  //   pwm_max_L = pwm_max * right_to_left_ratio_list[sectionIndex];
  // }

  if (LINE_COLOR == 1){
    if (line_control > 0 ){
      PWM_L_Value = pwm_max_L - LINE_COLOR * (line_control * Kp + diff_control * Kd);
      PWM_R_Value = pwm_max_R;
    }else{
      PWM_L_Value = pwm_max_L;
      PWM_R_Value = pwm_max_R + LINE_COLOR * (line_control * Kp + diff_control * Kd);
    }
  }
  else if (LINE_COLOR == -1){
    if (line_control > 0 ){
      PWM_L_Value = pwm_max_L;
      PWM_R_Value = pwm_max_R + LINE_COLOR * (line_control * Kp - diff_control * Kd);
    }else{
      PWM_L_Value = pwm_max_L - LINE_COLOR * (line_control * Kp - diff_control * Kd);
      PWM_R_Value = pwm_max_R;
    }
  }

  if (PWM_L_Value <= PWM_MIN) {
    PWM_L_Value = PWM_MIN; //モーター制御値上下ガード処理
  }

  if (PWM_R_Value <= PWM_MIN) {
    PWM_R_Value = PWM_MIN; //モーター制御値上下ガード処理
  }
  
  PWM_L_Value = int(PWM_L_Value);
  PWM_R_Value = int(PWM_R_Value);

  if (is_setting_mode) {
    PWM_L_Value = 0;
    PWM_R_Value = 0;
  }

  digitalWrite(DIR_L_PIN, CW_L);//モーター前進設定
  digitalWrite(DIR_R_PIN, CW_R);//モーター前進設定
  analogWrite(PWM_L_PIN, PWM_L_Value);//モーターPWM設定
  analogWrite(PWM_R_PIN, PWM_R_Value);//モーターPWM設定

}

void loop() {
  // put your main code here, to run repeatedly:

  // エンコーダの変化を検出したときに割り込みでエンコーダカウントを行う
  attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), handleRightEncoder, CHANGE);
  
  get_AD();
  left_marker_check();
  detect_course_out();
  trace_line();
  right_marker_check();

  // スタート時にブザーを鳴らす
  // ゴール時にぶらーを鳴らし、停止する。走行モードを初期化して２回目走行を可能な状態にする
  // if (run_state == 1) {
  //   BUZZER_DRIVE(1, 50, 50);
  // }
  if (run_state == 7) {
    continue_run_count++;
  }

  if(continue_run_count > CONTINUE_RUN_TIME){
    analogWrite(PWM_L_PIN, 0);
    analogWrite(PWM_R_PIN, 0);
    // RUN_STOP();
    has_course_out = false;
    ready_to_rerun();
  }

  if (is_setting_mode){
    print_param();
  }

  delay(1);
}
