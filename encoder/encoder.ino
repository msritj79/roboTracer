
struct CourseData
{
    unsigned long timestamp; // 時間（ミリ秒単位）
    int left_marker_count;   // 左のマーカのカウント
    int right_position;      // 右のエンコーダの値
    int left_position;       // 左のロボットの速度
};

#define MAX_DATA_POINTS 500

CourseData courseData[MAX_DATA_POINTS];

// エンコーダーの値を保存するための変数
int dataCount = 0;
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int left_marker_count = 0;

void collectData()
{
    if (dataCount < MAX_DATA_POINTS)
    {
        courseData[dataCount].left_marker_count = left_marker_count;
        courseData[dataCount].timestamp = millis();
        courseData[dataCount].right_position = rightEncoderCount;
        courseData[dataCount].left_position = leftEncoderCount;
        dataCount++;
    }
}

// ピンの設定
int DIR_R_Pin = D12;
int DIR_L_Pin = D0;
int PWM_R_Pin = D11;
int PWM_L_Pin = D10;

int SW1_Pin = D7;
int SW2_Pin = D8;

int POWER_Pin = A6;
int LINE_L2_Pin = A5;
int LINE_L1_Pin = A4;
int LINE_R1_Pin = A3;
int LINE_R2_Pin = A2;

int LED_Pin = D13;
int BUZZER_Pin = D2;

// エンコーダーのピン
int leftEncoderAPin = D1;  // LHallA
int leftEncoderBPin = D9;  // LHallB
int rightEncoderAPin = A1; // RHallA
int rightEncoderBPin = A0; // RHallB

// 変数たち
float pwm_max = 50;
float pwm = 0;
float Kp = 1;
float Kd = 0.0;
long Line_Controll_before = 0;
long diff_control;
int cnt = 1;

long Line_Controll;
int inside_offset = 0;
int outside_offset = 0;
int L2_Value = 0;
int L1_Value = 0;
int R1_Value = 0;
int R2_Value = 0;
int ML_Value = 0;
int MR_Value = 0;

int CW_R = LOW;
int CCW_R = HIGH;
int CW_L = HIGH;
int CCW_L = LOW;
float PWM_R_Value = 0;
float PWM_L_Value = 0;

// ラインが白の場合:1、ラインが黒の場合-1
int Line_signed = -1;

// 0:マーカー未検出、クロス未検出
// 1:Startマーカーを検出
// 3:マーカー未検出、クロス未検出（通常走行）
// 4:クロスライン通過中
// 5:マーカー未検出、クロス未検出
// 6:ダミーマーカー通過中
// 7:ゴールマーカー検出
int line_State = 3;

void get_AD(void)
{
    R2_Value = analogRead(LINE_R2_Pin);
    R1_Value = analogRead(LINE_R1_Pin);
    L1_Value = analogRead(LINE_L1_Pin);
    L2_Value = analogRead(LINE_L2_Pin);
    ML_Value = adc_read_value(PB_1, 10);
    MR_Value = adc_read_value(PB_0, 10);

    // 床の反射率が一定ではないため、特定の箇所で調査し、平均化したオフセットを算出
    outside_offset = -0.1579 * L2_Value + 113.47;
    inside_offset = -0.1352 * L1_Value + 15.276;
}

// 左エンコーダーの割り込み処理関数
void handleLeftEncoder()
{
    if (digitalRead(leftEncoderBPin) == HIGH)
    {
        leftEncoderCount++;
    }
    else
    {
    }
}

// 右エンコーダーの割り込み処理関数
void handleRightEncoder()
{
    if (digitalRead(rightEncoderBPin) == HIGH)
    {
        rightEncoderCount++;
    }
    else
    {
    }
}

void firstRun()
{
    get_AD();
    // Line_signedで不等号が変わるように
    // クロスラインの処理もいる
    if (ML_Value < 100)
    {
        left_marker_count++;
        collectData();
    }
    diff_control = Line_Controll - Line_Controll_before;
    Line_Controll_before = Line_Controll;
    // ラインセンサの値から制御量を算出する
    Line_Controll = (L1_Value - R1_Value - inside_offset) + 2 * (L2_Value - R2_Value - outside_offset);
    if (cnt <= 200)
    {
        if (cnt < pwm_max)
        {
            pwm = (pwm_max / 10) + cnt * 1;
        }
        cnt++;
    }

    if (Line_Controll > 0)
    {
        PWM_L_Value = pwm;
        PWM_R_Value = pwm + Line_signed * Line_Controll * Kp - diff_control * Kd;
    }
    else
    {
        PWM_R_Value = pwm;
        PWM_L_Value = pwm - Line_signed * Line_Controll * Kp + diff_control * Kd;
    }

    PWM_L_Value = int(PWM_L_Value);
    PWM_R_Value = int(PWM_R_Value);

    digitalWrite(DIR_L_Pin, CW_L); // モーター前進設定
    if (PWM_L_Value > pwm_max)
    {
        PWM_L_Value = pwm_max; // モーター制御値上下ガード処理
    }
    if (PWM_L_Value <= 0)
    {
        PWM_L_Value = 0; // モーター制御値上下ガード処理
    }
    analogWrite(PWM_L_Pin, PWM_L_Value);

    digitalWrite(DIR_R_Pin, CW_R); // モーター前進設定
    if (PWM_R_Value > pwm_max)
    {
        PWM_R_Value = pwm_max; // モーター制御値上下ガード処理
    }
    if (PWM_R_Value <= 0)
    {
        PWM_R_Value = 0; // モーター制御値上下ガード処理
    }
    analogWrite(PWM_R_Pin, PWM_R_Value);

    delay(1);
}

void setup()
{
    // put your setup code here, to run once:
    // IOポート設定
    pinMode(BUZZER_Pin, OUTPUT);
    pinMode(LED_Pin, OUTPUT);
    pinMode(SW1_Pin, INPUT_PULLUP);
    pinMode(SW2_Pin, INPUT_PULLUP);
    pinMode(DIR_R_Pin, OUTPUT);
    pinMode(DIR_L_Pin, OUTPUT);

    pinMode(leftEncoderAPin, INPUT_PULLUP);
    pinMode(rightEncoderAPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftEncoderAPin), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncoderAPin), handleRightEncoder, CHANGE);

    LED_DRIVE(2, 100, 100);
    Serial.begin(115200);
    Serial.printf("time, leftEncoderCount, rightEncoderCount");
    collectData();

    while (1)
    {
        if (digitalRead(SW2_Pin) == LOW)
        {
            BUZZER_DRIVE(1, 100, 100);
            while (1)
            {
                firstRun();
                // Line_signedで不等号が変わるように
                // クロスラインの処理もいる
                if (MR_Value < 70)
                {
                    analogWrite(PWM_L_Pin, 0);
                    analogWrite(PWM_R_Pin, 0);
                    break;
                }
            }
        }

        // 左のスイッチを押したら、serialPrint
        if (digitalRead(SW1_Pin) == LOW)
        {
            BUZZER_DRIVE(2, 70, 70);
            Serial.printf("--------------------------print memory-------------------\n");
            Serial.printf("left_marker_count, time, leftEncoderCount, rightEncoderCount\n");
            for (unsigned int i = 0; i < dataCount; i++)
            {
                Serial.printf("%d, %lu, %d, %d\n", courseData[i].left_marker_count, courseData[i].timestamp, courseData[i].left_position, courseData[i].right_position);
            }
            Serial.printf("%d, %lu, %d, %d\n", left_marker_count, millis(), leftEncoderCount, rightEncoderCount);
            // break;
        }
    }
}

void loop()
{
}