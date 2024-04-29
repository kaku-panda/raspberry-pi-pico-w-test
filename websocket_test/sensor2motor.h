#include <stdint.h>
#include <stdlib.h>  // abs関数のために追加

// モータアクセル値の配列
uint32_t motor_accel_values[256];

// 値を0と255の間に制限する関数
uint8_t clip_value(int value) {
    if (value < 0) {
        return 0;
    } else if (value > 255) {
        return 255;
    } else {
        return (uint8_t)value;
    }
}

void setup_motor_accel_values() {
    for (int i = 0; i < 256; i++) {
        int left_motor = 128;  // 左モータの初期値
        int right_motor = 128; // 右モータの初期値

        // 全てのセンサーがHighまたはLowの場合はモーターを停止
        if (i == 0 || i == 255) {
            motor_accel_values[i] = 0;
            continue;  // 次のループへ
        }

        // 左側のセンサー（センサー1から4）がどれだけHighかを数える
        int left_sensors_high = ((i >> 4) & 1) + ((i >> 5) & 1) + ((i >> 6) & 1) + ((i >> 7) & 1);
        // 右側のセンサー（センサー5から8）がどれだけHighかを数える
        int right_sensors_high = (i & 1) + ((i >> 1) & 1) + ((i >> 2) & 1) + ((i >> 3) & 1);

        // 左側のセンサーが黒線を検出した場合の処理
        if (left_sensors_high > 0) {
            right_motor += 30 * left_sensors_high;  // 右モータを強くする
            left_motor -= 30 * left_sensors_high;  // 左モータを弱くする
        }

        // 右側のセンサーが黒線を検出した場合の処理
        if (right_sensors_high > 0) {
            left_motor += 30 * right_sensors_high;  // 左モータを強くする
            right_motor -= 30 * right_sensors_high; // 右モータを弱くする
        }

        // アクセル値を調整し、前進または後退の値を決定
        uint8_t left_motor_forward = clip_value(left_motor > 0 ? left_motor : 0);
        uint8_t left_motor_backward = clip_value(left_motor < 0 ? -left_motor : 0);
        uint8_t right_motor_forward = clip_value(right_motor > 0 ? right_motor : 0);
        uint8_t right_motor_backward = clip_value(right_motor < 0 ? -right_motor : 0);

        // アクセル値を32ビット整数に組み立てる
        uint32_t accel_value = ((uint32_t)left_motor_forward << 24) | 
                               ((uint32_t)left_motor_backward << 16) |
                               ((uint32_t)right_motor_forward << 8) |
                               (uint32_t)right_motor_backward;

        // 配列に値を設定
        motor_accel_values[i] = accel_value;
    }
}

#define LINE_MIDDLE 4.5
#define SPEED       0.6
#define MAX_SPEED   255

float get_average_line_position(uint8_t sensor_value){
    int length = 0;
    int sum    = 0;
    for(int n = 0; n < 8; n++){
        if(sensor_value >> n & 1){
            sum += n+1;
            length++;
        }
    }
    if(length == 0){
        return 0;
    }else{
       return sum / length;
    }
}

static float integral = 0.0;    // 積分値
static float last_error = 0.0;  // 前回のエラー

uint32_t control_motors(uint8_t sensor_value, float Kp, float Ki, float Kd) {
    float position = get_average_line_position(sensor_value);
    float error = LINE_MIDDLE - position;

    // 積分値の更新
    integral += error;

    // 微分値の計算
    float derivative = error - last_error;

    // PID制御の計算
    float correction = Kp * error + Ki * integral + Kd * derivative;

    // モーター速度の設定
    int motor_speed_left = clip_value((int)(MAX_SPEED * (SPEED + correction)));
    int motor_speed_right = clip_value((int)(MAX_SPEED * (SPEED - correction)));

    // エラーの更新
    last_error = error;

    // モーター制御値を32ビット整数に組み立てる
    return ((uint32_t)motor_speed_left << 24) | ((uint32_t)motor_speed_right << 8);
}