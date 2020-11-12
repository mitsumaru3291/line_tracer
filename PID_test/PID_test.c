#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

//#define MAX_BUFFER_SIZE 256
#define Threshold 400
#define base_motor_velocity 20
#define KP 0.2
//int argc, char **argv
// entree point of the controller
int main() {
  // Webots APIを呼び出して初期化
  wb_robot_init();
  printf("Hello\n");
  // ローカル変数
  float v_motorL = 0; 
  float v_motorR = 0;
  //bool is_sensor_on_track[6]={0, 0, 0, 0, 0, 0};
  //float line_position = 0;
 
 
  // for文で使う変数を定義
  int i;

  // Robotノード内のセンサを使うための初期化の処理
  // DistanceSensor の name 欄で付けた名前を使用する
  // WbDeviceTag はアクチュエータやセンサの情報を扱うためのもの
  WbDeviceTag gs[6];
  char gs_names[6][10] = {"gs0","gs1","gs2","gs3","gs4","gs5"};
  for (i = 0; i < 6; i++) {
    // gs_name の配列で指定された名前のデバイスの情報をシミュレーション環境から取得
    gs[i] = wb_robot_get_device(gs_names[i]); 
    // 取得したデバイスを距離センサとして使うための設定
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // モータを初期化
  // wheel_names には HingeJoint > device > RotationalMotor の name で指定した名前を指定する
  WbDeviceTag wheels[2];
  char wheels_names[2][8] = {"WheelBL","WheelBR"};
  for (i = 0; i < 2; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    // モータの回転角を指定する関数。今回はモータの回転速度を指令値とし、回転角指令は用いない。
    // wb_motor_set_position の第二引数（回転角）に INFINITY を指定する。
    wb_motor_set_position(wheels[i], INFINITY);
    // wb_motor_set_velocity でモータの回転速度を指示する。
    // 第一引数が対象のモータ、第二引数が回転速度
    wb_motor_set_velocity(wheels[i], 0);
  }

  // ライントレーサのアルゴリズム
  float diff[6][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
  float diff_R[2] = {0,0};
  float diff_L[2] = {0,0};
  while (wb_robot_step(TIME_STEP) != -1) {
  
    unsigned short gs_value[6] = {0, 0, 0, 0, 0, 0};
    
    diff_R[0] = diff_R[1];
    diff_L[0] = diff_L[1];
    diff_R[1] = 0;
    diff_L[1] = 0;
    for(i = 0; i < 6; i++){
      gs_value[i] = wb_distance_sensor_get_value(gs[i]); // gs[i]のセンサからセンサの値を読み込む
      printf("gs[%d]:\t%u\n", i, gs_value[i]);
      diff[i][0] = diff[i][1];
      diff[i][1] = gs_value[i] - Threshold;
      if (i <= 2){
        diff_R[1] += diff[i][1]/6;
      }else{
        diff_L[1] += diff[i][1]/6;
      }
      //line_position += (is_sensor_on_track[i] * (i-2.5)) / 6;
    }
    printf("diff_R: %lf\t diff_L: %lf\n", diff_R[1], diff_L[1]);
    
    // 移動モードの決定
    /*
    if (base_motor_velocity + diff_L[1] * KP > 100){
      v_motorL = 100;
    }else{
      v_motorL = base_motor_velocity + diff_L[1] * KP;
    }
    
    if (base_motor_velocity - diff_R[1] * KP < -100){
      v_motorR = -100;
    }else{
      v_motorR = base_motor_velocity - diff_R[1] * KP;
    }
    
    printf("v_motorL: %lf\t v_motorR:%lf\n", v_motorL, v_motorR);
    */
     
    // write actuators inputs
    wb_motor_set_velocity(wheels[0], v_motorL);
    wb_motor_set_velocity(wheels[1], v_motorR);
  }

  // webots APIの終了の処理
  wb_robot_cleanup();
  return 0; 
}