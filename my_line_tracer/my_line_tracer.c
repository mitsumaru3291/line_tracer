#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

// time in [ms] of a simulation step
#define TIME_STEP 1 //改善の余地あり

#define MAX_BUFFER_SIZE 256
#define KP 0.03
#define KD 0.008 //ゲインテキトー

// entree point of the controller
int main(int argc, char **argv) {
  float diff[2] = {0,0};

  // ローカル変数
  float v_motorL = 0; 
  float v_motorR = 0;
  float base_velocity = 20;
  float d_term;
  // Webots APIを呼び出して初期化
  wb_robot_init();

  int i;

  // Robotノード内のセンサを使うための初期化の処理
  // DistanceSensor の name 欄で付けた名前を使用する
  // WbDeviceTag はアクチュエータやセンサの情報を扱うためのもの
  WbDeviceTag gs[2];
  char gs_names[2][10] = {"gs0","gs1"};
  for (i = 0; i < 2; i++) {
    // gs_name の配列で指定された名前のデバイスの情報をシミュレーション環境から取得
    gs[i] = wb_robot_get_device(gs_names[i]); 
    // 取得したデバイスを距離センサとして使うための設定
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // モータを初期化
  // wheel_names には HingeJoint > device > RotationalMotor の name で指定した名前を指定する
  WbDeviceTag wheels[2];
  char wheels_names[2][8] = {"wheelL", "wheelR"};
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
  while (wb_robot_step(TIME_STEP) != -1) {

    unsigned short gs_value[2] = {0, 0};
    for(i = 0; i < 2; i++){
      gs_value[i] = wb_distance_sensor_get_value(gs[i]); // gs[i]のセンサからセンサの値を読み込む
      printf("gs[%d]:\t%u\n", i, gs_value[i]);
    }
    
    //誤差を求める 0:L,1:R
    diff[0] = diff[1];
    diff[1] = gs_value[1] - gs_value[0];
    d_term = (diff[1] - diff[0])/TIME_STEP;
    
    // 速度決定
    if (base_velocity + diff[1]*KP + d_term*KD> 50){
      v_motorL = 50;
    }else if(base_velocity + diff[1]*KP + d_term*KD < -50){
      v_motorL = -50;  
    }else{
      v_motorL = base_velocity + diff[1]*KP + d_term*KD;
    }
    
    if (base_velocity - diff[1]*KP + d_term*KD > 50){
      v_motorR = 50;
    }else if(base_velocity - diff[1]*KP + d_term*KD < -50){
      v_motorR = -50;  
    }else{
      v_motorR = base_velocity - diff[1]*KP + d_term*KD;
    }
    
    printf("diff: %lf\n", diff[1]);
    printf("v_motorL: %lf\t v_motorR: %lf\n", v_motorL, v_motorR);
     
   // write actuators inputs
   wb_motor_set_velocity(wheels[0], v_motorL);
   wb_motor_set_velocity(wheels[1], v_motorR);
  }

  // webots APIの終了の処理
  wb_robot_cleanup();
  return 0; 
}