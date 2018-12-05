/* This file makes this project Arduino like, so that you don't need to mess
too much with the system. And it should be easy to test.
	Although, it is possible that you still need to mess with code outside this
file to achieve certain functions that you might need.
*/

/****************************************************************************
	Some of the configurations has to be changed inside the code, please record
them in this part of the annotations.

Parameters changing paths:
	PWM frequencies: (all ports are in the same frequency)
		main.h: #define PWM_FREQUENCE xxxxxxxx
****************************************************************************/

#include "control_functions.h"
#include "robot_configs.h"

static float angle_ratio = 815; // Ratio between required angle and the actual output for M2006

/* chassis motor control parameters */
PID_TypeDef motor_pid[4];
PID_TypeDef motor_position_pid[4];
float last_motor_target [4]; // start at 0

/* gimbal yaw motor control */
PID_TypeDef yaw_speed_pid;
PID_TypeDef yaw_position_pid;

/* remote control parameters */
int spinning = 0; // 0 for not; 1 for one direction; -1 for another;
float spinning_speed = 0;

/* overall parameters for gimbal movement */
float assuming_angle; // Since there is not slip ring on the top, we assume a delta angle.
float gimbal_target;

/** @brief start given PWM ports, which should be put in __init__()
**/
void TIMs_start() {
	/* start specific pin port onboard */
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3); // servo
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1); // servo
	// Now chaning the value in TIMx->CCRx 
	// (x refers to a number) should change the output of PWM port
}
/** @brief change the duty of specific tim channel port (PWM duty)
**/
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty){
	float value = (PWM_RESOLUTION*duty) - 1;
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = value;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = value;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = value;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = value;break;
	}
}


/**	@brief Arduino like process of initializing the peripherals.
**/
void _init_() {
	TIMs_start();
	/*< 初始化PID参数 >*/
	
	// Initializing chassis motor pid
  for(int i=0; i<4; i++) {
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0, \
															4,0.05,0);
  }
	
	// Initializing yaw motor pid
	pid_init(&yaw_speed_pid);
	yaw_speed_pid.f_param_init(&yaw_speed_pid, PID_Speed, 16384,5000,10,0,8000,0, \
															4,0.05,0);
	pid_init(&yaw_position_pid);
	yaw_position_pid.f_param_init(&yaw_position_pid, PID_Position, 65535, 65535, 0.5,10,5,0, \
															0.5,0.2,0);
	
	/* Initialization Done */
	HAL_Delay(20); // wait for 20ms
}
/**	@brief Arduino like process of keep looping and you can perform testing.
**/
void _loop_() {
		if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){
			//如果500ms都没有收到遥控器数据，证明遥控器可能已经离线
			set_moto_current(&hcan1, 0, 0, 0, 0);   // loose the motor
			set_moto_current_MORE(&hcan1, 0, 0, 0, 0); // loose the motor
			HAL_Delay(10);
			return;
    }
		/* calcualte and set pid targets */
    // remote_control.ch4*8000; // 摇杆度数
		// configure spinning switch
		if (remote_control.switch_left == 1) { // left up
		spinning = 1;
		} else if (remote_control.switch_left == 2) { // left down
			spinning = -1;
		} else { // left middle
			spinning = 0;
		}
		if (remote_control.switch_right == 1) { // right up
			spinning_speed = 500;
		} else if (remote_control.switch_right == 2) { // right down
			spinning_speed = 150;
		} else { //left middle
			spinning_speed = 0;
		}
		// configure chiassis operations
		float chassis_movement[4] = {0};
		float chassis_motor_output[4] = {0};
		assuming_angle += spinning_speed * spinning / HAL_FREQUENCY;
		spinning_top( remote_control.ch4, 
									remote_control.ch3,
									remote_control.ch1,
									spinning_speed * spinning,
									assuming_angle, chassis_movement);
		mecanum_calc( chassis_movement[0],
									chassis_movement[1],
									chassis_movement[2], chassis_motor_output);
		// assign value to pid target
		yaw_speed_pid.target = chassis_movement[3];
		for (int i=0; i<4; i++) motor_pid[i].target = chassis_motor_output[i];
		
		
		/* proceed PID caluclation and send output */
		for(int i=0; i<4; i++){																						
			motor_pid[i].f_cal_pid(&motor_pid[i], moto_chassis[i].speed_rpm);
		}
		yaw_speed_pid.f_cal_pid(&yaw_speed_pid, moto_chassis_more[0].speed_rpm);
		//将PID的计算结果通过CAN发送到电机
		set_moto_current(&hcan1, motor_pid[0].output,
														motor_pid[1].output,
														motor_pid[2].output,
														motor_pid[3].output);
		set_moto_current_MORE(&hcan1, yaw_speed_pid.output, 0,0,0);
		
		HAL_Delay(1000/HAL_FREQUENCY);      	//PID控制频率100HZ
}
