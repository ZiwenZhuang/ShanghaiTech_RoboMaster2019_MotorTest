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

static float angle_ratio = 815; // Ratio between required angle and the actual output for M2006

/* chassis motor control parameters */
PID_TypeDef motor_pid[4];
PID_TypeDef motor_position_pid[4];
float motor_speed_target [4]; // set for control motors speed directly.
float motor_target [4]; // recording the angle for each motor.
float last_motor_target [4]; // start at 0

/* gimbal yaw motor control */
PID_TypeDef yaw_speed_pid;
PID_TypeDef yaw_position_pid;
float yaw_speed_target;
float yaw_position_target = 0; // both are used to store the last time target


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
			for (int i=0; i<4; i++) last_motor_target[i] = motor_target[i];
			set_moto_current(&hcan1, 0, 0, 0, 0);   // loose the motor
			set_moto_current_MORE(&hcan1, 0, 0, 0, 0); // loose the motor
			HAL_Delay(10);
			return;
    }else{ // 设定各部位的目标值
      // remote_control.ch4*8000; // 摇杆度数
			if (remote_control.switch_left == 1) { // left up
				
			} else if (remote_control.switch_left == 2) { // left down
				
			} else { // left middle
				
			}
			if (remote_control.switch_right == 1) { // right up
				
			} else if (remote_control.switch_right == 2) { // right down
				
			} else { //left middle
				
			}
			for (int i = 0; i < 4; i++) motor_speed_target[i] = 0;
    }
		
		// Proceed the operations
		for(int i=0; i<4; i++){	
			motor_pid[i].target = motor_speed_target[i]; // M2006																							
			motor_pid[i].f_cal_pid(&motor_pid[i], moto_chassis[i].speed_rpm);
		}
		yaw_position_pid.target = yaw_position_target;
		//将PID的计算结果通过CAN发送到电机
		set_moto_current(&hcan1, motor_pid[0].output,
														motor_pid[1].output,
														motor_pid[2].output,
														motor_pid[3].output);
		set_moto_current_MORE(&hcan1, 550, -550, 1000, -1000);
		
		HAL_Delay(10);      	//PID控制频率100HZ
}
