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

/****************************
	This is a slip ring rigid test
****************************/


static float angle_ratio = 815; // Ratio between required angle and the actual output
PID_TypeDef motor_pid[4];
PID_TypeDef motor_position_pid[4];
float motor_target [4]; // 记录目标角度(轴)
float last_motor_target [4]; // start at 0
/*	0: loader motor (can id: 1)
		1: yaw motor (can id: 2) (will set speed only)
*/
int16_t yaw_speed_target = 0;

float pwm_duty = 0.05f; // This is defined bwtween 0 ~ 1; defined for fraction wheel

float pitch_pwm_duty = 0.075f; // 俯仰伺服机的参数
float pitch_reading_ratio = 8000;
float rc2pitchPWM(int16_t rc_reading) {
	double output = 0.075f + (double)rc_reading / 36500 * 5;
	if (output > 0.1) output = 0.1;
	if (output < 0.05) output = 0.05;
	return output;
}

/** @brief start given PWM ports
**/
void TIMs_start() {
	/* start specific pin port onboard */
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3); // fraction wheel (PH12)
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2); // fraction wheel (PH11)
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1); // pitch servo (PH10)
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
  for(int i=0; i<4; i++)
  {
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0, \
															4,0.05,0);
    pid_init(&motor_position_pid[i]); // M2006
		motor_position_pid[i].f_param_init(&motor_position_pid[i], PID_Position, 65535, 65535, 0.5,10,5,0, \
															0.5,0.2,0); // M2006
  }
	PWM_SetDuty(&htim5, TIM_CHANNEL_3, pwm_duty);
	PWM_SetDuty(&htim5, TIM_CHANNEL_2, pwm_duty);
	PWM_SetDuty(&htim5, TIM_CHANNEL_1, pitch_pwm_duty);
	HAL_Delay(20); // wait for 20ms
}
/**	@brief Arduino like process of keep looping and you can perform testing.
**/
void _loop_() {
		if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){
			//如果500ms都没有收到遥控器数据，证明遥控器可能已经离线
			for (int i=0; i<4; i++) last_motor_target[i] = motor_target[i];
			set_moto_current(&hcan1, 0, 0, 0, 0);   // loose the motor
			PWM_SetDuty(&htim5, TIM_CHANNEL_3, 0.05f); // stop the fraction wheel
			PWM_SetDuty(&htim5, TIM_CHANNEL_2, 0.05f); // stop the fraction wheel
			HAL_Delay(10);
			return;
    }else{ // 设定各部位的目标值
      // remote_control.ch4*8000; // 摇杆度数
			
			// set motor target
			if (remote_control.switch_right == 1) { // right up
				yaw_speed_target = 32767;
			} else if (remote_control.switch_right == 2) { // right down
				yaw_speed_target = -32767;
			} else {
				yaw_speed_target = 0;
			}
			
			// set fraction wheel target
			if (remote_control.switch_left == 1) { // left up
				pwm_duty = 0.065;
			} else if (remote_control.switch_left == 3) { // left middle
				pwm_duty = 0.05;
			} else if (remote_control.switch_left == 2) { // left down
				pwm_duty = 0.075;
			}
			
			// set pitch target
			pitch_pwm_duty = rc2pitchPWM(remote_control.ch4);
    }
		
		// 执行目标
		PWM_SetDuty(&htim5, TIM_CHANNEL_3, pwm_duty);
		PWM_SetDuty(&htim5, TIM_CHANNEL_2, pwm_duty);
		PWM_SetDuty(&htim5, TIM_CHANNEL_1, pitch_pwm_duty);
		
		// 计算loader的pid
		motor_position_pid[0].target = motor_target[0]; // M2006
		motor_position_pid[0].f_cal_pid(&motor_position_pid[0], moto_chassis[0].total_angle);
		motor_pid[0].target = motor_position_pid[0].output; // M2006																							
		motor_pid[0].f_cal_pid(&motor_pid[0], moto_chassis[0].speed_rpm);
		
		
		
		//将PID的计算结果通过CAN发送到电机
		set_moto_current(&hcan1, motor_pid[0].output,
														yaw_speed_target,
														motor_pid[2].output,
														motor_pid[3].output);
		HAL_Delay(10);      	//PID控制频率100HZ
}
