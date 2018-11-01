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


/** @brief start given PWM ports
**/

static float angle_ratio = 815; // Ratio between required angle and the actual output
PID_TypeDef motor_pid[4];
PID_TypeDef motor_position_pid[4];
float motor_speed_target [4]; // set for control motors speed directly.
float motor_target [4]; // ��¼Ŀ��Ƕ�(��)
float last_motor_target [4]; // start at 0
/*	0: loader motor (can id: 1)
		1: yaw motor (can id: 2)
*/
float yaw_pos(int16_t rc_reading) {
	return rc_reading * 1500;
}

// This is defined bwtween 0 ~ 1; defined for fraction wheel
float pwm_duty0 = 0.05f;
float pwm_duty1 = 0.05f;

float pitch_reading_ratio = 8000;
float rc2pitchPWM(int16_t rc_reading) {
	float output = 0.075f + (float)rc_reading / 36500 * 5;
	if (output > 0.1) output = 0.1;
	if (output < 0.05) output = 0.05;
	return output;
}

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
	/*< ��ʼ��PID���� >*/
  for(int i=0; i<4; i++)
  {
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0, \
															4,0.05,0);
    pid_init(&motor_position_pid[i]); // M2006
		motor_position_pid[i].f_param_init(&motor_position_pid[i], PID_Position, 65535, 65535, 0.5,10,5,0, \
															0.5,0.2,0); // M2006
  }
	HAL_Delay(20); // wait for 20ms
}
/**	@brief Arduino like process of keep looping and you can perform testing.
**/
void _loop_() {
		if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){
			//���500ms��û���յ�ң�������ݣ�֤��ң���������Ѿ�����
			for (int i=0; i<4; i++) last_motor_target[i] = motor_target[i];
			set_moto_current(&hcan1, 0, 0, 0, 0);   // loose the motor
			HAL_Delay(10);
			return;
    }else{ // �趨����λ��Ŀ��ֵ
      // remote_control.ch4*8000; // ҡ�˶���
			
			// set chassis movement
			mecanum_calc(remote_control.ch1 * -3,
									remote_control.ch2 * 3,
									remote_control.ch3 / 10,
									motor_speed_target);
			
			// servo poss
			if (remote_control.switch_left == 1) { // left up
				pwm_duty0 = 0.05f;
			} else if (remote_control.switch_left == 2) { // left down
				pwm_duty0 = 0.1f;
			}
			if (remote_control.switch_right == 1) { // right up
				pwm_duty1 = 0.1f;
			} else if (remote_control.switch_right == 2) { // right down
				pwm_duty1 = 0.05f;
			}
    }
		
		// Proceed the operations
		PWM_SetDuty(&htim5, TIM_CHANNEL_3, pwm_duty0);
		PWM_SetDuty(&htim5, TIM_CHANNEL_1, pwm_duty1);
		for(int i=0; i<4; i++){	
			motor_pid[i].target = motor_speed_target[i]; // M2006																							
			motor_pid[i].f_cal_pid(&motor_pid[i], moto_chassis[i].speed_rpm);
		}
		//��PID�ļ�����ͨ��CAN���͵����
		set_moto_current(&hcan1, motor_pid[0].output,
														motor_pid[1].output,
														motor_pid[2].output,
														motor_pid[3].output);
		HAL_Delay(10);      	//PID����Ƶ��100HZ
}
