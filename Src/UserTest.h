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

/* 4 Motor control parameters. */
PID_TypeDef motor_position_pid[4];
PID_TypeDef motor_speed_pid[4];

float pwm_duty[4];
/** @brief start given PWM ports
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
	// Initialize modor pids
	for (int i = 0; i < 4; i++) {
		pid_init(&motor_speed_pid[i]);
    motor_speed_pid[i].f_param_init(&motor_speed_pid[i],PID_Speed,16384,5000,10,0,8000,0, \
															4,0.05,0);
	}
	HAL_Delay(20); // wait for 20ms
}
/**	@brief Arduino like process of keep looping and you can perform testing.
**/
void _loop_() {
		uint8_t send_buffer[2048];
		if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){
			//如果500ms都没有收到遥控器数据，证明遥控器可能已经离线
			set_moto_current(&hcan1, 0, 0, 0, 0);   // loose the motor
			HAL_Delay(10);
			return;
    }else{ 
			memcpy(send_buffer, UART6_RxBuffer, 2048);
			HAL_UART_Transmit(&huart6, send_buffer, 2048, 100);
    }
		
		// Proceed the operations
		// PWM_SetDuty(&htim5, TIM_CHANNEL_3, pwm_duty[0]);
		for(int i=0; i<4; i++){	
			motor_speed_pid[i].target = 0;																						
			motor_speed_pid[i].f_cal_pid(&motor_speed_pid[i], moto_chassis[i].speed_rpm);
		}
		//将PID的计算结果通过CAN发送到电机
		set_moto_current(&hcan1, motor_speed_pid[0].output,
														motor_speed_pid[1].output,
														motor_speed_pid[2].output,
														motor_speed_pid[3].output);
		HAL_Delay(10);      	//PID控制频率100HZ
}
