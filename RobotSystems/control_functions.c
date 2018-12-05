#include "control_functions.h"
#include "robot_configs.h"
#include "math.h"

/** @brief calculate chassis movement given spinning gimbal parameters
	* @param vx, vy, vw(mm/s): the movement for gimbal
	                           +vy-forward; +vx-leftward; +vw-counterClockwise;
	*        cw       (deg/s): the rotating speed for chassis againest to the ground (counter-clockwise)
	*        theta      (deg): chassis's forward - gimbal's forward
	*	       chassis_target  : a 4-element array align with (vx,vy,vw,dw) which are aimed for 
														 chassis movement and yaw rotation speed.
														 yaw speed is chassis speed related to gimbal.
  */
void spinning_top(float vx, float vy, float vw, float cw, float theta, 
									float *chassis_target) {
  chassis_target[0] = vx*cos(theta) - vy*sin(theta);
	chassis_target[1] = vx*sin(theta) + vy*cos(theta);
	chassis_target[2] = cw;
	chassis_target[3] = cw - vw;
}

/**
  * @brief given the target of chassis movement, calculate the supposed speed for each wheels.
  * @param input : forward=+vy(mm/s)  leftward =+vx(mm/s)  couter-clockwise=+vw(deg/s)
  *        output: every wheel speed(rpm) (It has to be a 4-element array)
  * @note  1=FR 2=BR 3=BL 4=FL
  */
void mecanum_calc(float vx, float vy, float vw, float *speed){
  static float rotate_ratio_fr=((WHEELBASE+WHEELTRACK)/2.0f)/RADIAN_COEF;
  static float rotate_ratio_fl=((WHEELBASE+WHEELTRACK)/2.0f)/RADIAN_COEF;
  static float rotate_ratio_bl=((WHEELBASE+WHEELTRACK)/2.0f)/RADIAN_COEF;
  static float rotate_ratio_br=((WHEELBASE+WHEELTRACK)/2.0f)/RADIAN_COEF;
  static float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);
  
  
  VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
  VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
  
  float wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[2] = ( vx + vy + vw * rotate_ratio_fr) * wheel_rpm_ratio;   //  back- left
  wheel_rpm[3] = ( -vx + vy + vw * rotate_ratio_fl) * wheel_rpm_ratio;	 // forward- left
	// these wheels are reversed due to sysmetry
  wheel_rpm[0] = ( -vx - vy + vw * rotate_ratio_bl) * wheel_rpm_ratio;  // forward right
  wheel_rpm[1] = ( vx - vy + vw * rotate_ratio_br) * wheel_rpm_ratio;		// back -right

  //find max item 
  for (uint8_t i = 0; i < 4; i++)
  {
    if (ABS(wheel_rpm[i]) > max)
      max = ABS(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
	
	// return to speed.
  for (int i = 0; i < 4; i++) speed[i] = wheel_rpm[i];
}
