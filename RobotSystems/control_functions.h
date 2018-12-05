#ifndef CONTROL_H
#define CONTROL_H

#include "stdlib.h"
#include "stdint.h"
#include "string.h"

/** @brief calculate chassis movement given spinning gimbal parameters
	* @param vx, vy, vw(m/s): the movement for gimbal
	*        cw      (deg/s): the rotating speed for chassis againest to the ground
	*        theta     (deg): value of the angle between chassis's forward and gimbal's forward
	*	       chassis_target : a 4-element array align with (vx,vy,vw,dw) which are aimed for 
														chassis movement and yaw rotation speed.
  */
void spinning_top(float vx, float vy, float vw, float cw, float theta, float *chassis_target);

/**
  * @brief given the target of chassis movement, calculate the supposed speed for each wheels.
  * @param input : forward=+vy(mm/s)  leftward =+vx(mm/s)  couter-clockwise=+vw(deg/s)
  *        output: every wheel speed(rpm) (It has to be a 4-element array)
  * @note  1=FR 2=BR 3=BL 4=FL
  */
void mecanum_calc(float vx, float vy, float vw, float *speed);

#endif
