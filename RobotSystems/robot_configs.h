/***** math relevant ***************************************/

#define RADIAN_COEF 57.3f // radian coefficient 180/PI
#define PI          3.1416f

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define ABS(val) (val > 0 ? val:-val)


/************************ chassis parameter ********************************************************/

#define RADIUS     76  // the radius of wheel(mm)
#define PERIMETER  478 //the perimeter of wheel(mm)

#define WHEELTRACK 740  // wheel track distance(mm)
#define WHEELBASE  550 // wheelbase distance(mm)

#define GIMBAL_X_OFFSET 150 // gimbal is relative to chassis center x axis offset(mm) 
#define GIMBAL_Y_OFFSET 0 /* gimbal is relative to chassis center y axis offset(mm) */

#ifdef CHASSIS_EC60
  /* chassis motor use EC60 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        400   //440rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //415rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300
#else
  /* chassis motor use 3508 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f/19.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 360   //5000rpm
#endif
