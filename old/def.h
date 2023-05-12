
//Motor control pin declrations
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62
#define Z_MIN_PIN 18
#define Z_MAX_PIN 19

#define ZservoP 11


#define l1 (long)200//Scara link 1 length
#define l2 (long)140//Scara link 2 length
#define l1_2 (long)l1 * l1 // Required for inverse kinematics
#define l2_2 (long)l2 * l2 // Required for inverse kinematics
#define l12 (long)l1*l2 // Required for inverse kinematics


#define rad2degree 180 / PI //factor to convert radians to degree, required for invese kinematics
#define degree2rad PI / 180 //factor to convert radians to degree, required for invese kinematics

#define STEPS_deg_Y 71.111111111111111111111111111111//Number of steps to take 1 degree for theta 1
#define STEPS_deg_X 71.111111111111111111111111111111//Number of steps to take 1 degree for theta 1
#define STEPS_mm_Z 2560.000//Number of steps to take 1mm along z axis

#define Xmax 155 //max position of theta 1
#define Xmin -155 //min position of theta 1

#define Ymax 142 //max position of theta 2
#define Ymin -142 //min position of theta 2

#define Zmax 100 //max position of theta 2
#define Zmin 0 //min position of theta 2

#define XHomming_steps 50000 //(Xmax-Xmin)*STEPS_deg_Y+400
#define ZHomming_steps 256000.0 //(Xmax-Xmin)*STEPS_deg_Y+400


#define seg_per_mm 1

#define MOTOR_X_RPM 180// Target RPM for X axis motor
#define MOTOR_Y_RPM 180// Target RPM for Y axis motor
#define MOTOR_Z_RPM 1500// Target RPM for Z axis motor

#define homming_rpm_x_y 15
#define homming_rpm_z 200

#define X_transformation 220
#define Y_transformation 0
