#include "def.h"
#include "A4988.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "BasicStepperDriver.h"
#include <gcode.h>
#include <Servo.h>

Servo ZservoM;




#define MOTOR_STEPS_X_Y 200// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS_Z 48


// 1=full step, 2=half step, 4 = 1/4, 8 = 1/8 and 16 = 1/6 Microsteps.
#define MICROSTEPS_XY 16
#define MICROSTEPS_Z 16

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
A4988 stepperX(MOTOR_STEPS_X_Y, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN);
A4988 stepperY(MOTOR_STEPS_X_Y, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN);
A4988 stepperZ(MOTOR_STEPS_Z, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN);

// synchronized move, trajectory is a straight line
SyncDriver controller(stepperX, stepperY, stepperZ);

// G code Commands functions
void homing();
void moviment();
void gotoLocation();
void disable_motor();
void moviment_rela();
void testing();
void ik();
void linear_interp();
void SetSpeed();
void get_cartesian();
void update_cartesian();
void home_ignore();
void get_cartesian_transformed();
void set_servo();

/*
  G28 - HOMING
  G0 - RELATIVE MOVEMENT
  G5 - MOVEMENT IN JOINT AXES
  G99 - DISALE MOTORS
  G8 - TEST CODE RUN
  G1 - INVERSE KINEMATICS BASED POSITIONING
  G6 - SetSpeed
  G1 - linear_interp
  G9 - Ignore homming warning
  M114 get_cartesian_transformed
*/
#define NUMCOMMANDS 12
commandscallback commands[NUMCOMMANDS] = {{"M105", set_servo}, {"M115", get_cartesian_transformed}, {"G9", home_ignore}, {"M114", get_cartesian}, {"G6", SetSpeed}, {"G28", homing}, {"G0", moviment_rela}, {"G5", moviment}, {"G99", disable_motor}, {"G8", testing}, {"G4", ik}, {"G1", linear_interp}};
gcode Commands(NUMCOMMANDS, commands);



bool homming = 0; //Homing status, 0 for not homed and 1 for homed
//defining angles theta1 and theta2 as X and Y
double X;
double Y;
//defining Z co ordinate
double Z = 0;
//defining cartesian x and y co ordinates
double x_mm = 0;
double y_mm = 0;
void setup()
{
  //Set limit switches.//
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  ZservoM.attach(ZservoP);
  ZservoM.write(90);


  Commands.begin(115200);// i=Initiate G code interpreter
//  Serial.print("l1"); Serial.println(l1);
//  Serial.print("l2"); Serial.println(l2);
//  Serial.print("l1_2 "); Serial.println(l1_2);
//  Serial.print("l2_2 "); Serial.println(l2_2);
Serial.println("start");

  //Set target motors RPM and microsteps//
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS_XY);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS_XY);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS_Z);

  //Set speed profile// CONSTANT_SPEED or LINEAR_SPEED
  stepperX.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 500, 500);
  stepperY.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 500, 500);
  stepperZ.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 2000, 2000);

  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);

  Serial.println("start");
}

void loop()
{
  Commands.available();
}
void homing()
{
  // code to home machine
  home_y_axis();
  //Y = -142;
  stepperY.enable();
  controller.move(0, 142 * STEPS_deg_X, 0);
  //gotoLocation(0, 0, 0, 0);
  home_x_axis();
  X = -155;
  Y = 0;
  homming = 1;
  gotoLocation(0, 0, 0, 0);
  //home_z_axis();
  Z = 0;

}
void home_z_axis()
{
  Serial.print("Z homming");
  stepperZ.setRPM(homming_rpm_z);
  stepperZ.enable();
  stepperZ.startMove(-ZHomming_steps);
  uint8_t buttonz = 1;
  while (buttonz)
  {
    stepperZ.nextAction();
    buttonz = digitalRead(Z_MIN_PIN);
  }
  stepperZ.stop();
  stepperZ.setRPM(MOTOR_Z_RPM);
  Serial.println(" completed");
}
void home_x_axis()
{
  Serial.print("X homming");
  stepperX.setRPM(homming_rpm_x_y);
  stepperX.enable();
  stepperX.startMove(-XHomming_steps);
  uint8_t buttonx = 1;
  while (buttonx)
  {
    stepperX.nextAction();
    buttonx = digitalRead(X_MIN_PIN);
  }
  stepperX.stop();
  //stepperX.disable(); // DISABLE MOTOR X
  stepperX.setRPM(MOTOR_X_RPM);
  Serial.println(" completed");
}

void home_y_axis()
{
  Serial.print("Y homming");
  stepperY.setRPM(homming_rpm_x_y);
  stepperY.enable();
  stepperY.startMove(-XHomming_steps);
  uint8_t buttony = 1;
  while (buttony)
  {
    stepperY.nextAction();
    buttony = digitalRead(Y_MIN_PIN);
  }
  stepperY.stop();
  stepperY.disable(); // DISABLE MOTOR X
  stepperY.setRPM(MOTOR_Y_RPM);
  Serial.println(" completed");
}
void moviment_rela()
{
  double newXValue = X;
  double newYValue = Y;
  double newZValue = Z;
  if (Commands.availableValue('X')) // ADDED parameter X in G0
    newXValue = Commands.GetValue('X');
  if (Commands.availableValue('Y')) // ADDED parameter Y in G0
    newYValue = Commands.GetValue('Y');
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
    newZValue = Commands.GetValue('Z');

  gotoLocation(X + newXValue, Y + newYValue, Z + newZValue, 0);
}

void gotoLocation(double x, double y, double z, bool disabling)
{
  if (!homming)
  {
    Serial.println("Complete homming first");
  }
  if (homming)
  {
    if (x > Xmax)
    {
      x = Xmax;
      Serial.println("Xmax hit");
    }
    if (x < Xmin)
    {
      x = Xmin;
      Serial.println("Xmax hit");
    }
    if (y > Ymax)
    {
      y = Ymax;
      Serial.println("Ymax hit");
    }
    if (y < Ymin)
    {
      y = Ymin;
      Serial.println("Ymin hit");
    }

    if (z > Zmax)
    {
      z = Zmax;
      Serial.println("ZZmax hit");
    }
    if (z < Zmin)
    {
      z = Zmin;
      Serial.println("Zmin hit");
    }


    int stepsx = (x - X) * STEPS_deg_X; // DISTANCE VARIATION X
    int stepsy = (y - Y) * STEPS_deg_Y; // DISTANCE VARIATION Y
    long stepsz = (z - Z) * STEPS_mm_Z; // DISTANCE VARIATION Y
    stepperX.enable(); // ENABLE MOTOR X
    stepperY.enable();
    stepperZ.enable();
    controller.move(stepsx, stepsy, stepsz); //SEND CURRENT STEPS FOR DRIVE
    X = x;                              // SET LAST POSITION
    Y = y;
    Z = z;
    if (!disabling)
    {
      stepperX.disable(); // DISABLE MOTOR Y
      stepperY.disable();
      stepperZ.disable();
    }
    //Commands.comment("theta1:" + String(x) + "; theta2:" + String(y) + ";Z:" + String(z)); // DEBUG SERIAL
    update_cartesian();
  }
}

// added paramenter x and y in function MOVIMENT > SET GOTO LOCATION;
void moviment()//G5
{
  double newXValue = X;
  double newYValue = Y;
  double newZValue = Z;
  if (Commands.availableValue('X')) // ADDED parameter X in G0
    newXValue = Commands.GetValue('X');
  if (Commands.availableValue('Y')) // ADDED parameter Y in G0
    newYValue = Commands.GetValue('Y');
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
    newZValue = Commands.GetValue('Z');
  gotoLocation(newXValue, newYValue, newZValue, 0);
}
void disable_motor()
{
  Serial.println("Motor_disabled");
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
}

void testing()//G8
{
  double sig;
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
    sig = Commands.GetValue('Z');
  ZservoM.write((int)sig);
  Serial.print("Servo Set to ");
  Serial.println(sig);
}
void get_cartesian()//m114
{
  if (homming)
  {
    Serial.print("X: ");
    Serial.print(x_mm);
    Serial.print("  Y:");
    Serial.print(y_mm);
    Serial.print("  Z:");
    Serial.println(Z);
  }
  if (!homming)
  {
    Serial.println("Complete homing first");
  }
}

void get_cartesian_transformed()//m115
{
  if (homming)
  {
    Serial.print("X: ");
    Serial.print(x_mm - X_transformation);
    Serial.print("  Y:");
    Serial.print(y_mm);
    Serial.print("  Z:");
    Serial.println(Z);
  }
  if (!homming)
  {
    Serial.println("Complete homing first");
  }
}
void update_cartesian()
{
  x_mm = l1 * cos(X * degree2rad) + l2 * cos((X + Y) * degree2rad);
  y_mm = l1 * sin(X * degree2rad) + l2 * sin((X + Y) * degree2rad);
}


void ik()//G4
{
  double newXValue = x_mm;
  double newYValue = y_mm;
  double newZValue = Z;
  if (Commands.availableValue('X')) // ADDED parameter X in G0
    newXValue = Commands.GetValue('X');
  if (Commands.availableValue('Y')) // ADDED parameter Y in G0
    newYValue = Commands.GetValue('Y');
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
    newZValue = Commands.GetValue('Z');

  double l3 = sqrt(newXValue * newXValue + newYValue * newYValue);
  double cos_alpha = (l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3);
  double theta1 = ((acos(cos_alpha) + atan(newYValue / newXValue)) * 180) / PI;
  double cos_beta = (l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2);
  double beta = (acos(cos_beta) * 180) / PI;
  double theta2 = 180 - beta;
  //  Serial.print("theta1:: ");
  //  Serial.println(theta1);
  //  Serial.print("theta2:: ");
  //  Serial.println(theta2);
  gotoLocation(theta1, -theta2, newZValue, 0);
}

void linear_interp()//g1
{
  double newXValue = x_mm;
  double newYValue = y_mm;
  double newZValue = Z;
  if (Commands.availableValue('X')) // ADDED parameter X in G0
    newXValue = Commands.GetValue('X');
  if (Commands.availableValue('Y')) // ADDED parameter Y in G0
    newYValue = -Commands.GetValue('Y');
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
    newZValue = Commands.GetValue('Z');

  //newXValue=newXValue-X_transformation;
newXValue=newXValue+180;
  double xtravel = newXValue - x_mm;
  double ytravel = newYValue - y_mm;
  double ztravel = newZValue - Z;

  double dist = sqrt(xtravel * xtravel + ytravel * ytravel);
  Serial.print("Dist is");
  Serial.println(dist);
  int steps = seg_per_mm * (int)dist;
  if (steps == 0)
    steps = 1;

  Serial.print("Steps is");
  Serial.println(steps);

  //  double xint;
  //  double yint;

  double xint = (double)x_mm;
  double yint = (double)y_mm;
  double zint = (double)Z;


  //Set speed profile// CONSTANT_SPEED or LINEAR_SPEED
  stepperX.setSpeedProfile(BasicStepperDriver::CONSTANT_SPEED, 500, 500);
  stepperY.setSpeedProfile(BasicStepperDriver::CONSTANT_SPEED, 500, 500);

  float x_old_rpm = stepperX.getRPM();
  float y_old_rpm = stepperY.getRPM();
  stepperX.setRPM(7.0);
  stepperY.setRPM(7.0);

  xtravel = xtravel / steps;
  ytravel = ytravel / steps;
  ztravel = ztravel / steps;
  Serial.print("x_travel:");
  Serial.println(xtravel);
  Serial.print("y_travel:");
  Serial.println(ytravel);
  Serial.print("z_travel:");
  Serial.println(ztravel);

  for (int i = 0; i < steps; i++)
  {
    xint = xint + xtravel;
    yint = yint + ytravel;
    zint = zint + ztravel;
    //      Serial.print("l1 ");Serial.println(l1);
    //      Serial.print("l2 ");Serial.println(l2);
    //      Serial.print("l1_2 ");Serial.println(l1_2);
    //      Serial.print("l2_2 ");Serial.println(l2_2);
    //      Serial.print("l12 ");Serial.println(l12);
    //

    Serial.print("X: ");
    Serial.print(xint);
    Serial.print("Y: ");
    Serial.print(yint);
    double l3 = sqrt(xint * xint + yint * yint);
    //      Serial.print("l3 ");Serial.println(l3);
    //      Serial.print("l1_2 ");Serial.println(l1_2);
    //      Serial.print("l3 x l3");Serial.println(l3*l3);
    //      Serial.print("l2_2 ");Serial.println(l2_2);
    double cos_alpha = (double)(((long)l1_2 + (long)l3 * l3 - (long)l2_2) / (long)(2 * l1 * l3));
    //          Serial.print("cos alpha ");Serial.println(cos_alpha);
    double theta1 = ((acos(cos_alpha) + atan(yint / xint)) * 180) / PI;
    //     Serial.print("theta1 ");Serial.println(theta1);
    double cos_beta = (double)(((long)l1_2 + (long)l2_2 - (long)l3 * l3) / (long)(2 * l12));
    //          Serial.print("cos_beta ");Serial.println(cos_beta);
    double beta = (acos(cos_beta) * 180) / PI;
    //          Serial.print("beta ");Serial.println(beta);
    double theta2 = 180 - beta;
    //               Serial.print("theta2 ");Serial.println(theta2);
    gotoLocation(theta1, -theta2, zint, 1);
    Serial.print(" theta1=");
    Serial.print(theta1);
    Serial.print(" theta2= ");
    Serial.println(theta2);
    stepperX.disable();
    stepperY.disable();
  }

  //Set speed profile// CONSTANT_SPEED or LINEAR_SPEED
  stepperX.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 500, 500);
  stepperY.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 500, 500);

  stepperX.setRPM(x_old_rpm);
  stepperY.setRPM(y_old_rpm);

  Serial.print("X: ");
  Serial.print(xint);
  Serial.print("Y: ");
  Serial.println(yint);
  Serial.println("ok");
}

void get_thetas()
{
  Serial.print("theta1:: ");
  Serial.println(X);
  Serial.print("theta2:: ");
  Serial.println(Y);
}

void SetSpeed()
{
  if (Commands.availableValue('X')) // ADDED parameter X in G0
  {
    stepperX.setRPM((int)(Commands.GetValue('X')));
    Serial.println("theta1 speed set");
  }
  if (Commands.availableValue('Y')) // ADDED parameter Y in G0
  {
    stepperY.setRPM((int)(Commands.GetValue('Y')));
    Serial.println("theta2 speed set");
  }
  if (Commands.availableValue('Z')) // ADDED parameter Z in G0
  {
    stepperZ.setRPM((int)(Commands.GetValue('Z')));
    Serial.println("z speed set");
  }
}

void home_ignore()//G9
{
  homming = 1;
  X = 0;
  Y = 0;
  update_cartesian();
}
void set_servo()//M105
{
//  double newZValue=0;
//  if (Commands.availableValue('Z'))
//    newZValue = Commands.GetValue('Z');
}
