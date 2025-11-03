// BOARD: Tools/Board/Arduino AVR boards/Arduino UNO
// PORT: Tools/Port/COM3

#include <MeMCore.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Me7SegmentDisplay.h>
#include <MeGyro.h>

//Me7SegmentDisplay disp(2);

MeDCMotor motorLeft(M1);
MeDCMotor motorRight(M2);

int TIME_TO_STOP = 0; // target time between 50 - 75 seconds, no need to change it here, it is set in hard().
const int DEFAULT_SPEED = 220; // 220; // value from 1 to 255, 162
const int SLOW_SPEED = 150; // value from 1 to 255, 162
const int TURN_SPEED = 200; // value from 1 to 255
const int TURN_SPEED_SLOW = 75; // value from 1 to 255
const int DISTANCE_TO_WALL = 20;  // cm to the wall, + 2 cm obstacle + 9 cm to the tobot center - 6cm for inertia= 25cm.
const int ONE_BOX_MOVING_MS = 1000;  // 1 seconds to move one box (50cm)
const int LEFT_OFFSET = 2; // negative to slow down left, positive to speed up left. so the right/left is balanced.
const float TURN_90_ANGLE = 65.5; // turn this angle for the 90.
const float TURN_90_ANGLE_SLOW = 80; // turn this angle for the 90.
const int FIRST_MOVE_MS = 700; 

MeGyro gyro;
MeUltrasonicSensor ultraSensorFront(3); 
MeUltrasonicSensor ultraSensorLeft(1);  
MeUltrasonicSensor ultraSensorRight(2);

int total_steps = 40;   // How many box moves the robot need to make. 
int steps_taken = 0;
long start_millis = 0;
long machine_startup = 0;
float heading = 0;

void test_gyro() {
  do {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    sleep(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    sleep(1000);                      // wait for a second
  } while(fabs(checkGyro()) > 1.0);
}

void init_robot() {
  sleep(100);  // sleep for 0.1 second.
  steps_taken = 0;
  start_millis = millis();
}

void sleep(unsigned long sleep_ms) {
  unsigned long start = millis();
  while (millis() - start < sleep_ms) {
    checkGyro();
  }
}

void front();
void back();
void frontWall(bool no_pause = false);
void leftWall();
void rightWall();
void leftShortWall();
void rightShortWall();
void leftBottle();
void rightBottle();
void right(); // do not count as a step
void left();  // do not count as a step
void rightSlow(); // do not count as a step
void leftSlow();  // do not count as a step
void backByFrontWall(); // do not count as a step

void hard() {
  sleep(1000); // breathing
  moveForward(ONE_BOX_MOVING_MS / 50 * 36); // first step
  right();
  front(); 
  left();
  leftBottle();
  right(); 
  back(); 
  rightSlow();
  back();
  front();
  left();
  leftShortWall(); 
  left(); 
  leftWall();
  front();
  left(); 
  rightShortWall();
  back(); 
  left();
  back();
}

void leftBottle() {
  bottle(ultraSensorLeft);
}

void rightBottle() {
  bottle(ultraSensorRight);
}

void lastStep() {
  int left_distance = ultraSensorLeft.distanceCm();
  int right_distance = ultraSensorRight.distanceCm();
  if (left_distance < 35) {
    turnLeft();
  }
  else if (right_distance < 35) {
    turnRight();
  }
  else {
    moveBackward(ONE_BOX_MOVING_MS/4); 
  }
  backByFrontWall();
}

void backByFrontWall() {
  int DISTANCE_TO_STOP = 20; // 20 + 2cm wall + 3cm inertia = 25cm
  int front_distance = ultraSensorFront.distanceCm();
  // move backward
  motorLeft.run(DEFAULT_SPEED + LEFT_OFFSET);
  motorRight.run(-DEFAULT_SPEED);
  while (front_distance < DISTANCE_TO_STOP) {
    front_distance = ultraSensorFront.distanceCm();
  }
  stop();
}

void setup()
{
  delay(1000); // sleep 1 second for breathing...
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  gyro.begin();
  test_gyro();
}

void loop()
{
  checkGyro();  // Check gyro to keep it up to date.
  // Wait for the button press.
  int button_pressed = buttonPressed();
  if (button_pressed) {
    hard();
  }
}

void leftWall() {
  const int MOVE_AFTER_SEEN_WALL = 350;  
  sideWall(ultraSensorLeft, MOVE_AFTER_SEEN_WALL);
}

void rightWall() {
  const int MOVE_AFTER_SEEN_WALL = 350;  
  sideWall(ultraSensorRight, MOVE_AFTER_SEEN_WALL);
}

void leftShortWall() {
  const int MOVE_AFTER_SEEN_WALL = 500;  
  sideWall(ultraSensorLeft, MOVE_AFTER_SEEN_WALL);
}

void rightShortWall() {
  const int MOVE_AFTER_SEEN_WALL = 500;  
  sideWall(ultraSensorRight, MOVE_AFTER_SEEN_WALL);
}

void startMotor(int finalSpeed = DEFAULT_SPEED){
    motorLeft.run(-finalSpeed);
    motorRight.run(finalSpeed);
}

void stopMotor(){
  const int STOP_SPEED = 10;  
  for(int speed = DEFAULT_SPEED - 20; speed >= STOP_SPEED; speed = speed - 20) {
    motorLeft.run(speed);
    motorRight.run(-speed);
    sleep(15);
  }
  stop();
}

void bottle(MeUltrasonicSensor& ultraSensor) {
  const int BOTTLE_DISTANCE = 55;
  const int MOVE_AFTER_MS = 30;
  int distance = ultraSensor.distanceCm();
  startMotor();
  while (distance < BOTTLE_DISTANCE) { // skip old walls if any.
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  } ;
  while (distance > BOTTLE_DISTANCE) {
    distance = ultraSensor.distanceCm();
    checkGyro();
  }
  sleep(MOVE_AFTER_MS);
  stopMotor();
  pause();
}

void sideWall(MeUltrasonicSensor& ultraSensor, int move_after_ms) {
  const int MAX_WALL_DISTANCE = 55;
  startMotor();
  int distance = ultraSensor.distanceCm();
  while (distance < MAX_WALL_DISTANCE) { // skip old walls if any.
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  } ;
  while (distance > MAX_WALL_DISTANCE) { // looking for the new wall
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  }
  sleep(move_after_ms);
  stopMotor();
  pause();
}

void test() {
  init_robot();
  sleep(2000);
}

void left(){
  turnLeft();
}

void right(){
  turnRight();
}

void front() {
  moveForward(ONE_BOX_MOVING_MS);
}

void back() {
  moveBackward(ONE_BOX_MOVING_MS);
}

void frontWall(bool no_pause = false) {
  int measured_distance_front = ultraSensorFront.distanceCm();
  startMotor();
  while (measured_distance_front > DISTANCE_TO_WALL) {
    measured_distance_front = ultraSensorFront.distanceCm();
    checkGyro();
  }
  stopMotor();
  if (!no_pause) {
    pause();
  }
}

void turnLeft() {
  turn(TURN_90_ANGLE, 'l', TURN_SPEED);
  adjustPosition();
}

void turnRight() {
  turn(TURN_90_ANGLE, 'r', TURN_SPEED);
  adjustPosition();
}

void stop() {
  motorLeft.stop();
  motorRight.stop();
}

//******* FUNCTION turn left or right for specific angle controlled by a GYRO-sensor ***
void turn(float angle, char rightorleft, uint8_t velocity)
{
  float start_heading = checkGyro();
  float heading = 0;
  while (heading < angle)
  {
    if (rightorleft == 'r')
    {
      motorLeft.run(-velocity); 
      motorRight.run(-velocity);
    }
    else if (rightorleft == 'l')
    {
      motorLeft.run(velocity); 
      motorRight.run(velocity); 
    }
    heading = fabs(checkGyro() - start_heading);
    if (heading > 180.0) {
      heading = 360.0 - heading;  // 179  - -179
    }
  }
  stop();
  sleep(100);
}

void leftSlow() {
  turnSlow(80, 'l', 150);
}

void rightSlow() {
  turnSlow(80, 'r', 150);
}

void turnSlow(float angle, char rightorleft, uint8_t velocity)
{
  float start_heading = checkGyro();
  float heading = 0;
  while (heading < angle)
  {
    if (rightorleft == 'r')
    {
      motorLeft.run(0); 
      motorRight.run(-velocity);
    }
    else if (rightorleft == 'l')
    {
      motorLeft.run(velocity); 
      motorRight.run(0); 
    }
    heading = fabs(checkGyro() - start_heading);
    if (heading > 180.0) {
      heading = 360.0 - heading;  // 179  - -179
    }
  }
  stop();
  sleep(100);
}

void moveForward(int move_millis) {
  startMotor();
  sleep(move_millis - 50);
  stopMotor();
  pause(); // to meet the target time
}

void moveBackward(int move_millis) {
  motorLeft.run(DEFAULT_SPEED + LEFT_OFFSET);
  motorRight.run(-DEFAULT_SPEED);
  sleep(move_millis);
  stopMotor();
  // STOP to meet the target time 
  pause();
}

void pause() {
  ++steps_taken;
  int remain_steps = total_steps - steps_taken;
  if (remain_steps < 1) {
    remain_steps = 1;
  }
  long millis_remain = TIME_TO_STOP * 1000L - (millis() - start_millis);
  long each_step_millis = millis_remain / remain_steps;
  long wait_time = each_step_millis - ONE_BOX_MOVING_MS * 1.6;
  if (wait_time > 2500) {
    sleep(2500);
  } else if (wait_time > 75) {
    sleep(wait_time);
  } else {
    sleep(100); // stop for breathing.
  }
  adjustPosition();
}

bool buttonPressed() { //hard mode, 3 bonus scores
  return analogRead(A7) <= 10;
}

void display(float value) {
  //disp.display(value);
}

bool display_heading = true;

float checkGyro() {
  gyro.update();
  float heading = gyro.getAngleZ();
  if (display_heading) {
    display(heading);
  }
  return heading;
}

void adjust_by_wall() {
  int left_distance = ultraSensorLeft.distanceCm();
  int right_distance = ultraSensorRight.distanceCm();
  if (left_distance < 10) { // robot width: 14 : ideal width 25 - 7 = 18 
    turnLeft();
    backByFrontWall();
    turnRight();
  } else if (left_distance > 40 && left_distance < 50) {
    turnLeft();
    frontWall(true);
    turnRight();
  } else if (right_distance < 10) {
    turnRight();
    backByFrontWall();
    turnLeft();
  } else if (right_distance > 40 && right_distance < 50) {
    turnRight();
    frontWall(true);
    turnLeft();
  }
}

void adjustPosition() {
  adjust_by_wall();
  float heading_float = checkGyro();
  int heading = ((int)round(checkGyro()) + 360) % 90;
  display(heading_float);
  display_heading = false;
  display_heading = true;
  if (heading >= 4 && heading <= 20) {
    turn(max(1.0, heading - 8), 'l', TURN_SPEED);
  }  
  if (heading >= 70 && heading <= 86) {
    turn(max(1.0, 90- heading - 6), 'r', TURN_SPEED);
  }
}
