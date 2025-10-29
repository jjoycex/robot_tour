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
const int DEFAULT_SPEED = 220; // value from 1 to 255, 162
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
    delay(1000);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(1000);                      // wait for a second
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

void frontWall();
void rightWall();
void leftWall();
void leftShortWall();
void rightShortWall();
void enterGate();
void bottleLeft();
void bottleRight();

void hard() {
  init_robot();

  TIME_TO_STOP = 55; // target time between 55 - 80 seconds
  total_steps = 34;   // How many box moves the robot need to make. 

  moveForward(ONE_BOX_MOVING_MS/50*36);  // move to the center of the first box.
  right(); front();
  left(); bottleLeft();
  right(); back();
  turnRightSlow(); back();
  front();
  left(); front();
  left(); leftWall();
  bottleLeft();
  right(); back();
  turnRightSlow(); back();
  frontWall();
}

void hard1() {
  init_robot();

  TIME_TO_STOP = 55; // target time between 55 - 80 seconds
  total_steps = 34;   // How many box moves the robot need to make. 

  moveForward(ONE_BOX_MOVING_MS/50*36);  // move to the center of the first box.
  right(); front();
  left(); front();
  leftWall();
  front();
  leftWall();
  back();
  turnLeftSlow(); rightShortWall();
  rightShortWall();
  right(); leftShortWall();
  left(); leftWall();
  back();
  turnLeftSlow(); frontWall();
  right(); rightWall();
  left(); leftShortWall();
  frontWall();
  left(); frontWall();
  right(); front();
  right(); rightWall();
  back();
  turnRightSlow(); front();
  frontWall();
  left(); front();
  right(); frontWall();
  right(); rightWall();
  leftShortWall();
  leftShortWall();
  right(); rightShortWall();
  front();
  right(); frontWall();
  right(); frontWall();
  back();
  turnRightSlow(); leftShortWall();
  left(); leftWall();
  front();
  left(); rightShortWall();
  right(); leftWall();
  lastStep();
}

void lastStep() {
  int DISTANCE_TO_STOP = 20; // 20 + 2cm wall + 3cm inertia = 25cm
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
  const int START_SPEED = 120;  
  for(int speed = START_SPEED; speed <= finalSpeed; speed = speed + 10) {
    motorLeft.run(-speed);
    motorRight.run(speed);
    sleep(10);
  }
}

void stopMotor(){
  const int STOP_SPEED = 10;  
  for(int speed = DEFAULT_SPEED - 20; speed >= STOP_SPEED; speed = speed - 20) {
    motorLeft.run(speed);
    motorRight.run(-speed);
    sleep(10);
  }
  stop();
}

void sideWall(MeUltrasonicSensor& ultraSensor, int move_after_ms) {
  const int MAX_WALL_DISTANCE = 55;
  startMotor();
 
  int distance = ultraSensor.distanceCm();
  while (distance > MAX_WALL_DISTANCE) {
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  } ;

  sleep(move_after_ms);
  stop();
  pause();
}

void bottleLeft() {
  bottle(ultraSensorLeft);
  if (distance < 10) {
    turnLeft();
    moveBackward(100);  // REMOVE THE PAUSE
    turnRight();    
  }
  pause();
}

void bottleRight() {
  int distance = bottle(ultraSensorRight);
  if (distance < 10) {
    turnRight();
    moveBackward(100);   // REMOVE THE PAUSE
    turnLeft();
  }
  pause();
}

int bottle(MeUltrasonicSensor& ultraSensor) {
  const int MAX_WALL_DISTANCE = 55;
  startMotor(120);
 
  int distance = ultraSensor.distanceCm();
  while (distance < MAX_WALL_DISTANCE) {
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  }

  while (distance > MAX_WALL_DISTANCE) {
    distance = ultraSensor.distanceCm();
    checkGyro();  // keep gyro up to date
  }
  sleep(200);
  stop();
  return distance;
}


void enterGate() {
  startMotor();
  sleep(ONE_BOX_MOVING_MS/2);
  stop();
  sleep(150); // breath
  // move backward
  moveBackward(ONE_BOX_MOVING_MS/2 + 100);
}

void test() {
  init_robot();
  enterGate();
  sleep(2000);
  enterGate();
  sleep(2000);
  enterGate();
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

void frontWall() {
  int measured_distance_front = ultraSensorFront.distanceCm();
  if (measured_distance_front > 95) { // safety guard, not seeing any wall
    front();
    return;
  }
  startMotor();
  while (measured_distance_front > DISTANCE_TO_WALL) {
    measured_distance_front = ultraSensorFront.distanceCm();
    checkGyro();
  }
  stop();
  pause();
}

void turnLeft() {
  turn(TURN_90_ANGLE, 'l', TURN_SPEED);
  adjustPosition();
}

void turnRight() {
  turn(TURN_90_ANGLE, 'r', TURN_SPEED);
  adjustPosition();
}

void turnLeftSlow() {
  turn(TURN_90_ANGLE_SLOW, 'l', TURN_SPEED_SLOW);
  adjustPosition();
}

void turnRightSlow() {
  turn(TURN_90_ANGLE_SLOW, 'r', TURN_SPEED_SLOW);
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

void moveForward(int move_millis) {
  startMotor();
  sleep(move_millis - 50);
  stop();
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

void adjustPosition() {
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
