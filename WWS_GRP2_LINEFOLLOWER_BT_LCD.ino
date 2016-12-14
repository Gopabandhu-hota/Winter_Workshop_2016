/*******BNO LIBRARIES*************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/****************************/
/***************BLUETOOTH*************/
#include <SoftwareSerial.h>
SoftwareSerial BT(2,3);
/********************************/
float time1 = 0,time2 = 0;
#define angular_velocity  48.80
#define radius_of_wheel 3.50
#define pi 3.1415
volatile float x = 0, y = 0;
int count_BNO = 0;
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  BT.println("------------------------------------");
  BT.print  ("Sensor:       "); BT.println(sensor.name);
  BT.print  ("Driver Ver:   "); BT.println(sensor.version);
  BT.print  ("Unique ID:    "); BT.println(sensor.sensor_id);
  BT.print  ("Max Value:    "); BT.print(sensor.max_value); BT.println(" xxx");
  BT.print  ("Min Value:    "); BT.print(sensor.min_value); BT.println(" xxx");
  BT.print  ("Resolution:   "); BT.print(sensor.resolution); BT.println(" xxx");
  BT.println("------------------------------------");
  BT.println("");
  delay(500);
}

void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the BT Monitor */
  BT.println("");
  BT.print("System Status: 0x");
  BT.println(system_status, HEX);
  BT.print("Self Test:     0x");
  BT.println(self_test_results, HEX);
  BT.print("System Error:  0x");
  BT.println(system_error, HEX);
  BT.println("");
  delay(500);
}

/************ DEFINING LDR INPUT PINS***************/
int LDR_left[8];

void LDR_ARRAY_INIT() {
  int i,j = 0;
  for (i = 38; i <= 52 ; i = i+2)
  {
    LDR_left[j] = i;
  pinMode(LDR_left[j], INPUT);      // set pin to input
  digitalWrite(LDR_left[j], HIGH);       // turn on pullup resistors
  j++;
  }
  
}
/************************/

/********* DEFINING L293D PINS ****************/
const int enable_left_motor = 4;
const int enable_right_motor = 5;
const int left_motor_forward = 11;
const int left_motor_backward = 10;
const int right_motor_forward = 8;
const int right_motor_backward = 9;

void L293D_INIT(){
  pinMode(enable_left_motor, OUTPUT);      // set pin to output
  pinMode(enable_right_motor, OUTPUT);      // set pin to output
  pinMode(left_motor_forward, OUTPUT);      // set pin to output
  pinMode(left_motor_backward, OUTPUT);      // set pin to output
  pinMode(right_motor_forward, OUTPUT);      // set pin to output
  pinMode(right_motor_backward, OUTPUT);      // set pin to output
  digitalWrite(enable_left_motor, HIGH);      //enabling left motor
  digitalWrite(enable_right_motor, HIGH);      //enabling right motor  
}

void go_forward(){
  analogWrite(left_motor_forward, 150);
  analogWrite(right_motor_forward, 150);
  analogWrite(left_motor_backward, 0);
  analogWrite(right_motor_backward, 0);
}

void go_right(){
  analogWrite(left_motor_forward, 150);
  analogWrite(right_motor_forward, 0);
  analogWrite(left_motor_backward, 0);
  analogWrite(right_motor_backward, 100);
}

void go_left(){
  analogWrite(left_motor_forward, 0);
  analogWrite(right_motor_forward, 150);
  analogWrite(left_motor_backward, 100);
  analogWrite(right_motor_backward, 0);
}

void stop_motor(){
  analogWrite(left_motor_forward, 0);
  analogWrite(right_motor_forward, 0);
  analogWrite(left_motor_backward, 0);
  analogWrite(right_motor_backward, 0);
}
/**************************/

/*********TURNING DEVICE ON*********/

/*******************************/

void setup() {
  // put your setup code here, to run once:
  LDR_ARRAY_INIT();
  L293D_INIT();
  BT.begin(9600);
    BT.begin(9600);
  BT.println("Orientation Sensor Test"); BT.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    BT.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();
  pinMode (13,INPUT);
  bno.setExtCrystalUse(true);
  }

void loop() {
 // while(digitalRead(13)!= HIGH); //waits till pin13 becomes high 
  int d = 1,e,f;
  while(d == 1){
  // put your main code here, to run repeatedly:
  int LDR_STATE[8];
  int i;
  for (i = 0; i < 8; i++)
  {
    LDR_STATE[i] = digitalRead(LDR_left[i]); 
  }
/***RUNNING FORWARD***/
  if ((LDR_STATE[0] == HIGH)&&(LDR_STATE[1] == HIGH)&&(LDR_STATE[2] == HIGH)&&(LDR_STATE[3] == HIGH)&&(LDR_STATE[4] == HIGH)&&(LDR_STATE[5] == HIGH)&&(LDR_STATE[6] == HIGH)&&(LDR_STATE[7] == HIGH)){
    stop_motor();
  }
  else if ((LDR_STATE[0] == HIGH)&&(LDR_STATE[1] == HIGH)&&(LDR_STATE[2] == HIGH)&&(LDR_STATE[3] == LOW)&&(LDR_STATE[4] == LOW)&&(LDR_STATE[5] == LOW)&&(LDR_STATE[6] == HIGH)&&(LDR_STATE[7] == HIGH)){
    go_forward();
  }
  else if ((LDR_STATE[0] == HIGH)&&(LDR_STATE[1] == HIGH)&&(LDR_STATE[2] == LOW)&&(LDR_STATE[3] == LOW)&&(LDR_STATE[4] == LOW)&&(LDR_STATE[5] == HIGH)&&(LDR_STATE[6] == HIGH)&&(LDR_STATE[7] == HIGH)){
    go_forward();
  }
  else if ((LDR_STATE[7] == HIGH)&&((LDR_STATE[6] == LOW)||(LDR_STATE[5] == LOW))&&(LDR_STATE[0] == HIGH)){
        go_right();
      }
  else if ((LDR_STATE[7] == HIGH)&&((LDR_STATE[1] == LOW)||(LDR_STATE[2] == LOW))&&(LDR_STATE[0] == HIGH)){
        go_left();
      }
  else if ((LDR_STATE[0] == HIGH)&&(LDR_STATE[1] == HIGH)&&(LDR_STATE[6] == HIGH)&&(LDR_STATE[7] == HIGH)){
    if ((LDR_STATE[3] == LOW)||(LDR_STATE[4] == LOW)){
      go_forward();
    }
    }
  else if (LDR_STATE[7] == HIGH){
    if (LDR_STATE[2] == LOW){
      if ((LDR_STATE[6] == HIGH)&&(LDR_STATE[5] == HIGH)&&(LDR_STATE[4] == HIGH)){
      go_left();
      }
      else if ((LDR_STATE[1] == LOW)&&(LDR_STATE[6] == HIGH)){
        go_left();
      }
      }
      else if ((LDR_STATE[0] == LOW)){
        go_left();
      }
      else if ((LDR_STATE[0] == HIGH)&&(LDR_STATE[1] == LOW) &&(LDR_STATE[2] == HIGH)){
        go_left();
      }
  }
  else if (LDR_STATE[0] == HIGH){
    if (LDR_STATE[5] == LOW){
      if ((LDR_STATE[1] == HIGH)&&(LDR_STATE[2] == HIGH)&&(LDR_STATE[3] == HIGH)){
      go_right();
      }
      else if ((LDR_STATE[6] == LOW)&&(LDR_STATE[1] == HIGH)){
        go_right();
      }
      }
      else if (LDR_STATE[7] == LOW){
        go_right();
      }
      
  }
  else if ((LDR_STATE[7] == HIGH)&&(LDR_STATE[6] == LOW) &&(LDR_STATE[5] == HIGH)&&(LDR_STATE[0] == HIGH)){
        go_right();
      }
  if ((millis() - time1) > BNO055_SAMPLERATE_DELAY_MS){
    displayCalStatus();
    time2 = millis() - time1;
    time1 = millis();
  sensors_event_t event;
  bno.getEvent(&event);
  static float a = event.orientation.x,b = event.orientation.x,c = event.orientation.x;
  BT.print(event.orientation.x, 4);
  b = event.orientation.x;
  if (e < 5){
  if ((event.orientation.x - a) > 20.00) {
    e++;
    if (e == 5){
      f = 1;
    }
  }
  else if ((a - event.orientation.x) > 20.00){
    e++;
    if (e == 5){
      f = 2;
    }
  }
  }
  if (f == 1){
    if((c - event.orientation.x) > 1.00);
  }
  double sine = sin((PI*b)/180.00);
  double cosine = cos((PI*b)/180.00); 
  if(count_BNO >0)
  {
  x = x + (time2*angular_velocity*PI*2.0*radius_of_wheel*sine)/60000.00;
  y = y + (time2*angular_velocity*PI*2.0*radius_of_wheel*cosine)/60000.00;
  }
  count_BNO++;
  BT.print(",");
  //BT.print("X:");
  BT.print(x);
  BT.print(",");
  //BT.print("Y:");
  BT.print(y);
  BT.print(",");
  BT.println(time2);
  }
  }
}
