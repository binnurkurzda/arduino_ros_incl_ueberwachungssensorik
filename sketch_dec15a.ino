
#include <ros.h>
//#include <ros2arduino.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>  //Doku: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html
#include <stdio.h>
//#include "MPU9250.h"



// Range message parameters
#define US_MAX_RANGE 2.0  //[m]
#define US_MIN_RANGE 0.0  //[m]
#define US_FOV 0.1        //[rad]

#define NUMBER_OF_SENSORS 6

//#define IMU_PERIOD 10;

//MPU9250 IMU(SPI, 22);

//function pointer for the interupt service routines
typedef void (*us_isr_cb)(void);

typedef struct{
  uint8_t state;
  uint8_t index;
  unsigned long timeout;  
}range_measurement_t;

typedef struct{
  double temperature;
  unsigned long timeout;
}imu_measurement_t;

typedef struct{
  unsigned long timeout;
}callbacks_t;


int Bat_Spannung_Pin = A5; 
float Bat_Spannung = 0;
int Ladekabel_Pin = 60; //A6
int Ladekabel = 0;
int Schuetz_Schluss_Pin = 61; //A7
int Schuetz_Schluss = 0;


//struct that contains all data that is needed for the readout of the corresponding sensor
typedef struct{
  unsigned long t1;     //timestamp of the rising flank on the echo pin
  unsigned long t2;     //timestamp of the falling flank 
  bool active;          //true if the sensor is currently active
  uint8_t trigger_pin;  //the pin that the trigger ouput is connected to
  uint8_t echo_pin;     //the pin that the echo input is connected to (must be interrupt capable!)
  us_isr_cb isr;        //callback function to the interrupt service function
}ultrasound_sensor_t;

ultrasound_sensor_t sensors[NUMBER_OF_SENSORS]; //one sensor struct for each sensor


ros::NodeHandle  nh;  //initialize ros node handle

sensor_msgs::Range range_msg[NUMBER_OF_SENSORS];  //array of messages; one for each sensor
//sensor_msgs::Imu imu_msg;
//sensor_msgs::Temperature temp_msg;

//Initialize the msg publishers
ros::Publisher pub_range_1( "/ultrasound_1", &range_msg[0]);
ros::Publisher pub_range_2( "/ultrasound_2", &range_msg[1]);
ros::Publisher pub_range_3( "/ultrasound_3", &range_msg[2]);
ros::Publisher pub_range_4( "/ultrasound_4", &range_msg[3]);
ros::Publisher pub_range_5( "/ultrasound_5", &range_msg[4]);
ros::Publisher pub_range_6( "/ultrasound_6", &range_msg[5]);
//ros::Publisher pub_imu("/imu", &imu_msg);
//ros::Publisher pub_temp("/temperature", &temp_msg);

const char *str_buffer[NUMBER_OF_SENSORS];

//initialize the msg header frame_ids
const char sensor_1[] = "ultrasound_1";
const char sensor_2[] = "ultrasound_2";
const char sensor_3[] = "ultrasound_3";
const char sensor_4[] = "ultrasound_4";
const char sensor_5[] = "ultrasound_5";
const char sensor_6[] = "ultrasound_6";
//const char imu_header[] = "imu";
//const char temp_header[] = "temperature";


//function prototypes
void setup_sensors(void);
void setup_messages(void);
void setup_gpio(void);
//void publish_imu_data(void);
void us_1_isr(void);
void us_2_isr(void);
void us_3_isr(void);
void us_4_isr(void);
void us_5_isr(void);
void us_6_isr(void);

//initialize the software


//imu_measurement_t imu_s;
range_measurement_t range_s;
callbacks_t cb_s;


void setup()
{  
  nh.initNode();    //initialize ros node

  setup_messages(); //initialize messages

  setup_sensors();  //initialize sensor structs

  setup_gpio();     //initialite the gpio pins

  setup_structs();
  
  Serial.begin(9600);
}



void loop()
{
  measure_range();

  //measure_acceleration();

  Bat_Spannung = (float) analogRead(Bat_Spannung_Pin);
  Bat_Spannung = (Bat_Spannung/1024)*5;
  Serial.println(Bat_Spannung);

  Ladekabel = digitalRead(Ladekabel_Pin);
  // Wenn Pin HIGH ist, schreibe H
  if (digitalRead(Ladekabel_Pin) == HIGH) {
    Serial.println(("Ladekabel_HIGH"));
    Serial.print("\n");
  }
  // Sonst, schreibe L
  else {
    Serial.println(("Ladekabel_LOW"));
    Serial.print("\n");
  }
  
  Schuetz_Schluss = digitalRead(Schuetz_Schluss_Pin);
  // Wenn Pin HIGH ist, schreibe H
  if (digitalRead(Schuetz_Schluss_Pin) == HIGH) {
    Serial.println(("Schuetz_HIGH"));
    Serial.print("\n");
  }
  // Sonst, schreibe L
  else {
    Serial.println(("Schuetz_LOW"));
    Serial.print("\n");
  }
  
  
  call_callbacks();

//  delay(1000);
}







/***************************************************************************************************************/
/*
 *  void measure_range(void)
 *  
 * is responsible for the asynchronous ultrasound sensor readout
 * 
 */

void measure_range(){

  switch(range_s.state){
     case 0:
      start_measurement(range_s.index); //start the measturement for the current sensor
      range_s.timeout = millis() + 50;  //set timeout
      range_s.state++;                  //go to the next state
     break;

     case 1:
      if(range_s.timeout < millis()){   //wait until the timeout is over
        evaluate_result(range_s.index); //evaluate results in the sensor struct and publish the message
        range_s.state++;                //go to the next state
        range_s.index++;                //select next sensor
      }
      //timeout not over yet -> do nothing
     break;

     case 2:
      if(range_s.index >= NUMBER_OF_SENSORS){ //check if all sensors were read out
        range_s.index = 0;                      //reset counter to the first sensor
      }
      range_s.state = 0;                        //return to the first state
     break;
  }
}

/***************************************************************************************************************/
/*
 *  void measure_acceleration(void)
 *  
 * is responsible for the IMU readout
 * 
 */
/*void measure_acceleration(){
  if(imu_s.timeout < millis()){
    publish_imu_data();
    imu_s.timeout = millis() + IMU_PERIOD;
  }
}*/

/***************************************************************************************************************/
/*
 *  void call_callbacks(void)
 *  
 * just there to call spinOnce()
 * 
 */
void call_callbacks(){

  if(cb_s.timeout < millis()){
    nh.spinOnce();
    cb_s.timeout = millis() + 1000;  
  }
}


void setup_messages(){

  str_buffer[0] = sensor_1;
  str_buffer[1] = sensor_2;
  str_buffer[2] = sensor_3;
  str_buffer[3] = sensor_4;
  str_buffer[4] = sensor_5;
  str_buffer[5] = sensor_6;

  nh.advertise(pub_range_1);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  nh.advertise(pub_range_4);
  nh.advertise(pub_range_5);
  nh.advertise(pub_range_6);

//  nh.advertise(pub_imu);
//  nh.advertise(pub_temp);
  
  for(uint8_t i = 0; i < NUMBER_OF_SENSORS; i++){
    range_msg[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg[i].header.frame_id =  str_buffer[i];
    range_msg[i].field_of_view = US_FOV;         //[rad]     
    range_msg[i].min_range = US_MIN_RANGE;       //[m]
    range_msg[i].max_range = US_MAX_RANGE;       //[m]
  }

//  temp_msg.header.frame_id = temp_header;
//  imu_msg.header.frame_id = imu_header;
}

void setup_sensors(){ //Ultraschall
 sensors[0].active = false;   //set the sensor to inactive
 sensors[0].trigger_pin = 67;  //trigger pin for the first sensor
 sensors[0].echo_pin = 21;     //echo pin for the first sensor
 sensors[0].isr = us_1_isr;   //set the callback for the isr
 
 sensors[1].active = false;
 sensors[1].trigger_pin = 66;
 sensors[1].echo_pin = 20;
 sensors[1].isr = us_2_isr;
 
 sensors[2].active = false;   //set the sensor to inactive
 sensors[2].trigger_pin = 17;  //trigger pin for the first sensor
 sensors[2].echo_pin = 2;     //echo pin for the first sensor
 sensors[2].isr = us_3_isr;   //set the callback for the isr
 
 sensors[3].active = false;
 sensors[3].trigger_pin = 16;
 sensors[3].echo_pin = 3;
 sensors[3].isr = us_4_isr;
  
 sensors[4].active = false;
 sensors[4].trigger_pin = 65; 
 sensors[4].echo_pin = 19;     
 sensors[4].isr = us_5_isr;
  
 sensors[5].active = false;
 sensors[5].trigger_pin = 64;  
 sensors[5].echo_pin = 18;       
 sensors[5].isr = us_6_isr;


 

// IMU.begin();
}


void setup_gpio(){

  for(uint8_t i = 0; i < NUMBER_OF_SENSORS; i++){
    pinMode(sensors[i].trigger_pin, OUTPUT);        //set pinmode
    digitalWrite(sensors[i].trigger_pin, HIGH);     //set pin to high
    pinMode(sensors[i].echo_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(sensors[i].echo_pin), sensors[i].isr, CHANGE);  //attach interrupt to the pin with the given isr callback
  }

  //Batterie-Spannungssensor A5

  //Ladekabel-Sensor A6

  //Schütz-Schlusssensor A7
  
}

void setup_structs(void){
//  imu_s.timeout = 0;
//  imu_s.temperature = 0;

  range_s.timeout = 0;
  range_s.state = 0;
  range_s.index = 0;

  cb_s.timeout = 0;  
}

//a falling flank occured for the given sensor
void isr_falling(uint8_t sensor){
    if(sensors[sensor].active == true){ //check if the sensor is still acitve (if the flank is still valid)
    sensors[sensor].t2 = micros();      //set the timestamp 2
  }
}

//a rising flank occured for the given sensor
void isr_rising(uint8_t sensor){
  if(sensors[sensor].active == true){   //check if the sensor is still active (if the flank is still valid)
    sensors[sensor].t1 = micros();      //set the timestamp 1
  }
}

void start_measurement(uint8_t sensor){
  if(sensor < NUMBER_OF_SENSORS){
    sensors[sensor].active = true;  //set the state in the struct to active. Only the flanks that occure while the active flag is high are valid. 
    sensors[sensor].t1 = 0;         //set the timestamps to 0
    sensors[sensor].t2 = 0;
  
    digitalWrite(sensors[sensor].trigger_pin, LOW); //trigger the sensor
  }
}

void evaluate_result(uint8_t sensor){
  if(sensor < NUMBER_OF_SENSORS){
    sensors[sensor].active = false;     //set the sensor to inactive
    digitalWrite(sensors[sensor].trigger_pin, HIGH);  //set the trigger pin to high for the next measurement
    
    range_msg[sensor].range = -1; //set range to invalid
    
    if((sensors[sensor].t1 != 0) && (sensors[sensor].t1 != 0)){   //check if both flanks occured within the time frame

      float c_air = 0;
      /*if((imu_s.temperature > 0) && (imu_s.temperature < 100)){
        c_air = 331.5+0.6*imu_s.temperature;
      }
      else{
        c_air = 343.5;
      }*/
      
      float t_diff = (float)(sensors[sensor].t2 - sensors[sensor].t1);   //calculate the time difference between the flanks
      if(t_diff > 0){ //check if the time difference is positive
        range_msg[sensor].range = c_air*(0.000001*t_diff)/2; //both flanks occured and time difference is positive -> calculate the distance and set the range to the correct value
      }
      //else: something went wrong during the measurement -> leave range at the invalid value      
    }
    range_msg[sensor].header.stamp = nh.now();  //set current timestamp
    publish_message(sensor);  //publish message
  }
}


void publish_message(uint8_t sensor){ //awful implementation but i dont know how to do it better:

  switch(sensor){ //switch - case statement because i dont know how to do an array of objects
    case 0:
      pub_range_1.publish(&range_msg[0]);
    break;

    case 1:
      pub_range_2.publish(&range_msg[1]);
    break;

    case 2:
      pub_range_3.publish(&range_msg[2]);
    break;

    case 3:
      pub_range_4.publish(&range_msg[3]);
    break;

    case 4:
      pub_range_5.publish(&range_msg[4]);
    break;

    case 5:
      pub_range_6.publish(&range_msg[5]);
    break;

    default:
    break;
  }
}

/*void publish_imu_data(void){
  IMU.readSensor();

  imu_msg.linear_acceleration.x = IMU.getAccelX_mss();
  imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
  imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();

  imu_msg.angular_velocity.x = IMU.getGyroX_rads();
  imu_msg.angular_velocity.y = IMU.getGyroY_rads();
  imu_msg.angular_velocity.z = IMU.getGyroZ_rads();

  imu_s.temperature = IMU.getTemperature_C();
  
  imu_msg.header.stamp = nh.now();  //set current timestamp

  pub_imu.publish(&imu_msg);


  temp_msg.header.stamp = nh.now();
  temp_msg.temperature = imu_s.temperature;

  pub_temp.publish(&temp_msg);

}*/

//interrupt service routine for the first sensor
void us_1_isr(void){                  
  if(digitalRead(sensors[0].echo_pin) == HIGH){ //check if the flank was falling or rising
    isr_rising(0);  //rising flank
  }
  else
    isr_falling(0); //falling flank
}

//the same as the previous isr but for sensor 2
void us_2_isr(void){  
  if(digitalRead(sensors[1].echo_pin) == HIGH){
    isr_rising(1);
  }
  else
    isr_falling(1);
}

//interrupt service routine for the first sensor
void us_3_isr(void){                  
  if(digitalRead(sensors[2].echo_pin) == HIGH){ //check if the flank was falling or rising
    isr_rising(2);  //rising flank
  }
  else
    isr_falling(2); //falling flank
}

//the same as the previous isr but for sensor 2
void us_4_isr(void){  
  if(digitalRead(sensors[3].echo_pin) == HIGH){
    isr_rising(3);
  }
  else
    isr_falling(3);
}

//interrupt service routine for the first sensor
void us_5_isr(void){                  
  if(digitalRead(sensors[4].echo_pin) == HIGH){ //check if the flank was falling or rising
    isr_rising(4);  //rising flank
  }
  else
    isr_falling(4); //falling flank
}

//the same as the previous isr but for sensor 2
void us_6_isr(void){  
  if(digitalRead(sensors[5].echo_pin) == HIGH){
    isr_rising(5);
  }
  else
    isr_falling(5);
}
