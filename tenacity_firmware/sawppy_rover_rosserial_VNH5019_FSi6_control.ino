#include <IBusBM.h>
#include <stdlib.h>
#include <SPI.h>
#include "Adafruit_VL53L0X.h"

//motor control for Tenacity using the VNH5019 with an FS-16X RC Controller 
//Using the VNH5019 Motor Shield Library and IBus Library
//Control flow gets flipped around 
//Motor control goes right from FS-16 into VNH5019, skipping /joy & rosserial
//Publish steering data into /cmd_vel and teleop_corner_twist.py will take care of the rest

//The VL53L0X sensors are used for cliff detection

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>

#include "DualVNH5019MotorShield.h"

#define encoder_1_A 38    
#define encoder_1_B 40
    
#define encoder_2_A 42
#define encoder_2_B 44
    
#define encoder_4_A 25 //21
#define encoder_4_B 24 //20
    
#define encoder_3_A 23 //19
#define encoder_3_B 22 //18

#define TELEOP_STATUS 52
    
#define SPEED_MIN 50
#define SPEED_CRUISE 150
#define SPEED_FAST 250
#define SPEED_MAX 400

// define SHUT pins for VLX sensors
#define SHT_LOX_FRONT 6
#define SHT_LOX_REAR 7

//define I2C addresses for the VLX sensors
#define LOX_FRONT_ADDRESS 0x30
#define LOX_REAR_ADDRESS 0x31

/* 
 *   Motor labeling/numbering is as follows:
 *      FRONT
 *  [1][A] [2][B]
 *    \\   //
 *[3][C]== ==[4][D]
 *    //   \\
 *[5][E]     [6][F]                                              
 *      REAR
 */

    int rotDirection = 0;
    int pressed = false;

    float twist_linear=0.0;
    float twist_angular=0.0;
    
// variables will change:
    int buttonState = 0;         // variable for reading the pushbutton status
    int buttonPos = 0;
    
    boolean in_place = false;
    int tip_dir = 0;
    unsigned long e3a_ts;
    unsigned long e3b_ts;
    unsigned long e4a_ts;
    unsigned long e4b_ts;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::NodeHandle nh;

//std_msgs::String tip_state_msg;
char tip_state_str[9]="inactive";
char fsi_msg_str[80];
char vlx_msg_str[80];

std_msgs::Int32 left_ticks_out;
std_msgs::Int32 right_ticks_out;
std_msgs::String fsi_msg;
std_msgs::Float64 pan_cmd_msg;
std_msgs::Float64 tilt_cmd_msg;

const geometry_msgs::Twist vel_msg;

sensor_msgs::Range vlx_front_msg;
sensor_msgs::Range vlx_rear_msg;

float cliff_max = 0.60;

volatile long left_ticks=0;
volatile long right_ticks=0;

DualVNH5019MotorShield md;

// objects for the vl53l0x
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_rear = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure_front;
VL53L0X_RangingMeasurementData_t measure_rear;


IBusBM IBus; // IBus object for receivig signals from transmitter/receiver

//ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", TwistCb); 
ros::Publisher drive_pub("cmd_vel",&vel_msg);
ros::Publisher fsi_pub("flysky",&fsi_msg);

//Control happens inside the iBus node now and output goes right to the motors, so there is "no Joy"
//ros::Subscriber<sensor_msgs::Joy> sub_joy("joy", JoyModeCb);

//ros::Publisher left_enc_pub("left_ticks",&left_ticks_out);
//ros::Publisher right_enc_pub("right_ticks",&right_ticks_out);
ros::Publisher tilt_pub("mastcam_tilt_controller/command",&tilt_cmd_msg);
ros::Publisher pan_pub("mastcam_pan_controller/command",&pan_cmd_msg);
ros::Publisher vlx_front_pub("vlx/front_cliff",&vlx_front_msg);
ros::Publisher vlx_rear_pub("vlx/rear_cliff",&vlx_rear_msg);

void left_enc_Cb() {
   
  e3a_ts = micros();

  if (digitalRead(encoder_3_A) && !digitalRead(encoder_3_B)) {
      left_ticks++;
  } 

  if (digitalRead(encoder_3_A) && digitalRead(encoder_3_B)){ 
      left_ticks--;
  }
  
}

void right_enc_Cb() {

  e4a_ts = micros();
  if (digitalRead(encoder_4_A) && !digitalRead(encoder_4_B)) {
      right_ticks++;
  } 

  if (digitalRead(encoder_4_A) && digitalRead(encoder_4_B)){ 
      right_ticks--;
  }

}

void vlx_remap_IDs() {
  Serial.println("Reached setID");
  // all reset
  digitalWrite(SHT_LOX_FRONT, LOW);    
  digitalWrite(SHT_LOX_REAR, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX_FRONT, HIGH);
  digitalWrite(SHT_LOX_REAR, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX_FRONT, HIGH);
  digitalWrite(SHT_LOX_REAR, LOW);
 
  pinMode(SHT_LOX_FRONT,INPUT);
  // initing LOX1
  if(!lox_front.begin(LOX_FRONT_ADDRESS,true)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);
  Serial.println(F("Started first VL53L0X"));
  delay(100);
  
  // activating LOX2
  digitalWrite(SHT_LOX_REAR, HIGH);
  delay(10);
  
  pinMode(SHT_LOX_REAR,INPUT);
  //initing LOX2
  if(!lox_rear.begin(LOX_REAR_ADDRESS,true)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
    Serial.println(F("Started second VL53L0X"));
    delay(100);
}

void setup() {

// Set up encoder interrupts

   pinMode(encoder_3_A,INPUT_PULLUP);
   pinMode(encoder_3_B,INPUT_PULLUP);
 
   pinMode(encoder_4_A,INPUT_PULLUP);
   pinMode(encoder_4_B,INPUT_PULLUP);

   attachInterrupt(digitalPinToInterrupt(encoder_3_A),left_enc_Cb,RISING);
   attachInterrupt(digitalPinToInterrupt(encoder_4_A),right_enc_Cb,RISING);

   pinMode(TELEOP_STATUS,OUTPUT);

   digitalWrite(TELEOP_STATUS,LOW);

   //VLX sensors come with the same I2C address by default, so we have to remap those. 
   vlx_remap_IDs();

  //Fill out the .data fields of the VLX Range msgs
   
  vlx_front_msg.radiation_type=sensor_msgs::Range::INFRARED;
  vlx_front_msg.field_of_view = 0.44; //25 degrees
  vlx_front_msg.min_range = 0.03;
  vlx_front_msg.max_range = 8.20;

  vlx_rear_msg.radiation_type=sensor_msgs::Range::INFRARED;
  vlx_rear_msg.field_of_view = 0.44; //25 degrees
  vlx_rear_msg.min_range = 0.03;
  vlx_rear_msg.max_range = 8.20;

  //init the ROS nodes
  
    nh.initNode();    
//    nh.advertise(left_enc_pub);
//    nh.advertise(right_enc_pub);
    nh.advertise(drive_pub);
    nh.advertise(fsi_pub);
    nh.advertise(tilt_pub);
    nh.advertise(pan_pub);
    nh.advertise(vlx_front_pub);
    nh.advertise(vlx_rear_pub);
    md.init(); 

      Serial.begin(115200);
    //debug serial 
      Serial3.begin(115200);
    //Serial.begin(57600);
    // Init iBus 
    IBus.begin(Serial2);    // iBUS connected to Serial2
                            // but only uses pin 17(RX)
     
    geometry_msgs::Twist start_tw;
      start_tw.linear.x= 0.0;
      start_tw.linear.y=0.0;
      start_tw.linear.z=0.0;
      
      start_tw.angular.z= 0.0;
      start_tw.angular.x= 0.0;
      start_tw.angular.y= 0.0;
 }
 
 //We make direct calls to manipulate the motors for speed
 //as function calls can be expensive time-wise

 void loop() {

     int drive_cmd=1024;
     float drive_speed=0.0;
     float drive_angle=0.0;
     int drive_dir=0;
     int dirbtn=0;
//int rcvals={0,0,0,0,0,0,0,0,0,0};
     int rcval_safety=0;
     int rcval_z=0;
     int rcval_x=0;
     int rcval_pan=0;
     int rcval_tilt=0;
     int rcval_autonomy=0;
     
     float linear_x=0.0;
     float angular_z=0.0;
     float mastcam_tilt=0.0;
     float mastcam_pan=0.0;

     float vlx_front_range=0.0;
     float vlx_rear_range=0.0;
     
     int tip_dir=0;
     int tip_state=0;
           
   //  long leftTick,rightTick;

    //  drive_speed = twist_linear * SPEED_FAST;
    //Channels:
    //Channels on the controller are 1-based 
    //   Channel 0 - Right Stick lateral controls yaw
    //   Channel 2 - Left Stick Up/Down controls X(forward) velocity
    //   Channel 4 - Safety Switch. Motors are disabled in the up position, enabled with it down. 
    //   Channel 5 - Turn-in-place toggle. Up TIP mode is disabled, down enabled. 
    //   Channel 8 - mastcam pan
    //   Channel 9 - mastcam tilt
    //   Channel 6 - Fault Toggle. run - middle up - ACK down - do clearing action 
    //   Channel 7 - Autonomy Mode  

      rcval_z = readChannel(0,-60,60,0);
      rcval_x = readChannel(2,-100,100,0);
      rcval_safety=readSwitch(4,false); //SWA
      tip_state = readSwitch(5,false);  //SWB
      
    if(rcval_safety == 1) { 
       digitalWrite(TELEOP_STATUS,HIGH);      
    } else {
       digitalWrite(TELEOP_STATUS,LOW);
    }

    //mastcam channels
      rcval_pan = readChannel(8,-100,100,0);  //VRA dial
      rcval_tilt = readChannel(9,-100,100,0); //VRB dial 
      rcval_autonomy = readSwitch(7,false); //SWD

    // read VLX for cliff state
    VL53L0X_RangingMeasurementData_t measure_front;
    VL53L0X_RangingMeasurementData_t measure_rear;
   
    lox_front.rangingTest(&measure_front, false); 
    lox_rear.rangingTest(&measure_rear,false);
      
    if (measure_front.RangeStatus != 4) {  // phase failures have incorrect data
       vlx_front_range = measure_front.RangeMilliMeter;
       vlx_front_msg.range = measure_front.RangeMilliMeter/1000.0f;
    }

    if (measure_rear.RangeStatus !=4) { 
       vlx_rear_range = measure_rear.RangeMilliMeter;
       vlx_rear_msg.range = measure_rear.RangeMilliMeter/1000.0f;
    }
    
    //Publish cliff sensor data constantly 
    sprintf(vlx_msg_str,"vlx_front:%f vlx_rear:%f",vlx_front_msg.range,vlx_rear_msg.range);
    vlx_front_pub.publish(&vlx_front_msg);
    vlx_rear_pub.publish(&vlx_rear_msg);

   //Drive Speed is a direct map between IBUS controller values and VNH Controller values
   //Not relying on /joy value translation
      
   drive_speed = map(rcval_x,-100,100,-250,250);   
 
   linear_x = rcval_x * 0.01;
   angular_z = rcval_z * 0.01;

   mastcam_pan = rcval_pan *0.01;
   mastcam_tilt = rcval_tilt *0.01;
   sprintf(fsi_msg_str,"%d %d %d %d %d",rcval_x,rcval_z,tip_state,rcval_tilt,rcval_safety);
   Serial3.print("FSI:");
   Serial3.println(fsi_msg_str);
   Serial3.print("VLX:");
   Serial3.println(vlx_msg_str);

   
  // Serial3.println("VLX:%s",vlx_msg_str);
   
   if (tip_state == 1) { 
       in_place = true;
       tip_dir = readSwitch(7,false);
       
   } else {
       in_place = false;
   }
   //Check to see if Drive Safety has been engaged
   //Flipping this up should cut the motors immediately. 
   if (rcval_safety == 1) {
     if (vlx_front_msg.range < cliff_max) { 
      md.setM1Speed(drive_speed);
      md.setM2Speed(drive_speed);
     } else {
       linear_x = 0.00;
       //angular_z = 0.00;
       md.setM1Speed(0);
       md.setM2Speed(0);
       
       //Continue to read data off the iBus and dump it on the floor while the cliff sensor is triggered.
       readChannel(0,-60,60,0);
       readChannel(2,-100,100,0);
     }
  
//Check for turn-in-place flag 
       if (drive_speed == 0) {
            if  (in_place) { 
              if (tip_dir == -1) { 
                //TIP to the left
                    //Left Side goes in reverse
                    md.setM1Speed(SPEED_CRUISE*-1);
      
                    //Right Side goes forward
                    md.setM2Speed(SPEED_CRUISE);
                }

                if (tip_dir == 1) { 
                //TIP to the right
                    //Left Side goes forward
                    md.setM1Speed(SPEED_CRUISE);
      
                    //Right Side goes in reverse
                    md.setM2Speed(SPEED_CRUISE*-1);
                }
            }

           //If horizontal and angular twist are 0, stop the motors.
           //We send a 0 to the motor controller to zero-out any residual PWM
           if ( (rcval_z == 0) && (!in_place)) {
            md.setM1Speed(0);
            md.setM2Speed(0);
           }
            delay(10);
            //See if we can live without this
            //Spin should happen at the bottom of the control loop
            //nh.spinOnce();
       }
   } else {
           linear_x=0.00;
           drive_speed=0.00;       
   }
     //make sure we publish motor topics outside of rcval check so we don't starve subscribers
     //if we're in a safety condition, we should zero out the Twist message vs. not publishing
  /*   
       left_ticks_out.data=left_ticks;
       right_ticks_out.data=right_ticks;
       
       left_enc_pub.publish(&left_ticks_out);
       right_enc_pub.publish(&right_ticks_out);

   */

      // cmd_vel publish
      geometry_msgs::Twist tw;
      
      tw.linear.x= linear_x;
      tw.linear.y=0.0;
      tw.linear.z=0.0;
      
      tw.angular.z= angular_z;
      tw.angular.x= 0.0;
      tw.angular.y= 0.0;

      tilt_cmd_msg.data=mastcam_tilt;
      pan_cmd_msg.data=mastcam_pan;

      fsi_msg.data = fsi_msg_str;
      
      drive_pub.publish(&tw);  
      fsi_pub.publish(&fsi_msg);

      tilt_pub.publish(&tilt_cmd_msg);
      pan_pub.publish(&pan_cmd_msg);
       
       nh.spinOnce();
 }
 
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
