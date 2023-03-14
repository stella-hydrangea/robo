/*
servo turn and scan for 180
while laser recieve and compare distance to find min
return degree needed to turn from facing angle of the launcher
*/
#include <Arduino.h>
#include <SoftwareSerial.h> 
#include <Servo.h> 
#include <AccelStepper.h>

SoftwareSerial SerialLaser(2, 3); // define software serial port name as Serial and define pin2 as RX & pin3 as TX
Servo servo1;
const int HEADER = 0x59;
int uart[9];                      // data measured by LiDAR

// servo
int driverPUL = 6;    // PUL- pin
int driverDIR = 7;    // DIR- pin
int spd = A0;     // Potentiometer
int pd = 3000;       // Pulse Delay period
boolean setdir = LOW; // Set Direction
AccelStepper stepper = AccelStepper(1, driverPUL, driverDIR);

int motor_pin = 9;
bool scannfing_state = true;

//data arrays
int dist[50];//distance
int strn[50]; //strength
int angl[50]; //angle
int index = 0; //current index
unsigned long currentMills = millis();
unsigned long previousMills;

bool success = false;  
int strength, i, j, check, pos, min=300, range=50, null_counter=0;
float distance;

void setup() {
  servo1.attach(motor_pin);
  Serial.begin(9600);             //set bit rate of serial port connecting Arduino with computer
  SerialLaser.begin(115200);      //set bit rate of serial port connecting LiDAR with Arduino
  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
}

void loop() {

  
  //currentMills = millis();
  // if (currentMills - previousMills >= 500){
  //   previousMills = currentMills;
    
    digitalWrite(driverDIR,setdir);
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);
    //servo1.attach(motor_pin);
    for (pos = 0; pos <= range; pos += 1) {           // scanning, 180 degree
      while (!success){
        // read laser distance data
        if (SerialLaser.available()) {                // check if serial port has data input
          if (SerialLaser.read() == HEADER ){         // assess data package frame header 0x59
            uart[0] = HEADER;
            if (SerialLaser.read() == HEADER) {
              uart[1] = HEADER;
              for (i = 2; i < 9; i++)
                uart[i] = SerialLaser.read();
              check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
              if (uart[8] == (check & 0xff)) {        // verify the received data as per protocol
                distance = uart[2] + uart[3] * 256;   // calculate distance value
                strength = uart[4] + uart[5] * 256;   // calculate signal strength value

                //store to data array
                if (distance>300 || (distance<=1 && pos==0)) distance=300;
                else if (distance<=1 && pos!=0) distance=dist[index-1];
                dist[index] = distance;
                strn[index] = strength;
                angl[index] = pos;
                if (dist[index]<min)  min=dist[index];
                index++;
                success = true;
                Serial.print("dist = "+String(distance)+'\t'+"strength ="+String(strength)+'\t'+"angle = "+String(pos)+'\n');
              }
              else null_counter++;           
            }
          }
        }
        if (null_counter==5){
          distance=300;
          dist[index] = distance;
          strn[index] = strength;
          angl[index] = pos;
          index++;
          success = true;          
          Serial.print("dist = "+String(distance)+'\t'+"strength ="+String(strength)+'\t'+"angle = "+String(pos)+"(none) \n");
        }
      }
      success = false;  // goes from 0 degrees to 180 degrees
      null_counter=0;
      
      stepper.moveTo(pos*111);
      delayMicroseconds(pd);
      //servo1.write(pos);
      //delay(100);
    }

  // for (pos = 50; pos >= 0; pos -= 1) {           // goes from 100 degrees to 0 degrees
  //     servo1.write(pos);
  // }
  
  int count_min=0, starting=0, temp;
  bool multi=false;
  for (i=0; i<range; i++) {
    if (dist[i]==min) {
      if(dist[i-1]==dist[i] && count_min>1){
        count_min++;
        multi=true;
        temp=i;     
      }
      else{
        if (!starting && !multi) starting=i;
        count_min++;
      }
    }
  }
  Serial.println(count_min);  
  if (multi) starting = (temp-count_min/2);
  
  int L=starting, R=starting, edgeL=-1, edgeR=-1, diff_left, diff_right, same_left=0, same_right=0;
  while (L>=0 || R<=range-1){
    if (L==0) edgeL=L;
    else if (starting-L-1 > 5) edgeL=L;
    if (R==range-1) edgeR=R;
    else if (R+1-starting > 5) edgeR=R;

    // do if 
    if (edgeL==-1 || edgeR ==-1){

      diff_left = dist[L-1]-dist[L];
      diff_right = dist[R+1]-dist[R];

      // when diff is +ve
      if (diff_left>3)
        edgeL=L;      // found edge
      else if (diff_left<=3 && diff_left>0 && same_left!=0)
        same_left=0;  // reset counter

      if (diff_right>3)      
        edgeR=R;      // found edge
      else if (diff_left<=3 && diff_right>0 && same_left!=0)
        same_right=0; //reset counter
      

      // when diff = 0, same value
      if (diff_left==0)
        same_left++;        
      if (diff_right==0)            
        same_right++;
      
      if (same_left>3){
        edgeL=L;
        Serial.print("edgeL is set by many continous same value ");
        Serial.println(dist[edgeL], edgeL);
      }
  
      if (same_right>3){
        edgeR=R;
        Serial.print("edgeR is set by many continous same value ");
        Serial.println(dist[edgeR], edgeR);
      }
      if (edgeL==-1) L=L-1;
      if (edgeR==-1) R=R+1;
    }
    if (edgeL!=-1 && edgeR!=-1) break;    
  }
  Serial.println("edgeL : "+String(edgeL)+" edgeR : "+String(edgeR));
  Serial.println("distance : "+String(dist[starting])+" angle : "+String(starting));
  
  Serial.print("\n\nLoop ended\n\n");
  stepper.setCurrentPosition(pos*111);
  delayMicroseconds(pd);
  //servo1.write(starting);
  delay(5000);
}

