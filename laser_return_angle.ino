/*
servo turn and scan for 180
while laser recieve and compare distance to find min
return degree needed to turn from facing angle of the launcher
*/

#include <SoftwareSerial.h> 
#include <Servo.h>

SoftwareSerial SerialLaser(2, 3); // define software serial port name as Serial and define pin2 as RX & pin3 as TX
Servo servo1;

int motor_pin = 9;
float min[2], temp[2];            // min[0] = angle, min[1] = distance
const int HEADER = 0x59;
int uart[9];                      // data measured by LiDAR
int pos = 0, facing_angle = 90;   // set facing angle of the launcher

void setup() {
  servo1.attach(motor_pin);
  Serial.begin(9600);             //set bit rate of serial port connecting Arduino with computer
  SerialLaser.begin(115200);      //set bit rate of serial port connecting LiDAR with Arduino
}

void loop() {
  int strength, check, i, pos, interval=40;
  float distance, temp_sum, min_average=500, temp[2]={0,500};
  
  for (pos = 0; pos <= 180; pos += 1) {           // scanning, 180 degree
    
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
            Serial.println(distance);             // plot data to graph
            Serial.print("dist = ");
            Serial.print(distance);               // output measure distance value of LiDAR
            Serial.print('\t');
            Serial.print("strength = ");
            Serial.print(strength);
            Serial.print('\t');
            Serial.print("angle = ");
            Serial.print(pos);
            Serial.print('\n');
          }
        }
      }
    }

    // find the lefter-most closest object (elimiated random short point situation) 
    // every interval do:
    if(pos!=0 && pos%interval==0 || pos==180){    
      if(pos%interval==0)
        if(min_average >= temp_sum/interval){
          min_average = temp_sum/interval;      // update min average, min distance & angle if average
          min[0]=temp[0];                       // of current interval might be the pole
          min[1]=temp[1]; 
        }
      else
        if(min_average >= temp_sum/(180%interval)){
          min_average = temp_sum/(180%interval);
          min[0]=temp[0];
          min[1]=temp[1];  
        }
    }
    // not interval yet do:
    else{
      temp_sum+=distance;
      if(distance>min[1] && distance>0){
        temp[0]=pos;           // store angle of temp min distance in current interval
        temp[1]=distance;      // store temp min distance in current interval
      }
    }
    
    // move servo
    servo1.write(pos);
    delay(10);
  }

  // retuning?
  for (pos = 180; pos >= 0; pos -= 1) {           // goes from 0 degrees to 180 degrees
    servo1.write(pos);
    delay(10);
  }

  // display result
  Serial.print("min distance = ");
  Serial.print(min[1]);                // output shortest distance
  Serial.print('\t');
  Serial.print("min angle = ");
  Serial.print(min[0]);                // output angle of min distance
  Serial.print('\n');
  Serial.print("when facing ");
  Serial.print(facing_angle);
  if(min[0]>facing_angle){
    Serial.print(", turn left ");
    Serial.print(facing_angle-min[0]);
    Serial.print('\n');
    //return facing_angle-min[0];
  }
  else{
    Serial.print(", turn right ");
    Serial.print(min[0]-facing_angle);
    Serial.print('\n');
    //return min[0]-facing_angle;
  }
  Serial.print("\n\nLoop ended\n\n");
}
