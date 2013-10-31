#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>

Servo servo1;
ros::NodeHandle nh;
int v;

void msgCb(const std_msgs::Int32& message) {
  servo1.write(message.data);
} 

ros::Subscriber<std_msgs::Int32> sub("/cameramount/servo/setpoint", &msgCb);

void setup() {
  pinMode(1,OUTPUT);
  servo1.attach(14); //analog pin 0
  servo1.setMinimumPulse(875);
  servo1.setMaximumPulse(2100);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  Serial.begin(115200);
  Serial.println("Ready");
  
  servo1.write(90);
  v = 90;
}

void loop() {

  nh.spinOnce();
  delay(1);  
  
  if ( false && Serial.available()) {
    char ch = Serial.read();
    Serial.print("Received: ");    
    Serial.println(ch);    
    switch(ch) {
       case 'm':
        v = 180;
        servo1.write(v);
        Serial.print("Writing ");
        Serial.print(v);
        Serial.println(" to servo");    
        break;
      case '0'...'9':
        v = (ch - '0') * 10;
        servo1.write(v);
        Serial.print("Writing ");
        Serial.print(v);
        Serial.println(" to servo");    
        break;
      case 'd':
        servo1.detach();
        Serial.println("Detatching servo");      
        break;
      case 'a':
        servo1.attach(14);
        Serial.println("Attatching servo");      
        break;
      case '+':
         v = v + 10;
         Serial.print("Writing ");
         Serial.print(v);
         Serial.println(" to servo");    
         servo1.write(v);
        break;
      case '-':
         v = v - 10;
         Serial.print("Writing ");
         Serial.print(v);
         Serial.println(" to servo");
         servo1.write(v);    
        break;
    }
  }
  
  Servo::refresh();
} 
