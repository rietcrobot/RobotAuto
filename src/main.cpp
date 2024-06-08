#include <Arduino.h>
#include <function_positioncontrol.h>





void setup()
{
  Serial.begin(115200);
  InitRobot();
  
  
}

void loop()
{
  ReadGyro();
  //startMotion();
  //while(!RobotControl(0,100));
  //ReadMPU();
  //Scani2c();
  //digitalWrite(PC13,!digitalRead(PC13));
  //delay(100);
  /*
  startMotion();
  while(!RobotControl(0,100));
  RobotControlStop();

  startMotion();
  while(!RobotControl(0,0));
  RobotControlStop();

  while(1);*/
}
