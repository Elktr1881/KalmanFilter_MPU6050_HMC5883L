#include <Wire.h>
#include <QMC5883LCompass.h>
#include <MPU6050_light.h>

QMC5883LCompass compass;
MPU6050 mpu(Wire);

/*
 * mark1 : depan
 * mark2 : kanan
 * mark3 : tengah
 * marj4 : belakang
 * mark5 : kiri
 */
long sendTime, seTime;
int markSensor[5] = {12,11,10,9,8};
int mark[5];
float yaw_kalman = 0; //Yaw update Kalman
bool start;
char sign;

void setup() {
  Serial.begin(9600);
  for(int i=0; i<5; i++){pinMode(markSensor[i],INPUT);}
  
  pinMode(3, OUTPUT);     //buzzer I/O
  digitalWrite(3, HIGH);  //buzOFF
  
  Wire.begin();
  compass.init();
  mpu.begin();

  delay(3000);
  mpu.calcGyroOffsets();
  for(int i=0; i<3; i++)
  {
    digitalWrite(3, LOW); delay(100); //buzON
    digitalWrite(3, HIGH);delay(100); //buzOFF
  }
  digitalWrite(3, HIGH);  //buzOFF
}

void loop() {
//  if(Serial.available()){sign = Serial.read(); if(sign=='R'){start = true;}}
//  while(start)
//  {
    getMark();
    getData();
//    sendTime = millis()-seTime;
//    if(sendTime > 10000)
//    { 
//      Serial.println(String(get_gyro_measurement()) + "," + String(get_mag_measurement()) + "," + String(getData()));
      Serial.print(mark[0]);
      Serial.print(mark[1]);
      Serial.print(mark[2]);
      Serial.print(mark[3]);
      Serial.print(mark[4]);
      Serial.print(",");
      Serial.println((int)yaw_kalman);
//      seTime = millis();
//    }
//    if(Serial.available()){sign = Serial.read(); if(sign=='R'){start = true;}else{start = false;}}
//    else{start = false;}
//  }
}
