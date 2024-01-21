#include <stdio.h>
#include <math.h>

// Kalman Filter variables
float Q = 0.05; // Process noise covariance
float R = 0.1;  // Measurement noise covariance
float x[2] = {0, 0};  // State [yaw, yaw_rate]
float P[2][2] = {{1, 0}, {0, 1}}; // State covariance
//float A[2][2] = {{1, 1}, {0, 1}}; // State transition matrix
//float C[1][2] = {{1, 0}}; // Measurement matrix
int flag = 1;

// Get gyro and mag measurements 
float get_gyro_measurement() {
  mpu.update();
  float gyro = mpu.getAngleZ(); 
  return gyro;
}

float get_mag_measurement() {
  compass.read();
  int kompas = compass.getAzimuth();
  return kompas;
}

void normalizeYaw(){
  int gyro_sample, sample;

  // Get minimum degree at first activated
  if(flag<4 && flag>0)
  {
    if(flag == 1){flag++; sample = yaw_kalman;}
    else{gyro_sample = yaw_kalman; flag=0;}
//    Serial.print("s:" + String(gyro_sample) +", ");
  }

  yaw_kalman -= gyro_sample;
//  Serial.print(", g:" + String(yaw_kalman) +", ");  
//  if(yaw_kalman > 360){yaw_kalman = yaw_kalman-360;}  // Limit until 360 degree
//  if(yaw_kalman < 0){yaw_kalman += gyro_sample; yaw_kalman = 360+yaw_kalman;}    // Limit until 0 degree

  int zz;
  if(yaw_kalman > 360){zz = (yaw_kalman/360);yaw_kalman = yaw_kalman - (360 * zz);}else
  if(yaw_kalman < 0){zz = (yaw_kalman-360)/360;yaw_kalman = yaw_kalman - (360 * zz);}else
  {yaw_kalman = yaw_kalman;}
}

// Get Yaw Kalman Filter from measurement 
int getData() {
    // Prediction step
    float mag_measurement = get_mag_measurement();
    x[0] = x[0] + x[1] + mag_measurement; /*Serial.print(x[0]);*/
    P[0][0] = P[0][0] + P[0][1] + P[1][0] + P[1][1] + Q; /*Serial.print(" p00=" + String(P[0][0]));*/

    // Measurement step
    float gyro_measurement = get_gyro_measurement(); /*Serial.print(", " + String(gyro_measurement));*/
    float y = gyro_measurement - x[0];  // Selisih antara pengukuran gyro dan nilai prediksi state
    float S = P[0][0] + R;              // Nilai skalar yang merupakan jumlah dari P[0][0]dan R(matriks kovarians dari pengukuran).
    float K[2] = {P[0][0] / S, P[1][0] / S}; /*Serial.print(" k0=" + String(K[0]) + " k1=" + String(K[1]));*/
    
    // Update step
    x[0] = x[0] + K[0] * y;
    x[1] = x[1] + K[1] * y;
    P[0][0] = (1 - K[0]) * P[0][0];
    P[0][1] = (1 - K[0]) * P[0][1];
    P[1][0] = (1 - K[1]) * P[1][0];
    P[1][1] = (1 - K[1]) * P[1][1];
    
    // Update yaw_gyro with the Kalman-filtered value
    yaw_kalman = x[0]; /*Serial.println(", " + String(x[0]));*/
//    Serial.print("g:" + String(yaw_kalman) +", ");
    
    // Normalize Yaw degree
    normalizeYaw();

    return yaw_kalman;
}

void getMark()
{
  for(int i=0; i<5; i++)
  {
    mark[i] = digitalRead(markSensor[i]);
  }
}
