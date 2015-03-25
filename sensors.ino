

void imu_Valget ()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //we store gyro and accel values in an array
  AN[0] = gx;   
  AN[1] = gy;
  AN[2] = gz;
  AN[3] = ax;
  AN[4] = ay;
  AN[5] = az;

}


float read_adc(int select)
{
  if (SENSOR_SIGN[select]<0) {
    return (AN_OFFSET[select]-AN[select]);
  }
  else {
    return (AN[select]-AN_OFFSET[select]);
  }
}


void calib_gyro()
{

   for(int c=0; c<10; c++)
   { 
     digitalWrite(led, HIGH);
      delay(200);
      imu_Valget ();
      digitalWrite(led, LOW); 
      delay(200);
   }
  
   imu_Valget ();
   delay(20);
   imu_Valget ();
   for(int y=0; y<=2; y++)   // Read first initial ADC values for offset.
      AN_OFFSET[y]=AN[y];

   for(int i=0;i<100;i++)    // We take some readings...
   {
      imu_Valget ();
      for(int y=0; y<=2; y++)   // Read initial ADC values for offset (averaging).
         AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
      delay(20);

        }
  
   //AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
 
}


void IMU_print ()
{


Serial.print(pitch);
Serial.print("     ");
Serial.print(roll);
Serial.print("     ");
Serial.print(yaw);
Serial.print("     ");


Serial.println();
}
