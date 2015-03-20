/*******************************************************************************

  Sensor Hub with GPS
  Copyright(c) 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

    Contact Information:
  Joel Rosenzweig, joel.b.rosenzweig@intel.com
    
    We wish to thank the following authors for their contributions to the open source community:
    
    Mikal Hart, for TinyGPS++, http://arduiniana.org/libraries/tinygps/
    Bill Porter, for EasyTransfer, http://www.billporter.info/2011/05/30/easytransferarduino-library/
    Adafruit’s, Kevin Townsend, for the Sensor libraries, https://learn.adafruit.com/adafruit-10-dof-imu-breakout-lsm303-l3gd20-bmp180/software/
    I.Kövesdi, for the Great Circle Distance and Great Circle Bearing calculation, http://obex.parallax.com/object/256
    Alex, for his blog posting on magnetometer calibration, http://theboredengineers.com/2014/11/magnometer-calibration/
    Alex, for his blog posting on a low pass filter, http://theboredengineers.com/2012/09/the-quadcopter-get-its-orientation-from-sensors/
    
*******************************************************************************/
// Arduino Pro/Mini, ATMEGA 328P, 3.3v 8MHZ
// The target for this source code is the ATMEGA 328P, residing on the Sparkfun Arduino breakout board.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <EasyTransfer.h>
#include <EEPROM.h>

#define EDISON 0

#define MAGIC_VALUE 0xAF
#define BUFFER_SIZE 20
#define CALIBRATED 0x10
#define UNCALIBRATED 0x20

EasyTransfer ET; 

struct RECEIVE_DATA_STRUCTURE{
 // from accelerometer
 float pitch;    
 float roll;     
 float accelerometer_x; 
 float accelerometer_y;
 float accelerometer_z;
 // from gyro
 float gyro_x;
 float gyro_y;
 float gyro_z;
 // from magnetometer
 float heading;  
 // from barometer
 float altitude; 
 // from GPS
 float latitude; 
 float longitude;
 float gps_heading;
 float gps_altitude;
 long num_satellites; 
 long valid_fix;
 long hour;
 long minute;
 long second;
 long sample_time;
};

RECEIVE_DATA_STRUCTURE sensor_data;

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float initial_alt = 0;
int calibration_cycles = 0;

// magnetometer calibration
float mag_x_min = 0;
float mag_x_max = 0;
float mag_y_min = 0;
float mag_y_max = 0;
float mag_z_min = 0;
float mag_z_max = 0;
float mag_x_offset = 0;
float mag_y_offset = 0;
float mag_z_offset = 0;
float mag_avg_field = 0;
float mag_x_scale_factor = 0;
float mag_y_scale_factor = 0;
float mag_z_scale_factor = 0;

// Used for low pass filter for altitude calculation
int indexBuffer = 0;
float circularBuffer[BUFFER_SIZE];
float sensorDataCircularSum;
float filteredOutput;
float sensorRawData; 
int state = 0;

boolean initSensors()
{
  if (!gyro.begin())
  {
    Serial.println(F("Problem starting gyro sensor."));
    return false;
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    return false;
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    return false;
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    return false;
  }
  return true;
 
  
}

void calibrate_mag(void)
{
  sensors_event_t mag_event;
  Serial.println("Calibrating magnetometer.  Rotate sensor in all axis for 30 seconds.\n");
  
  //long time = millis();
  //while (millis() < (time + 30000))
  
  // Finish calibration routine when we remove the signal wire on pin 9.
  // A timer is ok, but this ensures we never run out of time.
  while ((digitalRead(9) == 0))
  {
      mag.getEvent(&mag_event);
      if (mag_event.magnetic.x > mag_x_max)
        mag_x_max = mag_event.magnetic.x;
      if (mag_event.magnetic.x < mag_x_min)
        mag_x_min = mag_event.magnetic.x;
  
      if (mag_event.magnetic.y > mag_y_max)
        mag_y_max = mag_event.magnetic.y;
      if (mag_event.magnetic.y < mag_y_min)
        mag_y_min = mag_event.magnetic.y;
  
      if (mag_event.magnetic.z > mag_z_max)
        mag_z_max = mag_event.magnetic.z;
      if (mag_event.magnetic.z < mag_z_min)
        mag_z_min = mag_event.magnetic.z;

      Serial.print("X: ");
      Serial.print(mag_x_min);
      Serial.print(" ");
      Serial.print(mag_x_max);
      Serial.print(" Y: ");
      Serial.print(mag_y_min);
      Serial.print(" ");
      Serial.print(mag_y_max);
      Serial.print(" Z: ");
      Serial.print(mag_z_min);
      Serial.print(" ");
      Serial.println(mag_z_max);
      if ((state++ % 10) > 5)
       digitalWrite(13, HIGH);
      else
       digitalWrite(13, LOW);      
  }
  Serial.println("Done reading mag data.\n");
  
  mag_x_offset = (mag_x_min + mag_x_max) / 2;
  mag_y_offset = (mag_y_min + mag_y_max) / 2;
  mag_z_offset = (mag_z_min + mag_z_max) / 2;
  mag_avg_field = (mag_x_offset + mag_y_offset + mag_z_offset) / 3;
  mag_x_scale_factor = (mag_avg_field / mag_x_offset);
  mag_y_scale_factor = (mag_avg_field / mag_y_offset);
  mag_z_scale_factor = (mag_avg_field / mag_z_offset);
  
  write_mag_calibration();
   
}

void write_mag_calibration(void)
{
 
   Serial.println("Storing magnetometer calibration... ");
   
   Serial.print("Mag X Scale Factor: ");
   Serial.println(mag_x_scale_factor);
   Serial.print("Mag Y Scale Factor: "); 
   Serial.println(mag_y_scale_factor);
   Serial.print("Mag Z Scale Factor: "); 
   Serial.println(mag_z_scale_factor);
  
   Serial.print("Mag X Offset: ");
   Serial.println(mag_x_offset);
   Serial.print("Mag Y Offset: ");
   Serial.println(mag_y_offset);
   Serial.print("Mag Z Offset: ");
   Serial.println(mag_z_offset);
  
   uint8_t *p = (uint8_t *)&mag_x_scale_factor;
 
   EEPROM.write(0, *p++);
   EEPROM.write(1, *p++);
   EEPROM.write(2, *p++);
   EEPROM.write(3, *p++);
   
   p = (uint8_t *)&mag_y_scale_factor;
   EEPROM.write(4, *p++);
   EEPROM.write(5, *p++);
   EEPROM.write(6, *p++);
   EEPROM.write(7, *p++);
   
   p = (uint8_t *)&mag_z_scale_factor;
   EEPROM.write(8, *p++);
   EEPROM.write(9, *p++);
   EEPROM.write(10, *p++);
   EEPROM.write(11, *p++);
   
   p = (uint8_t *)&mag_x_offset;
   EEPROM.write(12, *p++);
   EEPROM.write(13, *p++);
   EEPROM.write(14, *p++);
   EEPROM.write(15, *p++);
  
   p = (uint8_t *)&mag_y_offset;
   EEPROM.write(16, *p++);
   EEPROM.write(17, *p++);
   EEPROM.write(18, *p++);
   EEPROM.write(19, *p++);
  
   p = (uint8_t *)&mag_z_offset;
   EEPROM.write(20, *p++);
   EEPROM.write(21, *p++);
   EEPROM.write(22, *p++);
   EEPROM.write(23, *p++);
  
   EEPROM.write(511, MAGIC_VALUE); // Just a magic value that means we wrote the calibration data.  If it's not read back upon boot, then mag calibration was not performed.
   Serial.println("Finished..."); 
  
}

int read_mag_calibration(void)
{
 
  // Read the magic value to see if Mag calibration was already performed
 int calibrated = EEPROM.read(511); 
 
 // If the mag was calibrated, then go ahead and read those values.
 if (calibrated == MAGIC_VALUE) 
 { 
  
     uint8_t *p = (uint8_t *)&mag_x_scale_factor;
     
     Serial.println("Reading magnetometer calibration... ");
     
     *p++ = EEPROM.read(0);
     *p++ = EEPROM.read(1);
     *p++ = EEPROM.read(2);
     *p++ = EEPROM.read(3);
     
     p = (uint8_t *)&mag_y_scale_factor;
     *p++ = EEPROM.read(4);
     *p++ = EEPROM.read(5);
     *p++ = EEPROM.read(6);
     *p++ = EEPROM.read(7);
     
     p = (uint8_t *)&mag_z_scale_factor;
     *p++ = EEPROM.read(8);
     *p++ = EEPROM.read(9);
     *p++ = EEPROM.read(10);
     *p++ = EEPROM.read(11);
     
     p = (uint8_t *)&mag_x_offset;
     *p++ = EEPROM.read(12);
     *p++ = EEPROM.read(13);
     *p++ = EEPROM.read(14);
     *p++ = EEPROM.read(15);
    
     p = (uint8_t *)&mag_y_offset;
     *p++ = EEPROM.read(16);
     *p++ = EEPROM.read(17);
     *p++ = EEPROM.read(18);
     *p++ = EEPROM.read(19);
    
     p = (uint8_t *)&mag_z_offset;
     *p++ = EEPROM.read(20);
     *p++ = EEPROM.read(21);
     *p++ = EEPROM.read(22);
     *p++ = EEPROM.read(23);
    
     Serial.println("Finished..."); 
     Serial.print("Mag X Scale Factor: ");
     Serial.println(mag_x_scale_factor);
     Serial.print("Mag Y Scale Factor: "); 
     Serial.println(mag_y_scale_factor);
     Serial.print("Mag Z Scale Factor: "); 
     Serial.println(mag_z_scale_factor);
    
     Serial.print("Mag X Offset: ");
     Serial.println(mag_x_offset);
     Serial.print("Mag Y Offset: ");
     Serial.println(mag_y_offset);
     Serial.print("Mag Z Offset: ");
     Serial.println(mag_z_offset);
     return CALIBRATED;
     } else {
       Serial.println("MAG WAS NOT CALIBRATED!");
       delay(2000);
       return UNCALIBRATED;
     }
}



void smoothSensorReadings()
{
    // We remove the oldest value from the buffer
   sensorDataCircularSum= sensorDataCircularSum - circularBuffer[indexBuffer];
   // The new input from the sensor is placed in the buffer
   circularBuffer[indexBuffer]=sensorRawData;
   // It is also added to the total sum of the last  BUFFER_SIZE readings
   // This method avoids to sum all the elements every time this function is called.
   sensorDataCircularSum+=sensorRawData;
   // We increment the cursor
   indexBuffer++;
 
   if (indexBuffer>=BUFFER_SIZE) indexBuffer=0;// We test if we arrived to the end
   //of the buffer, in which case we start again from index 0
   filteredOutput =(sensorDataCircularSum/BUFFER_SIZE); // The output is the the mean
   //value of the circular buffer.
}

/**************************************************************************/
/*

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(57600);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, OUTPUT); 
  digitalWrite(10, LOW);
 
  /* Initialise the sensors */
  // This could loop forever if there is a problem.
  // The LED will not blink if this routine fails, so check the LED for status.
  while(initSensors() == false);
  
  if ((read_mag_calibration() == UNCALIBRATED) || (digitalRead(9) == 0))
  {
    calibrate_mag();
  } else {
    Serial.println("Successfully read mag calibration parameters.");
  }

  ET.begin(details(sensor_data), &Serial);
  
}

void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_event_t gyro_event;
  sensors_vec_t   orientation;
  sensors_vec_t   ahrs_orientation;

  float alt = 0;
  float pitch = 0;
  float roll = 0;
  
  gyro.getEvent(&gyro_event);
  sensor_data.gyro_x = gyro_event.gyro.x;
  sensor_data.gyro_y = gyro_event.gyro.y;
  sensor_data.gyro_z = gyro_event.gyro.z;
     
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  sensor_data.accelerometer_x = accel_event.acceleration.x;
  sensor_data.accelerometer_y = accel_event.acceleration.y;
  sensor_data.accelerometer_z = accel_event.acceleration.z;

  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
     sensor_data.pitch = orientation.pitch;
     sensor_data.roll = orientation.roll;
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);

  mag_event.magnetic.x = mag_x_scale_factor * (mag_event.magnetic.x - mag_x_offset);
  mag_event.magnetic.y = mag_y_scale_factor * (mag_event.magnetic.y - mag_y_offset);
  mag_event.magnetic.z = mag_z_scale_factor * (mag_event.magnetic.z - mag_z_offset);
  
  // Now use the calibrated values to determine heading.
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
      sensor_data.heading = orientation.heading;
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    
    if (calibration_cycles < 20)
    {
      initial_alt = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
      calibration_cycles ++;
    } 
    
    alt = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature) - initial_alt;

    
    sensorRawData = alt;
    smoothSensorReadings();   
    sensor_data.altitude = filteredOutput; // Altitude in meters
  }

  GetGPSData();
  
  sensor_data.sample_time = millis();
  
  ET.sendData();
  if ((state++ % 25) > 12)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

}

void GetGPSData()
{

  char data[24];
  int i=0;
  
  float latitude, longitude, heading, altitude;
  unsigned long status;
  unsigned long hour, minute, second;
  
  byte *p;
  
  Wire.requestFrom(2, 24);    // request 24 bytes from slave device #2 (our GPS)
  while(Wire.available())
  {
    data[i++] = Wire.read();
  }
  p = (byte *)&latitude;
  *p++ = (data[0] & 0xff);
  *p++ = (data[1] & 0xff);
  *p++ = (data[2] & 0xff);
  *p++ = (data[3] & 0xff);
  
  p = (byte *)&longitude;
  *p++ = (data[4] & 0xff);
  *p++ = (data[5] & 0xff);
  *p++ = (data[6] & 0xff);
  *p++ = (data[7] & 0xff);
 
  p = (byte *)&heading;
  *p++ = (data[8] & 0xff);
  *p++ = (data[9] & 0xff);
  *p++ = (data[10] & 0xff);
  *p++ = (data[11] & 0xff);
  
  p = (byte *)&altitude;
  *p++ = (data[12] & 0xff);
  *p++ = (data[13] & 0xff);
  *p++ = (data[14] & 0xff);
  *p++ = (data[15] & 0xff);

  p = (byte *)&status;
  *p++ = (data[16] & 0xff);
  *p++ = (data[17] & 0xff);
  *p++ = (data[18] & 0xff);
  *p++ = (data[19] & 0xff);   
 
  // not using data[20];
  hour = data[21] & 0xff;
  minute = data[22] & 0xff;
  second = data[23] & 0xff;
 
 
  sensor_data.latitude = latitude;
  sensor_data.longitude = longitude;
  sensor_data.gps_heading = heading;
  sensor_data.gps_altitude = altitude;
  sensor_data.num_satellites = ((status >> 8) & 0xff);
  sensor_data.valid_fix = (status & 0x1)?true:false;
 
  sensor_data.hour = hour;
  sensor_data.minute = minute;
  sensor_data.second = second;
}
