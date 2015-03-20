/*******************************************************************************

  GPSToI2C
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
// Arduino Pro / Pro Mini 16MHz, 5v
// The target for this source code is the ATMEGA 328P that resides between the GPS receiver and the sensor hub.  
// The sensor hub is itself based upon an ATMEGA 328P.

#include <TinyGPS++.h>
#include <Wire.h>
TinyGPSPlus gps;

unsigned char _115200Cmd[] = {                          
     0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
   };

unsigned char _57600Cmd[] = {                          
     0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xC9
     
   };

unsigned char _10HzCmd[] = {                          
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
   };

unsigned char _5HzCmd[] = {                          
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x05, 0x00, 0x01, 0x00, 0x7E, 0x22
   };


unsigned char _ACK[]  = {
   0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06
   };  

double latitude, longitude, altitude, heading;
unsigned long status;

#define _UpdateRateLen (14)
#define _BaudRateLen (28)

int blink_rate = 0;
long last_blink_time = 0;
long last_sensor_hub_transaction_time = 0;
void blinkStatus()
{
  long current_time = millis();
  long delta = current_time - last_blink_time;
  
  switch(blink_rate)
  {
    case 0:
      digitalWrite(13, HIGH);
      last_blink_time = current_time;
      break;
    case 1: // 1 second blink
      if (delta <= 1000)
      {
        digitalWrite(13, HIGH);
      } else if (delta <= 2000)
      {
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, LOW);
        last_blink_time = current_time;
      }
      break;
    case 2: // 1/2 second blink
      if (delta <= 500)
      {
        digitalWrite(13, HIGH);
      } else if (delta <= 1000)
      {
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, LOW);
        last_blink_time = current_time;
      }
      break;
    case 3: // 1/4 second blink
      if (delta <= 250)
      {
        digitalWrite(13, HIGH);
      } else if (delta <= 500)
      {
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, LOW);
        last_blink_time = current_time;
      }
      break;      
    case 4: // 1/8 second blink
      if (delta <= 125)
      {
        digitalWrite(13, HIGH);
      } else if (delta <= 250)
      {
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, LOW);
        last_blink_time = current_time;
      }
      break;
     default:
       break;
  }
  
}

int SetUpdateRate(unsigned char *update_rate)
{
  int status = false;
  int attempts = 0;
    
  while (attempts <= 5)
  {
    status = SendGPSCommand(update_rate, _UpdateRateLen, true);
    if (status == true)
      return true;
    attempts++;  
  }
  return status;
}

int SwitchGPSBaudRate(unsigned char *baud_rate)
{
  int status = SendGPSCommand(baud_rate, _BaudRateLen, false);
  return status;
}

int SendGPSCommand(unsigned char *cmd, int len, boolean look_for_ack)
{
  int chars_read = 0;
  unsigned char *p = cmd;
  unsigned char ch[7];
  
  while(len > 0)
  {
    Serial.write(*p++);
    len--;
  }
  if (look_for_ack == false)
    return true;

  while (ch[0] != (unsigned char)0xB5)
  {
    if (Serial.available())
    {
      ch[0] = Serial.read();
      chars_read++;
      
    }
    if (chars_read++ > 1000)
    {
      return false;
    }
  }
  delay(100);
  ch[1] = Serial.read();
  ch[2] = Serial.read();
  ch[3] = Serial.read();
  ch[4] = Serial.read();
  ch[5] = Serial.read();
  ch[6] = Serial.read();
 
  if ((ch[0] == _ACK[0]) && (ch[1] == _ACK[1]) && (ch[2] == _ACK[2]) && (ch[3] == _ACK[3]) && (ch[4] == _ACK[4]) && (ch[5] == _ACK[5]) && (ch[6] == _ACK[6]))
  {
    return true;
  }
  return false;
}

void setup()
{
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event

  // The power on reset baud rate for the uBLOX GPS is 9600 baud.
  // Start there, then send the initialization commands to increase the update rate, then switch the GPS module baud rate.
  
  Serial.begin(9600);

  SetUpdateRate(_5HzCmd);
  SetUpdateRate(_5HzCmd);
  SetUpdateRate(_5HzCmd);
  SetUpdateRate(_5HzCmd);
  SetUpdateRate(_5HzCmd);

  SwitchGPSBaudRate(_57600Cmd);
  SwitchGPSBaudRate(_57600Cmd);
  SwitchGPSBaudRate(_57600Cmd);
  SwitchGPSBaudRate(_57600Cmd);
  SwitchGPSBaudRate(_57600Cmd);

  Serial.begin(57600);
  
  last_blink_time = millis();
  blink_rate = 0;
  last_sensor_hub_transaction_time = millis();
  
}

void SendData()
{
  char data[24];
  int i=0;

  byte *p = (byte *)&latitude;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  
  p = (byte *)&longitude;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  
   p = (byte *)&heading;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  
  p = (byte *)&altitude;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;  
  
  p = (byte *)&status;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;
  data[i++] = *p++;  
  
  data[i++] = 0;
  //    Corrected for -5 GMT offset for EST.
  //    data[i++] = (gps.time.hour() > 5)? (gps.time.hour() - 5): (gps.time.hour() + 24 - 5);
  // Provide UTC from the GPS module without the GMT offset.
  data[i++] = gps.time.hour();
  data[i++] = gps.time.minute();
  data[i++] = gps.time.second();
  
  Wire.write(data, i);
}

void requestEvent()
{

 status = gps.failedChecksum() & 0xff;
 status <<= 8;
 status |= gps.passedChecksum() & 0xff;
 status <<= 8;
 status |= gps.satellites.value() & 0xff;
 status <<= 8;
 if (gps.location.isValid())
 {
   status |= 0x1;
 }

 if (gps.location.isValid())
 {
   latitude = gps.location.lat();
   longitude = gps.location.lng();
 }

 if (gps.course.isValid())
 { 
   if ((gps.course.deg() <= 360.0) && (gps.course.deg() >= 0))
     heading = gps.course.deg();
 }

 if (gps.altitude.isValid())
 {
   altitude = gps.altitude.meters();
 }

 SendData();
 
 last_sensor_hub_transaction_time = millis();
 
}

void loop()
{
  // Dispatch incoming characters
   while (Serial.available() > 0)
   {
     gps.encode(Serial.read());
   } 

  // GPS has valid fix?  Blink rate 2
  // GPS has invalid fix? Blink rate 4
  // If sensor hub hasn't requested data within the last 1000ms, use blink rate 0.

  if (gps.location.isValid())
    blink_rate = 2;
  else
    blink_rate = 4;
  
  if ((millis() - last_sensor_hub_transaction_time) > 1000)
    blink_rate = 0;
    
   blinkStatus();
}
