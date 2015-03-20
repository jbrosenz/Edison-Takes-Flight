/*******************************************************************************

  Intel Edison High Level Flight Controller
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

// The target for this source code is the Intel(R) Edison.  Use the Arduino for Edison tools to build it.


#include "Telemetry.h"

#define FLIGHT_STATUS_IDLE              0x1000
#define FLIGHT_STATUS_GOTO_WAYPOINT     0x2000
#define FLIGHT_STATUS_WAYPOINT_REACHED  0x4000

#define FLAG_NONE                   0x0
#define FLAG_RETURN_HOME            0x1
#define FLAG_LAND                   0x2



#define NEUTRAL_PWM                 120000
#define NEUTRAL_PITCH               120000
#define NEUTRAL_YAW                 120000
#define NEUTRAL_THROTTLE            120000
#define NEUTRAL_ROLL                120000
#define MODERATE                    25000
#define AGGRESSIVE                  35000
#define ADJUSTMENT_RANGE            35000

// If necessary to reverse a channel output, change the sign of the value here.
#define PITCH_AXIS_SENSE            -1
#define ROLL_AXIS_SENSE             -1
#define YAW_AXIS_SENSE              -1
#define ALT_AXIS_SENSE              1

#define PITCH_AXIS_GAIN             0.75
#define ROLL_AXIS_GAIN              1
#define YAW_AXIS_GAIN               0.35
#define ALTITUDE_GAIN               1

// expressed in meters
#define MIN_ALTITUDE_FOR_YAW      5


// We will consider the waypoint reached if the aircraft is within this distance, expressed in meters.
#define WAYPOINT_REACHED_THRESHOLD  3

#define MEAN_RADIUS_EARTH_KM        6371.01

#define DegreesToRadians(x)        (x * PI/180.0)
#define RadianstoDegrees(x)        (x * 180.0/PI)

// At my home location, magnetic declination is 14.62 degrees WEST.
// It means that the magnetic reading (from the magnetometer) is 14.62 degrees WEST (counter clockwise) of TRUE NORTH.
// When the magnetometer reads 0.0 degrees, it is actually pointing to -14.62 degrees, i.e. 345.38 degrees true north.
// Navigation bearings are based on TRUE NORTH.
//
// Subtract a "WEST" declination from or add an "EAST" declination to the magnetometer reading to correct the heading information.
// Find your magnetic declination here:  http://www.ngdc.noaa.gov/geomag-web/#declination
// When defining your own magnetic declination, provide a negative value for WEST declination readings and a positive value for EAST declination readings.
#define MAGNETIC_DECLINATION (-14.62)

#include <EasyTransfer.h>
EasyTransfer ET; 

struct SENSOR_DATA{
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
 long  num_satellites;
 long  valid_fix;
 long  hour;
 long  minute;
 long  second;
 long  sample_time;
};
SENSOR_DATA sensor_data;

#define NUM_WAYPOINTS 10

struct WAYPOINT{
  float latitude; 
  float longitude; 
  float altitude; // relative altitude in meters above ground level (AGL) in meters
  float gps_altitude; // actual altitude above sea level (ASL) in meters
  float distance; // The computed distance from our current location to this waypoint.  Updated each frame.
  float heading; // The computed bearing from our current location to this waypoint.  Updated each frame.
  float camera_heading; // Upon arrival, swivel the camera (or the airframe) to this angle.
  float camera_pitch_angle; // The desired camera pitch angle that should be obtained once at the waypoint. Expressed in degrees.
  int loiter_duration; // Determines how long the aircaft will loiter at the waypoint before processing the next command. Expressed in milliseconds.
  int special_flags; // FLAG_NONE, FLAG_LAND, FLAG_HOLD_POSITION or FLAG_RETURN_HOME
  boolean valid;
};

WAYPOINT current_location;
WAYPOINT target_location;
WAYPOINT home_location;
WAYPOINT waypoint_list[NUM_WAYPOINTS];
int waypoint_index = 0;

int sensor_readings = 0;
long time_waypoint_reached;
long time_min_altitude_reached;
boolean at_min_altitude = false;
boolean home_location_set = false;
boolean telemetry_ok = false;
float initial_altitude = 0.0;
float initial_heading = 0.0;

void FlyToWaypoint(SENSOR_DATA sensor_data, WAYPOINT target);
float HeadingError(float current_heading, float target_heading);

long   flight_status = FLIGHT_STATUS_IDLE;

long  HLC_PITCH;
long  HLC_ROLL;
long  HLC_THROTTLE;
long  HLC_YAW;

boolean seeking_higher_altitude = false;
boolean seeking_lower_altitude = false;
long current_time;
long last_sample_time;
float integrated_gyro_z;
float gyro_z_error = 0.0;
float gyro_heading = 0.0;
long  gyro_samples_collected = 0;
long last_sensor_hub_transfer_time = 0;

void setup() {
   // Serial1 transmits from Edison UART1 to the MUX
   // Serial1 receives from the Arduino Sensor hub 
   Serial1.begin(57600);
   
   // Serial is the virtual serial port over the composite USB channel.  Fine to use when plugged into the computer.
   Serial.begin(115200);

   ET.begin(details(sensor_data), &Serial1);
   pinMode(3, OUTPUT);
   pinMode(5, OUTPUT);
   pinMode(6, OUTPUT);
   pinMode(7, OUTPUT);
   pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);
   
 
   HLC_PITCH    = NEUTRAL_PWM;
   HLC_ROLL     = NEUTRAL_PWM;
   HLC_THROTTLE = NEUTRAL_PWM;
   HLC_YAW      = NEUTRAL_PWM;
   flight_status = FLIGHT_STATUS_IDLE;

   // In the absence of some commands, then we want to stay put.
   target_location.latitude = sensor_data.latitude;
   target_location.longitude = sensor_data.longitude;
   target_location.heading = sensor_data.heading;
   target_location.altitude = sensor_data.altitude;
 
   flight_status = FLIGHT_STATUS_IDLE;
   
   for (int i = 0; i < NUM_WAYPOINTS; i++)
   {
     waypoint_list[i].valid = false;
   }
   
   
   // Embedded waypoints for testing until integration with web server
   
   // center of field
   waypoint_list[0].latitude = 42.3810296;   
   waypoint_list[0].longitude = -71.5590512; 
   waypoint_list[0].altitude = 15.0; // meters
   waypoint_list[0].loiter_duration = 5000;
   waypoint_list[0].camera_pitch_angle = 90.0;
   waypoint_list[0].special_flags = FLAG_NONE;
   waypoint_list[0].valid = true;
   
   // South of field, near goal post, centered.
   waypoint_list[1].latitude = 42.3807165; 
   waypoint_list[1].longitude = -71.5590137; 
   waypoint_list[1].altitude = 15.0; 
   waypoint_list[1].loiter_duration = 5000;
   waypoint_list[1].camera_pitch_angle = 90.0;
   waypoint_list[1].special_flags = FLAG_NONE;
   waypoint_list[1].valid = true;

   
   seeking_higher_altitude = false;
   seeking_lower_altitude = false;

   integrated_gyro_z = 0.0;
   last_sample_time = 0.0;
   
   InitTelemetry();
    
}

void SendChannelData(char cmd, long value)
{
  Serial1.write('$');
  Serial1.write(cmd);
  Serial1.write(  ((value & 0xff0000) >> 16) & 0xff );
  Serial1.write(  ((value & 0x00ff00) >> 8) & 0xff );
  Serial1.write(   (value & 0x0000ff) & 0xff );
  Serial1.write('!');
}

void SendTelemetryData()
{
  
  char buffer_wireless[1024];
  char buffer_file[1024];
  
  sprintf(buffer_wireless, "Waypoint %.2d, GPS Status: %s, Satellites: %.2d, Time: %.2d:%.2d:%.2d GMT, Computed Bearing: %.2f, GPS Lat/Long %.6f,%.6f, GPS Heading %.2f, Mag Heading %.2f Gyro Heading %.2f GPS Altitude: %.2f, Baro Alt: %.2f, DTT: %.2f, Waypoint reached: %s\n",
     waypoint_index, 
     (sensor_data.valid_fix == true)?"  VALID":"INVALID", 
     sensor_data.num_satellites,
     sensor_data.hour,
     sensor_data.minute,
     sensor_data.second,
     target_location.heading,
     sensor_data.latitude,
     sensor_data.longitude,
     sensor_data.gps_heading,
     sensor_data.heading,
     gyro_heading,
     sensor_data.gps_altitude,
     sensor_data.altitude,
     target_location.distance,
     (flight_status & FLIGHT_STATUS_WAYPOINT_REACHED)?" TRUE":"FALSE"
  );
 
  if (telemetry_ok)
  {
    Serial2.println(buffer_wireless);
  }


  sprintf(buffer_file, "%.2d,%s,%.2d,%.2d:%.2d:%.2d,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d\n",
       waypoint_index, 
       (sensor_data.valid_fix == true)?"  VALID":"INVALID", 
       sensor_data.num_satellites,
       sensor_data.hour,
       sensor_data.minute,
       sensor_data.second,
       target_location.heading,
       sensor_data.gps_heading,
       sensor_data.heading,
       gyro_heading,
       sensor_data.latitude,
       sensor_data.longitude,
       sensor_data.gps_altitude,
       sensor_data.altitude,
       target_location.distance,
       (flight_status & FLIGHT_STATUS_WAYPOINT_REACHED)?1:0,
       HLC_PITCH, 
       HLC_ROLL, 
       HLC_YAW, 
       HLC_THROTTLE
       );
  
  LogTelemetry(buffer_file);

}

// Write an out of band telemetry message over the wireless telemetry.  Does not log to the file to keep it clean for easier post processing.
void SendTelemetryString(char *buffer)
{

  if (telemetry_ok)
  {
    Serial2.print(buffer);
  }
 
}


void loop() 
{
  // Slowly blink the red status LED to show the main loop is running. 
  if ((millis() % 1000) < 500)
     digitalWrite(8, HIGH);
  else
     digitalWrite(8, LOW);
     
  if (ET.receiveData())
  {
       last_sensor_hub_transfer_time = millis();
       
       if (telemetry_ok)
       {
         // Telemetry is ready to send.  Blink at the high rate.
         if ((millis() % 100) < 50)
           digitalWrite(7, HIGH);
         else
           digitalWrite(7, LOW);
        } else {
          // Telemetry not ready to transmit.  Blink at the low rate.
          if ((millis() % 500) < 250)
            digitalWrite(7, HIGH);
          else
            digitalWrite(7, LOW);
       } 
  
       // Allow the system to finish booting and to stop transmitting data to the console.
       // Once we are satisfied that the console has stopped printing data, we can initialize it at our desired
       // baud rate and start sending telemetry through it.  
       // Initializing this too early causes the boot to hang.
       // To debug:  Even when we wait long enough, need multiple attempts to begin the Serial2 stream.  5 attempts always works.  1 does not always work.
       if (telemetry_ok == false)
       {
         if (sensor_readings++ == 300)
         {
            Serial2.begin(57600);
            Serial2.begin(57600);
            Serial2.begin(57600);
            Serial2.begin(57600);
            Serial2.begin(57600);
            SendTelemetryString("Initialization complete!\n");
            telemetry_ok = true;
            
         }
       }
      
       
       // Add the declination from the magnetic reading to get a true north reading.
       // In our case, declination is negative.
       sensor_data.heading += MAGNETIC_DECLINATION;

       if (sensor_data.heading > 360)
          sensor_data.heading -= 360.0;
       else if (sensor_data.heading < 0)
          sensor_data.heading += 360.0;
       
       if (sensor_readings == 1)
       {
         initial_heading = sensor_data.heading;
       }
       
       if (last_sample_time > 0.0)
       {
         // integrate the gyro data to determine rotation in radians
         // sample time is expressed in milliseconds, and rotation is expressed in radians/second.
         // Remove the static bias and multiply by the time interval in seconds.
         integrated_gyro_z += (sensor_data.gyro_z - 0.01730) * ((sensor_data.sample_time - last_sample_time) / 1000.0);
 
         // handle the wrap around (useful for the experiment trying to determine actual heading instead of raw degrees of rotation)
         if (integrated_gyro_z > (2 * PI))
           integrated_gyro_z -= (2 * PI);
         if (integrated_gyro_z < 0)
           integrated_gyro_z += (2 * PI);
         
         // For now, try for fun to compute an absolute heading based upon the initial heading read from the mag at boot up + the estimated rotation from the gyro.
         // If this were perfect, it would maintain an accurate heading to augment the mag.  But it's not, and not expected to be.
         // In future experiments, we will simply track relative degrees of rotation to improve the yaw routine.  Actual heading won't matter for that purpose.
         gyro_heading = (-1 * RadianstoDegrees(integrated_gyro_z)) + initial_heading;
         if (gyro_heading > 360)
           gyro_heading -= 360.0;
         else if (gyro_heading < 0)
           gyro_heading += 360.0;

       }
       last_sample_time = sensor_data.sample_time;

       SendTelemetryData();
       
 
       // If the aircraft is sitting still, we can look at the average gyro error by taking many samples and averaging them. 
       // The static error is: (gyro_z_error / gyro_samples_collected), and that's what was used to calculate the number 0.01730 used above.
       // Only makes sense to measure this bias when the aircraft is not moving.  So something a little more sophisticated is required to automatically
       // measure and obtain the static error.  
       // gyro_z_error += sensor_data.gyro_z;
       // gyro_samples_collected ++;
            
   } 
 
   // if home location information has been determined, store home location and rename the telemetry file 
   // to match the time from the GPS to make file identification easier later.  The full date would make this even nicer.  Save that for another day.
   if ((home_location_set == false) && (sensor_data.valid_fix == true))
   {
     char buffer[16];
     
     home_location.latitude = sensor_data.latitude;
     home_location.longitude = sensor_data.longitude;
     home_location.altitude = sensor_data.altitude;
     home_location.gps_altitude = sensor_data.gps_altitude;
     home_location_set = true;
     initial_altitude = sensor_data.altitude;
     
     // The GPS data is now good.  Therefore, the time/date, and lat/long are now valid.
     // Rename the log file to a name that reflects the current time so it's easier to reference later.
     sprintf(buffer, "%.2d%.2d%.2d", sensor_data.hour, sensor_data.minute, sensor_data.second);
     RenameTelemetryFile(buffer);
   }
      
   HighLevelFlightStateMachine();

   // Did we lose communication with the sensor hub?  If so, auto hover.
   if (millis() - last_sensor_hub_transfer_time > 2000)
   {
     // Bad news bears.
     // No communication with the sensor hub
     SendTelemetryString("Communications lost with sensor hub!\n");
     digitalWrite(7, HIGH); // Make this LED solid
 
     // maintain auto hover    
     HLC_YAW = NEUTRAL_PWM;
     HLC_PITCH = NEUTRAL_PWM;
     HLC_ROLL = NEUTRAL_PWM;
     HLC_THROTTLE = NEUTRAL_PWM;
   }
  
   // Send the outputs to the MUX for delivery to the low level flight controller   
   SendChannelData('P', HLC_PITCH);
   SendChannelData('R', HLC_ROLL);
   SendChannelData('T', HLC_THROTTLE);
   SendChannelData('Y', HLC_YAW);  

}

// Assert that the aircraft does not fly outside the box defined by this 4 point geo fence.
// If the aircraft crosses the fence, put it into an automatic, position hold hover.
// The box is based upon one max longitude, one min longitude, one max latitude, and one min latitude.
// Eventually, modify this function so that vertices for an arbitrary convex polygon can be passed in and the bounding box follows that precise shape.
// For now, the max/min lat/long are embedded.  Nothing fancy, but it does the job.  
// Perhaps it would be interesting to have the aircraft fly home if it hits the fence.  
// Just depends on what your goal is in the first place.  We didn't want to fly over a road by mistake!
void AssertGeoFence()
{
  
  // Don't go EAST of the geofence:
  if (sensor_data.longitude > -71.5578576)
  {
       // Maintain these neutral values, which equate to a stationary hover, unless the state machine makes a move to update them.
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    SendTelemetryString("EXCEEDING EAST GEOFENCE.  AUTO HOVER ENABLED!\n");
  }

  // Don't go WEST of the geofence:
  if (sensor_data.longitude < -71.559402)
  {
       // Maintain these neutral values, which equate to a stationary hover, unless the state machine makes a move to update them.
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    SendTelemetryString("EXCEEDING WEST GEOFENCE.  AUTO HOVER ENABLED!\n");
  }
  
  // Don't go NORTH of the geofence:
  if (sensor_data.latitude > 42.381570)
  {
       // Maintain these neutral values, which equate to a stationary hover, unless the state machine makes a move to update them.
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    SendTelemetryString("EXCEEDING NORTH GEOFENCE.  AUTO HOVER ENABLED!\n");
  }
  
  // Don't go SOUTH of the geofence:
  if (sensor_data.latitude < 42.380480)
  {
       // Maintain these neutral values, which equate to a stationary hover, unless the state machine makes a move to update them.
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    SendTelemetryString("EXCEEDING SOUTH GEOFENCE.  AUTO HOVER ENABLED!\n");
  }
}

// As long as the GPS signal is still good, keep flying as normal.  
// We want 3+ satellites and a GPS reported "VALID" fix to continue navigation.
// If that condition isn't met, try to auto-hover.  
// This might be impossible if the low level flight controller also lost GPS. 
// But it might buy some time for things to recover.
void AssertGPSValid()
{
  if (sensor_data.num_satellites < 3)
  {
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    if (sensor_data.altitude >= MIN_ALTITUDE_FOR_YAW)
      SendTelemetryString("SATELLITE LOCK LOST.  AUTO HOVER ENABLED!\n");   
  }
  
  if (sensor_data.valid_fix == false)
  {
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    if (sensor_data.altitude >= MIN_ALTITUDE_FOR_YAW)
      SendTelemetryString("SATELLITE LOCK LOST.  AUTO HOVER ENABLED!\n");   
  }
  
}
void HighLevelFlightStateMachine()
{
    // TODO: Integrate routine to read messages from the web server and modify the state machine accordingly.
    // The state machine is currently geared for use without the web server.  Therefore, with hardcoded waypoints used for testing, the state machine
    // moves from FLIGHT_STATUS_IDLE to FLIGHT_STATUS_GOTO_WAYPOINT immediately.  Once we modify the state machine to read messages from the web server,
    // we will add waypoints dynamically, then wait for the "GO" command to transition from FLIGHT_STATUS_IDLE to FLIGHT_STATUS_GOTO_WAYPOINT.
    // Also, look for exceptions that tell us to abort a mission (read from the web server to see if there are any messages we need to know about)

    // Maintain these neutral values, which equate to a stationary hover, unless the state machine makes a move to update them.
    HLC_YAW = NEUTRAL_PWM;
    HLC_PITCH = NEUTRAL_PWM;
    HLC_ROLL = NEUTRAL_PWM;
    HLC_THROTTLE = NEUTRAL_PWM;
    
    if (flight_status & FLIGHT_STATUS_IDLE)
    {
      // Get the next job from the job list
      if (waypoint_index < NUM_WAYPOINTS)
      {
        if (waypoint_list[waypoint_index].valid == true)
        {
         
          char buffer[64];
          sprintf(buffer, "Processing waypoint [%d]\n", waypoint_index);
          SendTelemetryString(buffer); 
         
          flight_status = FLIGHT_STATUS_GOTO_WAYPOINT;
          target_location.latitude = waypoint_list[waypoint_index].latitude;
          target_location.longitude = waypoint_list[waypoint_index].longitude;
          target_location.altitude = waypoint_list[waypoint_index].altitude;
          target_location.loiter_duration = waypoint_list[waypoint_index].loiter_duration;
          target_location.camera_pitch_angle =  waypoint_list[waypoint_index].camera_pitch_angle;
          target_location.special_flags = waypoint_list[waypoint_index].special_flags;
          time_min_altitude_reached = 0;
          at_min_altitude = false;
          
          if (waypoint_list[waypoint_index].special_flags == FLAG_RETURN_HOME)
          {
            target_location.latitude =  home_location.latitude;
            target_location.longitude = home_location.longitude;
            target_location.altitude = 30 + home_location.altitude; // Return home at 30 meters altitude.  Once we arrive, we can reduce altitude;
          }

          if (target_location.altitude > sensor_data.altitude)
            seeking_higher_altitude = true;
          else
            seeking_higher_altitude = false;
            
          if (target_location.altitude < sensor_data.altitude)
            seeking_lower_altitude = true;
          else
            seeking_lower_altitude = false;
          
        } else {
            SendTelemetryString("No more waypoints.");
        }
        waypoint_index++;
      }
     }

    if (flight_status & FLIGHT_STATUS_GOTO_WAYPOINT)
    {
         // Adjust the altitude to match the target altitude.
        SeekAltitude(sensor_data.altitude, target_location.altitude);

        target_location.heading = Calc_Great_Circle_Bearing(sensor_data.latitude, sensor_data.longitude, target_location.latitude, target_location.longitude);
        target_location.distance = Calc_Great_Circle_Distance(sensor_data.latitude, sensor_data.longitude, target_location.latitude, target_location.longitude);
     
        // Always want to seek the desired heading while the mission is active.
        // However, we don't want to yaw while on the ground so we don't break the aircraft or topple over.
        // Therefore, make sure the aircraft is above the minimum altitude threshold before we try to maintain yaw.
        if (sensor_data.altitude >= MIN_ALTITUDE_FOR_YAW)
        {
            // Take note of the time at which we reach safe yaw altitude because we want to delay yawing for a few seconds to establish a good GPS heading.
            if (at_min_altitude == false)
            {
              time_min_altitude_reached = millis();
              at_min_altitude = true;
            }
            
            // Check to see if we are close enough to our waypoint to consider it a proper arrival
            // Examine the distance to the waypoint (great circle) and also the altitude
            if (target_location.distance <= WAYPOINT_REACHED_THRESHOLD)
            {
              // Has aircraft has finished seeking the desired altitude?  If so, then the waypoint has been reached.
              if ((seeking_higher_altitude == false) && (seeking_lower_altitude == false))  
              {
                flight_status = FLIGHT_STATUS_WAYPOINT_REACHED;
                // Record the time of arrival
                time_waypoint_reached = millis();
                SendTelemetryString("WAYPOINT REACHED!\n");
                
              } else {
                SendTelemetryString("Waypoint LAT/LONG reached, waiting to seek to correct altitude...\n");
              }
            } else {
                           
              // If aircraft has been at the minimum altitude for 3 seconds or more, go ahead and provide yaw input.  
              // Otherwise, only provide pitch input so that the GPS can establish a heading.  
              if (millis() > (time_min_altitude_reached + 3000))
              {
                // Adjust the yaw so that the aircraft points towards the target.
                // If the magnetometer worked really well, we would want to yaw to the magnetic heading.
                // YawToHeading(sensor_data.heading, target_location.heading)
                // So far, the results from the magnetometer are not reliable enough, despite calibration.
                // We're using the GPS heading instead.  This is great if we're moving, and worthless if the aircraft is stationary, or skidding.
                // In the next set of experiments, we will use the gyro rotation instead of measured heading to yaw the correct amount.
                // This will require a change that disables yaw periodically to allow the GPS to establish a heading again before trying to correct
                // the heading again.
                
                // Despite that, by tweaking the yaw rates to limit skidding, yawing to the GPS heading worked reliably and got us to the waypoint every time.
                // But no awards given for efficiency of the flight path or the elegance of the flight track in the sky!
                YawToHeading(sensor_data.gps_heading, target_location.heading);
              }
              
              // Generate the pitch commands to guide the aircraft towards the waypoint.
              FlyToWaypoint(sensor_data, target_location);
            }
        } 
    }


    if (flight_status & FLIGHT_STATUS_WAYPOINT_REACHED)
    {
       target_location.heading = Calc_Great_Circle_Bearing(sensor_data.latitude, sensor_data.longitude, target_location.latitude, target_location.longitude);
       target_location.distance = Calc_Great_Circle_Distance(sensor_data.latitude, sensor_data.longitude, target_location.latitude, target_location.longitude);

       // Eventually, add routines such as these to adjust the camera angle or position as desired for a given waypoint.
       //AdjustCameraPosition(target_location.camera_pitch_angle);
       //AdjustCameraHeading(target_location.camera_heading);

       if ( (millis() - time_waypoint_reached) > target_location.loiter_duration)
       {
            // Get the next job from the job list
            flight_status = FLIGHT_STATUS_IDLE;
            SendTelemetryString("Loiter duration expired.  Loading next waypoint!\n");
       }
    }
    
    // After all is said and done, validate that we are not flying out of bounds.  
    // This puts the controls at neutral if anything went wrong.
    AssertGeoFence();

    // Make sure we don't continue trying to navigate if we lost satellite lock.
    // Like the geo fence, puts the controls at neutral if anything went wrong.
    AssertGPSValid();
}


// Seek the given altitude.  The aircraft will climb or descend as necessary, and will hold altitude once it reaches or exceeds the target altitude.
// Stop seeking the altitude once the desired altitude has been reached.  Don't modulate the throttle to maintain a given altitude.  
// Let the low level flight controller do that.
void SeekAltitude(float current_altitude, float target_altitude)
{
  float error = (target_altitude - current_altitude);

  HLC_THROTTLE = NEUTRAL_THROTTLE;
  
  if (seeking_higher_altitude == true)
  {
    if (error > 0) // still need to climb?
    {
        HLC_THROTTLE = NEUTRAL_THROTTLE + (MODERATE * ALT_AXIS_SENSE);
    }
    if (error <= 0) // aircraft reached the target altitude, or is now above it.  -> stop climbing
    {
        HLC_THROTTLE = NEUTRAL_THROTTLE;
        seeking_higher_altitude = false;
    }
  }
  
  if (seeking_lower_altitude == true)
  {
    if (error < 0) // still need to descend?
    {
      if (error < -8.0) // if difference is > 8 meters, use the aggressive throttle 
        HLC_THROTTLE = NEUTRAL_THROTTLE - (AGGRESSIVE * ALT_AXIS_SENSE);
      else // within 8 meters, use the moderate throttle
        HLC_THROTTLE = NEUTRAL_THROTTLE - (MODERATE * ALT_AXIS_SENSE);
    }      
    if (error >= 0) // aircraft reached the target altitude, or is now below it.  -> stop descending
    {
        HLC_THROTTLE = NEUTRAL_THROTTLE;
        seeking_lower_altitude = false;
    }
    
    // Could implement a virtual hard deck here //
    // If aircraft is not landing, and we're seeking a lower altitude, don't allow it to fly below the VIRTUAL_HARD_DECK.
    // if (sensor_data.altitude < VIRTUAL_HARD_DECK_ALTITUDE) && !(flight_status & LANDING_SEQUENCE))
    // {
    //     HLC_THROTTLE = NEUTRAL_THROTTLE;
    //     seeking_lower_altitude = false;
    // }
   }

}

void YawToHeading(int current_heading, int target_heading)
{
  float error;
  
  error = HeadingError(current_heading, target_heading);
  HLC_YAW = NEUTRAL_YAW + ((error/180 * ADJUSTMENT_RANGE * YAW_AXIS_GAIN) * YAW_AXIS_SENSE);

 }

// Compute the difference between two headings ranging from 0 to 360 degrees.
// Return that difference as a value between -180.0 and 180.0
// A negative value means the current heading is behind (counter clockwise of) the target heading.
// A positive value means the current heading is ahead (clockwise of) of the target heading 
float HeadingError(float current_heading, float target_heading)
{
   float error = 0; 
    
    if (target_heading > current_heading)
    {
      if ((target_heading - current_heading) <= 180)
          error = -(target_heading - current_heading);
      else
          error = (360 - (target_heading - current_heading));
    } else if (target_heading < current_heading) {
      if ((current_heading - target_heading) <= 180)
          error = (current_heading - target_heading);
      else
          error =  -(360 - (current_heading - target_heading));
    } 
    return error;
    
}
void FlyToWaypoint(SENSOR_DATA sensor_data, WAYPOINT target)
{
   // Allow travel up to full speed if the waypoint is > 30 meters away.
   // If closer than that, reduce the maximum speed to 25% - 50% of that full speed depending on distance to target.
   // This will let us approach the target more slowly
   if (abs(target.distance > 30)) // meters
   {   
     HLC_PITCH = NEUTRAL_PITCH + (ADJUSTMENT_RANGE * 0.5 * PITCH_AXIS_GAIN * PITCH_AXIS_SENSE);
   } else { // When within 30 meters of the target, approach at a slower speed.  The minimum speed will be 35% of full speed.  Maximum speed is 50% of full speed.
     // In this case, full speed is 50% of the Adjustment Range, so the % multipliers from each side will equal 50%.
     HLC_PITCH = NEUTRAL_PITCH + (ADJUSTMENT_RANGE * 0.35 * PITCH_AXIS_GAIN * PITCH_AXIS_SENSE) + (ADJUSTMENT_RANGE * 0.15 * PITCH_AXIS_GAIN * PITCH_AXIS_SENSE * (abs(target.distance / 30.0)));
   }

}

// Calculate the great circle distance between (la1,lo1) - (la2, lo2) in meters
float Calc_Great_Circle_Distance(float la1, float lo1, float la2, float lo2)
{
  float r1, r2;

  la1 = DegreesToRadians(la1);
  lo1 = DegreesToRadians(lo1);    
  la2 = DegreesToRadians(la2);
  lo2 = DegreesToRadians(lo2);

  r1 = sin((la2-la1)/2.0);
  r1 = r1 * r1;
  r2 = sin((lo2-lo1)/2.0);
  r2 = cos(la1) * (r2*r2);
  r2 = sqrt(r1 + (cos(la2) * r2));
  r1 = asin(r2) * 2.0;
  r1 = r1 * MEAN_RADIUS_EARTH_KM;
  r1 = r1 * 1000; // converted KM to meters

  return r1;
}


float Calc_Great_Circle_Bearing(float la1, float lo1, float la2, float lo2)
{
  float r1,r2,r3,r4;

  la1 = DegreesToRadians(la1);
  lo1 = DegreesToRadians(lo1);    
  la2 = DegreesToRadians(la2);
  lo2 = DegreesToRadians(lo2);

  r1 = lo2 - lo1;
  r2 = sin(r1) * cos(la2);
  r3 = cos(la1) * sin(la2);
  r4 = (sin(la1) * cos(la2)) * cos(r1);
  r1 = atan2(r2, (r3-r4));
  r1 = r1 * 180.0 / PI;
  if (r1 < 0.0)
    r1 = r1 + 360.0;
  
  return r1;
}

