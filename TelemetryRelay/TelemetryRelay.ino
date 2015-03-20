/*******************************************************************************

  Telemetry Relay
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
    
*******************************************************************************/
// In the original prototype, an ATMEGA 328P resides between the Intel(R) Edison and the XBEE radio.
// It shares the same UART as the Linux console.
// This ATMEGA is used to filter out the Linux console output when being sent at 115200 baud, which keeps it from putting
// the XBEE in a bad state.  The flight controller uses the XBEE radio at 57600.  The telemetry relay, simply forwards all
// characters that are valid, printable characters, to the radio.  Anything at the wrong baud rate is filtered out.
// A better solution is simply to reset the XBEE under program control (via GPIO) once Edison has finished booting.  
// But, time won out and it was faster to wire up this controller to do the job.

int i = 0;
char ch;

void setup() {
  // initialize serial:
  Serial.begin(57600);
  
  i = millis();
  // Delay 30s to allow Edison to startup fully.
  // Console output will cease at about 30 seconds. 
  // Console output is sent at 115200 baud.  The modem doesn't work great at 115200 baud, but 57600 works well.
  // Sending characters at 115200 baud from the console can confuse the modem and put it in a bad state.
  // So we wait until boot has completed.  And, filter out any characters that are undesirable to print.
 
  while (millis() < (i + 30000))
  {
    Serial.print("Telemetry online! Timer: ");
    Serial.print( ((i + 30000) - millis()) / 1000);
    Serial.println(" seconds remaining.");
    delay(1000);
  }
  
}

void loop() {
    if (Serial.available())
    {
      ch = Serial.read();
      if (((ch >= 32) && (ch <= 126)) || (ch == 10) || (ch == 13))
        Serial.write(ch);
    }
}
