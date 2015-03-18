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
    
*******************************************************************************/
#include <Arduino.h>
#include <trace.h>
#include <interrupt.h>
#include <sys/stat.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

int fd=0;
char logfilename[64];
char renamed_filename[64];

void LogTelemetry(char *msg)
{
  if (fd != -1)
    write(fd, msg, strlen(msg));
}

void setDate(const char* dataStr)  // format like MMDDYY
{
  char buf[3] = {0};

  strncpy(buf, dataStr + 0, 2);
  unsigned short month = atoi(buf);

  strncpy(buf, dataStr + 2, 2);
  unsigned short day = atoi(buf);

  strncpy(buf, dataStr + 4, 2);
  unsigned short year = atoi(buf);

  time_t mytime = time(0);
  struct tm* tm_ptr = localtime(&mytime);

  if (tm_ptr)
  {
    tm_ptr->tm_mon  = month - 1;
    tm_ptr->tm_mday = day;
    tm_ptr->tm_year = year + (2000 - 1900);
//    tm_ptr->tm_hour = hour;
//    tm_ptr->tm_min = minute;
//    tm_ptr->tm_sec = second;
    const struct timeval tv = {mktime(tm_ptr), 0};
    settimeofday(&tv, 0);
  }
}

void InitTelemetry(void)
{
  strcpy(logfilename, "/home/root/flight_log_XXXXXX");
  fd = mkstemp(logfilename);
  LogTelemetry("Logfile created ... \n");
}

void RenameTelemetryFile(char *timestamp)
{
  sprintf(renamed_filename, "/home/root/flight_log_%s.txt", timestamp);
 
  if (rename(logfilename, renamed_filename) != 0)
  {
    Serial.print("Failed to rename file.");
  }
}
