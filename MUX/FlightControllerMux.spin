/*******************************************************************************

  MUX
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
{{ FlightControllerMux.spin }}
{{     

Cog usage:  TX digitizer Throttle, Pitch, Roll, Yaw, Mode = 5
            Flight Controller PWM output                  = 1
            High Level Controller Input Serial I/O        = 1
            Main Routine                                  = 1
            

        
}}
CON

        _clkmode        = xtal1 + pll16x
        _xinfreq        = 5_000_000
        stack_size      = 100
        TPM             = 80 'Ticks Per Microsecond

        AUTO_MODE       = 1
        MANUAL_MODE     = 2

        ROLL_INPUT_PIN     = 0
        PITCH_INPUT_PIN    = 2
        THROTTLE_INPUT_PIN = 4
        YAW_INPUT_PIN      = 6
        MODE_INPUT_PIN     = 8
        
        LLC_PITCH_OUT_PIN    = 10
        PPM_OUT_PIN          = 10
        LLC_ROLL_OUT_PIN     = 12
        LLC_THROTTLE_OUT_PIN = 14
        LLC_YAW_OUT_PIN      = 16
        LLC_CAMERA_OUT_PIN   = 18
        LLC_MODE_OUT_PIN     = 20
        
        HLC_RX_PIN           = 22
        HLC_TX_PIN           = 24

        
        HLC_BAUD_RATE  = 57_600

        'For standard output mode
        'AUTO_STABILIZE_PWM = 147_750
        
        'For PPM Mode
        AUTO_STABILIZE_PWM = 145_875 '145_250 to 146_500
        
                 
        TRIGGER_MANUAL_MODE_PWM = 100_000
        CAMERA_NEUTRAL_PWM = 120_000
        PITCH_NEUTRAL_PWM = 120_000
        ROLL_NEUTRAL_PWM = 120_000
        THROTTLE_NEUTRAL_PWM = 120_000
        YAW_NEUTRAL_PWM = 120_000 
                    
VAR
  long  StackPitch[stack_size]
  long  StackRoll[stack_size]
  long  StackThrottle[stack_size]
  long  StackYaw[stack_size]
  long  StackMode[stack_size]
  long  StackServo[stack_size]

  'The inputs from the TX
  long  TX_PITCH
  long  TX_ROLL
  long  TX_THROTTLE
  long  TX_YAW
  long  TX_MODE

  'The outputs we send to the Low Level Controller (LLC)
  long  LLC_PITCH
  long  LLC_ROLL
  long  LLC_THROTTLE
  long  LLC_YAW
  long  LLC_MODE
  long  LLC_CAMERA
  
  BYTE  HLC_RX_DATA[256]
  BYTE ch, ch_a, ch_b, ch_c, ch_chk
   
OBJ
        HLC_SERIAL     : "FullDuplexSerialExtended"
        DBG            : "FullDuplexSerialPlus"
        
                                        
PUB Main | exit_time, HLC_HEART_BEAT, TX_HEART_BEAT, count, HLC_VALUE

   'DBG.Start(31, 30, 0, 57600)

   
   cognew(DigitizeServo(PITCH_INPUT_PIN,    @TX_PITCH),    @StackPitch)
   cognew(DigitizeServo(ROLL_INPUT_PIN,     @TX_ROLL),     @StackRoll)
   cognew(DigitizeServo(THROTTLE_INPUT_PIN, @TX_THROTTLE), @StackThrottle)
   cognew(DigitizeServo(YAW_INPUT_PIN,      @TX_YAW),      @StackYaw)
   cognew(DigitizeServo(MODE_INPUT_PIN,     @TX_MODE),     @StackMode)

   'Initialize the cog that will create the UART for the HLC communications
   HLC_SERIAL.start(HLC_RX_PIN, HLC_TX_PIN, 0, HLC_BAUD_RATE)


  LLC_PITCH    := PITCH_NEUTRAL_PWM
  LLC_ROLL     := ROLL_NEUTRAL_PWM
  LLC_THROTTLE := THROTTLE_NEUTRAL_PWM
  LLC_YAW      := YAW_NEUTRAL_PWM
  LLC_MODE     := AUTO_STABILIZE_PWM
  LLC_CAMERA   := CAMERA_NEUTRAL_PWM

  'Choose either PPM or standard output modes.
  'There are no additional cogs left to run both at the same time.
  cognew(OutputServoDataPPM, @StackServo)
  'cognew(OutputServoData, @StackServo)

  'The DJI flight controller wants to see steady, neutral PWM signals at startup,
  'on the pitch, roll, and yaw inputs.  To insure that this happens, the MUX will output
  'neutral PWM data for the first seconds while everything stabilizes.
  WAITCNT((CLKFREQ * 5) + cnt)
  
  repeat
     
     if (TX_MODE > TRIGGER_MANUAL_MODE_PWM)
         LLC_PITCH := TX_PITCH
         LLC_ROLL  := TX_ROLL
         LLC_THROTTLE := TX_THROTTLE
         LLC_YAW   := TX_YAW
         LLC_CAMERA := CAMERA_NEUTRAL_PWM

'         DBG.str(string("PR:"))
'         DBG.dec(LLC_PITCH)
'         DBG.str(string(" "))
'         DBG.str(string("RR:"))
'         DBG.dec(LLC_ROLL)
'         DBG.str(string(" "))
'         DBG.str(string("TR:"))
'         DBG.dec(LLC_THROTTLE)
'         DBG.str(string(" "))
'         DBG.str(string("YR:"))
'         DBG.dec(LLC_YAW)
'         DBG.str(string(" ", 10, 13))



     else
     
 '        DBG.str(string("P:"))
 '        DBG.dec(LLC_PITCH)
 '        DBG.str(string(" "))
 '        DBG.str(string("R:"))
 '        DBG.dec(LLC_ROLL)
 '        DBG.str(string(" "))
 '        DBG.str(string("T:"))
 '        DBG.dec(LLC_THROTTLE)
 '        DBG.str(string(" "))
 '        DBG.str(string("Y:"))
 '        DBG.dec(LLC_YAW)
 '        DBG.str(string(" ", 10, 13))
         'Input looks like this sequence of 6 bytes for each message:
         '  "$" "P" HIGH_BYTE MIDDLE_BYTE LOW_BYTE "!"
         '  "$" "R" HIGH_BYTE MIDDLE_BYTE LOW_BYTE "!"
         '  "$" "T" HIGH_BYTE MIDDLE_BYTE LOW_BYTE "!"
         '  "$" "Y" HIGH_BYTE MIDDLE_BYTE LOW_BYTE "!"
         '  "$" "C" HIGH_BYTE MIDDLE_BYTE LOW_BYTE "!"
         '
         ch := 0
         count := 0
         repeat until (ch == "$") OR (count == 100)
             ch := HLC_SERIAL.RXTIME(10)
             count++

         'Look for the start of a valid message, signaled by the "$".
         'If we find the "$", read the next 4 bytes to get the high data byte, low data byte, and a sync character "!"    
         if (ch == "$")
             ch   := HLC_SERIAL.RXTIME(10)
             ch_a := HLC_SERIAL.RXTIME(10)
             ch_b := HLC_SERIAL.RXTIME(10)
             ch_c := HLC_SERIAL.RXTIME(10)
             ch_chk := HLC_SERIAL.RXTIME(10)
             HLC_VALUE := (ch_a << 16) + (ch_b << 8) + ch_c
             if (ch_chk == "!")
                case ch
                   "P":
                         LLC_PITCH := HLC_VALUE
                   "R":
                         LLC_ROLL := HLC_VALUE
                   "T":
                         LLC_THROTTLE := HLC_VALUE
                   "Y":
                         LLC_YAW := HLC_VALUE
                   "C":
                         LLC_CAMERA := HLC_VALUE
                  
PUB OutputServoData | exit_time

  dira[LLC_PITCH_OUT_PIN] ~~
  dira[LLC_ROLL_OUT_PIN] ~~
  dira[LLC_THROTTLE_OUT_PIN] ~~
  dira[LLC_YAW_OUT_PIN] ~~
  dira[LLC_MODE_OUT_PIN] ~~
  dira[LLC_CAMERA_OUT_PIN] ~~

    repeat
      'Repeat the pulse train every 20ms
      'The Servo_Out routine will take up to ~2ms * 6channels to complete (~12ms) worst case.
      'Pad the remainder of the time to achieve a 20ms frame time.
      exit_time := (clkfreq / 1000 * 20) + cnt
      Servo_Out(LLC_PITCH, LLC_ROLL, LLC_THROTTLE, LLC_YAW, LLC_MODE, LLC_CAMERA)
      WAITCNT(exit_time)

PUB OutputServoDataPPM | exit_time

  dira[PPM_OUT_PIN] ~~
   
   repeat
      'The PPM format uses a sync pulse at the end of the frame that pads the frame to 20ms.
      'No need to check a start and exit time, because we already know how long this routine takes.
      Servo_Out_PPM(LLC_PITCH, LLC_ROLL, LLC_THROTTLE, LLC_YAW, LLC_MODE, LLC_CAMERA)

PUB DigitizeServo(Pin, OutputAddress) | start_time, end_time
  WaitPEQ(0, |< Pin, 0) 'Wait for the pin to go low.
  repeat
    WaitPEQ(|< Pin, |< Pin, 0) 'Wait for the pin to go high
    start_time := cnt
    WaitPEQ(0, |< Pin, 0) 'Wait for the pin to go low
    end_time := cnt
    'Store the elapsed time in the memory buffer pointed to by OutputAddress
    LONG[OutputAddress] := end_time - start_time
    
PUB Servo_Out(pos1, pos2, pos3, pos4, pos5, pos6)


 'PITCH
  if (pos1 > 160_000)
    pos1 := 160_000
  if (pos1 < 80_000)
    pos1 := 80_000
  outa[LLC_PITCH_OUT_PIN] := 1
  waitcnt(pos1 + cnt)
  outa[LLC_PITCH_OUT_PIN] := 0

  'ROLL
  if (pos2 > 160_000)
    pos2 := 160_000
  if (pos2 < 80_000)
    pos2 := 80_000
  outa[LLC_ROLL_OUT_PIN] := 1
  waitcnt(pos2 + cnt)
  outa[LLC_ROLL_OUT_PIN] := 0


  'THROTTLE
  if (pos3 > 160_000)
    pos3 := 160_000
  if (pos3 < 80_000)
    pos3 := 80_000
  outa[LLC_THROTTLE_OUT_PIN] := 1
  waitcnt(pos3 + cnt)
  outa[LLC_THROTTLE_OUT_PIN] := 0

  'YAW
  if (pos4 > 160_000)
    pos4 := 160_000
  if (pos4 < 80_000)
    pos4 := 80_000
  outa[LLC_YAW_OUT_PIN] := 1
  waitcnt(pos4 + cnt)
  outa[LLC_YAW_OUT_PIN] := 0

  'MODE 
  if (pos5 > 160_000)
    pos5 := 160_000
  if (pos5 < 80_000)
    pos5 := 80_000
  outa[LLC_MODE_OUT_PIN] := 1
  waitcnt(pos5 + cnt)
  outa[LLC_MODE_OUT_PIN] := 0

  'X1 (CAMERA)
  if (pos6 > 160_000)
    pos6 := 160_000
  if (pos6 < 80_000)
    pos6 := 80_000
  outa[LLC_CAMERA_OUT_PIN] := 1
  waitcnt(pos6 + cnt)
  outa[LLC_CAMERA_OUT_PIN] := 0
    
  return

PUB Servo_Out_PPM(pos1, pos2, pos3, pos4, pos5, pos6) | pos7, sync, start_time
  start_time := cnt

  'gap := 80' There are 80 ticks/uS

  'ROLL
  if (pos2 > 160_000)
    pos2 := 160_000
  if (pos2 < 80_000)
    pos2 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos2 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)


  'PITCH
  if (pos1 > 160_000)
    pos1 := 160_000
  if (pos1 < 80_000)
    pos1 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos1 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)

  'THROTTLE
  if (pos3 > 160_000)
    pos3 := 160_000
  if (pos3 < 80_000)
    pos3 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos3 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)

  'YAW
  if (pos4 > 160_000)
    pos4 := 160_000
  if (pos4 < 80_000)
    pos4 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos4 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)

  'X1 (CAMERA)
  if (pos6 > 160_000)
    pos6 := 160_000
  if (pos6 < 80_000)
    pos6 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos6 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)

  'X2 (UNUSED)
  pos7 := 120_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos7 + cnt)
  outa[PPM_OUT_PIN] := 0

  'waitcnt(gap + cnt)

  'MODE 
  if (pos5 > 160_000)
    pos5 := 160_000
  if (pos5 < 80_000)
    pos5 := 80_000
  outa[PPM_OUT_PIN] := 1
  waitcnt(pos5 + cnt)
  outa[PPM_OUT_PIN] := 0
  
  'waitcnt(gap + cnt)

  'Generate the SYNC pulse which occupies the rest of the 20ms frame.
  sync := 1_600_000 - (cnt - start_time)
  outa[PPM_OUT_PIN] := 1
  waitcnt(sync + cnt)
  outa[PPM_OUT_PIN] := 0
  
  return
               