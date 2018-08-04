#include <Romi32U4.h>
#include <Wire.h>
#include "debug.h"
#include "defaults.h"
#include "sound.h"
#include "motors.h"
#include "gyro.h"
#include "i2c_interface.h"
#include "pose_snapshotter.h"
#include "waypoint_navigator.h"

Sound               sound;
I2CInterface        i2c(&sound);
PoseSnapshotter     poseSnapshotter(&i2c);
Motors              motors(&sound);
Gyro                gyro(&sound, &motors, &poseSnapshotter);
WaypointNavigator   waypointNavigator(&sound, &motors, &gyro, &poseSnapshotter);

#ifdef __ENABLE_MAGNETOMETER__
Magnetometer        magnetometer(&sound, &motors);
#endif

/*****************************************************************************************************
 * Entrypoint
 *****************************************************************************************************/

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    pinMode(A4, INPUT);                 // use the ADC on A4 for ultrasonics

    if (USE_GYRO_FOR_HEADING || USE_GYRO_FOR_PIVOT_TURN)
    {
#ifdef __ENABLE_MAGNETOMETER__      
        magnetometer.calibrate();      // far less accurate than the gyro until we can use it for fusion
#endif
        gyro.calibrate();
    }    
}

void loop() 
{
   if (i2c.pollForCommands())
   {
      switch(i2c.cmdCtx.cmd)
      {
          case BotCmd::CalibrateMotors:
            motors.calibrate(2, i2c.cmdCtx.pivotTurnSpeed, &gyro);
            break;

          case BotCmd::ResetToOrigin:
            waypointNavigator.resetToOrigin();
            break;

          case BotCmd::SetPidParameters:
            waypointNavigator.setPIDParameters(i2c.cmdCtx.pidProportional, i2c.cmdCtx.pidIntegral, 0, i2c.cmdCtx.pidLoopIntervalMs);
            break;

          case BotCmd::TransitViaWaypoints:
            waypointNavigator.executeTransit(i2c.cmdCtx);
            break;
      }
   }

   delay(100);
}
 

