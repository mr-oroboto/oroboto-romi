#include <Romi32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "QMC5883L.h"

#define PID_PROPORTIONAL      50.00             // was 0.9
#define PID_INTEGRAL          0.001             // was 0.0005
#define PID_DERIVATIVE        0.00
#define PID_LOOP_INTERVAL_MS  50

#define MAX_VELOCITY         5                  // cm/s (slowest known good max velocity is "5")
#define VELOCITY_ON_APPROACH 1.0
#define MIN_SPEED            20                 // motors stall below this point
#define MAX_SPEED            300                // on-floor tests show 100 =~ 26.13cm/s, 200 =~ 44.24 cm/s, 300 =~ 65.49cm/s
#define MAX_SPEED_CM_PER_SEC 65.49

#define WHEELBASE        14.9225                // cm
#define BASERADIUS       7.46125                // cm
#define WHEELRADIUS      3.4925                 // cm (was 3.65125)

#define PIVOT_TURN true                         // pivot turn instead of PID mode for large heading corrections
#define PIVOT_TURN_SPEED 30                     // known good speed is 30
#define PIVOT_TURN_THRESHOLD 12.0               // use pivot if heading correction > 2*PI / PIVOT_TURN_THRESHOLD
#define PIVOT_TURN_SLEEP_MS 200                 // known good sleep is 500

#define CAP_ANGULAR_VELOCITY_SIGNAL true        // cap angular velocity correction allowed in PID mode
#define ANGULAR_VELOCITY_SIGNAL_LIMIT 0.2

#define ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES false   // true
#define DISTANCE_INCREASE_ITERATION_THRESHOLD 1
#define DISTANCE_INCREASE_DRIFT_FROM_INITIAL_VECTOR_THRESHOLD 0.05    // used when ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES is false

#define WAYPOINT_PROXIMITY_APPROACHING 5.0      // was 10.0
#define WAYPOINT_PROXIMITY_REACHED 2.0          // was 3.0
#define POST_WAYPOINT_SLEEP_MS 300              // known good sleep is 1000

#define PERIODIC_REFERENCE_HEADING_RESET true   // should we periodically recalculate required reference heading during waypoint finding?
#define PERIODIC_REFERENCE_HEADING_INTERVAL 5   // reset every n iterations of the PID loop (10 is known good)

#define COUNTS_PER_REVOLUTION 1440.0            // number of encoder counts per wheel revolution (should be ~1440)

#define USE_GYRO_FOR_HEADING false
#define USE_GYRO_FOR_PIVOT_TURN true
#define GYRODIGITS_TO_DPS 0.00875

#define I2C_PI_ADDR 0b10011                     // 0x0A
#define I2C_CMD_GETWAYPOINTS 'p'                // ping!
#define I2C_CMD_REPORTSNAPSHOTS 'r'       

#define I2C_WAYPOINTS_MAX 16                    // max waypoints that can be sent per I2C_CMD_GETWAYPOINTS
#define I2C_WAYPOINT_SIZE 4                     // 2 bytes each (signed short) for x and y co-ordinates 
#define I2C_WAYPOINTS_PER_SEGMENT 2             // number of waypoints that can be sent per segment for I2C_CMD_GETWAYPOINTS (don't exceed 16 byte max segment size)

#define I2C_SNAPSHOT_SIZE 11                      // 2 bytes (signed short) for x, y, 2 bytes for heading, 2 bytes (unsigned short) for distance, 2 bytes (unsigned short) for timestamp, 1 byte for extra detail (ie. aborted due to obstacle)
#define I2C_SNAPSHOTS_PER_SEGMENT 1               // number of snapshots that can be sent per segment for I2C_CMD_REPORTSNAPSHOTS (don't exceed 16 byte max segment size)
#define I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY 0x01      // reported on the last snapshot of the last waypoint as long as that waypoint was not interrupted by an obstacle: if we never reach the last waypoint this will never be sent
#define I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT 0x02     // reported on the last snapshot of every waypoint or obstacle avoidance maneuver
#define I2C_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT  0x04              // reported on the last snapshot if the current waypoint or obstacle avoidance maneuver itself was aborted due to an obstacle

#define I2C_MARKER_SEGMENT_START 0xA0
#define I2C_MARKER_SEGMENT_END 0xA1
#define I2C_MARKER_PAYLOAD_START 0xA2
#define I2C_MARKER_PAYLOAD_END 0xA3

#define OPTION1_ENABLE_RANGING 0x01
#define OPTION1_ENABLE_ABORT_AFTER_DISTANCE_INCREASES 0x02
#define OPTION1_ENABLE_CAP_ANGULAR_VELOCITY_SIGNAL 0x04
#define OPTION1_ENABLE_PERIODIC_REFERENCE_HEADING_RESET 0x08
#define OPTION1_OVERRIDE_CALIBRATE_MOTORS 0xB0
#define OPTION1_OVERRIDE_RESET_TO_ORIGIN  0xB1
#define OPTION1_OVERRIDE_SET_PID_PARAMETERS 0xB2

#define VCC 5.0
#define VOLTS_PER_CM_DIVISOR 1300.48              // 512.0 * 2.54 (from datasheet)
#define VOLTS_PER_CM VCC / VOLTS_PER_CM_DIVISOR   // 0.003844734251969
#define ADC_LSB_PER_VOLT 1023 / VCC               // 1023 / 5.0
#define CLOSE_PROXIMITY_THRESHOLD 17.0
#define MAX_CONSECUTIVE_CLOSE_PROXIMITY_READINGS 2
#define OBSTACLE_AVOIDANCE_DISTANCE 30.0

struct Pose {
  double        x;
  double        y;
  double        heading;
  double        distanceToObstacle;
  uint16_t      timestamp;  
};

Romi32U4Encoders  encoders;
Romi32U4Buzzer    buzzer;
Romi32U4Motors    motors;
LSM6              imu;
QMC5883L          compass;

int16_t waypointPayload[I2C_WAYPOINTS_MAX * 2]; // all the waypoints received via an I2C_CMD_GETWAYPOINTS are stored here
bool    waypointPayloadInProgress = false;
uint8_t waypointPayloadCurrentCount = 0;
uint8_t waypointPayloadExpectedCount = 0;
uint8_t checksum, checksumComputed;

uint8_t maxVelocity = MAX_VELOCITY;
uint8_t pivotTurnSpeed = PIVOT_TURN_SPEED;
uint8_t optionByte1, optionByte2;
bool    enableRanging = true;
bool    abortAfterDistanceToWaypointIncreases = ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES;

struct Pose currentPose;                        // (believed) current position and heading
struct Pose referencePose;                      // pose of reference waypoint (heading is heading required from currentPose)
uint32_t    waypointStartTime;

#define MAX_POSE_SNAPSHOTS 48
struct Pose poseSnapshots[MAX_POSE_SNAPSHOTS];  // capture snapshots of our pose along each waypoint path for reporting
int         poseSnapshotCount;

float     pidProportional    = PID_PROPORTIONAL;
float     pidIntegral        = PID_INTEGRAL;
bool      capAngularVelocity = CAP_ANGULAR_VELOCITY_SIGNAL;
bool      periodicReferenceHeadingReset = PERIODIC_REFERENCE_HEADING_RESET;
uint16_t  pidLoopIntervalMs  = PID_LOOP_INTERVAL_MS;

double headingError;              // for PID proportional term
double headingErrorPrev;          // for PID derivative term
double headingErrorIntegral;      // for PID integral term

double distLeft;                  // current distance travelled by left wheel in this waypoint segment
double distLeftPrev;              // previous current distance travelled by left wheel in this waypoint segment
double distRight;
double distRightPrev;
double distTotal;                 // total distance travelled in this waypoint segment
double distTotalPrev;             // previous total distance travelled in this waypoint segment

uint8_t consecutiveCloseProximityReadings;

int16_t       gyroOffset = 0;           // average reading on gyro Z axis during calibration
unsigned long gyroLastUpdateMicros = 0; // helps calculate dt for gyro readings (in microseconds)
double        gyroAngle;
double        gyroAngleRad;
double        gyroAngleDifference = 0;

#ifdef ENABLE_MAGNETOMETER
double        magnetoAngle;
double        magnetoAngleRad;
#endif

float * motorCalibrationRightToLeftRatio = NULL;
int16_t motorCalibrationBuckets[] = {30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240};     // Motor speed range is -300 to 300 (reverse to forward)
uint8_t motorCalibrationAdjustedBuckets = 0;
uint8_t motorCalibrationBucketCount = sizeof(motorCalibrationBuckets) / sizeof(int16_t);

#ifdef __DEBUG__                    
char report[80];
char floatBuf1[16], floatBuf2[16], floatBuf3[16], floatBuf4[16], floatBuf5[16];
#endif

#define FREQUENCY_DURATION 300
#define FREQUENCY_VOLUME 9
#define FREQUENCY_MODE_ABORTED_WAYPOINT 800
#define FREQUENCY_MODE_ALARM 200

const char soundFinished[] PROGMEM = "! L16 V8 cdefgab>cbagfedc";
const char soundOk[] PROGMEM = "v10>>g16>>>c16";

/*****************************************************************************************************
 * Entrypoint
 *****************************************************************************************************/

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    pinMode(A4, INPUT);               // use the ADC on A4

    if (USE_GYRO_FOR_HEADING || USE_GYRO_FOR_PIVOT_TURN)
    {
#ifdef ENABLE_MAGNETOMETER      
        calibrateMagnetometer();      // far less accurate than the gyro until we can use it for fusion
#endif
        calibrateGyro();
    }    
}

void loop() 
{
   if (pollForWaypoints())
   {
#ifdef __DEBUG__
      snprintf_P(report, sizeof(report), PSTR("START: range[%d] abort[%d] cap[%d] refReset[%d] P[%s] I[%s] dt[%d]"), enableRanging, abortAfterDistanceToWaypointIncreases, capAngularVelocity, periodicReferenceHeadingReset, ftoa(floatBuf1, pidProportional), ftoa(floatBuf2, pidIntegral), pidLoopIntervalMs);    
      Serial.println(report);
#endif           

      ledYellow(1);
      
      for (int8_t i = 0; i < waypointPayloadCurrentCount; i++)
      {
           bool abortedDueToObstacle = goToWaypoint((double)waypointPayload[(i*2)], (double)waypointPayload[(i*2)+1]);
           reportPoseSnapshots(abortedDueToObstacle, i == (waypointPayloadCurrentCount - 1));

           if (abortedDueToObstacle)
           {
              uint8_t attempts = 0;

              while (attempts < 7 && abortedDueToObstacle)
              {
                 // Rotate 45 degrees clockwise and try again
                 double avoidanceHeading = atan2(sin(currentPose.heading - (M_PI / 4)), cos(currentPose.heading - (M_PI / 4)));
                 double avoidanceWaypointX = currentPose.x + (OBSTACLE_AVOIDANCE_DISTANCE * cos(avoidanceHeading));
                 double avoidanceWaypointY = currentPose.y + (OBSTACLE_AVOIDANCE_DISTANCE * sin(avoidanceHeading));

#ifdef __DEBUG__
                 snprintf_P(report, sizeof(report), PSTR("Collision avoidance attempt[%d], target (%s,%s)"), attempts, ftoa(floatBuf1, avoidanceWaypointX), ftoa(floatBuf2, avoidanceWaypointY));    
                 Serial.println(report);
#endif

                 abortedDueToObstacle = goToWaypoint(avoidanceWaypointX, avoidanceWaypointY);   
                 reportPoseSnapshots(abortedDueToObstacle, false);    // last waypoint of journey can never be an obstacle avoidance waypoint

                 attempts++;
              }

#ifdef __DEBUG__
              snprintf_P(report, sizeof(report), PSTR("Collision avoidance finished %s after %d attempts"), abortedDueToObstacle ? "unsuccessfully" : "successfully", attempts);    
              Serial.println(report);
#endif

              if (attempts == 7 && abortedDueToObstacle)
              {
                 buzzer.playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
                 delay(FREQUENCY_DURATION);
                 buzzer.playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
                 delay(FREQUENCY_DURATION);

                 break;   // we're blocked on all sides!
              }
              else
              {
                 i--;     // try the current waypoint again
              }
           }
      }

      buzzer.playFromProgramSpace(soundFinished);
      delay(2000);

      ledYellow(0);
   }
   
   delay(100);
}

/*****************************************************************************************************
 * End entrypoint
 *****************************************************************************************************/


/**
 * Go to the specified waypoint from the current waypoint.
 */
bool goToWaypoint(double x, double y) 
{
    double    targetVectorMagnitude;                    // current distance between where we think we are and the waypoint
    double    targetVectorMagnitudeLast = 0.0;          // last distance between where we think we were and the waypoint
    double    targetVectorMagnitudeInitial = 0.0;       // initial distance between where we think were are the waypoint
    double    targetVectorMagnitudeAtStartOfDrift;      // if we begin to get further from target (rather than closer), what was our distance to the target when this started?
    uint8_t   consecutiveIncreasingDistances = 0;
    bool      approachingTarget = false;                // once we start getting close to waypoint, don't forget that
    bool      abortedDueToObstacle = false;

    double    velocityLeft = 0.0, velocityRight = 0.0;  // current velocity of left and right wheels

    resetDistancesForNewWaypoint();

    // @todo Should we really throw away PID error from the previous waypoint here?
    headingErrorPrev = 0.0;
    headingErrorIntegral = 0.0;

#ifdef __DEBUG__
    snprintf_P(report, sizeof(report), PSTR("************ WAYPOINT (%s,%s) from (%s,%s head [%s]) ************"), ftoa(floatBuf1, x), ftoa(floatBuf2, y), ftoa(floatBuf3, currentPose.x), ftoa(floatBuf4, currentPose.y), ftoa(floatBuf5, currentPose.heading));    
    Serial.println(report);      
#endif

    referencePose.x = x;
    referencePose.y = y;

    // We need to keep our heading as close to this reference heading as possible to reach the waypoint
    referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);

    uint32_t      iteration = 0;
    double        dt = pidLoopIntervalMs / 1000.0;
    unsigned long lastMillis = 0;
    int           lastLeftSpeed = 0, lastRightSpeed = 0;

    // Reset wheel encoder counts to zero, we don't care about their current values
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    
    while (true)
    {
        if (iteration != 0)
        {
            dt = (millis() - lastMillis) / 1000.0;

            // Recalculate reference heading every now and then as our current position changes
            if (periodicReferenceHeadingReset && (iteration % PERIODIC_REFERENCE_HEADING_INTERVAL == 0))
            {
                referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);

#ifdef __DEBUG__                
                snprintf_P(report, sizeof(report), PSTR("corrected reference heading to [%s])"), ftoa(floatBuf1, referencePose.heading));    
                Serial.println(report);      
#endif                
            }
        }

#ifdef __DEBUG__
//      snprintf_P(report, sizeof(report), PSTR("[%3d] to (%s, %s, head [%s]) dist: %s"), iteration, ftoa(floatBuf1, referencePose.x), ftoa(floatBuf2, referencePose.y), ftoa(floatBuf3, referencePose.heading), ftoa(floatBuf4, targetVectorMagnitudeInitial)); 
//      Serial.println(report);      
//      snprintf_P(report, sizeof(report), PSTR("   was (%s, %s, head [%s]) distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, targetVectorMagnitudeLast));    
//      Serial.println(report);      
#endif

        int16_t countsLeft  = encoders.getCountsAndResetLeft();         // 1440 per revolution = 2*PI*radius cm travel
        int16_t countsRight = encoders.getCountsAndResetRight();
        
        if (encoders.checkErrorLeft())
        {
            buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
        }
        if (encoders.checkErrorRight())
        {
            buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
        }

        if (iteration != 0)
        {
            //
            // Update heading (using odometry and/or gyroscope if this isn't the first iteration of the waypoint (ie. we've actually travelled somewhere)
            //

            if (USE_GYRO_FOR_HEADING)
            {
               updateGyroscopeHeading();
            }
            
            distLeft  += ((double)countsLeft / COUNTS_PER_REVOLUTION)  * (2*M_PI * WHEELRADIUS);        // cm travelled by left wheel (total, note the increment)
            distRight += ((double)countsRight / COUNTS_PER_REVOLUTION) * (2*M_PI * WHEELRADIUS);        // cm travelled by right wheel (total, note the increment)
            distTotal = (distLeft + distRight) / 2.0;                                                   // cm travelled forward (total)

#ifdef __DEBUG__
            snprintf_P(report, sizeof(report), PSTR("travelled L[%s] R[%s] dist[%s] distPrev[%s] dt[%s] [%d]"), ftoa(floatBuf1, distLeft), ftoa(floatBuf2, distRight), ftoa(floatBuf3, distTotal), ftoa(floatBuf4, distTotalPrev), ftoa(floatBuf5, dt), iteration); 
            Serial.println(report);      
#endif

            // How far have we travelled in this last iteration?
            double distDelta      = distTotal - distTotalPrev;
            double distLeftDelta  = distLeft - distLeftPrev;
            double distRightDelta = distRight - distRightPrev;

            currentPose.x += distDelta * cos(currentPose.heading);
            currentPose.y += distDelta * sin(currentPose.heading);

            // Update the heading as it has changed based on the distance travelled too
            currentPose.heading += ((distRightDelta - distLeftDelta) / WHEELBASE);

            if (USE_GYRO_FOR_HEADING)
            {
                currentPose.heading = gyroAngleRad;
            }

            // Ensure our heading remains sane (between -pi and +pi)
            currentPose.heading = atan2(sin(currentPose.heading), cos(currentPose.heading));
            
            distTotalPrev = distTotal;
            distLeftPrev  = distLeft;
            distRightPrev = distRight;
        }
        
        // What's the error between our required heading and our heading?
        double headingErrorRaw = referencePose.heading - currentPose.heading;
        headingError = atan2(sin(headingErrorRaw), cos(headingErrorRaw));

#ifdef __DEBUG__
        if (USE_GYRO_FOR_HEADING)
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] gyro[%s rad] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, gyroAngleRad), ftoa(floatBuf3, referencePose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, headingErrorRaw));          
        }
        else
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, referencePose.heading), ftoa(floatBuf3, headingError), ftoa(floatBuf4, headingErrorRaw));
        }
        Serial.println(report);
#endif

        if (PIVOT_TURN && ((headingError > ((2*M_PI) / PIVOT_TURN_THRESHOLD)) || (headingError < -((2*M_PI) / PIVOT_TURN_THRESHOLD))))
        {
            if (USE_GYRO_FOR_PIVOT_TURN)
            {
                correctHeadingWithPivotTurnGyro(headingError);
            }
            else
            {
                correctHeadingWithPivotTurn(headingError);              
            }

            // If using gyroscope, assume we got to whatever heading it is currently measuring, rather than what we asked for
            if ( ! USE_GYRO_FOR_HEADING)
            {
                // @todo We SHOULD set the current heading based on the arc length ACTUALLY travelled rather than that requested (because we can overshoot)
                currentPose.heading = referencePose.heading;
            }
            
            headingError = 0.0;   // @todo: if using the gyro we should update this for the current error NOW that we've spun via the gyro (and its newly reported heading)

            // Reset counters as if we'd never done this since last checking them
            encoders.getCountsAndResetLeft();
            encoders.getCountsAndResetRight();

            // @todo Should we really throw away PID error from the current waypoint here?
            headingErrorPrev = 0.0;
            headingErrorIntegral = 0.0;
        }

        if ((iteration % 8 == 0) && (poseSnapshotCount < MAX_POSE_SNAPSHOTS))   // don't poll for distance more often than every 400ms (loop has delay ~50ms)
        {
            recordSnapshot(currentPose.heading, currentPose.x, currentPose.y);

            if (enableRanging && consecutiveCloseProximityReadings >= MAX_CONSECUTIVE_CLOSE_PROXIMITY_READINGS)
            {
               motors.setSpeeds(0, 0);
#ifdef __DEBUG__
               Serial.println("   !!! ABORT: OBSTACLE DETECTED");      
#endif
               buzzer.playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
               delay(FREQUENCY_DURATION);

               abortedDueToObstacle = true;
               
               break;
            }
        }

        // What is the magnitude of the vector between us and our target?
        targetVectorMagnitude = sqrt(sq(referencePose.x - currentPose.x) + sq(referencePose.y - currentPose.y));
        if (iteration == 0)
        {
            targetVectorMagnitudeInitial = targetVectorMagnitude;        
        }

#ifdef __DEBUG__
        snprintf_P(report, sizeof(report), PSTR("   now (%s,%s) head [%s] err[%s] distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, targetVectorMagnitude));    
        Serial.println(report);      
#endif

        // Give up out of out-of-control situations
        if (iteration != 0)
        {
            if (targetVectorMagnitude >= targetVectorMagnitudeLast)
            {
                bool abortWaypoint = false;
                
                if (consecutiveIncreasingDistances == 0)
                {
                    targetVectorMagnitudeAtStartOfDrift = targetVectorMagnitude;
                }

                consecutiveIncreasingDistances++;

                if (abortAfterDistanceToWaypointIncreases && consecutiveIncreasingDistances >= DISTANCE_INCREASE_ITERATION_THRESHOLD)
                {
                    abortWaypoint = true;
                }
                else
                {
                    double distanceCoveredSinceStartOfDrift = targetVectorMagnitude - targetVectorMagnitudeAtStartOfDrift;
                    if (distanceCoveredSinceStartOfDrift >= (DISTANCE_INCREASE_DRIFT_FROM_INITIAL_VECTOR_THRESHOLD * targetVectorMagnitudeInitial))
                    {
                        abortWaypoint = true;
                    }
                }

                if (abortWaypoint)
                {
                    motors.setSpeeds(0, 0);

#ifdef __DEBUG__
                    Serial.println("   !!! ABORT: DISTANCE TO TARGET INCREASED");      
#endif

                    buzzer.playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
                    delay(FREQUENCY_DURATION);
                    break;                  
                }
            }
            else
            {
                consecutiveIncreasingDistances = 0;
            }
        }
        
        double forwardVelocity = maxVelocity;

        if (targetVectorMagnitude < WAYPOINT_PROXIMITY_APPROACHING || approachingTarget)
        {
            approachingTarget = true;

#ifdef __DEBUG__
            Serial.println("   >>> slowing...");      
#endif

            forwardVelocity = VELOCITY_ON_APPROACH;
        }

        if (targetVectorMagnitude <= WAYPOINT_PROXIMITY_REACHED)
        {
#ifdef __DEBUG__          
            Serial.println("   >>> WAYPOINT REACHED <<<");      
#endif            
            break;
        }

        targetVectorMagnitudeLast = targetVectorMagnitude;

        // Maintain the PID variables
        double headingErrorDerivative  = (headingError - headingErrorPrev) / dt;
        headingErrorIntegral          += (headingError * dt);
        headingErrorPrev               = headingError;

        double pidP = pidProportional * headingError;
        double pidI = pidIntegral * headingErrorIntegral;
        double pidD = PID_DERIVATIVE * headingErrorDerivative;

        // PID, this gives us the control signal, u, this is our required angular velocity to achieve our desired heading
        double u = pidP + pidI + pidD;

        if (capAngularVelocity)
        {
            if (u > ANGULAR_VELOCITY_SIGNAL_LIMIT)
            {
                u = ANGULAR_VELOCITY_SIGNAL_LIMIT;

#ifdef __DEBUG__                
                Serial.println("   !!! Capped angular velocity signal");      
#endif                
            }
            else if (u < (-1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT))
            {
                u = -1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT;

#ifdef __DEBUG__                
                Serial.println("   !!! Capped angular velocity signal");                      
#endif                
            }
        }

        // Angular velocity gives us new wheel velocities. We assume a constant forward velocity for simplicity.
        velocityRight = ((2.0 * forwardVelocity) + (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s
        velocityLeft  = ((2.0 * forwardVelocity) - (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s

        int leftSpeed  = convertVelocityToMotorSpeed(velocityLeft);
        int rightSpeed = convertVelocityToMotorSpeed(velocityRight); 

        if (approachingTarget)
        {
           // Avoid low speed oscillations that can occur and stall at low speed
           if (leftSpeed == lastRightSpeed && rightSpeed == lastLeftSpeed)
           {
              motors.setSpeeds(0, 0);
#ifdef __DEBUG__
              Serial.println("   !!! Stalled at low speed, aborting!"); 
#endif
              buzzer.playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
              delay(FREQUENCY_DURATION);
              break;                                
           }
        }

        lastLeftSpeed = leftSpeed;
        lastRightSpeed = rightSpeed;

        leftSpeed = getCalibratedLeftMotorSpeed(leftSpeed);

#ifdef __DEBUG__
//      snprintf_P(report, sizeof(report), PSTR("   P[%s] I[%s] D[%s] u[%s] -> required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, pidP), ftoa(floatBuf2, pidI), ftoa(floatBuf3, pidD), ftoa(floatBuf4, u), ftoa(floatBuf5, velocityLeft), leftSpeed, ftoa(floatBuf6, velocityRight), rightSpeed);    
        snprintf_P(report, sizeof(report), PSTR("   u[%s] required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, u), ftoa(floatBuf2, velocityLeft), leftSpeed, ftoa(floatBuf3, velocityRight), rightSpeed);    
        Serial.println(report);      
#endif

        motors.setSpeeds(leftSpeed, rightSpeed);

        iteration++;

        lastMillis = millis();
        delay(pidLoopIntervalMs);

        // Time passes, wheels respond to new control signal and begin moving at new velocities
    }

    motors.setSpeeds(0, 0);
    delay(POST_WAYPOINT_SLEEP_MS);

    return abortedDueToObstacle;
}


/****************************************************************************************************************
 * HELPERS 
 ****************************************************************************************************************/

/**
 * Poll the Raspberry Pi (acting as an I2C slave) for updated waypoints
 */
bool pollForWaypoints()
{
   bool receivedFullPayload = false;
   
   Wire.beginTransmission(I2C_PI_ADDR);
   Wire.write(I2C_CMD_GETWAYPOINTS);
   Wire.endTransmission();

   // The Pi's BSC FIFO is 16 bytes deep, keep the maximum transmit segment below that (10 bytes)
   Wire.requestFrom((uint8_t)I2C_PI_ADDR, (uint8_t)(I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE) + 2);  // 4 bytes per waypoint (x,y) + start and end markers
   uint8_t startMarker, endMarker, i, bufferOffset = 0, maxWaypointsPerSegment = I2C_WAYPOINTS_PER_SEGMENT;
   uint8_t waypointsBuffer[I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE];

   startMarker = Wire.read();
   for (i = 0; i < (I2C_WAYPOINTS_PER_SEGMENT * I2C_WAYPOINT_SIZE); i++)
   {
      waypointsBuffer[i] = Wire.read();
   }
   endMarker = Wire.read();
   Wire.endTransmission();  

   if (startMarker == I2C_MARKER_SEGMENT_START && endMarker == I2C_MARKER_SEGMENT_END)
   {
#ifdef __DEBUG__    
      Serial.println("Received valid I2C segment");
#endif      
      buzzer.playFromProgramSpace(soundOk);
      delay(500);

      // Valid payload, either part of an existing waypoint payload, or the start of a new one
      if (waypointsBuffer[0] == I2C_MARKER_PAYLOAD_START && waypointsBuffer[1] == I2C_MARKER_PAYLOAD_START)
      {
         ledRed(1);

#ifdef __DEBUG__
         Serial.println(" - Segment starts new waypoint payload");
#endif

         if (waypointsBuffer[2] == 0 || waypointsBuffer[2] > I2C_WAYPOINTS_MAX)
         {
#ifdef __DEBUG__          
            Serial.println(" - ERROR: mismatched payload count indicator");
#endif            
            ledRed(0);
         }
         else
         {
            waypointPayloadExpectedCount = waypointsBuffer[2]; 
            maxVelocity = waypointsBuffer[3];
            pivotTurnSpeed = waypointsBuffer[4];
            optionByte1 = waypointsBuffer[5];
            optionByte2 = waypointsBuffer[6];
            checksum = waypointsBuffer[7];
            checksumComputed = waypointPayloadExpectedCount + maxVelocity + pivotTurnSpeed + optionByte1 + optionByte2;

            waypointPayloadCurrentCount = 0;
            waypointPayloadInProgress = true;

#ifdef __DEBUG__
            snprintf_P(report, sizeof(report), PSTR(" - Expect %d waypoints (maxVelocity:%d, pivotTurnSpeed:%d, o1:%x, o2:%x, chk:%x)"), waypointPayloadExpectedCount, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2, checksum);      
            Serial.println(report);  
#endif

            memset(waypointPayload, 0, sizeof(waypointPayload));

            return false;  // expecting another segment with the first (and next) waypoints
         }
      }
      else
      {
         if ( ! waypointPayloadInProgress)
         {
#ifdef __DEBUG__
            Serial.println(" - Segment is a payload extension but no payload build is in progress, ignoring");
#endif            
            ledRed(0);
            
            return false;
         }
#ifdef __DEBUG__          
         else
         {
            Serial.println(" - Segment is a payload extension");
         }
#endif            
      }

      for (i = 0; i < maxWaypointsPerSegment && waypointPayloadCurrentCount < waypointPayloadExpectedCount; i++, waypointPayloadCurrentCount++)
      {
          int16_t x = (waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE)] << 8) | waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 1];
          int16_t y = (waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 2] << 8) | waypointsBuffer[bufferOffset + (i * I2C_WAYPOINT_SIZE) + 3];

          waypointPayload[(waypointPayloadCurrentCount * sizeof(int16_t))]   = x;
          waypointPayload[(waypointPayloadCurrentCount * sizeof(int16_t))+1] = y;  

          checksumComputed += ((x >> 8) & (x & 0xFF));
          checksumComputed += ((y >> 8) & (y & 0xFF));
      }

      if (waypointPayloadCurrentCount == waypointPayloadExpectedCount)
      {
#ifdef __DEBUG__                  
          Serial.println(" - Waypoint payload is full, ignoring further waypoint data");
#endif    
          if (checksum == checksumComputed)
          {
              enableRanging = (optionByte1 & OPTION1_ENABLE_RANGING);
              abortAfterDistanceToWaypointIncreases = (optionByte1 & OPTION1_ENABLE_ABORT_AFTER_DISTANCE_INCREASES);
              capAngularVelocity = (optionByte1 & OPTION1_ENABLE_CAP_ANGULAR_VELOCITY_SIGNAL);
              periodicReferenceHeadingReset = (optionByte1 & OPTION1_ENABLE_PERIODIC_REFERENCE_HEADING_RESET);
              
              receivedFullPayload = true; 

              // Overrides
              if (optionByte1 == OPTION1_OVERRIDE_CALIBRATE_MOTORS)
              {
                 calibrateMotors(2);
                 receivedFullPayload = false;  // ignore the waypoints, we're just calibrating  
              }
              else if (optionByte1 == OPTION1_OVERRIDE_RESET_TO_ORIGIN)
              {
                 resetToOrigin();
                 receivedFullPayload = false;  // ignore the waypoints                 
              }
              else if (optionByte1 == OPTION1_OVERRIDE_SET_PID_PARAMETERS)
              {
                 // When setting PID parameters the waypoints stand in as command bytes
                 pidLoopIntervalMs = maxVelocity;           // interval up to 255ms can be set via I2C
                 pidProportional = (uint8_t)(waypointPayload[0] >> 8) + ((uint8_t)(waypointPayload[0] & 0xFF) / 100.0);
                 pidIntegral = (uint8_t)(waypointPayload[1] >> 8) + ((uint8_t)(waypointPayload[1] & 0xFF) / 100.0);
#ifdef __DEBUG__
              snprintf_P(report, sizeof(report), PSTR(" - Updated PID. Interval %ums, P: %s I: %s"), pidLoopIntervalMs, ftoa(floatBuf1, pidProportional), ftoa(floatBuf2, pidIntegral));      
              Serial.println(report);
#endif 

                 receivedFullPayload = false;
              }
          }
          else
          {
#ifdef __DEBUG__                  
              snprintf_P(report, sizeof(report), PSTR(" - Invalid checksum [%x] expected [%x])"), checksumComputed, checksum);      
              Serial.println(report);
#endif                
          }

          ledRed(0);
      }
   }
   
   return receivedFullPayload;
}


/**
 * Report the pose snapshots from the last waypoint to the Raspberry Pi.
 * 
 * Snapshots are sent to the Raspberry Pi, acting as an I2C slave, encapsulated in a similar way
 * to how the Pi sends waypoint commands to the Arduino. Each snapshot buffer segment contains 
 * one snapshot (8 bytes) and the 1st and 10th byte are segment markers.
 * 
 * A snapshot consists of:
 *  x position (converted from double to signed 16-bit integer)
 *  y position (converted from double to signed 16-bit integer)
 *  heading (converted from double to 2 8-bit integers, the first considered signed and the second used for the floating point)
 *  distance to obstacle (converted from double to unsigned 16-bit integer)
 */
void reportPoseSnapshots(bool abortedDueToObstacle, bool lastWaypointOfJourney)
{
    // The last snapshot is always our current position
    if (poseSnapshotCount >= MAX_POSE_SNAPSHOTS)
    {
        poseSnapshotCount--;
    }

    recordSnapshot(currentPose.heading, currentPose.x, currentPose.y);

#ifdef __DEBUG__            
    snprintf_P(report, sizeof(report), PSTR("Reporting %d pose snapshots:"), poseSnapshotCount);      
    Serial.println(report);  
#endif
    
    ledGreen(1);

    unsigned char snapshotBuffer[(I2C_SNAPSHOTS_PER_SEGMENT * I2C_SNAPSHOT_SIZE) + 2];           // 1 snapshot + header / trailer markers
    size_t snapshotBufferSize = sizeof(snapshotBuffer);
    int i;
    
    for (i = 0; i < poseSnapshotCount; i++)
    {
        int bufferIndex = i % I2C_SNAPSHOTS_PER_SEGMENT;                     // allows for smaller snapshot reports in future

        int8_t  heading = (int8_t)poseSnapshots[i].heading;
        uint8_t headingFloat = (poseSnapshots[i].heading >= 0) ? ((uint8_t)((int)(poseSnapshots[i].heading * 100.0) % 100)) : ((uint8_t)((int)(poseSnapshots[i].heading * -100.0) % 100));

#ifdef __DEBUG__                  
        snprintf_P(report, sizeof(report), PSTR("   Adding snapshot [%d]: (%d,%d at %d.%d) dist %d ts: %u"), i, (int16_t)poseSnapshots[i].x, (int16_t)poseSnapshots[i].y, heading, headingFloat, (uint16_t)poseSnapshots[i].distanceToObstacle, poseSnapshots[i].timestamp);      
        Serial.println(report);  
#endif

        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+0] = (unsigned char)((int16_t)poseSnapshots[i].x >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+1] = (unsigned char)((int16_t)poseSnapshots[i].x & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+2] = (unsigned char)((int16_t)poseSnapshots[i].y >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+3] = (unsigned char)((int16_t)poseSnapshots[i].y & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+4] = (unsigned char)heading;
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+5] = (unsigned char)headingFloat;        
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+6] = (unsigned char)((int16_t)poseSnapshots[i].distanceToObstacle >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+7] = (unsigned char)((int16_t)poseSnapshots[i].distanceToObstacle & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+8] = (unsigned char)(poseSnapshots[i].timestamp >> 8);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+9] = (unsigned char)(poseSnapshots[i].timestamp & 0xFF);
        snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] = 0;

        if (i == (poseSnapshotCount - 1))
        {
           snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_WAYPOINT;

           if (abortedDueToObstacle)
           {
              // If the waypoint was aborted due to finding an obstacle, report that on the last snapshot (as that represents 
              // our current position and hence its distance measurement will have the best estimate for the obstacle location)
              snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_ABORTED_WAYPOINT;
           }
           else if (lastWaypointOfJourney)
           {
              // This is never reported if the last waypoint can't be reached due to an obstacle
              snapshotBuffer[1+(bufferIndex*I2C_SNAPSHOT_SIZE)+10] |= I2C_SNAPSHOT_DETAILBYTE_LAST_SNAPSHOT_FOR_JOURNEY;            
           }
        }

        if ((I2C_SNAPSHOTS_PER_SEGMENT == 1) || (i % (I2C_SNAPSHOTS_PER_SEGMENT - 1) == 0))
        {
#ifdef __DEBUG__                    
            Serial.println("   Writing snapshot buffer...");
#endif

            snapshotBuffer[0] = I2C_MARKER_SEGMENT_START;
            snapshotBuffer[snapshotBufferSize-1] = I2C_MARKER_SEGMENT_END;

            Wire.beginTransmission(I2C_PI_ADDR);
            Wire.write('r');                             // report
            for (int j = 0; j < snapshotBufferSize; j++)
            {
               Wire.write(snapshotBuffer[j]);
            }
            Wire.endTransmission();

            if (I2C_SNAPSHOTS_PER_SEGMENT > 1)
            {
                memset(snapshotBuffer, 0, snapshotBufferSize);
            }

            delay(50);    // don't write too fast
        }
    }

    if (i % I2C_SNAPSHOTS_PER_SEGMENT != 0)
    {
#ifdef __DEBUG__                
        snprintf_P(report, sizeof(report), PSTR("   Writing trailing snapshot buffer with %d snapshots"), i % I2C_SNAPSHOTS_PER_SEGMENT);      
        Serial.println(report);  
#endif

        snapshotBuffer[0] = I2C_MARKER_SEGMENT_START;
        snapshotBuffer[snapshotBufferSize-1] = I2C_MARKER_SEGMENT_END;

        Wire.beginTransmission(I2C_PI_ADDR);
        Wire.write('r');                             // report
        for (int j = 0; j < snapshotBufferSize; j++)
        {
           Wire.write(snapshotBuffer[j]);
        }
        Wire.endTransmission();

        delay(50);    // don't write too fast
    }    

    ledGreen(0);
}


/**
 * Record a snapshot of the current position and a range finding to the nearest obstacle.
 */
void recordSnapshot(double heading, double x, double y)
{
    if (poseSnapshotCount < MAX_POSE_SNAPSHOTS)
    {
        poseSnapshots[poseSnapshotCount].timestamp = (uint16_t)(millis() - waypointStartTime);
        poseSnapshots[poseSnapshotCount].heading = heading;
        poseSnapshots[poseSnapshotCount].x = x;
        poseSnapshots[poseSnapshotCount].y = y;

        if (enableRanging)
        {
            poseSnapshots[poseSnapshotCount].distanceToObstacle = getSonarRangedDistance();

            if (poseSnapshots[poseSnapshotCount].distanceToObstacle < CLOSE_PROXIMITY_THRESHOLD)
            {
               consecutiveCloseProximityReadings++;
            }
            else
            {
               consecutiveCloseProximityReadings = 0;
            }
        }
        else
        {
            poseSnapshots[poseSnapshotCount].distanceToObstacle = 0;                  
        }
        
        poseSnapshotCount++;          
    }
}


/**
 * Get the current distance measured from the ultrasonic sensor. Returns a value in centimeters.
 */
double getSonarRangedDistance()
{
    double distance = 0;

    int adc = analogRead(A4);
    double mv = ((double)adc / (ADC_LSB_PER_VOLT)) * 1000.0;

    distance = mv / (VOLTS_PER_CM * 1000.0);

    return distance;
}


/**
 * Reset pose to (0,0) with heading of 0 rad.
 */
void resetToOrigin() 
{
    headingError = 0.0;
    headingErrorPrev = 0.0;
    headingErrorIntegral = 0.0;

    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.heading = 0.0;
    currentPose.timestamp = 0;

    referencePose.x = 0.0;
    referencePose.y = 0.0;
    referencePose.heading = 0.0;
    referencePose.timestamp = 0;

    gyroAngle = 0.0;
    gyroAngleRad = 0.0;

    resetDistancesForNewWaypoint();
}


/**
 * Reset distance travelled measurements. This must be done at the start of each waypoint segment.
 */
void resetDistancesForNewWaypoint() 
{
    distLeft = 0.0;
    distLeftPrev = 0.0;
    distRight = 0.0;
    distRightPrev = 0.0;
    distTotal = 0.0;
    distTotalPrev = 0.0;  
    poseSnapshotCount = 0;
    consecutiveCloseProximityReadings = 0;
    
    waypointStartTime = millis();
    gyroLastUpdateMicros = micros();
}


/**
 * Determine heading required to get from a given global co-ordinate (facing in a given heading) to 
 * another global go-ordinate.
 * 
 * The current position and heading is always translated to (0,0) 0 radians (ie. looking along the x-axis from (0,0)) and
 * this function will return a value between 0..6.28 where radians grows counter-clockwise:
 * 
 * (10,0)     0 rad
 * (10,10)    0.78 rad (pi/4)
 * (0,10)     1.57 rad (pi/2)     
 * (-10,10)   2.36 rad (3pi/4) 
 * (-10,0)    3.14 rad (pi)
 * (-10,-10)  3.92 rad (5pi/4)
 * (0,-10)    4.71 rad (3pi/2)
 * (10,-10)   5.49 rad (7pi/4)
 * 
 * @param double toX              destination global x co-ord
 * @param double toY              destination global y co-ord
 * @param double fromX            current global x co-ord
 * @param double fromY            current global y co-ord
 * @param double currentHeading   current heading
 */
double getHeading(double toX, double toY, double fromX, double fromY, double currentHeading) 
{
    // Translate current pose to (0,0, 0 rad)
    double rad = currentHeading;
    double x = toX - fromX;
    double y = toY - fromY;

#ifdef __DEBUG__          
//  snprintf_P(report, sizeof(report), PSTR("to global   (%s, %s) from (%s, %s head [%s])"), ftoa(floatBuf1, toX), ftoa(floatBuf2, toY), ftoa(floatBuf3, fromX), ftoa(floatBuf4, fromY), ftoa(floatBuf5, currentHeading));    
//  Serial.println(report);
//  snprintf_P(report, sizeof(report), PSTR("to relative (%s, %s)"), ftoa(floatBuf1, x), ftoa(floatBuf2, y));    
//  Serial.println(report);
#endif

    if (x <= 0.01 && x >= -0.01)            // x == 0
    {
        if (y < 0.0)
        {
            rad = (double)(3.0*M_PI)/2.0;
        }
        else if (y > 0.0)
        {
            rad = M_PI / 2.0;
        }
#ifdef __DEBUG__                    
        else
        {
            snprintf_P(report, sizeof(report), PSTR("-> keeping current heading"));    
            Serial.println(report);      
            
        }
#endif        
    }
    else
    {
        rad = atan(y / x);

        if (x > 0)
        {
            rad += (2.0*M_PI);

            if (rad > (2.0*M_PI))
            {
                rad -= (2.0*M_PI); 
            }
        }
        else if (x < 0)
        {
            rad += M_PI;          
        }
    }

    /**
     * BUG: I think we're missing a condition here where y = 0. If y = 0 and x > 0 then we correctly maintain the current pose, but if x < 0 then we need to do a 180.
     */

    return rad;
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using odometry).
 */
void correctHeadingWithPivotTurn(double headingError)
{
    motors.setSpeeds(0, 0);
    delay(PIVOT_TURN_SLEEP_MS);
    
    double arcLength = ((headingError / (2*M_PI)) * (2*M_PI*BASERADIUS));

#ifdef __DEBUG__                        
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad, spin arc length [%s]"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, arcLength)); 
    Serial.println(report);      
#endif
    
    encoders.getCountsAndResetLeft();
    double spinDist = 0.0;
    arcLength = fabs(arcLength);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * pivotTurnSpeed, pivotTurnSpeed); 
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(pivotTurnSpeed, -1 * pivotTurnSpeed);
    }
    
    while (spinDist < arcLength)
    {
        int16_t countsLeft = abs(encoders.getCountsLeft());
        int16_t countsRight = abs(encoders.getCountsRight());
        int16_t counts = (countsLeft + countsRight) / 2;
        
        unsigned long absCounts = abs(counts);
        spinDist = ((double)absCounts / COUNTS_PER_REVOLUTION) * (2*M_PI * WHEELRADIUS); 
    }
    
    motors.setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED arc length [%s]"), ftoa(floatBuf2, spinDist)); 
    Serial.println(report);      
#endif
    
    delay(PIVOT_TURN_SLEEP_MS);    
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using gyroscope).
 */
void correctHeadingWithPivotTurnGyro(double headingError)
{
    motors.setSpeeds(0, 0);

    updateGyroscopeHeading();
    double diff, delaySafeHeadingError = 0.75 * headingError, gyroStartAngleRad = gyroAngleRad;
    bool skip;
    int i = 0;

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, gyroStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, gyroStartAngleRad)); 
    Serial.println(report);      
#endif

    delay(PIVOT_TURN_SLEEP_MS);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * pivotTurnSpeed, pivotTurnSpeed); 

        do
        {
           delay(10);
           updateGyroscopeHeading();
           skip = false;

           // Ignore noise, the angle SHOULD be increasing
           if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
           {
#ifdef __DEBUG__                                
//            Serial.println("Ignoring noise");
#endif
              skip = true;
              continue;
           }
          
           if (gyroAngleRad > gyroStartAngleRad)
           {
              diff = gyroAngleRad - gyroStartAngleRad;
           }
           else
           {
              diff = ((2*M_PI) - gyroStartAngleRad) + gyroAngleRad;
           }

           // Only record a snapshot (which requires an ADC read, which can impact timing) during the first 75% of the rotation and even then only every 400ms
           if ((diff < delaySafeHeadingError) && ((i++ % 40) == 0))
           {
              recordSnapshot(currentPose.heading + diff, currentPose.x, currentPose.y);
           }

#ifdef __DEBUG__                    
//           Serial.println(diff);
#endif
        }
        while (skip || (diff < headingError));
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(pivotTurnSpeed, -1 * pivotTurnSpeed);

        do
        {
            delay(10);
            updateGyroscopeHeading();
            skip = false;

            // Ignore noise, the angle SHOULD be decreasing
            if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
            {
#ifdef __DEBUG__                                  
//              Serial.println("Ignoring noise");
#endif
                skip = true;
                continue;
            }
            
            if (gyroAngleRad < gyroStartAngleRad)
            {
                diff = gyroAngleRad - gyroStartAngleRad;
            }
            else
            {
                diff = -1.0 * (gyroStartAngleRad + ((2*M_PI) - gyroAngleRad)); 
            }

           // Only record a snapshot (which requires an ADC read, which can impact timing) during the first 75% of the rotation and even then only every 400ms
           if ((diff > delaySafeHeadingError) && ((i++ % 40) == 0))
           {
              recordSnapshot(currentPose.heading + diff, currentPose.x, currentPose.y);
           }

#ifdef __DEBUG__                    
//            Serial.println(diff);
#endif
        }
        while (skip || (diff > headingError));
    }
    
    motors.setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current gyroAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, gyroAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      
#endif    

    delay(PIVOT_TURN_SLEEP_MS); 
}


#ifdef ENABLE_MAGNETOMETER
/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using magnetometer).
 */
void correctHeadingWithPivotTurnMagnetometer(double headingError)
{
    motors.setSpeeds(0, 0);

    updateMagnetometerHeading();
    double diff, magnetoStartAngleRad = magnetoAngleRad;
    bool skip;

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, magnetoStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, magnetoStartAngleRad)); 
    Serial.println(report);      
#endif

    delay(PIVOT_TURN_SLEEP_MS);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * pivotTurnSpeed, pivotTurnSpeed); 

        do
        {
           delay(10);
           updateMagnetometerHeading();
           skip = false;

           // Ignore noise, the angle SHOULD be increasing
           if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
           {
#ifdef __DEBUG__                    
//            Serial.println("Ignoring noise");
#endif
              skip = true;
              continue;
           }
          
           if (magnetoAngleRad > magnetoStartAngleRad)
           {
              diff = magnetoAngleRad - magnetoStartAngleRad;
           }
           else
           {
              diff = ((2*M_PI) - magnetoStartAngleRad) + magnetoAngleRad;
           }

#ifdef __DEBUG__                    
           Serial.println(diff);
#endif           
        }
        while (skip || (diff < headingError));
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(pivotTurnSpeed, -1 * pivotTurnSpeed);

        do
        {
            delay(10);
            updateMagnetometerHeading();
            skip = false;

            // Ignore noise, the angle SHOULD be decreasing
            if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
            {
#ifdef __DEBUG__                                  
//              Serial.println("Ignoring noise");
#endif
                skip = true;
                continue;
            }
            
            if (magnetoAngleRad < magnetoStartAngleRad)
            {
                diff = magnetoAngleRad - magnetoStartAngleRad;
            }
            else
            {
                diff = -1.0 * (magnetoStartAngleRad + ((2*M_PI) - magnetoAngleRad)); 
            }

#ifdef __DEBUG__                    
            Serial.println(diff);
#endif            
        }
        while (skip || (diff > headingError));
    }
    
    motors.setSpeeds(0, 0);

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current magnetoAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, magnetoAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      
#endif

    delay(PIVOT_TURN_SLEEP_MS); 
}
#endif


/**
 * Convert velocity expressed in cm/s to a motor speed integer in the range[-300, 300].
 */
int convertVelocityToMotorSpeed(double velocity)
{
    int speed = (velocity / MAX_SPEED_CM_PER_SEC) * MAX_SPEED;

    if (speed > 0)
    {
        if (speed < MIN_SPEED)
        {
           speed = MIN_SPEED;          
        }
        else if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
    }
    else if (speed < 0)
    {
       if (speed > (-1 * MIN_SPEED))
       {
          speed = -1 * MIN_SPEED;
       }
       else if (speed < (-1 * MAX_SPEED))
       {
          speed = -1 * MAX_SPEED;
       }
    }

    return speed;
}

/**
 * Get the calibrated motor speed for the left motor. 
 */
int16_t getCalibratedLeftMotorSpeed(int16_t desiredSpeed)
{
   if ( ! motorCalibrationRightToLeftRatio)
   {
#ifdef __DEBUG__    
      Serial.println("   Motor calibration has not been performed");
#endif      
      return desiredSpeed;
   }
   
   if (desiredSpeed < motorCalibrationBuckets[0] || desiredSpeed > motorCalibrationBuckets[motorCalibrationBucketCount - 1])
   {
#ifdef __DEBUG__        
       snprintf_P(report, sizeof(report), PSTR("No calibration data for speed %d"), desiredSpeed);
       Serial.println(report); 
#endif

       return desiredSpeed;
   }

   // Assume the buckets are equally divided into groups of 10
   uint8_t bucketNumber = (desiredSpeed - motorCalibrationBuckets[0]) / 10;
   int16_t leftSpeed = (float)desiredSpeed * motorCalibrationRightToLeftRatio[bucketNumber];

#ifdef __DEBUG__    
   snprintf_P(report, sizeof(report), PSTR("Using bucket %d (speed: %d) to adjust rightSpeed %d to leftSpeed %d (ratio: %s)"), bucketNumber, motorCalibrationBuckets[bucketNumber], desiredSpeed, leftSpeed, ftoa(floatBuf1, motorCalibrationRightToLeftRatio[bucketNumber]));
   Serial.println(report); 
#endif

   return leftSpeed;
}

/**
 * Update gyroscope heading.
 * 
 * Similar to getHeading(), this will always update gyroAngleRad to a value between 0 and 2*PI where the value grows in the
 * counter-clockwise direction.
 */
void updateGyroscopeHeading()
{
    while ( ! imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();

    if (imu.timeoutOccurred())
    {
       buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
       delay(FREQUENCY_DURATION);
    }

    // Update the orientation as determined by the gyroscope
    int16_t gyroTurnRate = imu.g.z - gyroOffset;      // int16_t

    unsigned long now = micros();                     // uint32_t
    unsigned long dt = now - gyroLastUpdateMicros;
    gyroLastUpdateMicros = now;

    // Determine how much we've rotated around Z axis since last measurement
    int32_t rotation = (int32_t)gyroTurnRate * dt;    // signed 16 bits -> signed 32 bits multiplied by unsigned long

    // rotation is measured in gyro digits * micro-seconds. convert to degrees and add to the current angle
    gyroAngleDifference = ((double)rotation * GYRODIGITS_TO_DPS) * 0.000001;
    gyroAngle += gyroAngleDifference;  

    while (gyroAngle < 0)
    {
        gyroAngle += 360;
    }
    while (gyroAngle > 360)
    {
        gyroAngle -= 360;
    }

    gyroAngleRad = gyroAngle * (2*M_PI / 360.0);
}


#ifdef ENABLE_MAGNETOMETER
/**
 * Update magnetometer heading (blocks until magnetometer ready)
 * 
 * Similar to getHeading(), this will always update magnetoAngleRad to a value between 0 and 2*PI where the value grows in the
 * counter-clockwise direction.
 */
void updateMagnetometerHeading()
{
    while ( ! compass.ready())
    {
        delay(1);
    }

    // 360 - heading converts compass so that it grows from 0..360 CCW (ie. opposite to actual compass)
    magnetoAngle = 360 - compass.readHeading();
    magnetoAngleRad = magnetoAngle * ((2*M_PI) / 360.0);
}
#endif

 
/**
 * Helper to print doubles
 */
char *ftoa(char *a, double f)
{
    char *ret = a;

    if (f < 0)
    {
       *a++ = '-';
    }

    f = fabs(f);

    long heiltal = (long)f;
    if ( ! itoa(heiltal, a, 10))
    {
       *a = 'X';
    }
    while (*a != '\0') a++;
    *a++ = '.';

    sprintf(a, "%04d", abs((long)((f - (double)heiltal) * 10000.0)));
    return ret;
}


/****************************************************************************************************************
 * Calibration 
 ****************************************************************************************************************/

/**
 * Calibrate fixed offset out of the gyroscope.
 */
void calibrateGyro()
{
    delay(5000);                          // give user time to place device at rest
    buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
    delay(FREQUENCY_DURATION);

#ifdef __DEBUG__                    
    Serial.println("Calibrating gyroscope ...");
#endif
    
    if ( ! imu.init())
    {
#ifdef __DEBUG__                          
        Serial.println("Failed to detect or initialize IMU, halting.");
#endif        
        while(1);
    }

    imu.enableDefault();
    delay(500);

    int32_t gyroTotal = 0;
    for (uint16_t i = 0; i < 1024; i++)
    {
        while ( ! imu.readReg(LSM6::STATUS_REG) & 0x08);
        imu.read();
        gyroTotal += imu.g.z;             // gyro values are int16_t (signed)
    }

    gyroOffset = gyroTotal / 1024;        // average across all 1024 samples

#ifdef __DEBUG__                        
    Serial.println("Gyroscope calibration complete");
#endif
    
    buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
    delay(FREQUENCY_DURATION);
}


#ifdef ENABLE_MAGNETOMETER
/**
 * Calibrate magnetometer
 */
void calibrateMagnetometer()
{
    delay(5000);                          // give user time to pick up device
    buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
    delay(FREQUENCY_DURATION);

#ifdef __DEBUG__                    
    Serial.println("Calibrating compass, please move device around all axes ...");
#endif

    compass.init();
    compass.resetCalibration();

    int calibrationTime = 0;
    while (calibrationTime < 20000)
    {
        compass.readHeading();
        delay(200);
        calibrationTime += 200;
    }

#ifdef __DEBUG__                    
    Serial.println("Compass calibration complete");
#endif    
    buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
    delay(FREQUENCY_DURATION);
}
#endif


/**
 * Calibrate motor speeds. We will adjust the left motor speed to try and balance it with the right.
 */
void calibrateMotors(uint8_t sampleCount) 
{
   uint16_t i;
   uint32_t * leftTotals, * rightTotals, countTravelled = 0;
   int16_t leftCount;

#ifdef __DEBUG__                    
   Serial.println("Calibrating motors ...");
#endif

   motorCalibrationRightToLeftRatio = (float*)malloc(motorCalibrationBucketCount * sizeof(float));
   leftTotals = (uint32_t*)malloc(motorCalibrationBucketCount * sizeof(uint32_t));
   rightTotals = (uint32_t*)malloc(motorCalibrationBucketCount * sizeof(uint32_t));

   memset(leftTotals, 0, motorCalibrationBucketCount * sizeof(uint32_t));
   memset(rightTotals, 0, motorCalibrationBucketCount * sizeof(uint32_t));

   buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
   delay(FREQUENCY_DURATION);

   for (i = 0; i < motorCalibrationBucketCount * sampleCount; i++)
   {
       uint16_t speedBucket = i % motorCalibrationBucketCount;

       delay(500);
       
       encoders.getCountsAndResetLeft();
       encoders.getCountsAndResetRight();
       
       motors.setSpeeds(motorCalibrationBuckets[speedBucket], motorCalibrationBuckets[speedBucket]);
       delay(3000);
       motors.setSpeeds(0, 0);

       leftCount = encoders.getCountsAndResetLeft();

       leftTotals[speedBucket]  += leftCount;
       rightTotals[speedBucket] += encoders.getCountsAndResetRight();
       countTravelled           += abs(leftCount);

       if (((countTravelled / COUNTS_PER_REVOLUTION) * WHEELRADIUS * 2 * M_PI) > 70)
       {
          correctHeadingWithPivotTurnGyro(M_PI / 2);
          countTravelled = 0;
       }
   }

   // Calculate required adjustment factors
   for (i = 0; i < motorCalibrationBucketCount; i++)
   {
       motorCalibrationRightToLeftRatio[i] = 1.0;

       if (leftTotals[i])
       {
           float observedRatio = (float)rightTotals[i] / (float)leftTotals[i];

           // What would left's new speed if using the adjustment factor?
           int left = (float)motorCalibrationBuckets[i] * observedRatio;

           // We only have integer granularity: if the difference between the two speeds is more than the adjustment factor, don't adjust or we'll overcook it.
           float correctedDiff = abs(left - motorCalibrationBuckets[i]) / (float)motorCalibrationBuckets[i];
           float observedDiff = fabs(1 - observedRatio);

           if ((left == motorCalibrationBuckets[i]) || (correctedDiff > observedDiff))
           {
#ifdef __DEBUG__                                
               snprintf_P(report, sizeof(report), PSTR("bucket[%d] l: %d diff: %s > %s, ignoring"), motorCalibrationBuckets[i], left, ftoa(floatBuf1, correctedDiff), ftoa(floatBuf2, observedDiff));
               Serial.println(report);
#endif                
           }
           else
           {
#ifdef __DEBUG__                                
               snprintf_P(report, sizeof(report), PSTR("bucket[%d] l: %d diff: %s <= %s, ADJUSTING"), motorCalibrationBuckets[i], left, ftoa(floatBuf1, correctedDiff), ftoa(floatBuf2, observedDiff));
               Serial.println(report); 
#endif

               motorCalibrationRightToLeftRatio[i] = observedRatio;
               motorCalibrationAdjustedBuckets++;
           }
       }
   }

   free(leftTotals);
   free(rightTotals);

   buzzer.playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
   delay(FREQUENCY_DURATION);

#ifdef __DEBUG__                    
   Serial.println("Motor calibration complete ...");
#endif   
}

/****************************************************************************************************************
 * Demos 
 ****************************************************************************************************************/

double waypointsSquare[][2] = {
    {100.0, 0.0},
    {100.0, 100.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

double waypointsLine[][2] = {
    {100.0, 0.0}
};

double waypointsRectangle[][2] = {
    {-50.0, 50.0},
    {50.0, 50.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

double waypointsStar[][2] = {
    {-40.0, 90.0},
    {-80.0, 0.0},
    {40.0, 60.0},
    {-100.0, 60.0},
    {0.0, 0.0}
};

double waypointsTriangles[][2] = {
    {50.0, 50.0},
    {100.0, 0.0},
    {100.0, 100.0},
    {50.0, 50.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

#define NUM_WAYPOINTS 6
double (*waypoints)[2] = waypointsTriangles;

void preprogrammedWaypoints()
{
#ifdef __DEBUG__                      
    snprintf_P(report, sizeof(report), PSTR("************************ START ************************"));    
    Serial.println(report);
#endif
    
    resetToOrigin();
    for (int i = 0; i < NUM_WAYPOINTS; i++)
    {
        goToWaypoint(waypoints[i][0], waypoints[i][1]);
    }

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("************************ END ************************"));    
    Serial.println(report);      
#endif

    buzzer.playFromProgramSpace(soundFinished);
}

void gyroBasedOrientation() 
{
    resetToOrigin();

    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57);  
}

#ifdef ENABLE_MAGNETOMETER
void magnetometerBasedOrientation() 
{
    resetToOrigin();

    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57);  
}
#endif

