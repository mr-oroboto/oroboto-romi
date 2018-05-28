#include <Romi32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "QMC5883L.h"

#define PID_PROPORTIONAL 0.90
#define PID_INTEGRAL     0.0005
#define PID_DERIVATIVE   0.00

#define MAX_VELOCITY         2                  // cm/s (known good max velocity is "2")
#define VELOCITY_ON_APPROACH 1.0
#define CORRECT_MOTOR_BIASES false              // left and right motor may have different input response curves, correct for them?

#define WHEELBASE        14.9225                // cm
#define BASERADIUS       7.46125                // cm
#define WHEELRADIUS      3.4925                 // cm (was 3.65125)

#define PIVOT_TURN true                         // pivot turn instead of PID mode for large heading corrections
#define PIVOT_TURN_SPEED 30                     // known good speed is 30
#define PIVOT_TURN_THRESHOLD 12.0               // use pivot if heading correction > 2*PI / PIVOT_TURN_THRESHOLD
#define PIVOT_TURN_SLEEP_MS 200                 // known good sleep is 500

#define CAP_ANGULAR_VELOCITY_SIGNAL true        // cap angular velocity correction allowed in PID mode
#define ANGULAR_VELOCITY_SIGNAL_LIMIT 0.2

#define ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES true
#define DISTANCE_INCREASE_ITERATION_THRESHOLD 1
#define DISTANCE_INCREASE_DRIFT_FROM_INITIAL_VECTOR_THRESHOLD 0.25

#define WAYPOINT_PROXIMITY_APPROACHING 5.0      // was 10.0
#define WAYPOINT_PROXIMITY_REACHED 1.0          // was 3.0
#define POST_WAYPOINT_SLEEP_MS 300              // known good sleep is 1000

#define PERIODIC_REFERENCE_HEADING_RESET false  // should we periodically recalculate required reference heading during waypoint finding?

#define COUNTS_PER_REVOLUTION 1440.0            // number of encoder counts per wheel revolution (should be ~1440)

#define USE_GYRO_FOR_HEADING true
#define GYRODIGITS_TO_DPS 0.00875

struct Pose {
  double        x;
  double        y;
  double        heading;
  uint32_t      timestamp;
};

Romi32U4Encoders  encoders;
Romi32U4Buzzer    buzzer;
Romi32U4Motors    motors;
LSM6              imu;
QMC5883L          compass;

struct Pose currentPose;          // (believed) current position and heading
struct Pose referencePose;        // pose of reference waypoint (heading is heading required from currentPose)

double headingError;              // for PID proportional term
double headingErrorPrev;          // for PID derivative term
double headingErrorIntegral;      // for PID integral term

double distLeft;                  // current distance travelled by left wheel in this waypoint segment
double distLeftPrev;              // previous current distance travelled by left wheel in this waypoint segment
double distRight;
double distRightPrev;
double distTotal;                 // total distance travelled in this waypoint segment
double distTotalPrev;             // previous total distance travelled in this waypoint segment

int16_t       gyroOffset = 0;           // average reading on gyro Z axis during calibration
unsigned long gyroLastUpdateMicros = 0; // helps calculate dt for gyro readings (in microseconds)
double        gyroAngle;
double        gyroAngleRad;
double        gyroAngleDifference = 0;
double        magnetoAngle;
double        magnetoAngleRad;

char report[80];
char floatBuf1[16], floatBuf2[16], floatBuf3[16], floatBuf4[16], floatBuf5[16], floatBuf6[16];

const char encoderErrorLeft[]  PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
const char starting[] PROGMEM = "! L16 V8 gcdgcdgcdgcd";
const char finished[] PROGMEM = "! L16 V8 cdefgab>cbagfedc";
const char alarm[]  PROGMEM = "!<c2";

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

/*****************************************************************************************************
 * Entrypoint
 *****************************************************************************************************/

void setup() 
{
    Serial.begin(9600);
    Wire.begin();

    if (USE_GYRO_FOR_HEADING)
    {
//      calibrateMagnetometer();      // far less accurate than the gyro until we can use it for fusion
        calibrateGyro();
    }
    
    /* --------------------------------------------------------------------------------------------- */
    
    delay(2000);
    snprintf_P(report, sizeof(report), PSTR("************************ START ************************"));    
    Serial.println(report);      
    
    buzzer.playFromProgramSpace(starting);
    delay(2000);

    resetToOrigin();
    for (int i = 0; i < NUM_WAYPOINTS; i++)
    {
        goToWaypoint(waypoints[i][0], waypoints[i][1]);
    }

    snprintf_P(report, sizeof(report), PSTR("************************ END ************************"));    
    Serial.println(report);      

    buzzer.playFromProgramSpace(finished);
}

void loop() 
{
}

/*****************************************************************************************************
 * End entrypoint
 *****************************************************************************************************/

/**
 * Go to the specified waypoint from the current waypoint.
 */
void goToWaypoint(double x, double y) 
{
    double    targetVectorMagnitude;                    // current distance between where we think we are and the waypoint
    double    targetVectorMagnitudeLast = 0.0;          // last distance between where we think we were and the waypoint
    double    targetVectorMagnitudeInitial = 0.0;       // initial distance between where we think were are the waypoint
    double    targetVectorMagnitudeAtStartOfDrift;      // if we begin to get further from target (rather than closer), what was our distance to the target when this started?
    uint8_t   consecutiveIncreasingDistances = 0;
    bool      approachingTarget = false;                // once we start getting close to waypoint, don't forget that

    double    velocityLeft = 0.0, velocityRight = 0.0;  // current velocity of left and right wheels

    resetDistancesForNewWaypoint();

    // @todo Should we really throw away PID error from the previous waypoint here?
    headingErrorPrev = 0.0;
    headingErrorIntegral = 0.0;

    snprintf_P(report, sizeof(report), PSTR("************************ WAYPOINT (%s,%s) ************************"), ftoa(floatBuf1, x), ftoa(floatBuf2, y));    
    Serial.println(report);      
    snprintf_P(report, sizeof(report), PSTR("to (%s, %s) from pose (%s, %s head [%s])"), ftoa(floatBuf1, x), ftoa(floatBuf2, y), ftoa(floatBuf3, currentPose.x), ftoa(floatBuf4, currentPose.y), ftoa(floatBuf5, currentPose.heading));    
    Serial.println(report);      

    referencePose.x = x;
    referencePose.y = y;

    // We need to keep our heading as close to this reference heading as possible to reach the waypoint
    referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);

    uint32_t      iteration = 0;
    double        dt = 0.05;                // 50ms
    unsigned long lastMillis = 0;

    // Reset wheel encoder counts to zero, we don't care about their current values
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    
    while (true)
    {
        if (iteration != 0)
        {
            dt = (millis() - lastMillis) / 1000.0;

            // Recalculate reference heading every now and then as our current position changes
            if ((iteration % 10 == 0) && PERIODIC_REFERENCE_HEADING_RESET)
            {
                referencePose.heading = getHeading(referencePose.x, referencePose.y, currentPose.x, currentPose.y, currentPose.heading);
                snprintf_P(report, sizeof(report), PSTR("corrected reference heading to [%s])"), ftoa(floatBuf1, referencePose.heading));    
                Serial.println(report);      
            }
        }

//      snprintf_P(report, sizeof(report), PSTR("[%3d] to (%s, %s, head [%s]) dist: %s"), iteration, ftoa(floatBuf1, referencePose.x), ftoa(floatBuf2, referencePose.y), ftoa(floatBuf3, referencePose.heading), ftoa(floatBuf4, targetVectorMagnitudeInitial)); 
//      Serial.println(report);      
//      snprintf_P(report, sizeof(report), PSTR("   was (%s, %s, head [%s]) distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, targetVectorMagnitudeLast));    
//      Serial.println(report);      

        int16_t countsLeft  = encoders.getCountsAndResetLeft();         // 1440 per revolution = 2*PI*radius cm travel
        int16_t countsRight = encoders.getCountsAndResetRight();
        
        if (encoders.checkErrorLeft())
        {
            buzzer.playFromProgramSpace(encoderErrorLeft);
        }
        if (encoders.checkErrorRight())
        {
            buzzer.playFromProgramSpace(encoderErrorRight);
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

            snprintf_P(report, sizeof(report), PSTR("[%d] travelled L[%s] R[%s] dist[%s] distPrev[%s]"), iteration, ftoa(floatBuf1, distLeft), ftoa(floatBuf2, distRight), ftoa(floatBuf3, distTotal), ftoa(floatBuf4, distTotalPrev)); 
            Serial.println(report);      

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

        if (USE_GYRO_FOR_HEADING)
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] gyro[%s / %s deg] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, gyroAngleRad), ftoa(floatBuf3, gyroAngle), ftoa(floatBuf4, referencePose.heading), ftoa(floatBuf5, headingError), ftoa(floatBuf6, headingErrorRaw));          
        }
        else
        {
            snprintf_P(report, sizeof(report), PSTR("head curr[%s] ref[%s] err[%s] (raw %s)"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, referencePose.heading), ftoa(floatBuf3, headingError), ftoa(floatBuf4, headingErrorRaw));
        }
        Serial.println(report);

        if (PIVOT_TURN && ((headingError > ((2*M_PI) / PIVOT_TURN_THRESHOLD)) || (headingError < -((2*M_PI) / PIVOT_TURN_THRESHOLD))))
        {
            if (USE_GYRO_FOR_HEADING)
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

        // What is the magnitude of the vector between us and our target?
        targetVectorMagnitude = sqrt(sq(referencePose.x - currentPose.x) + sq(referencePose.y - currentPose.y));
        if (iteration == 0)
        {
            targetVectorMagnitudeInitial = targetVectorMagnitude;        
        }

        snprintf_P(report, sizeof(report), PSTR("   now (%s, %s) head [%s] err[%s] distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, targetVectorMagnitude));    
        Serial.println(report);      

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

                if (ABORT_WAYPOINT_AFTER_DISTANCE_INCREASES && consecutiveIncreasingDistances >= DISTANCE_INCREASE_ITERATION_THRESHOLD)
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

                    snprintf_P(report, sizeof(report), PSTR("   !!! ABORT: DISTANCE TO TARGET INCREASED"));    
                    Serial.println(report);      

                    buzzer.playFromProgramSpace(encoderErrorLeft);
                    delay(500);
                    buzzer.playFromProgramSpace(encoderErrorRight);
                    delay(500);
                    buzzer.playFromProgramSpace(encoderErrorLeft);
                    delay(500);
                    buzzer.playFromProgramSpace(encoderErrorRight);
                    delay(500);
                    break;                  
                }
            }
            else
            {
                consecutiveIncreasingDistances = 0;
            }
        }
        
        double forwardVelocity = MAX_VELOCITY;

        if (targetVectorMagnitude < WAYPOINT_PROXIMITY_APPROACHING || approachingTarget)
        {
            approachingTarget = true;

            snprintf_P(report, sizeof(report), PSTR("   >>> slowing..."));    
            Serial.println(report);      

            forwardVelocity = VELOCITY_ON_APPROACH;
        }

        if (targetVectorMagnitude <= WAYPOINT_PROXIMITY_REACHED)
        {
            snprintf_P(report, sizeof(report), PSTR("   >>> WAYPOINT REACHED <<<"));    
            Serial.println(report);      
            break;
        }

        targetVectorMagnitudeLast = targetVectorMagnitude;

        // Maintain the PID variables
        double headingErrorDerivative  = (headingError - headingErrorPrev) / dt;
        headingErrorIntegral          += (headingError * dt);
        headingErrorPrev               = headingError;

        double pidP = PID_PROPORTIONAL * headingError;
        double pidI = PID_INTEGRAL * headingErrorIntegral;
        double pidD = PID_DERIVATIVE * headingErrorDerivative;

        // PID, this gives us the control signal, u, this is our required angular velocity to achieve our desired heading
        double u = pidP + pidI + pidD;

        if (CAP_ANGULAR_VELOCITY_SIGNAL)
        {
            if (u > ANGULAR_VELOCITY_SIGNAL_LIMIT)
            {
                u = ANGULAR_VELOCITY_SIGNAL_LIMIT;
                snprintf_P(report, sizeof(report), PSTR("   !!! Capped angular velocity signal"));    
                Serial.println(report);      
            }
            else if (u < (-1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT))
            {
                u = -1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT;
                snprintf_P(report, sizeof(report), PSTR("   !!! Capped angular velocity signal"));    
                Serial.println(report);                      
            }
        }

        // Angular velocity gives us new wheel velocities. We assume a constant forward velocity for simplicity.
        velocityRight = ((2.0 * forwardVelocity) + (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s
        velocityLeft  = ((2.0 * forwardVelocity) - (u * WHEELBASE)) / (2.0 * WHEELRADIUS);    // cm/s

        int leftSpeed  = convertVelocityToMotorSpeed(velocityLeft, true);
        int rightSpeed = convertVelocityToMotorSpeed(velocityRight, false); 

//      snprintf_P(report, sizeof(report), PSTR("   P[%s] I[%s] D[%s] u[%s] -> required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, pidP), ftoa(floatBuf2, pidI), ftoa(floatBuf3, pidD), ftoa(floatBuf4, u), ftoa(floatBuf5, velocityLeft), leftSpeed, ftoa(floatBuf6, velocityRight), rightSpeed);    
        snprintf_P(report, sizeof(report), PSTR("   u[%s] required velocities L%s (%4d) R%s (%4d)"), ftoa(floatBuf1, u), ftoa(floatBuf2, velocityLeft), leftSpeed, ftoa(floatBuf3, velocityRight), rightSpeed);    
        Serial.println(report);      

        motors.setSpeeds(leftSpeed, rightSpeed);

        iteration++;

        lastMillis = millis();
        delay(50);    // 50ms

        // Time passes, wheels respond to new control signal and begin moving at new velocities
    }

    motors.setSpeeds(0, 0);
    delay(POST_WAYPOINT_SLEEP_MS);
}


/****************************************************************************************************************
 * HELPERS 
 ****************************************************************************************************************/

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

//  snprintf_P(report, sizeof(report), PSTR("to global   (%s, %s) from (%s, %s head [%s])"), ftoa(floatBuf1, toX), ftoa(floatBuf2, toY), ftoa(floatBuf3, fromX), ftoa(floatBuf4, fromY), ftoa(floatBuf5, currentHeading));    
//  Serial.println(report);

//  snprintf_P(report, sizeof(report), PSTR("to relative (%s, %s)"), ftoa(floatBuf1, x), ftoa(floatBuf2, y));    
//  Serial.println(report);

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
        else
        {
            snprintf_P(report, sizeof(report), PSTR("-> keeping current heading"));    
            Serial.println(report);      
        }
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
    buzzer.playFromProgramSpace(encoderErrorRight);
    delay(PIVOT_TURN_SLEEP_MS);
    
    double arcLength = ((headingError / (2*M_PI)) * (2*M_PI*BASERADIUS));
    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad, spin arc length [%s]"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, arcLength)); 
    Serial.println(report);      
    
    encoders.getCountsAndResetLeft();
    double spinDist = 0.0;
    arcLength = fabs(arcLength);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * PIVOT_TURN_SPEED, PIVOT_TURN_SPEED); 
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(PIVOT_TURN_SPEED, -1 * PIVOT_TURN_SPEED);
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

    snprintf_P(report, sizeof(report), PSTR("   CORRECTED arc length [%s]"), ftoa(floatBuf2, spinDist)); 
    Serial.println(report);      
    
    delay(PIVOT_TURN_SLEEP_MS);    
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using gyroscope).
 */
void correctHeadingWithPivotTurnGyro(double headingError)
{
    motors.setSpeeds(0, 0);

    updateGyroscopeHeading();
    double diff, gyroStartAngleRad = gyroAngleRad;
    bool skip;

    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, gyroStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, gyroStartAngleRad)); 
    Serial.println(report);      

    buzzer.playFromProgramSpace(encoderErrorRight);
    delay(PIVOT_TURN_SLEEP_MS);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * PIVOT_TURN_SPEED, PIVOT_TURN_SPEED); 

        do
        {
           delay(10);
           updateGyroscopeHeading();
           skip = false;

           // Ignore noise, the angle SHOULD be increasing
           if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
           {
//            Serial.println("Ignoring noise");
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

           Serial.println(diff);
        }
        while (skip || (diff < headingError));
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(PIVOT_TURN_SPEED, -1 * PIVOT_TURN_SPEED);

        do
        {
            delay(10);
            updateGyroscopeHeading();
            skip = false;

            // Ignore noise, the angle SHOULD be decreasing
            if ((fabs(gyroStartAngleRad - gyroAngleRad) < 0.02) || (fabs(gyroStartAngleRad - gyroAngleRad) > ((2*M_PI) - 0.02)))
            {
//              Serial.println("Ignoring noise");
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

            Serial.println(diff);
        }
        while (skip || (diff > headingError));
    }
    
    motors.setSpeeds(0, 0);

    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current gyroAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, gyroAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      

    delay(PIVOT_TURN_SLEEP_MS); 
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID (using magnetometer).
 */
void correctHeadingWithPivotTurnMagnetometer(double headingError)
{
    motors.setSpeeds(0, 0);

    updateMagnetometerHeading();
    double diff, magnetoStartAngleRad = magnetoAngleRad;
    bool skip;

    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad %s, magnetoStartAngleRad [%s]"), ftoa(floatBuf1, headingError), ((headingError > 0) ? "CCW" : "CW"), ftoa(floatBuf2, magnetoStartAngleRad)); 
    Serial.println(report);      

    buzzer.playFromProgramSpace(encoderErrorRight);
    delay(PIVOT_TURN_SLEEP_MS);
    
    if (headingError > 0)
    {
        // left backwards, right forwards (counter clockwise ("left")): radians are positive CCW
        motors.setSpeeds(-1 * PIVOT_TURN_SPEED, PIVOT_TURN_SPEED); 

        do
        {
           delay(10);
           updateMagnetometerHeading();
           skip = false;

           // Ignore noise, the angle SHOULD be increasing
           if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
           {
//            Serial.println("Ignoring noise");
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

           Serial.println(diff);
        }
        while (skip || (diff < headingError));
    }
    else
    {
        // left forwards, right backwards (clockwise, ("right"))
        motors.setSpeeds(PIVOT_TURN_SPEED, -1 * PIVOT_TURN_SPEED);

        do
        {
            delay(10);
            updateMagnetometerHeading();
            skip = false;

            // Ignore noise, the angle SHOULD be decreasing
            if ((fabs(magnetoStartAngleRad - magnetoAngleRad) < 0.04) || (fabs(magnetoStartAngleRad - magnetoAngleRad) > ((2*M_PI) - 0.04)))
            {
//              Serial.println("Ignoring noise");
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

            Serial.println(diff);
        }
        while (skip || (diff > headingError));
    }
    
    motors.setSpeeds(0, 0);

    snprintf_P(report, sizeof(report), PSTR("   CORRECTED heading error of [%s], current magnetoAngleRad [%s] (diff [%s])"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, magnetoAngleRad), ftoa(floatBuf3, diff)); 
    Serial.println(report);      

    delay(PIVOT_TURN_SLEEP_MS); 
}


/**
 * Convert velocity expressed in cm/s to a motor speed integer in the range[-300, 300].
 */
int convertVelocityToMotorSpeed(double velocity, double left)
{
    int speed = (velocity / MAX_VELOCITY) * 300;

    if (CORRECT_MOTOR_BIASES)
    {
        if (left)
        {
            if (speed < 0)
            {
                speed -= 10;      // bias left, it's always slower to start than right
                if (speed > -30)
                {
                    speed = -30;  
                }
            }
            else
            {
                speed += 10;      // bias left          
  
                if (speed < 30)
                {
                    speed = 30; 
                }
            }
        }
        else
        {
            if (speed < 0)
            {
                if (speed > -20)
                {
                    speed = -20;      
                }
            }
            else
            {
                if (speed < 20)
                {
                    speed = 20;  
                }
            }
        }
    }

    return speed;
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
      buzzer.playFromProgramSpace(alarm);    
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
    itoa(heiltal, a, 10);
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
    buzzer.playFromProgramSpace(alarm);

    Serial.println("Calibrating gyroscope ...");
    if ( ! imu.init())
    {
        Serial.println("Failed to detect or initialize IMU, halting.");
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
    Serial.println("Gyroscope calibration complete");
    
    buzzer.playFromProgramSpace(alarm);  
}


/**
 * Calibrate magnetometer
 */
void calibrateMagnetometer()
{
    delay(5000);                          // give user time to pick up device
    buzzer.playFromProgramSpace(alarm);

    Serial.println("Calibrating compass, please move device around all axes ...");

    compass.init();
    compass.resetCalibration();

    int calibrationTime = 0;
    while (calibrationTime < 20000)
    {
        compass.readHeading();
        delay(200);
        calibrationTime += 200;
    }

    Serial.println("Compass calibration complete");
    buzzer.playFromProgramSpace(alarm);
}


/****************************************************************************************************************
 * Demos 
 ****************************************************************************************************************/

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


