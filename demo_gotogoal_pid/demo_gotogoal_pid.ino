#include <Romi32U4.h>

#define PID_PROPORTIONAL 0.90
#define PID_INTEGRAL     0.0005
#define PID_DERIVATIVE   0.00

#define MAX_VELOCITY         2                  // cm/s
#define VELOCITY_ON_APPROACH 1.0
#define CORRECT_MOTOR_BIASES false              // left and right motor may have different input response curves, correct for them?

#define WHEELBASE        14.9225                // cm
#define BASERADIUS       7.46125                // cm
#define WHEELRADIUS      3.65125                // cm

#define PIVOT_TURN true                         // pivot turn instead of PID mode for large heading corrections
#define PIVOT_TURN_SPEED 30
#define PIVOT_TURN_THRESHOLD 12.0               // use pivot if heading correction > 2*PI / PIVOT_TURN_THRESHOLD
#define PIVOT_TURN_SLEEP_MS 500

#define CAP_ANGULAR_VELOCITY_SIGNAL true        // cap angular velocity correction allowed in PID mode
#define ANGULAR_VELOCITY_SIGNAL_LIMIT 0.2

#define WAYPOINT_PROXIMITY_APPROACHING 10.0
#define WAYPOINT_PROXIMITY_REACHED 1.5
#define POST_WAYPOINT_SLEEP_MS 1000

#define PERIODIC_REFERENCE_HEADING_RESET false  // should we periodically recalculate required reference heading during waypoint finding?

#define COUNTS_PER_REVOLUTION 1440.0            // number of encoder counts per wheel revolution (should be ~1440)

struct Pose {
  double        x;
  double        y;
  double        heading;
  uint32_t      timestamp;
};

Romi32U4Encoders  encoders;
Romi32U4Buzzer    buzzer;
Romi32U4Motors    motors;

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

char report[80];
char floatBuf1[16], floatBuf2[16], floatBuf3[16], floatBuf4[16], floatBuf5[16], floatBuf6[16];

const char encoderErrorLeft[]  PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";
const char starting[] PROGMEM = "! L16 V8 gcdgcdgcdgcd";
const char finished[] PROGMEM = "! L16 V8 cdefgab>cbagfedc";

double waypointsSquare[][2] = {
    {100.0, 0.0},
    {100.0, 100.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

double waypointsLine[][2] = {
    {100.0, 0.0}
};

#define NUM_WAYPOINTS 4
double (*waypoints)[2] = waypointsSquare;

void setup() 
{
    resetToOrigin();

    snprintf_P(report, sizeof(report), PSTR("************************ START ************************"));    
    Serial.println(report);      
    
    buzzer.playFromProgramSpace(starting);
    delay(2000);
    
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

            // Ensure our heading remains sane
            currentPose.heading = atan2(sin(currentPose.heading), cos(currentPose.heading));

            distTotalPrev = distTotal;
            distLeftPrev  = distLeft;
            distRightPrev = distRight;
        }
        
        // What's the error between our required heading and our heading?
        double headingErrorRaw = referencePose.heading - currentPose.heading;
        headingError = atan2(sin(headingErrorRaw), cos(headingErrorRaw));

        snprintf_P(report, sizeof(report), PSTR("c[%s] r[%s] er[%s] e[%s]"), ftoa(floatBuf1, currentPose.heading), ftoa(floatBuf2, referencePose.heading), ftoa(floatBuf3, headingErrorRaw), ftoa(floatBuf4, headingError));
        Serial.println(report);

        if (PIVOT_TURN && ((headingError > ((2*M_PI) / PIVOT_TURN_THRESHOLD)) || (headingError < -((2*M_PI) / PIVOT_TURN_THRESHOLD))))
        {
            correctHeadingWithPivotTurn(headingError);

            // @todo We SHOULD set the current heading based on the arc length ACTUALLY travelled rather than that requested (because we can overshoot)
            currentPose.heading = referencePose.heading;
            headingError = 0.0;

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

        snprintf_P(report, sizeof(report), PSTR("   now (%s, %s, head [%s, e: %s]) distTgt[%s]"), ftoa(floatBuf1, currentPose.x), ftoa(floatBuf2, currentPose.y), ftoa(floatBuf3, currentPose.heading), ftoa(floatBuf4, headingError), ftoa(floatBuf5, targetVectorMagnitude));    
        Serial.println(report);      

        // Give up out of out-of-control situations
        if (iteration != 0)
        {
            if (targetVectorMagnitude >= targetVectorMagnitudeLast)
            {
                if (consecutiveIncreasingDistances == 0)
                {
                    targetVectorMagnitudeAtStartOfDrift = targetVectorMagnitude;
                }

                consecutiveIncreasingDistances++;

                double distanceCoveredSinceStartOfDrift = targetVectorMagnitude - targetVectorMagnitudeAtStartOfDrift;
                if (distanceCoveredSinceStartOfDrift >= (0.25 * targetVectorMagnitudeInitial))
                {
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
            snprintf_P(report, sizeof(report), PSTR("--- WAYPOINT REACHED ---"));    
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
            }
            else if (u < (-1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT))
            {
                u = -1.0 * ANGULAR_VELOCITY_SIGNAL_LIMIT;
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
}


/**
 * Determine heading required to get from a given global co-ordinate (facing in a given heading) to 
 * another global go-ordinate.
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

    if (x <= 0.01 && x >= -0.01)
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

    return rad;
}


/**
 * Our heading is so wrong we should correct with a pivot (on-the-spot) rotation rather than PID.
 */
void correctHeadingWithPivotTurn(double headingError)
{
    motors.setSpeeds(0, 0);
    buzzer.playFromProgramSpace(encoderErrorRight);
    delay(PIVOT_TURN_SLEEP_MS);
    
    double arcLength = ((headingError / (2*M_PI)) * (2*M_PI*BASERADIUS));
    
    snprintf_P(report, sizeof(report), PSTR("   CORRECTING [%s] rad, spin arc length [%s] [%s]"), ftoa(floatBuf1, headingError), ftoa(floatBuf2, arcLength)); 
    Serial.println(report);      
    
    if (headingError > 0)
    {
        motors.setSpeeds(-1 * PIVOT_TURN_SPEED, PIVOT_TURN_SPEED); 
    }
    else
    {
        motors.setSpeeds(PIVOT_TURN_SPEED, -1 * PIVOT_TURN_SPEED);
    }
    
    double spinDist = 0.0;
    arcLength = fabs(arcLength);
    
    encoders.getCountsAndResetLeft();
    
    while (spinDist < arcLength)
    {
        int16_t counts          = encoders.getCountsLeft();
        unsigned long absCounts = abs(counts);
    
        spinDist = ((double)absCounts / COUNTS_PER_REVOLUTION) * (2*M_PI * WHEELRADIUS); 
    
        delay(2);
    }
    
    motors.setSpeeds(0, 0);

    snprintf_P(report, sizeof(report), PSTR("   CORRECTED arc length [%s]"), ftoa(floatBuf2, spinDist)); 
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

