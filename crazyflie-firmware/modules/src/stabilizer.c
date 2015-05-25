//#define STOCKFIRMWARE


#ifdef STOCKFIRMWARE


/**
 
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
#include "lps25h.h"
#include "debug.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
static PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
static float accWZ     = 0.0;
static float accMAG    = 0.0;
static float vSpeedASL = 0.0;
static float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude

// Altitude hold & Baro Params
static float altHoldKp              = 0.5;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18;
static float altHoldKd              = 0.0;
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = -48;  // multiplier
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 0.05;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.98; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 60000; // max altitude hold thrust


RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

void stabilizerInit(void)
{
    if(isInit)
        return;
    
    motorsInit();
    imu6Init();
    sensfusion6Init();
    controllerInit();
    
    rollRateDesired = 0;
    pitchRateDesired = 0;
    yawRateDesired = 0;
    
    xTaskCreate(stabilizerTask, (const signed char * const)STABILIZER_TASK_NAME,
                STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
    
    isInit = true;
}

bool stabilizerTest(void)
{
    bool pass = true;
    
    pass &= motorsTest();
    pass &= imu6Test();
    pass &= sensfusion6Test();
    pass &= controllerTest();
    
    return pass;
}

static void stabilizerTask(void* param)
{
    uint32_t attitudeCounter = 0;
    uint32_t altHoldCounter = 0;
    uint32_t lastWakeTime;
    
    vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
    
    //Wait for the system to be fully started to start stabilization loop
    systemWaitStart();
    
    lastWakeTime = xTaskGetTickCount ();
    
    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
        
        // Magnetometer not yet used more then for logging.
        imu9Read(&gyro, &acc, &mag);
        
        if (imu6IsCalibrated())
        {
            commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
            commanderGetRPYType(&rollType, &pitchType, &yawType);
            
            // 250HZ
            if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
            {
                sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
                sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
                
                accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
                accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z);
                // Estimate speed from acc (drifts)
                vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;
                
                controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                             eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                             &rollRateDesired, &pitchRateDesired, &yawRateDesired);
                attitudeCounter = 0;
            }
            
            // 100HZ
            if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))
            {
                stabilizerAltHoldUpdate();
                altHoldCounter = 0;
            }
            
            if (rollType == RATE)
            {
                rollRateDesired = eulerRollDesired;
            }
            if (pitchType == RATE)
            {
                pitchRateDesired = eulerPitchDesired;
            }
            if (yawType == RATE)
            {
                yawRateDesired = -eulerYawDesired;
            }
            
            // TODO: Investigate possibility to subtract gyro drift.
            controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                                     rollRateDesired, pitchRateDesired, yawRateDesired);
            
            controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);
            
            if (!altHold || !imuHasBarometer())
            {
                // Use thrust from controller if not in altitude hold mode
                commanderGetThrust(&actuatorThrust);
            }
            else
            {
                // Added so thrust can be set to 0 while in altitude hold mode after disconnect
                commanderWatchdog();
            }
            
            if (actuatorThrust > 0)
            {
#if defined(TUNE_ROLL)
                distributePower(actuatorThrust, actuatorRoll, 0, 0);
#elif defined(TUNE_PITCH)
                distributePower(actuatorThrust, 0, actuatorPitch, 0);
#elif defined(TUNE_YAW)
                distributePower(actuatorThrust, 0, 0, -actuatorYaw);
#else
                distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
#endif
            }
            else
            {
                distributePower(0, 0, 0, 0);
                controllerResetAllPID();
            }
        }
    }
}

static void stabilizerAltHoldUpdate(void)
{
    // Get altitude hold commands from pilot
    commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);
    
    // Get barometer height estimates
    //TODO do the smoothing within getData
    lps25hGetData(&pressure, &temperature, &aslRaw);
    
    asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
    aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);
    
    // Estimate vertical speed based on successive barometer readings. This is ugly :)
    vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);
    
    // Estimate vertical speed based on Acc - fused with baro to reduce drift
    vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
    vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
    vSpeedAcc = vSpeed;
    
    // Reset Integral gain of PID controller if being charged
    if (!pmIsDischarging())
    {
        altHoldPID.integ = 0.0;
    }
    
    // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
    if (setAltHold)
    {
        // Set to current altitude
        altHoldTarget = asl;
        
        // Cache last integral term for reuse after pid init
        const float pre_integral = altHoldPID.integ;
        
        // Reset PID controller
        pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
                ALTHOLD_UPDATE_DT);
        // TODO set low and high limits depending on voltage
        // TODO for now just use previous I value and manually set limits for whole voltage range
        //                    pidSetIntegralLimit(&altHoldPID, 12345);
        //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /
        
        altHoldPID.integ = pre_integral;
        
        // Reset altHoldPID
        altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
    }
    
    // In altitude hold mode
    if (altHold)
    {
        // Update target altitude from joy controller input
        altHoldTarget += altHoldChange / altHoldChange_SENS;
        pidSetDesired(&altHoldPID, altHoldTarget);
        
        // Compute error (current - target), limit the error
        altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                               -altHoldErrMax, altHoldErrMax);
        pidSetError(&altHoldPID, -altHoldErr);
        
        // Get control from PID controller, dont update the error (done above)
        // Smooth it and include barometer vspeed
        // TODO same as smoothing the error??
        altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                                                                         (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));
        
        // compute new thrust
        actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                                                    limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));
        
        // i part should compensate for voltage drop
        
    }
    else
    {
        altHoldTarget = 0.0;
        altHoldErr = 0.0;
        altHoldPIDVal = 0.0;
    }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
    int16_t r = roll >> 1;
    int16_t p = pitch >> 1;
    motorPowerM1 = limitThrust(thrust - r + p + yaw);
    motorPowerM2 = limitThrust(thrust - r - p - yaw);
    motorPowerM3 =  limitThrust(thrust + r - p + yaw);
    motorPowerM4 =  limitThrust(thrust + r + p - yaw);
#else // QUAD_FORMATION_NORMAL
    motorPowerM1 = limitThrust(thrust + pitch + yaw);
    motorPowerM2 = limitThrust(thrust - roll - yaw);
    motorPowerM3 =  limitThrust(thrust - pitch + yaw);
    motorPowerM4 =  limitThrust(thrust + roll - yaw);
#endif
    
    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M2, motorPowerM2);
    motorsSetRatio(MOTOR_M3, motorPowerM3);
    motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
    if(value > UINT16_MAX)
    {
        value = UINT16_MAX;
    }
    else if(value < 0)
    {
        value = 0;
    }
    
    return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
    return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
    if (fabs(value) < threshold)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= threshold;
    }
    else if (value < 0)
    {
        value += threshold;
    }
    return value;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)




#else // our firmware

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "ledseq.h"
#include "param.h"
//#include "ms5611.h"
#include "lps25h.h"
#include "debug.h"
#include <stdlib.h>

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;
static float velZgoal;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

static float accWZ     = 0.0;
static float accMAG    = 0.0;
static float velZ = 0;
static float posZ = 0;
//static float vSpeedASL = 0.0;
//static float vSpeedAcc = 0.0;
//static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration

RPYType rollType;
RPYType pitchType;
RPYType yawType;

uint16_t actuatorThrust;
int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM1;
uint32_t motorPowerM2;
uint32_t motorPowerM3;
uint32_t motorPowerM4;

// Old stuffs
/*
static float K[4][8] = {
    {-0.0420390966593963,    -0.0423757987804794,	-0.345306369907288,	 -0.0133780084142088,     -0.0134851563916171,	-0.109886405272128,	0.494003712736536,	0.537138523654095},
    {-0.0420390966594088,	0.0423757987933134,  	0.345306369906335,	 -0.0133780084142088,     0.0134851563917864,	0.109886405270806,	0.494003712736537,	0.537138523654093},
    {0.0420390966761430, 	0.0423757987923129,  	-0.345306369907174,   0.0133780084142634,	  0.0134851563917854,	-0.109886405272127,	0.494003712736545,	0.537138523654096},
    {0.0420390966762987, 	-0.0423757987796651, 	0.345306369906239,    0.0133780084142635,	  -0.0134851563916162,	0.109886405270806,	0.494003712736537,	0.537138523654093}
};
*/

static float K[4][8] = {
    {-2.43708471149600,	-2.45660426508723,	-1.08941862403029e-05,	-0.0157731449865403,	-0.0158994782389946,	-0.109593394450376,	0.482595182319267,	1.54027012185378},
    {-2.43708471147221,	2.45660423894574,	1.10253165036707e-05,	-0.0157731449865166,	0.0158994780766410,     0.109593365395343,	0.482595182319271,	1.54027012185380},
    {2.43708475964789,	2.45660423896934,	-1.08940993711986e-05,	0.0157731453321723,     0.0158994780766644,     -0.109593394450293,	0.482595182319261,	1.54027012185379},
    {2.43708475962417,	-2.45660426506360,	1.10252631390818e-05,	0.0157731453321487,     -0.0158994782389711,	0.109593365395285,	0.482595182319260,	1.54027012185377}
};

float r[4];

#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ )) // 500hz


uint16_t thrustDesired;


static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);

static bool isInit;

void stabilizerInit(void)
{
    if(isInit)
        return;
    
    motorsInit();
    imu6Init();
    sensfusion6Init();
    controllerInit();
    
    rollRateDesired = 0;
    pitchRateDesired = 0;
    yawRateDesired = 0;
    
    xTaskCreate(stabilizerTask, (const signed char * const)STABILIZER_TASK_NAME,
                STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
    
    isInit = true;
}

bool stabilizerTest(void)
{
    bool pass = true;
    
    pass &= motorsTest();
    pass &= imu6Test();
    pass &= sensfusion6Test();
    pass &= controllerTest();
    
    return pass;
}

static uint16_t limitThrust(int32_t value)
{
    if(value > UINT16_MAX)
    {
        value = UINT16_MAX;
    }
    else if(value < 0)
    {
        value = 0;
    }
    
    return (uint16_t)value;
}

// Update the control signal with LQR gain (Called from stabilizerTask)
void updateControlSignal(void){
#define PI 3.14159265359
	float rollRad = (PI/180)*eulerRollActual;
	float pitchRad = (PI/180)*eulerPitchActual;
	float yawRad = (PI/180)*eulerYawActual;

	float goalRollRad = (PI/180)*eulerRollDesired;
	float goalPitchRad = (PI/180)*eulerPitchDesired;
	float goalYawRad = (PI/180)*eulerYawDesired;
	//find the T matrix
	float T[3][3] = { {1, 0, -sin(pitchRad)},
			{0, cos(rollRad), cos(pitchRad)*sin(rollRad)},
			{0, -sin(rollRad), cos(pitchRad)*cos(rollRad)}};

	//get omega in radians
	float omegaRad[3] = {(PI/180)*gyro.x,(PI/180)*gyro.y,(PI/180)*gyro.z};

	//find dropiya from dropiya = T\omega
	float temp = (T[0][0]*T[1][1]*T[2][2]+T[0][1]*T[2][0]*T[1][2]+T[0][2]*T[1][0]*T[2][1])-(T[0][0]*T[1][2]*T[2][1]+T[0][1]*T[1][0]*T[2][2]+T[0][2]*T[1][1]*T[2][0]);
	float dRoll = -((T[0][1]*T[2][2]*omegaRad[1]+T[0][2]*T[1][1]*omegaRad[2]+omegaRad[0]*T[1][2]*T[2][1])-(T[0][1]*T[1][2]*omegaRad[2]+T[0][2]*T[2][1]*omegaRad[1]+omegaRad[0]*T[1][1]*T[2][2]))/temp;
	float dPitch = -((T[0][0]*T[1][2]*omegaRad[2]+T[0][2]*T[2][0]*omegaRad[1]+omegaRad[0]*T[1][0]*T[2][2])-(T[0][0]*T[2][2]*omegaRad[1]+T[0][2]*T[1][0]*omegaRad[2]+omegaRad[0]*T[1][2]*T[2][0]))/temp;
	float dYaw = -((T[0][0]*T[2][1]*omegaRad[1]+T[0][1]*T[1][0]*omegaRad[2]+omegaRad[0]*T[1][1]*T[2][0])-(T[0][0]*T[1][1]*omegaRad[2]+T[0][1]*T[2][0]*omegaRad[1]+omegaRad[0]*T[1][0]*T[2][1]))/temp;

    float y[8] = {goalRollRad - rollRad, goalPitchRad - pitchRad, -yawRad, -dRoll, -dPitch, goalYawRad - dYaw, -posZ, 100*velZgoal - velZ};

    int i,j;
    /* Multiplying matrix a and b and storing in array mult. */
    for(i=0; i< 4; ++i){
        float temp2 = 0;/* Initializing elements of matrix mult to 0.*/
        for(j=0; j<8 ; ++j){ // Rows
            temp2-=K[i][j]*y[j];
        }
        r[i]=temp2; //+ 0.52; //adding hover pwm
    }

}

static void stabilizerTask(void* param){
    
    uint32_t lastWakeTime;
    
    vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
    
    //Wait for the system to be fully started to start stabilization loop
    systemWaitStart();
    
    lastWakeTime = xTaskGetTickCount ();
    
    while(1){
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
        
        // Magnetometer not yet used more then for logging.
        imu9Read(&gyro, &acc, &mag);
        
        if (imu6IsCalibrated()){
            commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
            commanderGetRPYType(&rollType, &pitchType, &yawType);
            sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
            sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
            accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
            
            velZ += (9.82*accWZ*FUSION_UPDATE_DT)/(1000);
            posZ += posZ*FUSION_UPDATE_DT;
            
            //input: gyro.x, -gyro.y, gyro.z
            commanderGetThrust(&thrustDesired);
            velZgoal = 3*thrustDesired/(float)60000;
            
            // Update control signal
            
            updateControlSignal();
            
            if (thrustDesired > 0.1){
                velZ = 0;
                posZ = 0;
                //write output to motors directly, m1 m2 m3 and m4 must be set
                motorsSetRatio(MOTOR_M1, limitThrust((uint32_t) UINT16_MAX*r[3])); //motors are between 0 and UINT16_MAX
                motorsSetRatio(MOTOR_M2, limitThrust((uint32_t) UINT16_MAX*r[2]));
                motorsSetRatio(MOTOR_M3, limitThrust((uint32_t) UINT16_MAX*r[1]));
                motorsSetRatio(MOTOR_M4, limitThrust((uint32_t) UINT16_MAX*r[0]));
                
            }else{
                velZ = 0;
                posZ = 0;
                motorsSetRatio(MOTOR_M1, 0); //motors are between 0 and UINT16_MAX
                motorsSetRatio(MOTOR_M2, 0);
                motorsSetRatio(MOTOR_M3, 0);
                motorsSetRatio(MOTOR_M4, 0);
            }
        }
    }
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

#endif
