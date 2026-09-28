/**
 ******************************************************************************
 * @file    pid_control.c
 * @brief   See pid_control.h.
 ******************************************************************************
 */
#include "pid_control.h"
#include "temperature_sensor.h"
#include "current_sensor.h"

extern TIM_HandleTypeDef htim3; /* PWM_CTRL / TIM3_CH1, initialised by MX_TIM3_Init() in main.c */

/* ---- Tunables, carried over from the original driver (converted from its
 * 0.1ms-tick units to milliseconds since HAL_GetTick() is now the timebase) */
#define PID_FREQ_HZ                    10.0f  /* PID calculation rate */
#define PID_CYCLE_MS                  100u    /* = 1000 / PID_FREQ_HZ */
#define RELAY_ACTUATION_MS             50u    /* time to energise the bistable relay coil */
#define CURRENT_CYCLE_MS               10u    /* actuation stage update rate */
#define CURRENT_UPDATE_COOLDOWN_MS     20u    /* settle time after a new control value before overcurrent checks resume */
#define RECAL_COOLDOWN_MS              20u    /* settle time after a duty-cycle recalibration */
#define FLIP_TIME_MS                 1000u    /* minimum time direction must be requested before the relay actually flips */

#define PWM_MAX_COUNT                65535u   /* matches htim3 ARR (Period = 65535, fixed by PM.ioc) */

#define DUTY_CYCLE_START                0.2f  /* initial test duty cycle for the startup short-circuit check */
#define CURRENT_LIMIT_START_A           3.0f  /* fixed startup short-circuit threshold, independent of the user-settable run-time limit */
#define I_SAT_P                         2.5f  /* integrator anti-windup, positive */
#define I_SAT_N                        -2.5f  /* integrator anti-windup, negative */

/* Weighted 11-point moving-average derivative filter, verbatim from the
 * original driver (err_FiltCo * weight for each of the last 11 error samples). */
#define ERR_FILT_CO                  0.0909f
static const float dWeight[11] = { 10.0f, 5.0f, 3.33f, 2.50f, 2.00f, 1.65f, 1.43f, 1.25f, 1.11f, 1.0f, 0.99f };
/* dWeight[0] pairs with the most recent history slot (Err_History[0] in the
 * original), dWeight[10] with the oldest (Err_History[10]). */

/* ---- PID state ----------------------------------------------------------- */
typedef enum { PID_STATE_OFF, PID_STATE_WAIT_FOR_DRIVER, PID_STATE_WAIT, PID_STATE_CALCULATE, PID_STATE_ERROR } PidState_t;

static PidState_t pidState = PID_STATE_OFF;
static uint8_t startRequested = 0;
static uint8_t stopRequested = 0;
static float T_SET = 25.0f;
static float kp = 1.0f, ki = 0.1f, kd = 1.0f;
static float P_val = 0.0f, I_val = 0.0f, D_val = 0.0f;
static float pidOut = 0.0f;
static float err = 0.0f, errStart = 0.0f;
static float errHistory[11];
static uint8_t tempUpdatedFlag = 0;
static uint32_t pidLastTick = 0;
static PidControl_Error_t lastError = PID_ERROR_NONE;

/* ---- Driver / actuation state --------------------------------------------- */
typedef enum { DRIVER_STATE_OFF, DRIVER_STATE_ON, DRIVER_STATE_ERROR } DriverState_t;

static DriverState_t driverState = DRIVER_STATE_OFF;
static uint8_t driverStartRequested = 0;
static uint8_t driverStopRequested = 0;
static float driverDirection = 1.0f;   /* +1 = heating (RELAY_SET), -1 = cooling (RELAY_RESET) */
static float controlDirection = 1.0f;
static uint8_t directionFlipCounter = 0;
static uint8_t directionFlipThreshold = 1; /* recomputed in Init */
static float currentLimit = 2.0f;          /* symmetric magnitude, I2C-settable */
static float dutyCycle = 0.0f, dutyCyclePrevious = 0.0f;
static float dutyCycleLimit = 1.0f, dutyCycleOverload = 1.0f;
static float currentMaxDC = 0.0f;
static float currentControl = 0.0f, currentControlPrevious = 0.0f;
static float measuredCurrent = 0.0f;
static uint8_t currentUpdatedSettlingCycles = 0;
static uint8_t recalibrationCooldownCounter = 0;
static uint8_t relayActuatedCooldownCounter = 0;
static uint32_t driverLastTick = 0;

/* ---- Small helpers --------------------------------------------------------- */
static void BusyWaitMs(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) { }
}

static void SetPwmDutyCycle(float dc)
{
    if (dc <= dutyCycleLimit && dc >= 0.0f)
        htim3.Instance->CCR1 = (uint32_t)(dc * (float)PWM_MAX_COUNT);
}

static void SetRelayHeating(void)
{
    HAL_GPIO_WritePin(RELAY_RESET_GPIO_Port, RELAY_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_SET_GPIO_Port, RELAY_SET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PID_COOLING_GPIO_Port, PID_COOLING_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_HEATING_GPIO_Port, PID_HEATING_Pin, GPIO_PIN_SET);
}

static void SetRelayCooling(void)
{
    HAL_GPIO_WritePin(RELAY_SET_GPIO_Port, RELAY_SET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_RESET_GPIO_Port, RELAY_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PID_HEATING_GPIO_Port, PID_HEATING_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_COOLING_GPIO_Port, PID_COOLING_Pin, GPIO_PIN_SET);
}

static void ReleaseRelayCoils(void)
{
    /* Bistable relay: only ever pulsed, never held - drop both coil drives
     * to reduce parasitic current, same as the original driver. */
    HAL_GPIO_WritePin(RELAY_SET_GPIO_Port, RELAY_SET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_RESET_GPIO_Port, RELAY_RESET_Pin, GPIO_PIN_RESET);
}

static void EnterDriverError(PidControl_Error_t reason)
{
    dutyCycle = 0.0f;
    SetPwmDutyCycle(0.0f);
    HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_COOLING_GPIO_Port, PID_COOLING_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_HEATING_GPIO_Port, PID_HEATING_Pin, GPIO_PIN_RESET);
    lastError = reason;
    driverState = DRIVER_STATE_ERROR;
}

/* ---- Driver / actuation stage ---------------------------------------------- */

/* One-time startup sequence: switch to heating direction, ramp to a test
 * duty cycle and measure the resulting current twice (catches short
 * circuits before real power is applied for any length of time), then
 * derive dutyCycleLimit so that currentLimit is not exceeded regardless of
 * supply voltage / Peltier resistance tolerance. Blocking (~100-150 ms) -
 * see the design note in pid_control.h for why that is fine here. */
static uint8_t RunStartupCalibration(void)
{
    driverDirection = 1.0f;
    SetRelayHeating();
    BusyWaitMs(RELAY_ACTUATION_MS);
    ReleaseRelayCoils();

    dutyCycle = 0.0f;
    dutyCycleLimit = 1.0f;
    dutyCycleOverload = 1.0f;
    currentMaxDC = 0.0f;
    SetPwmDutyCycle(0.0f);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    CurrentSensor_CalibrateZero(); /* PWM is at 0% here: genuine zero-current baseline */

    BusyWaitMs(10);

    dutyCycle = DUTY_CYCLE_START;
    SetPwmDutyCycle(dutyCycle);
    BusyWaitMs(4);

    measuredCurrent = CurrentSensor_Read();
    if (measuredCurrent > CURRENT_LIMIT_START_A)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        EnterDriverError(PID_ERROR_STARTUP_OVERCURRENT);
        return 0;
    }

    BusyWaitMs(4);
    measuredCurrent = CurrentSensor_Read();
    if (measuredCurrent > CURRENT_LIMIT_START_A)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        EnterDriverError(PID_ERROR_STARTUP_OVERCURRENT);
        return 0;
    }
    currentMaxDC = measuredCurrent;

    /* Target ~90% of the configured limit at the current/duty-cycle ratio just measured */
    dutyCycle = 0.9f * (currentLimit / currentMaxDC) * DUTY_CYCLE_START;
    if (dutyCycle > 1.0f) dutyCycle = 1.0f;
    SetPwmDutyCycle(dutyCycle);
    BusyWaitMs(8);
    currentMaxDC = CurrentSensor_Read();

    dutyCycleLimit = (currentLimit / currentMaxDC) * dutyCycle;
    if (dutyCycleLimit > 1.0f) dutyCycleLimit = 1.0f;
    dutyCycle = dutyCycleLimit;
    SetPwmDutyCycle(dutyCycle);
    BusyWaitMs(8);
    currentMaxDC = CurrentSensor_Read();

    if (currentMaxDC > (currentLimit + 0.5f))
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        EnterDriverError(PID_ERROR_STARTUP_OVERCURRENT);
        return 0;
    }

    dutyCycle = 0.0f;
    SetPwmDutyCycle(0.0f);
    currentControl = 0.0f;
    currentControlPrevious = 0.0f;
    dutyCyclePrevious = 0.0f;
    BusyWaitMs(5);

    HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_SET);
    driverLastTick = HAL_GetTick();
    return 1;
}

static void DriverStep(void)
{
    measuredCurrent = CurrentSensor_Read();

    switch (driverState)
    {
    case DRIVER_STATE_OFF:
        if (driverStartRequested)
        {
            driverStartRequested = 0;
            if (RunStartupCalibration())
                driverState = DRIVER_STATE_ON;
        }
        break;

    case DRIVER_STATE_ON:
        if (driverStopRequested)
        {
            driverStopRequested = 0;
            dutyCycle = 0.0f;
            SetPwmDutyCycle(0.0f);
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PID_COOLING_GPIO_Port, PID_COOLING_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PID_HEATING_GPIO_Port, PID_HEATING_Pin, GPIO_PIN_RESET);
            BusyWaitMs(4); /* let current settle to zero before the relay is allowed to switch */
            SetRelayHeating(); /* leave relay in a defined (heating) position, mirrors original driver */
            driverDirection = 1.0f;
            BusyWaitMs(RELAY_ACTUATION_MS);
            ReleaseRelayCoils();
            driverState = DRIVER_STATE_OFF;
            break;
        }

        if (pidState == PID_STATE_ERROR)
        {
            EnterDriverError(lastError == PID_ERROR_NONE ? PID_ERROR_SANITY : lastError);
            break;
        }

        if ((HAL_GetTick() - driverLastTick) < CURRENT_CYCLE_MS)
            break;
        driverLastTick = HAL_GetTick();

        if (relayActuatedCooldownCounter > 0)
            relayActuatedCooldownCounter--;
        else
            ReleaseRelayCoils();

        controlDirection = (pidOut > 0.0f) ? 1.0f : -1.0f;

        if (driverDirection * controlDirection < 0.0f)
        {
            /* Desired direction differs from the actuated one: hold at zero and
             * only actually flip the relay once the request is stable for
             * FLIP_TIME_MS, so we don't chatter the relay while the PID output
             * hovers around zero. */
            currentControl = 0.0f;
            directionFlipCounter++;
            if (directionFlipCounter > directionFlipThreshold)
            {
                driverDirection = controlDirection;
                if (driverDirection < 0.0f)
                    SetRelayCooling();
                else
                    SetRelayHeating();
                relayActuatedCooldownCounter = (uint8_t)(RELAY_ACTUATION_MS / CURRENT_CYCLE_MS);
                directionFlipCounter = 0;
            }
        }
        else
        {
            float currentControlMag = (currentControl < 0.0f) ? -currentControl : currentControl;

            if (directionFlipCounter > 0)
                directionFlipCounter--;

            if (currentUpdatedSettlingCycles > 1)
            {
                currentUpdatedSettlingCycles--;
                if (recalibrationCooldownCounter > 0)
                    recalibrationCooldownCounter--;
            }
            else
            {
                if (recalibrationCooldownCounter > 0)
                {
                    recalibrationCooldownCounter--;
                }
                else
                {
                    if (currentUpdatedSettlingCycles > 0)
                    {
                        currentUpdatedSettlingCycles--;
                    }
                    else if (measuredCurrent > (currentLimit + 0.5f))
                    {
                        /* current is single-ended/unipolar now (see current_sensor.h),
                         * so only one bound needs checking regardless of direction */
                        EnterDriverError(PID_ERROR_RUNTIME_OVERCURRENT);
                        break;
                    }
                }

                if (currentUpdatedSettlingCycles > 0)
                    currentUpdatedSettlingCycles--;

                /* Recalibrate dutyCycleLimit against the actually measured current
                 * whenever we're driving a significant load and the measurement
                 * has drifted from the commanded value - direction-agnostic now
                 * that current is a magnitude, unlike the original's separate
                 * heating/cooling branches. */
                if ((measuredCurrent > (0.5f * currentLimit) || currentControlMag > (0.5f * currentLimit)) &&
                    (measuredCurrent > (currentControlMag + 0.08f) || measuredCurrent < (currentControlMag - 0.08f)))
                {
                    dutyCycleLimit = (currentLimit / measuredCurrent) * dutyCycle;
                    if (dutyCycleLimit > 1.0f)
                    {
                        dutyCycleOverload = dutyCycleLimit;
                        dutyCycleLimit = 1.0f;
                    }
                    else
                    {
                        dutyCycleOverload = 1.0f;
                    }
                    recalibrationCooldownCounter = (uint8_t)(RECAL_COOLDOWN_MS / CURRENT_CYCLE_MS) + 1u;
                }
            }

            currentControl = pidOut;
            if (driverDirection > 0.0f)
            {
                if (currentControl > currentLimit) currentControl = currentLimit;
                if (currentControl < 0.0f) currentControl = 0.0f;
            }
            else
            {
                if (currentControl < -currentLimit) currentControl = -currentLimit;
                if (currentControl > 0.0f) currentControl = 0.0f;
            }
        }

        {
            float currentControlMagNow = (currentControl < 0.0f) ? -currentControl : currentControl;
            dutyCycle = dutyCycleLimit * dutyCycleOverload * (currentControlMagNow / currentLimit);
            if (measuredCurrent > 0.0f)
                dutyCycle -= ((measuredCurrent - ((currentControlPrevious < 0.0f) ? -currentControlPrevious : currentControlPrevious)) / measuredCurrent) * dutyCyclePrevious;
            if (dutyCycle < 0.0f) dutyCycle = 0.0f;
            if (dutyCycle > dutyCycleLimit) dutyCycle = dutyCycleLimit;
            SetPwmDutyCycle(dutyCycle);
        }

        currentControlPrevious = currentControl;
        dutyCyclePrevious = dutyCycle;
        break;

    case DRIVER_STATE_ERROR:
        /* Held until PidControl_ClearError() (I2C CONTROL register) - no local
         * button/LED exist on the PM board, unlike the original driver.
         * Discard any pending start/stop request accumulated while in this
         * state so clearing the error doesn't cause an unrequested auto-start. */
        driverStartRequested = 0;
        driverStopRequested = 0;
        break;
    }
}

/* ---- PID stage -------------------------------------------------------------- */
static void PushErrHistory(void)
{
    uint8_t i;
    for (i = 10; i > 0; i--)
        errHistory[i] = errHistory[i - 1];
    errHistory[0] = err;
}

static float WeightedDerivative(void)
{
    float sum = 0.0f;
    uint8_t i;
    for (i = 0; i < 11; i++)
        sum += ERR_FILT_CO * (err - errHistory[i]) * dWeight[i];
    return sum;
}

static void PidStep(void)
{
    switch (pidState)
    {
    case PID_STATE_OFF:
        PushErrHistory();
        err = T_SET - TempSensor_GetControlTemperature();

        /* A start request waits until the temperature measurement is valid
         * (calibration of the analog front-end right after boot takes ~300 ms),
         * so errStart / the sanity check are never based on a bogus 0 degC. */
        if (startRequested && TempSensor_IsReady())
        {
            startRequested = 0;
            errStart = err;
            tempUpdatedFlag = 0;
            driverStartRequested = 1;
            pidState = PID_STATE_WAIT_FOR_DRIVER;
        }
        break;

    case PID_STATE_WAIT_FOR_DRIVER:
        if (driverState == DRIVER_STATE_ON)
        {
            P_val = I_val = D_val = 0.0f;
            pidLastTick = HAL_GetTick();
            pidState = PID_STATE_WAIT;
        }
        else if (driverState == DRIVER_STATE_ERROR)
        {
            pidState = PID_STATE_OFF;
        }
        break;

    case PID_STATE_WAIT:
        if ((HAL_GetTick() - pidLastTick) >= PID_CYCLE_MS)
        {
            pidLastTick = HAL_GetTick();
            pidState = PID_STATE_CALCULATE;
        }
        break;

    case PID_STATE_CALCULATE:
        if (driverState == DRIVER_STATE_ERROR)
        {
            pidState = PID_STATE_OFF;
            break;
        }

        PushErrHistory();
        err = T_SET - TempSensor_GetControlTemperature();

        if (tempUpdatedFlag)
        {
            errStart = err;
            I_val = 0.0f;
            tempUpdatedFlag = 0;
        }
        else if ((errStart > 0.0f && err > (errStart + 3.0f)) || (errStart <= 0.0f && err < (errStart - 3.0f)))
        {
            lastError = PID_ERROR_SANITY;
            pidState = PID_STATE_ERROR;
            break;
        }

        P_val = kp * err;
        I_val += ki * err / PID_FREQ_HZ;
        if (I_val > I_SAT_P) I_val = I_SAT_P;
        if (I_val < I_SAT_N) I_val = I_SAT_N;
        D_val = kd * WeightedDerivative();
        pidOut = P_val + I_val + D_val;

        currentUpdatedSettlingCycles = (uint8_t)(CURRENT_UPDATE_COOLDOWN_MS / CURRENT_CYCLE_MS) + 1u;
        pidState = PID_STATE_WAIT;
        break;

    case PID_STATE_ERROR:
        /* Held until PidControl_ClearError(); driver error (if any) is cleared together with it. */
        break;
    }

    if (stopRequested)
    {
        startRequested = 0; /* also cancels a start that is still waiting for a valid temperature */

        if (pidState != PID_STATE_OFF && pidState != PID_STATE_ERROR)
        {
            driverStopRequested = 1;
            pidState = PID_STATE_OFF;
        }
    }
    stopRequested = 0;
}

/* ---- Public API --------------------------------------------------------------- */
void PidControl_Init(void)
{
    uint8_t i;
    for (i = 0; i < 11; i++)
        errHistory[i] = 0.0f;

    directionFlipThreshold = (uint8_t)(FLIP_TIME_MS / CURRENT_CYCLE_MS);

    ReleaseRelayCoils();
    HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_HEATING_GPIO_Port, PID_HEATING_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PID_COOLING_GPIO_Port, PID_COOLING_Pin, GPIO_PIN_RESET);
}

void PidControl_Process(void)
{
    PidStep();
    DriverStep();
}

void PidControl_Start(void)
{
    if (pidState == PID_STATE_OFF)
        startRequested = 1;
}

void PidControl_Stop(void)
{
    stopRequested = 1;
}

uint8_t PidControl_IsRunning(void)
{
    return (driverState == DRIVER_STATE_ON) ? 1u : 0u;
}

void PidControl_ClearError(void)
{
    if (driverState == DRIVER_STATE_ERROR)
        driverState = DRIVER_STATE_OFF;
    if (pidState == PID_STATE_ERROR)
        pidState = PID_STATE_OFF;
    lastError = PID_ERROR_NONE;
}

uint8_t PidControl_IsError(void)
{
    return (driverState == DRIVER_STATE_ERROR || pidState == PID_STATE_ERROR) ? 1u : 0u;
}

PidControl_Error_t PidControl_GetErrorCode(void)
{
    return lastError;
}

void PidControl_SetSetpoint(float tSetDegC)
{
    T_SET = tSetDegC;
    tempUpdatedFlag = 1;
}

float PidControl_GetSetpoint(void) { return T_SET; }

void PidControl_SetGains(float newKp, float newKi, float newKd)
{
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void PidControl_GetGains(float *outKp, float *outKi, float *outKd)
{
    if (outKp) *outKp = kp;
    if (outKi) *outKi = ki;
    if (outKd) *outKd = kd;
}

void PidControl_SetCurrentLimit(float limitA)
{
    currentLimit = (limitA < 0.0f) ? -limitA : limitA;
}

float PidControl_GetCurrentLimit(void) { return currentLimit; }

float PidControl_GetControllerOutput(void) { return pidOut; }
float PidControl_GetMeasuredCurrent(void) { return measuredCurrent; }
float PidControl_GetDutyCycle(void) { return dutyCycle; }

PidControl_Direction_t PidControl_GetDirection(void)
{
    return (driverDirection < 0.0f) ? PID_DIRECTION_COOLING : PID_DIRECTION_HEATING;
}
