/**
 ******************************************************************************
 * @file    i2c_comm.c
 * @brief   See i2c_comm.h.
 ******************************************************************************
 */
#include "i2c_comm.h"
#include "pid_control.h"
#include "temperature_sensor.h"
#include "current_sensor.h"
#include <string.h>
#include <stddef.h>

extern I2C_HandleTypeDef hi2c1; /* initialised by MX_I2C1_Init() in main.c */

#define I2C_SLAVE_ADDR_7BIT   0x28u  /* TODO: confirm/adjust to the address the master expects */

typedef struct __attribute__((packed))
{
    uint8_t  status;      /* 0x00 RO */
    uint8_t  control;     /* 0x01 RW, write-triggered */
    uint8_t  errorCode;   /* 0x02 RO */
    uint8_t  channelSel;  /* 0x03 RW */
    int16_t  tSet;        /* 0x04 RW, degC x100 */
    int16_t  tExt;        /* 0x06 RW, degC x100 */
    int16_t  tActual;     /* 0x08 RO, degC x100 */
    int16_t  tCh1;        /* 0x0A RO, degC x100 */
    int16_t  tCh2;        /* 0x0C RO, degC x100 */
    int16_t  tCh3;        /* 0x0E RO, degC x100 */
    uint16_t kp;           /* 0x10 RW, x1000 */
    uint16_t ki;           /* 0x12 RW, x1000 */
    uint16_t kd;           /* 0x14 RW, x1000 */
    uint16_t iLimit;       /* 0x16 RW, mA */
    uint16_t iActual;      /* 0x18 RO, mA */
    uint16_t dutyCycle;    /* 0x1A RO, x1000 */
} RegisterMap_t;

#define REG_MAP_SIZE ((uint16_t)sizeof(RegisterMap_t))

static RegisterMap_t regs;
static uint8_t regPointer = 0;
static uint8_t rxTemp[1u + sizeof(RegisterMap_t)]; /* pointer byte + up to a full register-map write */
static volatile uint8_t pendingControlWrite = 0;
static volatile uint8_t pendingConfigWrite = 0;

/* ---- main-loop side ------------------------------------------------------- */
void I2cComm_Init(void)
{
    memset(&regs, 0, sizeof(regs));
    regPointer = 0;
    pendingControlWrite = 0;
    pendingConfigWrite = 0;

    /* MX_I2C1_Init() (CubeMX-generated) leaves OwnAddress1 at 0; set the real
     * 7-bit slave address here rather than editing the generated init code. */
    hi2c1.Instance->OAR1 = I2C_OAR1_OA1EN | ((uint32_t)I2C_SLAVE_ADDR_7BIT << 1);

    HAL_I2C_EnableListen_IT(&hi2c1);
}

void I2cComm_Update(void)
{
    /* Refresh read-only telemetry */
    regs.status = (uint8_t)((PidControl_IsRunning() ? 0x01u : 0u) |
                             (PidControl_IsError() ? 0x02u : 0u) |
                             ((PidControl_GetDirection() == PID_DIRECTION_COOLING) ? 0x04u : 0u));
    regs.errorCode = (uint8_t)PidControl_GetErrorCode();
    regs.tActual = (int16_t)(TempSensor_GetControlTemperature() * 100.0f);
    regs.tCh1 = (int16_t)(TempSensor_GetTemperature(TEMP_CHANNEL_1) * 100.0f);
    regs.tCh2 = (int16_t)(TempSensor_GetTemperature(TEMP_CHANNEL_2) * 100.0f);
    regs.tCh3 = (int16_t)(TempSensor_GetTemperature(TEMP_CHANNEL_3) * 100.0f);
    regs.iActual = (uint16_t)(PidControl_GetMeasuredCurrent() * 1000.0f);
    regs.dutyCycle = (uint16_t)(PidControl_GetDutyCycle() * 1000.0f);
    /* NOTE: regs is written here (main-loop context) and read back-to-back
     * by the I2C ISR while servicing a master read. On the rare chance a
     * read transaction spans exactly across this update, a multi-byte
     * telemetry field could be read torn (half old / half new value); the
     * next poll self-corrects. Acceptable for telemetry; add double
     * buffering here if a consumer needs stricter consistency. */

    if (pendingControlWrite)
    {
        pendingControlWrite = 0;
        if (regs.control & 0x01u) PidControl_Start();
        if (regs.control & 0x02u) PidControl_Stop();
        if (regs.control & 0x04u) PidControl_ClearError();
        regs.control = 0;
    }

    if (pendingConfigWrite)
    {
        pendingConfigWrite = 0;
        PidControl_SetSetpoint(regs.tSet / 100.0f);
        TempSensor_SetExternalTemperature(regs.tExt / 100.0f);
        if (regs.channelSel <= TEMP_SOURCE_CH3)
            TempSensor_SetSource((TempSensor_Source_t)regs.channelSel);
        PidControl_SetGains(regs.kp / 1000.0f, regs.ki / 1000.0f, regs.kd / 1000.0f);
        PidControl_SetCurrentLimit(regs.iLimit / 1000.0f);
    }
}

/* ---- ISR side --------------------------------------------------------------
 * Kept minimal on purpose: copy bytes in/out of the register map and set
 * flags; all actual application logic runs from I2cComm_Update() above. */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;
    if (hi2c->Instance != I2C1)
        return;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT)
    {
        /* Master is writing to us: receive the pointer byte plus however
         * many data bytes follow before the master issues a STOP. */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxTemp, sizeof(rxTemp), I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
        if (regPointer >= REG_MAP_SIZE)
            regPointer = 0;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, ((uint8_t *)&regs) + regPointer, (uint16_t)(REG_MAP_SIZE - regPointer), I2C_FIRST_AND_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint16_t received;

    if (hi2c->Instance != I2C1)
        return;

    /* XferCount is decremented as bytes arrive; what's left tells us how
     * many of the requested sizeof(rxTemp) bytes were actually clocked in
     * before the master's STOP (a slave receiver does not know the
     * transaction length in advance). */
    received = (uint16_t)sizeof(rxTemp) - hi2c->XferCount;

    if (received >= 1u)
    {
        regPointer = rxTemp[0];
        if (regPointer >= REG_MAP_SIZE)
            regPointer = 0;

        if (received > 1u)
        {
            uint16_t dataLen = received - 1u;
            uint16_t space = REG_MAP_SIZE - regPointer;
            if (dataLen > space)
                dataLen = space;
            memcpy(((uint8_t *)&regs) + regPointer, &rxTemp[1], dataLen);

            if (regPointer == offsetof(RegisterMap_t, control))
                pendingControlWrite = 1;
            else
                pendingConfigWrite = 1;
        }
    }

    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
        return;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
        return;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
        return;
    HAL_I2C_EnableListen_IT(hi2c);
}
