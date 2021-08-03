/**
  I2C1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1.c

  @Summary
    This is the generated driver implementation file for the I2C1 driver.

  @Description
    This file provides common enumerations for I2C1 driver.
    Generation Information :
        Product Revision  :   - 
        Device            :  
        Driver Version    :  1.0.2
    The generated drivers are tested against the following:
        Compiler          :  XC8 v2.20 and above
        MPLAB             :  MPLABX v5.40  and above
*/

/*
Copyright (c) [2012-2020] Microchip Technology Inc.  

    All rights reserved.

    You are permitted to use the accompanying software and its derivatives 
    with Microchip products. See the Microchip license agreement accompanying 
    this software, if any, for additional info regarding your rights and 
    obligations.
    
    MICROCHIP SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
    WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
    LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
    AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP OR ITS
    LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT 
    LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE 
    THEORY FOR ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT 
    LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, 
    OR OTHER SIMILAR COSTS. 
    
    To the fullest extend allowed by law, Microchip and its licensors 
    liability will not exceed the amount of fees, if any, that you paid 
    directly to Microchip to use this software. 
    
    THIRD PARTY SOFTWARE:  Notwithstanding anything to the contrary, any 
    third party software accompanying this software is subject to the terms 
    and conditions of the third party's license agreement.  To the extent 
    required by third party licenses covering such third party software, 
    the terms of such license will apply in lieu of the terms provided in 
    this notice or applicable license.  To the extent the terms of such 
    third party licenses prohibit any of the restrictions described here, 
    such restrictions will not apply to such third party software.
*/

#include <xc.h>
#include "../i2c1.h"

// I2C1 STATES
typedef enum 
{
    I2C1_IDLE = 0,
    I2C1_SEND_ADR_READ,
    I2C1_SEND_ADR_WRITE,
    I2C1_TX,
    I2C1_RX,
    I2C1_TX_EMPTY,
    I2C1_RX_EMPTY,
    I2C1_SEND_RESTART_READ,
    I2C1_SEND_RESTART_WRITE,
    I2C1_SEND_RESTART,
    I2C1_SEND_STOP,
    I2C1_RX_ACK,
    I2C1_TX_ACK,
    I2C1_RX_NACK_STOP,
    I2C1_RX_NACK_RESTART,
    I2C1_RESET,
    I2C1_ADDRESS_NACK,
    I2C1_BUS_COLLISION,
    I2C1_BUS_ERROR
} i2c1_fsm_states_t;

// I2C1 Event callBack List
typedef enum 
{
    I2C1_DATA_COMPLETE = 0,
    I2C1_WRITE_COLLISION,
    I2C1_ADDR_NACK,
    I2C1_DATA_NACK,
    I2C1_TIMEOUT,
    I2C1_NULL
} i2c1_callbackIndex_t;

typedef struct
{
    size_t len;
    uint8_t *data;
} i2c1_buffer_t;

static  i2c1_operations_t rdBlkRegCompleteHandler(void *ptr);

// I2C1 Status Structure
typedef struct
{
    i2c1_callback_t callbackTable[6];
    void *callbackPayload[6];           //  each callBack can have a payload
    uint16_t time_out;                  // I2C1 Timeout Counter between I2C1 Events.
    uint16_t time_out_value;            // Reload value for the timeouts
    i2c1_address_t address;             // The I2C1 Address
    uint8_t *data_ptr;                  // pointer to a data buffer
    size_t data_length;                 // Bytes in the data buffer
    i2c1_fsm_states_t state;            // Driver State
    i2c1_error_t error;
    unsigned addressNackCheck:2;
    unsigned busy:1;
    unsigned inUse:1;
    unsigned bufferFree:1;

} i2c1_status_t;

static void I2C1_SetCallback(i2c1_callbackIndex_t idx, i2c1_callback_t cb, void *ptr);
static void I2C1_Poller(void);
static inline void I2C1_ClearInterruptFlags(void);
static inline void I2C1_HostFsm(void);

/* I2C1 interfaces */
static inline bool I2C1_HostOpen(void);
static inline void I2C1_HostClose(void);
static inline uint8_t I2C1_HostGetRxData(void);
static inline void I2C1_HostSendTxData(uint8_t data);
static inline void I2C1_HostSetCounter(uint8_t counter);
static inline uint8_t I2C1_HostGetCounter();
static inline void I2C1_HostResetBus(void);
static inline void I2C1_HostEnableRestart(void);
static inline void I2C1_HostDisableRestart(void);
static inline void I2C1_HostStop(void);
static inline bool I2C1_HostIsNack(void);
static inline void I2C1_HostSendAck(void);
static inline void I2C1_HostSendNack(void);
static inline void I2C1_HostClearBusCollision(void);
static inline bool I2C1_HostIsRxBufFull(void);
static inline bool I2C1_HostIsTxBufEmpty(void);
static inline bool I2C1_HostIsStopFlagSet(void);
static inline bool I2C1_HostIsCountFlagSet(void);
static inline bool I2C1_HostIsNackFlagSet(void);
static inline void I2C1_HostClearStopFlag(void);
static inline void I2C1_HostClearCountFlag(void);
static inline void I2C1_HostClearNackFlag(void);

/* Interrupt interfaces */
static inline void I2C1_HostEnableIrq(void);
static inline bool I2C1_HostIsIrqEnabled(void);
static inline void I2C1_HostDisableIrq(void);
static inline void I2C1_HostClearIrq(void);
static inline void I2C1_HostWaitForEvent(void);

static i2c1_fsm_states_t I2C1_DO_IDLE(void);
static i2c1_fsm_states_t I2C1_DO_SEND_ADR_READ(void);
static i2c1_fsm_states_t I2C1_DO_SEND_ADR_WRITE(void);
static i2c1_fsm_states_t I2C1_DO_TX(void);
static i2c1_fsm_states_t I2C1_DO_RX(void);
static i2c1_fsm_states_t I2C1_DO_TX_EMPTY(void);
static i2c1_fsm_states_t I2C1_DO_RX_EMPTY(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_READ(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_WRITE(void);
static i2c1_fsm_states_t I2C1_DO_SEND_RESTART(void);
static i2c1_fsm_states_t I2C1_DO_SEND_STOP(void);
static i2c1_fsm_states_t I2C1_DO_RX_ACK(void);
static i2c1_fsm_states_t I2C1_DO_TX_ACK(void);
static i2c1_fsm_states_t I2C1_DO_RX_NACK_STOP(void);
static i2c1_fsm_states_t I2C1_DO_RX_NACK_RESTART(void);
static i2c1_fsm_states_t I2C1_DO_RESET(void);
static i2c1_fsm_states_t I2C1_DO_ADDRESS_NACK(void);
static i2c1_fsm_states_t I2C1_DO_BUS_COLLISION(void);
static i2c1_fsm_states_t I2C1_DO_BUS_ERROR(void);

typedef i2c1_fsm_states_t (*i2c1FsmHandler)(void);
const i2c1FsmHandler i2c1_fsmStateTable[] = 
{
    I2C1_DO_IDLE,
    I2C1_DO_SEND_ADR_READ,
    I2C1_DO_SEND_ADR_WRITE,
    I2C1_DO_TX,
    I2C1_DO_RX,
    I2C1_DO_TX_EMPTY,
    I2C1_DO_RX_EMPTY,
    I2C1_DO_SEND_RESTART_READ,
    I2C1_DO_SEND_RESTART_WRITE,
    I2C1_DO_SEND_RESTART,
    I2C1_DO_SEND_STOP,
    I2C1_DO_RX_ACK,
    I2C1_DO_TX_ACK,
    I2C1_DO_RX_NACK_STOP,
    I2C1_DO_RX_NACK_RESTART,
    I2C1_DO_RESET,
    I2C1_DO_ADDRESS_NACK,
    I2C1_DO_BUS_COLLISION,
    I2C1_DO_BUS_ERROR
};

/**
  Section: Driver Interface
*/
const struct I2C_HOST_INTERFACE i2c1_host_Interface = {
  .Initialize = I2C1_Initialize,
  .Write = I2C1_Write,
  .Read = I2C1_Read,
  .WriteRead = I2C1_WriteRead,
  .TransferSetup = NULL,
  .ErrorGet = I2C1_ErrorGet,
  .IsBusy = I2C1_IsBusy,
  .CallbackRegister = I2C1_CallbackRegister
};

/**
 Section: Private Variable Definitions
*/
static void (*I2C1_InterruptHandler)(void) = NULL;
i2c1_status_t I2C1_Status = {0};
enum I2C_ERROR i2c1ErrorState = I2C_ERROR_NONE;

i2c1_error_t I2C1_Open(i2c1_address_t address)
{
    i2c1_error_t returnValue = I2C1_BUSY;
    
    if(!I2C1_Status.inUse)
    {
        I2C1_Status.address = address;
        I2C1_Status.busy = 0;
        I2C1_Status.inUse = 1;
        I2C1_Status.addressNackCheck = 0;
        I2C1_Status.state = I2C1_RESET;
        I2C1_Status.time_out_value = 500; // MCC should determine a reasonable starting value here.
        I2C1_Status.bufferFree = 1;

        // set all the call backs to a default of sending stop
        I2C1_Status.callbackTable[I2C1_DATA_COMPLETE]=I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE] = NULL;
        I2C1_Status.callbackTable[I2C1_WRITE_COLLISION]=I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_WRITE_COLLISION] = NULL;
        I2C1_Status.callbackTable[I2C1_ADDR_NACK]=I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_ADDR_NACK] = NULL;
        I2C1_Status.callbackTable[I2C1_DATA_NACK]=I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[I2C1_DATA_NACK] = NULL;
        I2C1_Status.callbackTable[I2C1_TIMEOUT]=I2C1_CallbackReturnReset;
        I2C1_Status.callbackPayload[I2C1_TIMEOUT] = NULL;
        
        I2C1_HostClearIrq();
        I2C1_HostOpen();
        i2c1ErrorState = I2C_ERROR_NONE;
        returnValue = I2C1_NOERR;
    }
    return returnValue;
}

i2c1_error_t I2C1_Close(void)
{
    i2c1_error_t returnValue = I2C1_BUSY;
    if(!I2C1_Status.busy)
    {
        I2C1_Status.inUse = 0;
        I2C1_Status.address = 0xff;
        I2C1_HostClearIrq();
        I2C1_HostDisableIrq();
        I2C1_HostClose();
        returnValue = I2C1_Status.error;
    }
    return returnValue;
}


i2c1_error_t I2C1_HostRead(void)
{
    i2c1_error_t returnValue = I2C1_BUSY;
    if(!I2C1_Status.busy)
    {
        I2C1_Status.busy = true;
        returnValue = I2C1_NOERR;
        I2C1_HostSetCounter((uint8_t) I2C1_Status.data_length);
        I2C1_Status.state = I2C1_RX;
        I2C1_DO_SEND_ADR_READ();
        I2C1_Poller();
    }
    return returnValue;
    
}

i2c1_error_t I2C1_HostWrite(void)
{
    i2c1_error_t returnValue = I2C1_BUSY;
    if(!I2C1_Status.busy)
    {
        I2C1_Status.busy = true;
        returnValue = I2C1_NOERR;
        I2C1_HostSetCounter((uint8_t) I2C1_Status.data_length);
        I2C1_Status.state = I2C1_TX;
        I2C1_DO_SEND_ADR_WRITE();
        I2C1_Poller();
    }
    return returnValue;
}

void I2C1_SetTimeout(uint8_t timeOutValue)
{
    I2C1_HostDisableIrq();
    I2C1_Status.time_out_value = timeOutValue;
    I2C1_HostEnableIrq();
}

void I2C1_SetBuffer(void *buffer, size_t bufferSize)
{
    if(I2C1_Status.bufferFree)
    {
        I2C1_Status.data_ptr = buffer;
        I2C1_Status.data_length = bufferSize;
        I2C1_Status.bufferFree = false;
    }
}

void I2C1_SetDataCompleteCallback(i2c1_callback_t cb, void *ptr)
{
    I2C1_SetCallback(I2C1_DATA_COMPLETE, cb, ptr);
}

void I2C1_SetWriteCollisionCallback(i2c1_callback_t cb, void *ptr)
{
    I2C1_SetCallback(I2C1_WRITE_COLLISION, cb, ptr);
}

void I2C1_SetAddressNackCallback(i2c1_callback_t cb, void *ptr)
{
    I2C1_SetCallback(I2C1_ADDR_NACK, cb, ptr);
}

void I2C1_SetDataNackCallback(i2c1_callback_t cb, void *ptr)
{
    I2C1_SetCallback(I2C1_DATA_NACK, cb, ptr);
}

void I2C1_SetTimeoutCallback(i2c1_callback_t cb, void *ptr)
{
    I2C1_SetCallback(I2C1_TIMEOUT, cb, ptr);
}

static void I2C1_SetCallback(i2c1_callbackIndex_t idx, i2c1_callback_t cb, void *ptr)
{
    if(cb)
    {
        I2C1_Status.callbackTable[idx] = cb;
        I2C1_Status.callbackPayload[idx] = ptr;
    }
    else
    {
        I2C1_Status.callbackTable[idx] = I2C1_CallbackReturnStop;
        I2C1_Status.callbackPayload[idx] = NULL;
    }
}

static void I2C1_Poller(void)
{
    while(I2C1_Status.busy)
    {
        I2C1_HostWaitForEvent();
        I2C1_HostFsm();
    }
}

static inline void I2C1_HostFsm(void)

{
    I2C1_ClearInterruptFlags();

    if(I2C1_Status.addressNackCheck && I2C1_HostIsNack())
    {
        I2C1_Status.state = I2C1_ADDRESS_NACK;
    }
    I2C1_Status.state = i2c1_fsmStateTable[I2C1_Status.state]();
}

static inline void I2C1_ClearInterruptFlags(void)
{
    if(I2C1_HostIsCountFlagSet())
    {
        I2C1_HostClearCountFlag();
    }
    else if(I2C1_HostIsStopFlagSet())
    {
        I2C1_HostClearStopFlag();
    }
    else if(I2C1_HostIsNackFlagSet())
    {
        I2C1_HostClearNackFlag();
    }
}

static i2c1_fsm_states_t I2C1_DO_IDLE(void)
{
    I2C1_Status.busy = false;
    I2C1_Status.error = I2C1_NOERR;
    return I2C1_RESET;
}

static i2c1_fsm_states_t I2C1_DO_SEND_ADR_READ(void)
{
    I2C1_Status.addressNackCheck = 2;
    if(I2C1_Status.data_length ==  1)
    {
        I2C1_DO_RX_EMPTY();
    }
    I2C1_HostSendTxData((uint8_t) (I2C1_Status.address << 1 | 1));
    return I2C1_RX;
}

static i2c1_fsm_states_t I2C1_DO_SEND_ADR_WRITE(void)
{
    I2C1_Status.addressNackCheck = 2;
    I2C1_HostSendTxData((uint8_t) (I2C1_Status.address << 1));
    return I2C1_TX;
}

static i2c1_fsm_states_t I2C1_DO_TX(void)
{
    if(I2C1_HostIsNack())
    {
        switch(I2C1_Status.callbackTable[I2C1_DATA_NACK](I2C1_Status.callbackPayload[I2C1_DATA_NACK]))
        {
            case I2C1_RESTART_READ:
                return I2C1_DO_SEND_RESTART_READ();
            case I2C1_RESTART_WRITE:
                  return I2C1_DO_SEND_RESTART_WRITE();
            default:
            case I2C1_CONTINUE:
            case I2C1_STOP:
                return I2C1_IDLE;
        }
    }
    else if(I2C1_HostIsTxBufEmpty())
    {
        if(I2C1_Status.addressNackCheck)
        {
            I2C1_Status.addressNackCheck--;
        }
        uint8_t dataTx = *I2C1_Status.data_ptr++;
        i2c1_fsm_states_t retFsmState = (--I2C1_Status.data_length)?I2C1_TX:I2C1_DO_TX_EMPTY();
        I2C1_HostSendTxData(dataTx);
        return retFsmState;
    }
    else
    {
        return I2C1_TX;
    }
}

static i2c1_fsm_states_t I2C1_DO_RX(void)
{
    if(!I2C1_HostIsRxBufFull())
    {
        return I2C1_RX;
    }
    if(I2C1_Status.addressNackCheck)
    {
        I2C1_Status.addressNackCheck--;
    }

    if(--I2C1_Status.data_length)
    {
        *I2C1_Status.data_ptr++ = I2C1_HostGetRxData();
        return I2C1_RX;
    }
    else
    {
        i2c1_fsm_states_t retFsmState = I2C1_DO_RX_EMPTY();
        *I2C1_Status.data_ptr++ = I2C1_HostGetRxData();
        return retFsmState;
    }
}

static i2c1_fsm_states_t I2C1_DO_TX_EMPTY(void)
{
    I2C1_Status.bufferFree = true;
    switch(I2C1_Status.callbackTable[I2C1_DATA_COMPLETE](I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE]))
    {
        case I2C1_RESTART_READ:
            I2C1_HostEnableRestart();
            return I2C1_SEND_RESTART_READ;
        case I2C1_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C1_HostSetCounter((uint8_t) (I2C1_Status.data_length + 1));
            return I2C1_TX;
        default:
        case I2C1_STOP:
            I2C1_HostDisableRestart();
            return I2C1_SEND_STOP;
    }
}

static i2c1_fsm_states_t I2C1_DO_RX_EMPTY(void)
{
    I2C1_Status.bufferFree = true;
    switch(I2C1_Status.callbackTable[I2C1_DATA_COMPLETE](I2C1_Status.callbackPayload[I2C1_DATA_COMPLETE]))
    {
        case I2C1_RESTART_WRITE:
            I2C1_HostEnableRestart();
            return I2C1_SEND_RESTART_WRITE;
        case I2C1_RESTART_READ:
            I2C1_HostEnableRestart();
            return I2C1_SEND_RESTART_READ;
        case I2C1_CONTINUE:
            // Avoid the counter stop condition , Counter is incremented by 1
            I2C1_HostSetCounter((uint8_t) I2C1_Status.data_length + 1);
            return I2C1_RX;
        default:
        case I2C1_STOP:
            if(I2C1_Status.state != I2C1_SEND_RESTART_READ)
            {
                I2C1_HostDisableRestart();
            }
            return I2C1_RESET;
    }
}

static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_READ(void)
{
    I2C1_HostSetCounter((uint8_t) I2C1_Status.data_length);
    return I2C1_DO_SEND_ADR_READ();
}

static i2c1_fsm_states_t I2C1_DO_SEND_RESTART_WRITE(void)
{
    return I2C1_SEND_ADR_WRITE;
}


static i2c1_fsm_states_t I2C1_DO_SEND_RESTART(void)
{
    return I2C1_SEND_ADR_READ;
}

static i2c1_fsm_states_t I2C1_DO_SEND_STOP(void)
{
    I2C1_HostStop();
    if(I2C1_HostGetCounter())
    {
        I2C1_HostSetCounter(0);
        I2C1_HostSendTxData(0);
    }
    return I2C1_IDLE;
}

static i2c1_fsm_states_t I2C1_DO_RX_ACK(void)
{
    I2C1_HostSendAck();
    return I2C1_RX;
}

static i2c1_fsm_states_t I2C1_DO_TX_ACK(void)
{
    I2C1_HostSendAck();
    return I2C1_TX;
}

static i2c1_fsm_states_t I2C1_DO_RX_NACK_STOP(void)
{
    I2C1_HostSendNack();
    I2C1_HostStop();
    return I2C1_DO_IDLE();
}

static i2c1_fsm_states_t I2C1_DO_RX_NACK_RESTART(void)
{
    I2C1_HostSendNack();
    return I2C1_SEND_RESTART;
}

static i2c1_fsm_states_t I2C1_DO_RESET(void)
{
    I2C1_HostResetBus();
    I2C1_Status.busy = false;
    I2C1_Status.error = I2C1_NOERR;
    return I2C1_RESET;
}
static i2c1_fsm_states_t I2C1_DO_ADDRESS_NACK(void)
{
    I2C1_Status.addressNackCheck = 0;
    I2C1_Status.error = I2C1_FAIL;
    I2C1_Status.busy = false;
    switch(I2C1_Status.callbackTable[I2C1_ADDR_NACK](I2C1_Status.callbackPayload[I2C1_ADDR_NACK]))
    {
        case I2C1_RESTART_READ:
        case I2C1_RESTART_WRITE:
            return I2C1_DO_SEND_RESTART();
        default:
            i2c1ErrorState = I2C_ERROR_NACK;
            return I2C1_RESET;
    }
}

static i2c1_fsm_states_t I2C1_DO_BUS_COLLISION(void)
{
    // Clear bus collision status flag
    I2C1_HostClearIrq();

    I2C1_Status.error = I2C1_FAIL;
    switch (I2C1_Status.callbackTable[I2C1_WRITE_COLLISION](I2C1_Status.callbackPayload[I2C1_WRITE_COLLISION])) {
    case I2C1_RESTART_READ:
        return I2C1_DO_SEND_RESTART_READ();
    case I2C1_RESTART_WRITE:
        return I2C1_DO_SEND_RESTART_WRITE();
    default:
        i2c1ErrorState = I2C_ERROR_BUS_COLLISION;
        return I2C1_DO_RESET();
    }
}

static i2c1_fsm_states_t I2C1_DO_BUS_ERROR(void)
{
    I2C1_HostResetBus();
    I2C1_Status.busy  = false;
    I2C1_Status.error = I2C1_FAIL;
    return I2C1_RESET;
}

void I2C1_BusCollisionIsr(void)
{
    I2C1_HostClearBusCollision();
    I2C1_Status.state = I2C1_RESET;
}

i2c1_operations_t I2C1_CallbackReturnStop(void *funPtr)
{
    return I2C1_STOP;
}

i2c1_operations_t I2C1_CallbackReturnReset(void *funPtr)
{
    return I2C1_RESET_LINK;
}

i2c1_operations_t I2C1_CallbackRestartWrite(void *funPtr)
{
    return I2C1_RESTART_WRITE;
}

i2c1_operations_t I2C1_CallbackRestartRead(void *funPtr)
{
    return I2C1_RESTART_READ;
}

/* I2C1 Register Level interfaces */
static inline bool I2C1_HostOpen(void)
{
    if(!I2C1CON0bits.EN)
    {
        //WRIF Data byte not detected; CNTIF Byte count is not zero; RSCIF Restart condition not detected; PCIF Stop condition not detected; ACKTIF Acknowledge sequence not detected; ADRIF Address not detected; SCIF Start condition not detected; 
        I2C1PIR = 0x0;
        //CNTIE disabled; RSCIE disabled; ACKTIE disabled; SCIE disabled; PCIE disabled; ADRIE disabled; WRIE disabled; 
        I2C1PIE = 0x0;
        //BTOIE disabled; BTOIF No bus timeout; NACKIF No NACK/Error detected; BCLIE disabled; BCLIF No bus collision detected; NACKIE disabled; 
        I2C1ERR = 0x0;
        //Count register
        I2C1CNTL = 0x0;
        I2C1CNTH = 0x0;
        //Enable I2C1
        I2C1CON0bits.EN = 1;
        return true;
    }
    return false;
}

static inline void I2C1_HostClose(void)
{
    //Disable I2C1
    I2C1CON0bits.EN = 0;
    //WRIF Data byte not detected; CNTIF Byte count is not zero; RSCIF Restart condition not detected; PCIF Stop condition not detected; ACKTIF Acknowledge sequence not detected; ADRIF Address not detected; SCIF Start condition not detected; 
    I2C1PIR = 0x0;
    //Set Clear Buffer Flag
    I2C1STAT1bits.CLRBF = 1;
}

static inline uint8_t I2C1_HostGetRxData(void)
{
    return I2C1RXB;
}

static inline void I2C1_HostSendTxData(uint8_t data)
{
    I2C1TXB  = data;
}

static inline uint8_t I2C1_HostGetCounter()
{
    return I2C1CNTL;
}

static inline void I2C1_HostSetCounter(uint8_t counter)
{
    I2C1CNTL = counter;
    I2C1CNTH = 0x00;
}

static inline void I2C1_HostResetBus(void)
{
    //Disable I2C1
    I2C1CON0bits.EN = 0;
    //Set Clear Buffer Flag
    I2C1STAT1bits.CLRBF = 1;
    //Enable I2C1
    I2C1CON0bits.EN = 1;
}

static inline void I2C1_HostEnableRestart(void)
{
    //Enable I2C1 Restart
    I2C1CON0bits.RSEN = 1;
}

static inline void I2C1_HostDisableRestart(void)
{
    //Disable I2C1 Restart
    I2C1CON0bits.RSEN = 0;
}

static inline void I2C1_HostStop(void)
{
    //Clear Start Bit
    I2C1CON0bits.S = 0;
}

static inline bool I2C1_HostIsNack(void)
{
    return I2C1CON1bits.ACKSTAT;
}

static inline void I2C1_HostSendAck(void)
{
    I2C1CON1bits.ACKDT = 0;
}

static inline void I2C1_HostSendNack(void)
{
    I2C1CON1bits.ACKDT = 1;
}

static inline void I2C1_HostClearBusCollision(void)
{
    I2C1ERRbits.BCLIF = 0;
    I2C1ERRbits.BTOIF = 0;
    I2C1ERRbits.NACKIF = 0;
}

static inline bool I2C1_HostIsRxBufFull(void)
{
    return I2C1STAT1bits.RXBF;
}

static inline bool I2C1_HostIsTxBufEmpty(void)
{
    return I2C1STAT1bits.TXBE;
}

static inline bool I2C1_HostIsStopFlagSet(void)
{
    return I2C1PIRbits.PCIF;
}

static inline bool I2C1_HostIsCountFlagSet(void)
{
    return I2C1PIRbits.CNTIF;
}

static inline bool I2C1_HostIsNackFlagSet(void)
{
    return I2C1ERRbits.NACKIF;
}

static inline void I2C1_HostClearStopFlag(void)
{
    I2C1PIRbits.PCIF = 0;
}

static inline void I2C1_HostClearCountFlag(void)
{
    I2C1PIRbits.CNTIF = 0;
}

static inline void I2C1_HostClearNackFlag(void)
{
    I2C1ERRbits.NACKIF = 0;
}

static inline void I2C1_HostEnableIrq(void)
{
    PIE7bits.I2C1IE = 1;
    PIE7bits.I2C1EIE = 1;
    PIE7bits.I2C1RXIE = 1;
    PIE7bits.I2C1TXIE = 1;

    I2C1PIEbits.PCIE = 1; 
    I2C1PIEbits.CNTIE = 1; 
    I2C1ERRbits.NACKIE = 1; 
}

static inline bool I2C1_HostIsIrqEnabled(void)
{
    return (PIE7bits.I2C1RXIE && PIE7bits.I2C1TXIE && PIE7bits.I2C1IE);
}

static inline void I2C1_HostDisableIrq(void)
{
    PIE7bits.I2C1IE = 0;
    PIE7bits.I2C1EIE = 0;
    PIE7bits.I2C1RXIE = 0;
    PIE7bits.I2C1TXIE = 0;
    I2C1PIEbits.SCIE = 0;
    I2C1PIEbits.PCIE = 0;
    I2C1PIEbits.CNTIE = 0;
    I2C1PIEbits.ACKTIE = 0;
    I2C1PIEbits.RSCIE = 0;
    I2C1ERRbits.BCLIE = 0;
    I2C1ERRbits.BTOIE = 0;
    I2C1ERRbits.NACKIE = 0;
}

static inline void I2C1_HostClearIrq(void)
{
    I2C1PIR = 0x00;
}

static inline void I2C1_HostWaitForEvent(void)
{
    while(1)
    {
        if(PIR7bits.I2C1TXIF)
        {    
            break;
        }
        if(PIR7bits.I2C1RXIF)
        {  
            break;
        } 
        if(I2C1PIRbits.PCIF)
        {
            break;
        } 
        if(I2C1PIRbits.CNTIF)
        {
            break;
        }
        if(I2C1ERRbits.NACKIF)
        {
            break;
        }
    }
}

static  i2c1_operations_t rdBlkRegCompleteHandler(void *ptr)
{
     I2C1_SetBuffer((( i2c1_buffer_t *)ptr)->data,(( i2c1_buffer_t*)ptr)->len);
     I2C1_SetDataCompleteCallback(NULL,NULL);
    return  I2C1_RESTART_READ;
}

/**
 Section: Driver Interface Function Definitions
*/
void I2C1_Initialize(void)
{
    //CSTR Enable clocking; S Cleared by hardware after Start; MODE 7-bit address; EN disabled; RSEN disabled; 
    I2C1CON0 = 0x4;
    //TXU No underflow; CSD Clock Stretching enabled; RXO No overflow; ACKDT Acknowledge; ACKCNT Not Acknowledge; 
    I2C1CON1 = 0x80;
    //ABD disabled; GCEN disabled; ACNT disabled; SDAHT 30 ns hold time; BFRET 8 I2C Clock pulses; FME disabled; 
    I2C1CON2 = 0x18;
    //CLK MFINTOSC; 
    I2C1CLK = 0x3;
    //WRIF Data byte not detected; CNTIF Byte count is not zero; RSCIF Restart condition not detected; PCIF Stop condition not detected; ACKTIF Acknowledge sequence not detected; ADRIF Address not detected; SCIF Start condition not detected; 
    I2C1PIR = 0x0;
    //CNTIE disabled; RSCIE disabled; ACKTIE disabled; SCIE disabled; PCIE disabled; ADRIE disabled; WRIE disabled; 
    I2C1PIE = 0x0;
    //BTOIE disabled; BTOIF No bus timeout; NACKIF No NACK/Error detected; BCLIE disabled; BCLIF No bus collision detected; NACKIE disabled; 
    I2C1ERR = 0x0;
    //Count register
    I2C1CNTL = 0x0;
    I2C1CNTH = 0x0;
}

bool I2C1_Write(uint16_t address, uint8_t *data, size_t dataLength)
{
    bool status = true;
    while(!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_SetBuffer(data,dataLength);
    I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
    I2C1_HostWrite();
    while(I2C1_BUSY == I2C1_Close());
    return status;
}

bool I2C1_Read(uint16_t address, uint8_t *data, size_t dataLength)
{
    bool status = true;
    while(!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_SetBuffer(data,dataLength);
    I2C1_HostRead();
    while(I2C1_BUSY == I2C1_Close()); // sit here until finished.
    return status;
}

bool I2C1_WriteRead(uint16_t address, uint8_t *writeData, size_t writeLength, uint8_t *readData, size_t readLength)
{
    bool status = true;
    i2c1_buffer_t bufferBlock; // result is little endian
    bufferBlock.data = readData;
    bufferBlock.len = readLength;
    while(!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_SetDataCompleteCallback(rdBlkRegCompleteHandler,&bufferBlock);
    I2C1_SetBuffer(writeData,writeLength);
    I2C1_SetAddressNackCallback(NULL,NULL); //NACK polling?
    I2C1_HostWrite();
    while(I2C1_BUSY == I2C1_Close()); // sit here until finished.    
    return status;
}

enum I2C_ERROR I2C1_ErrorGet(void)
{
    return i2c1ErrorState;
}

bool I2C1_IsBusy(void)
{
    return I2C1_Status.inUse;
}

void I2C1_CallbackRegister(void (*handler)(void))
{
    I2C1_InterruptHandler = handler;
}