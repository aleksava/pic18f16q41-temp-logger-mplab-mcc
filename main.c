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
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/i2c_host/i2c_simple_host.h"
#include "mcc_generated_files/data_streamer/data_streamer.h"

/* Temperature sensor */
#define I2C_MCP9800_CLIENT_ADDR         0x49	/* 7-bit address */
#define MCP9800_REG_ADDR_TEMP           0x00
#define MCP9800_REG_ADDR_CONFIG         0x01
#define MCP9800_REG_DATA_CONFIG         0x60

/* IO - expander */
#define I2C_MCP23008_CLIENT_ADDR        0x20
#define MCP23008_REG_ADDR_IODIR         0x00
#define MCP23008_REG_ADDR_GPIO          0x09
#define PINS_DIGITAL_OUTPUT             0x00

uint8_t tempToGPIO(uint16_t rawTempValue);

bool TC_flag = false;

void TC_overflow_cb(void){
    LED0_Toggle();
    TC_flag = true;
}

int main(void)
{
    SYSTEM_Initialize();

    Timer.TimeoutCallbackRegister(TC_overflow_cb);

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();
    
    /* Declare variables */
    float degreesCelsius;
    uint16_t rawTemp;
    uint8_t dataRead[2];
    uint8_t dataWrite[2];

    
    /* Set the extended pins as digital output */
    dataWrite[0] = MCP23008_REG_ADDR_IODIR;
    dataWrite[1] = PINS_DIGITAL_OUTPUT;
    i2c_writeNBytes(I2C_MCP23008_CLIENT_ADDR,dataWrite, 2);
    
    
    /* Set the resolution to 12-bits */
    dataWrite[0] = MCP9800_REG_ADDR_CONFIG;
    dataWrite[1] = MCP9800_REG_DATA_CONFIG;
    i2c_writeNBytes(I2C_MCP9800_CLIENT_ADDR,dataWrite, 2);
    
    /* Set register we want to read from to Temperature register */
    dataWrite[0] = MCP9800_REG_DATA_CONFIG;
    i2c_writeNBytes(I2C_MCP9800_CLIENT_ADDR,dataWrite, 1);
    
    while(1)
    {
        if(TC_flag)
        {
            /* Read out the 12-bit raw temperature value */
            i2c_readNBytes(I2C_MCP9800_CLIENT_ADDR, dataRead, 2);

            /* Make on 16-bit value from the 2 bytes read from the Temp sensor */
            rawTemp = (uint16_t) ((dataRead[0] << 4) | (dataRead[1] >> 4));
            
            /* Write to I/O Expander based on rawTemp value */
            dataWrite[0] = MCP23008_REG_ADDR_GPIO;
            dataWrite[1] = tempToGPIO(rawTemp);
            i2c_writeNBytes(I2C_MCP23008_CLIENT_ADDR, dataWrite, 2);

            /* Convert the raw temperature data to degrees Celsius */
            degreesCelsius = (float) rawTemp / 16.0;
            
            /* Write to data visualizer */
            variableWrite_SendFrame(degreesCelsius);
            TC_flag = false;
        }
    }    
}


uint8_t tempToGPIO(uint16_t rawTemp)
{
    uint16_t minTemp = 24*16;
    uint16_t interval = 8;
    uint16_t i = (rawTemp - minTemp) / interval;
    if (i > 7) i = 7;
    switch(i)
    {
        case 0 : return 0x01;
        case 1 : return 0x03;
        case 2 : return 0x07;
        case 3 : return 0x0F;
        case 4 : return 0x1F;
        case 5 : return 0x3F;
        case 6 : return 0x7F;
        case 7 : return 0xFF;
    }
}