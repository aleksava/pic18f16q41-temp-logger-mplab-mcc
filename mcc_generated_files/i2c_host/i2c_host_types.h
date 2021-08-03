/**
  I2C Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c_host_types.h

  @Summary
    This is the generated driver types header file for the I2C driver.

  @Description
    This file provides common enumerations for I2C driver.
    Generation Information :
        Product Revision  :   - 
        Device            :  
        Driver Version    :  1.0.0
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

#ifndef I2C_HOST_TYPES_H
#define	I2C_HOST_TYPES_H

/**
  I2C_ERROR Enumeration
 
  @Description
    I2C ERROR code
     
*/
enum I2C_ERROR
{
    I2C_ERROR_NONE,             /* No Error */
    I2C_ERROR_NACK,             /* Client returned NACK */
    I2C_ERROR_BUS_COLLISION,    /* Bus Collision Error */
};

/**
  I2C_TRANSFER_SETUP structure
 
  @Description
    I2C Clock Speed (100KHZ to 1MHZ)
     
*/
struct I2C_TRANSFER_SETUP
{
  uint32_t clkSpeed;            // I2C Clock Speed
};

#endif // end of I2C_HOST_TYPES_H