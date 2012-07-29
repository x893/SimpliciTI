/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Include file for BSP utility macros.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

#ifndef BSP_MACROS_H
#define BSP_MACROS_H


/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */
/* bit value */
#ifndef BV
#define BV(n)      (1 << (n))
#endif

/*
 *  This macro is for use by other macros to form a fully valid C statement.
 *  Without this, the if/else conditionals could show unexpected behavior.
 *
 *  For example, use...
 *    #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
 *  instead of ...
 *    #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
 *  or
 *    #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
 *  The last macro would not behave as expected in the if/else construct.
 *  The second to last macro will cause a compiler error in certain uses
 *  of if/else construct
 *
 *  It is not necessary, or recommended, to use this macro where there is
 *  already a valid C statement.  For example, the following is redundant...
 *    #define CALL_FUNC()   st(  func();  )
 *  This should simply be...
 *    #define CALL_FUNC()   func()
 *
 * (The while condition below evaluates false without generating a
 *  constant-controlling-loop type of warning on most compilers.)
 */
#define st(x)      do { x } while (__LINE__ == -1)

#define REVERSE_16( v )\
  st( uint8_t t = *((uint8_t*)&(v));\
      *((uint8_t*)&(v)) = *(((uint8_t*)&(v)) + 1);\
      *(((uint8_t*)&(v)) + 1) = t; )

#define REVERSE_32( v )\
  st( uint8_t t = *((uint8_t*)&(v));\
      *((uint8_t*)&(v)) = *(((uint8_t*)&(v)) + 3);\
      *(((uint8_t*)&(v)) + 3) = t;\
      t = *(((uint8_t*)&(v)) + 1);\
      *(((uint8_t*)&(v) + 1)) = *(((uint8_t*)&(v)) + 2);\
      *(((uint8_t*)&(v)) + 2) = t; )


/* bit field operations */
/* In the following macros,
 * reg  is a register variable suitable for access or assignment
 * val  is the value to put into a bit field
 * ofst is the offset of the lsb of the bit field in bits
 * sz   is the size of the bit field in bits
 */

// BF_MSK0 generates a bit field mask assuming the offset is zero
#ifndef BF_MSK0
#define BF_MSK0(sz) (~(~0ul << (sz)))
#endif

// BF_MSK generates a bit field mask which can be used to isolate
// a bit field
#ifndef BF_MSK
#define BF_MSK(ofst, sz) (BF_MSK0(sz) << (ofst))
#endif

// BF_CLR generates a complimented bit field mask which can be
// used to clear a bit field before oring in its value
#ifndef BF_CLR
#define BF_CLR(ofst, sz) (~BF_MSK(ofst, sz))
#endif

// BF_GEN generates a bit field value appropriate for oring with
// other bit field values for the same register.
#ifndef BF_GEN
#define BF_GEN(val, ofst, sz) (((val) & BF_MSK0(sz)) << (ofst))
#endif

// BF_GET extracts a bit field value from the given register value
#ifndef BF_GET
#define BF_GET(reg, ofst, sz) (((reg) >> (ofst)) & BF_MSK0(sz))
#endif

// BF_SET assigns a given bit field value to the bit field in
// the register reference provided
#ifndef BF_SET
#define BF_SET(reg, val, ofst, sz ) (reg = (reg) & BF_CLR(ofst, sz)\
                                          | BF_GEN(val, ofst, sz))
#endif

/*******************************************************************************
 * min and max function like macros
 ******************************************************************************/
#ifndef min
#define min(a, b) (( (a) < (b) ) ? (a) : (b) )
#endif
#ifndef max
#define max(a, b) (( (a) < (b) ) ? (b) : (a) )
#endif



/**************************************************************************************************
 */
#endif
