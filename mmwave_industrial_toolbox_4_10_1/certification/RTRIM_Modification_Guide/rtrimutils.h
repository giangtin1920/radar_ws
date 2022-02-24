/**
 *   @file  rtrimutils.h
 *
 *   @brief
 *      This is the header file for the APIs which allow applications
 *      to set and retrieve the RTRIM_REP_APLL(27:31) field in the 
 *      CLK_CTRL_REG2_APLL register. 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef XWR6843_TRIM_APLL_API_H
#define XWR6843_TRIM_APLL_API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Analog Functions */

/**
 *  @b Description
 *  @n
 *      Pass Trim Value to APLL
 *      Swing voltage is updated by writing to RTRM_REP_APLL field in CLK_CTRL_REG2_APLL register
 *      Set and reset MSS memory region accordingly
 *
 *  @param[in] uint8_t trimValue
 *      Trim Value
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

int32_t rtrim_set(uint8_t);

/**
 *  @b Description
 *  @n
 *      Read Trim Value from APLL
 *      Value should be 0x12 at reset.
 *      Set and reset MSS memory region accordingly
 *
 *  @param[in]
 *
 *  @retval
 *      uint8_t trimValue
 *  @retval
 *      Error   -   <0
 */
int32_t rtrim_get();



#ifdef __cplusplus
}
#endif

#endif /* XWR6843_TRIM_APLL_API_H */


