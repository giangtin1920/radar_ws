/**
 *   @file  libsleep_xwr68xx.h
 *
 *   @brief
 *      This is the header file for the libsleep_xwr68xx library which exposes the
 *      exported API which can be used by the
 *      applications to use the power down functions.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 Texas Instruments, Inc.
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

#ifndef LIBSLEEP_XWR68XX_H
#define LIBSLEEP_XWR68XX_H

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief
 *  Idle Mode Configuration
 *
 * @details
 *  The structure contains the parameters for the various components in the
 *  Idle Mode Power Function
 *
 *
 */
typedef struct IdleModeCfg_t
{

    //Enable DSP Shutdown
    int8_t enDSPpowerdown;
    //Enable DSS Clock Gate (No Shutdown)
    int8_t enDSSclkgate;
    //Enable switching of MSS VCLK
    int8_t enMSSvclkgate;
    //Enable BSS Clock Gate
    int8_t enBSSclkgate;
    //Enable Power switching of RF
    int8_t enRFpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLpowerdown;
    //Enable Power switching of APLL
    int8_t enAPLLGPADCpowerdown;

    //Delay between each successive component
    uint32_t componentMicroDelay;

    //Delay between when device has reached lowest power state
    uint32_t idleModeMicroDelay;


} IdleModeCfg;


/* Digital Functions */

/**
 *  @b Description
 *  @n
 *     Register writes to power down the DSS subsystem.
 *     CBUFF and LVDS INTF clkgated
 */
void xWR6843_dss_power_down(void);


/**
 *  @b Description
 *  @n
 *     Register writes to power up the DSS subsystem.
 *     CBUFF and LVDS INTF ungated
 */
void xWR6843_dss_power_up(void);


/**
 *  @b Description
 *  @n
 *     Register writes to clkgate the DSP.
 */
void xWR6843_dss_clock_gate(void);


/**
 *  @b Description
 *  @n
 *     Register writes to ungate the DSP.
 */
void xWR6843_dss_clock_ungate(void);


/**
 *  @b Description
 *  @n
 *     Register writes to set MSS_VCLK to 40 MHz
 *     Configured OS timer for 40 MHz with UDCP0 init.
 */
void xWR6843_mss_vclk_40M(void);


/**
 *  @b Description
 *  @n
 *     Register writes to set MSS_VCLK to 120 MHz
 *     Configured OS timer for 120 MHz with UDCP0 init.
 */
void xWR6843_mss_vclk_120M(void);


/**
 *  @b Description
 *  @n
 *     Register writes to set MSS_VCLK to 200MHz
 *     Configured OS timer for 200 MHz with UDCP0 init.
 */
void xWR6843_mss_vclk_200M(void);


#ifdef __cplusplus
}
#endif

#endif /* LIBSLEEP_XWR68XX_H */
