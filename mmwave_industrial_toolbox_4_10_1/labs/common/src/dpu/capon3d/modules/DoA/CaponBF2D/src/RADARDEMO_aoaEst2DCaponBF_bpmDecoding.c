
/**
 *  \file   RADARDEMO_aoaEst2DCaponBF_bpmDecoding.c
 *
 *   \brief   Estimate the angle of arrival using 2D BF.
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/

#include <common/src/dpu/capon3d/modules/DoA/CaponBF2D/src/RADARDEMO_aoaEst2DCaponBF_priv.h>
#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_bpmDecoding
 *
 *   \brief   Per range bin, decode the BPM from the input signal.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    bpmPosPhaseAntIdx
 *               index of TX Antenna in BPM mode with positive phase (only 2 TX BPM is supported)
 *
 *   \param[in]    bpmNegPhaseAntIdx
 *               index of TX Antenna in BPM mode with negative phase (only 2 TX BPM is supported)
 *
 *   \param[in/out]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void        RADARDEMO_aoaEst2DCaponBF_bpmDecoding(
                IN int32_t nRxAnt,
                IN int32_t nChirps,
                IN int32_t bpmPosPhaseAntIdx,
                IN int32_t bpmNegPhaseAntIdx,
                INOUT cplx16_t * inputAntSamples)
{
#ifdef RADARDEMO_AOARADARCUDE_RNGCHIRPANT
    int32_t     antIdx, chirpIdx, tdmAntIdx;
    cplx16_t    * RESTRICT input1;
    cplx16_t    * RESTRICT input2;
    cplx16_t    * RESTRICT input3;
    cplx16_t    * RESTRICT input4;
    cplx16_t    * RESTRICT input5;
    cplx16_t    * RESTRICT input6;
    int64_t     intAdd, intSub;

#ifdef _TMS320C6X
    _nassert(nRxAnt %4  ==  0);
    _nassert(nRxAnt /4  ==  3); /* Only 3TX is supported */
    _nassert(nChirps %8 ==  0);
#endif

    antIdx      =   0;
    tdmAntIdx   =   3-bpmPosPhaseAntIdx-bpmNegPhaseAntIdx; /* Supporting 3TX with 2TX BPM and 1TX TDM */

    input1      =   (cplx16_t *) &inputAntSamples[antIdx + 4*bpmPosPhaseAntIdx];   /* RX1 of 1st BPM TX */
    input2      =   (cplx16_t *) &inputAntSamples[antIdx + 4*bpmNegPhaseAntIdx];   /* RX1 of 2nd BPM TX */
    input3      =   (cplx16_t *) &inputAntSamples[antIdx + 2+4*bpmPosPhaseAntIdx]; /* RX3 of 1st BPM TX */
    input4      =   (cplx16_t *) &inputAntSamples[antIdx + 2+4*bpmNegPhaseAntIdx]; /* RX3 of 2nd BPM TX */
    input5      =   (cplx16_t *) &inputAntSamples[antIdx + 4*tdmAntIdx];           /* RX1 of TDM TX */
    input6      =   (cplx16_t *) &inputAntSamples[antIdx + 2+4*tdmAntIdx];         /* RX3 of TDM TX */

    for (chirpIdx = 0; chirpIdx < nRxAnt * nChirps; chirpIdx += nRxAnt)
    {
        intAdd  =   _dsadd2(_amem8(&input1[chirpIdx]), _amem8(&input2[chirpIdx]));
        intSub  =   _dssub2(_amem8(&input1[chirpIdx]), _amem8(&input2[chirpIdx]));
        _amem8(&input1[chirpIdx])   =   intAdd;
        _amem8(&input2[chirpIdx])   =   intSub;

        intAdd  =   _dsadd2(_amem8(&input3[chirpIdx]), _amem8(&input4[chirpIdx]));
        intSub  =   _dssub2(_amem8(&input3[chirpIdx]), _amem8(&input4[chirpIdx]));
        _amem8(&input3[chirpIdx])   =   intAdd;
        _amem8(&input4[chirpIdx])   =   intSub;

        intAdd  =   _dsadd2(_amem8(&input5[chirpIdx]), _amem8(&input5[chirpIdx]));
        _amem8(&input5[chirpIdx])   =   intAdd;
        intAdd  =   _dsadd2(_amem8(&input6[chirpIdx]), _amem8(&input6[chirpIdx]));
        _amem8(&input6[chirpIdx])   =   intAdd;
    }

#endif

}
