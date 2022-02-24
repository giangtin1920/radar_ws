
/**
 *  \file   RADARDEMO_aoaEstBF_priv.c
 *
 *   \brief   Estimate the angle of arrival using BF.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "RADARDEMO_aoaEstCaponBF_priv.h"

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_covInv
 *
 *   \brief   Per range bin, estimate the covariance matrices from input 1D FFT results, and calculate the inverse of these matrices.
 *
 *   \param[in]    invFlag
 *               Flag to indicate matrix inversion will be performed. 
 *               If set to 1, output invRnMatrices will contain inversion of covariance matrices.
 *               If set to 0, output invRnMatrices will contain covariance matrices without inversion.
 *
 *   \param[in]    clutterRmFlag
 *               Flag to indicate clutter removal will be performed if set to 1. Otherwise, disabled.
 *
 *   \param[in]    gamma
 *               Scaling factor for diagnal loading.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size TBD!!!
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
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
void		RADARDEMO_aoaEstCaponBF_covInv(
				IN uint8_t invFlag,
				IN uint8_t clutterRmFlag,
				IN float gamma,
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN int32_t * scratch,
				IN cplx16_t * inputAntSamples,
				OUT cplxf_t  * invRnMatrices,
				OUT cplxf_t  * static_information)
{
	int32_t		antIdx, chirpIdx, i, j, scratchOffset, rnIdx, tempNchirps;
	cplx16_t	* RESTRICT input1;
	cplx16_t	* RESTRICT input2;
	__float2_t     * RESTRICT Rn;
	int64_t		lltemp, llinput1, llinput2;
	__float2_t     acc, acc1, acc2, acc3, scale2;
	int32_t       itemp1;
	cplxf_t		* RESTRICT invRn;
	float       ftemp;

	scratchOffset	=	0;
	Rn			=	(__float2_t *) &scratch[scratchOffset];
	scratchOffset	=	scratchOffset + 2 * nRxAnt * (1 + (nRxAnt>>1));  /*72 32-bit word for 8 antennas*/

	ftemp			=	_rcpsp((float)nChirps);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	ftemp			=	ftemp * (2.f - (float)nChirps * ftemp);
	
	scale2			=	_ftof2(ftemp, ftemp);

	/*Rn estimation */
	if (clutterRmFlag)
	{
		int64_t		mean2;
		int64_t		intAcc;

		for (antIdx = 0; antIdx < nRxAnt; antIdx++)
		{
			if (nChirps & 0x7)
				tempNchirps		=	nChirps - 8;
			else
				tempNchirps		=	nChirps;
			input1		=	(cplx16_t *) &inputAntSamples[antIdx * nChirps];
			acc			=	_ftof2(0.f, 0.f);
			acc1		=	_ftof2(0.f, 0.f);
			for (chirpIdx = 0; chirpIdx < tempNchirps; chirpIdx += 8)
			{
				intAcc	=	_dsadd2(_amem8(&input1[chirpIdx]), _amem8(&input1[chirpIdx+2]));
				itemp1	=	_sadd2(_hill(intAcc), _loll(intAcc));
				acc		=	_daddsp(acc, _dinthsp(itemp1));
				intAcc	=	_dsadd2(_amem8(&input1[chirpIdx+4]), _amem8(&input1[chirpIdx+6]));
				itemp1	=	_sadd2(_hill(intAcc), _loll(intAcc));
				acc1	=	_daddsp(acc1, _dinthsp(itemp1));
			}
			acc			=	_daddsp(acc, acc1);
			
			if (nChirps & 0x7)
			{
				for ( ; chirpIdx < nChirps; chirpIdx++)
				{
					itemp1	=	_amem4(&input1[chirpIdx]);
					acc		=	_daddsp(acc, _dinthsp(itemp1));
				}
			}			
			acc			=	_dmpysp(acc, scale2);
			//*static_information = acc;
			//static_information++;
			_amem8_f2(static_information) = acc;
			static_information++;
			itemp1		=	_dspinth(acc);
			mean2		=	_itoll(itemp1,itemp1);
			if (nChirps & 0x3)
				tempNchirps		=	nChirps - 4;
			else
				tempNchirps		=	nChirps;
			for (chirpIdx = 0; chirpIdx < tempNchirps; chirpIdx += 4)
			{
				llinput1	=	_amem8(&input1[chirpIdx]);
				_amem8(&input1[chirpIdx])	=	_dssub2(llinput1, mean2);
				llinput1	=	_amem8(&input1[chirpIdx + 2]);
				_amem8(&input1[chirpIdx + 2])	=	_dssub2(llinput1, mean2);
			}
			if (nChirps & 0x3)
			{
				for ( ; chirpIdx < nChirps; chirpIdx++)
				{
					_amem4(&input1[chirpIdx])	=	_ssub2(_amem4(&input1[chirpIdx]), itemp1);
				}
			}
		}
	}

	rnIdx	=	0;
	for (antIdx = 0; antIdx < nRxAnt; antIdx++)
	{
		input1		=	(cplx16_t *) &inputAntSamples[antIdx * nChirps];

		//i = antIdx case
		acc			=	_ftof2(0.f, 0.f);
		acc1		=	_ftof2(0.f, 0.f);
		acc2		=	_ftof2(0.f, 0.f);
		acc3		=	_ftof2(0.f, 0.f);
		if (nChirps & 0x7)
			tempNchirps		=	nChirps - 8;
		else
			tempNchirps		=	nChirps;
		for (chirpIdx = 0; chirpIdx < tempNchirps; chirpIdx += 8)
		{
			llinput1	=	_amem8(&input1[chirpIdx]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc			=	_daddsp(acc, _dintsp(lltemp));
			llinput1	=	_amem8(&input1[chirpIdx + 2]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc2		=	_daddsp(acc2, _dintsp(lltemp));
			
			llinput1	=	_amem8(&input1[chirpIdx + 4]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc1		=	_daddsp(acc1, _dintsp(lltemp));
			llinput1	=	_amem8(&input1[chirpIdx + 6]);
			itemp1		=	_hill(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_cmpy(_hill(llinput1), itemp1);
			itemp1		=	_loll(llinput1);
			itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
			lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
			acc3		=	_daddsp(acc3, _dintsp(lltemp));
			
		}
		acc							=	_daddsp(acc, acc1);
		acc							=	_daddsp(acc, acc2);
		acc							=	_daddsp(acc, acc3);
		if (nChirps & 0x7)
		{
			int32_t itemp2;
			for ( ; chirpIdx < nChirps; chirpIdx++)
			{
				itemp2		=	_amem4(&input1[chirpIdx]);
				itemp1		=	_packhl2(itemp2, _ssub2(0, itemp2));
				lltemp		=	_cmpy(itemp2, itemp1);
				acc			=	_daddsp(acc, _dintsp(lltemp));
			}
		}			
		acc							=	_dmpysp(acc, scale2);
		_amem8_f2(&Rn[rnIdx++])		=	_ftof2(_hif2(acc), 0.f);

		for (i = antIdx + 1; i < nRxAnt; i++)
		{
			input2		=	(cplx16_t *) &inputAntSamples[i * nChirps];

			acc			=	_ftof2(0.f, 0.f);
			acc1		=	_ftof2(0.f, 0.f);
			if (nChirps & 0x3)
				tempNchirps		=	nChirps - 4;
			else
				tempNchirps		=	nChirps;
			for (chirpIdx = 0; chirpIdx < tempNchirps; chirpIdx += 4)
			{
				llinput1	=	_amem8(&input1[chirpIdx]);
				llinput2	=	_amem8(&input2[chirpIdx]);
				itemp1		=	_hill(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_cmpy(_hill(llinput1), itemp1);
				itemp1		=	_loll(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
				acc			=	_daddsp(acc, _dintsp(lltemp));
				llinput1	=	_amem8(&input1[chirpIdx + 2]);
				llinput2	=	_amem8(&input2[chirpIdx + 2]);
				itemp1		=	_hill(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_cmpy(_hill(llinput1), itemp1);
				itemp1		=	_loll(llinput2);
				itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
				lltemp		=	_dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
				acc1		=	_daddsp(acc1, _dintsp(lltemp));
			}
			acc							=	_daddsp(acc, acc1);
			if (nChirps & 0x3)
			{
				int32_t itemp2;
				for ( ; chirpIdx < nChirps; chirpIdx++)
				{
					itemp2		=	_amem4(&input1[chirpIdx]);
					itemp1		=	_amem4(&input2[chirpIdx]);
					itemp1		=	_packhl2(itemp1, _ssub2(0, itemp1));
					lltemp		=	_cmpy(itemp2, itemp1);
					acc			=	_daddsp(acc, _dintsp(lltemp));
				}
			}			
			acc							=	_dmpysp(acc, scale2);
			_amem8_f2(&Rn[rnIdx++])		=	acc;
		}
	}

	if (invFlag)
	{
		/*add diagnal loading */
		j = 0;
		ftemp = 0;
		itemp1 = nRxAnt;
		for (i = 0; i < nRxAnt; i++)
		{
			ftemp	+=	_hif2(_amem8_f2(&Rn[j]));
			j		+=	itemp1;
			itemp1--;
		}
		if (nRxAnt == 8)
			ftemp	*=	0.125f;
		else if (nRxAnt == 4)
			ftemp	*=	0.25f;
		j = 0;
		ftemp *= gamma;
		acc = _ftof2(ftemp, 0.f);
		itemp1 = nRxAnt;
		for (i = 0; i < nRxAnt; i++)
		{
			_amem8_f2(&Rn[j])	=	_daddsp(_amem8_f2(&Rn[j]), acc);
			j		+=	itemp1;
			itemp1--;
		}


		/* matrix inversion */
		if (nRxAnt == 8)
		{
			MATRIX_single8x8MatInv (  
									(cplxf_t	*) Rn,
									&scratch[scratchOffset],
									invRnMatrices);
		}
		else if (nRxAnt == 4)
		{
			cplxf_t		* RESTRICT inputRn;
			__float2_t f2temp1;

			inputRn			=	(cplxf_t *)&scratch[scratchOffset]; // size 2 * 16
			invRn			=	(cplxf_t *)&scratch[scratchOffset + 2 * 16]; // size 2 * 16

			rnIdx		=	0;
			for (i = 0; i < nRxAnt; i++)
			{
				for (j = i; j < nRxAnt; j++)
				{
					_amem8_f2(&inputRn[i * nRxAnt + j]) = _amem8_f2(&Rn[rnIdx++]);
				}
			}
			for (i = 1; i < nRxAnt; i++)
			{
				for (j = 0; j < i; j++)
				{
					f2temp1		=	_amem8_f2(&inputRn[j * nRxAnt + i]);

					_amem8_f2(&inputRn[i * nRxAnt + j]) = _ftof2(_hif2(f2temp1), -_lof2(f2temp1));
				}
			}

			MATRIX_4x4_BWInversionfp (  
								inputRn,
								invRn);
			rnIdx	=	0;
			for (i = 0; i < nRxAnt; i++)
			{
				for (j = i; j < nRxAnt; j++)
				{
					_amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&invRn[i * nRxAnt + j]);
				}
			}
		}
	}
	else
	{
			
		if (nRxAnt == 8)
			rnIdx	=	36;
		else if (nRxAnt == 4)
			rnIdx	=	10;

		for (i = 0; i < rnIdx; i++)
		{
			_amem8_f2(&invRnMatrices[i]) = _amem8_f2(&Rn[i]);
		}
	}
}

/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_heatmap
 *
 *   \brief   Use Capon beamforming to generate range azimuth heatmap per range bin.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    numAzimuthBins
 *               number of Azimuth bins
 *
 *   \param[in]    steeringVec
 *              steering vector for beamforming.
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size of 360 32-bit words for 8 antennas, and 82 32-bit words for 4 antennas.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    rangeAzimuthHeatMap
 *               Output range azimuth heatmap, in the format of numInputRangeBins by numAzimuthBins
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
void RADARDEMO_aoaEstCaponBF_heatmap(
				IN uint8_t bfFlag,
				IN int32_t  nRxAnt,
				IN int32_t  numAzimuthBins,
				IN cplxf_t * steeringVec,
				IN cplxf_t * invRnMatrices,
				OUT float  * rangeAzimuthHeatMap)
{
	int32_t azimIdx;
	__float2_t * RESTRICT steeringVecPtr;
	__float2_t * RESTRICT invRnMatPtr;
	float	 * RESTRICT heatMapPtr;
	__float2_t  f2temp, steerVecIn;
	float		output, result;

	if (nRxAnt == 8)
	{
		steeringVecPtr		=	(__float2_t *) &steeringVec[0];
		invRnMatPtr		=	(__float2_t *)&invRnMatrices[0];

		//this may need to change depending on which dimension to search first.
		heatMapPtr		=	(float *) &rangeAzimuthHeatMap[0];
		for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
		{
			output		=	_hif2(_amem8_f2(&invRnMatPtr[0]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[8]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[15]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[21]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[26]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[30]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[33]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[35]));

			f2temp		=	_amem8_f2(&invRnMatPtr[1]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[9]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[16]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[22]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[27]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[31]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[34]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[2]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[10]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[17]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[23]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[28]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[32]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[3]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[11]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[18]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[24]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[29]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[4]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[12]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[19]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[25]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[5]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[13]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[20]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[6]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[14]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[7]);
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			result		=	_rcpsp(output);
			result		=	result * (2.f - output * result);
			result		=	result * (2.f - output * result);
			//result		=	result * result;

			if (!bfFlag) result = output;
			heatMapPtr[azimIdx]		=	result;
		}
	}
	else if (nRxAnt == 4)
	{
		steeringVecPtr		=	(__float2_t *) &steeringVec[0];
		invRnMatPtr		=	(__float2_t *)&invRnMatrices[0];

		//this may need to change depending on which dimension to search first.
		heatMapPtr		=	(float *) &rangeAzimuthHeatMap[0];
		for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
		{
			output		=	_hif2(_amem8_f2(&invRnMatPtr[0]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[4]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[7]));
			output		+=	_hif2(_amem8_f2(&invRnMatPtr[9]));

			f2temp		=	_amem8_f2(&invRnMatPtr[1]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[5]));
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[8]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[2]);
			f2temp		=	_daddsp(f2temp, _amem8_f2(&invRnMatPtr[6]));
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			f2temp		=	_amem8_f2(&invRnMatPtr[3]);
			steerVecIn	=	_amem8_f2(steeringVecPtr++);
			f2temp		=	_dmpysp(f2temp, steerVecIn);
			output		+=	2.f * (_hif2(f2temp)  + _lof2(f2temp));

			result		=	_rcpsp(output);
			result		=	result * (2.f - output * result);
			result		=	result * (2.f - output * result);
			//result		=	result * result;

			if (!bfFlag) result = output;
			heatMapPtr[azimIdx]		=	result;
		}
	}
}


#define RADARDEMO_AOAESTBF_VAREST_CONST 1        //!< A constant used in angle variance estimation

int32_t  RADARDEMO_aoaEstimationBFSinglePeak_static_use(
                            IN  uint8_t numAnt,
                            IN  float   noise,
                            IN  float   estResolution,
                            IN  cplxf_t * sigIn,
                            IN  cplxf_t * steeringVec,
                            IN  int32_t steeringVecSize,
                            IN  float   * scratchPad,
                            OUT float   * peakVal,
                            OUT float   * estVar,
                            OUT int32_t * angleEst)
 {
    int32_t 	i;
	//int32_t maxInd, leftInd, rightInd, temp3dBSpan;
    float 		* RESTRICT doaSpectrum;
    float 		tempPow, maxPow;
	//float totPower, signalPower, maxPow, thre3dB, tempVar, inputPower, tempVarSqrInv, tempInterpol;
	__float2_t 	input0;
	__float2_t 	* RESTRICT tempSteerVecPtr;
	//__float2_t 	* RESTRICT tempInPtr;
    /*
     Solve: spectrum = A'*Rn*A = |A'x|.^2;
     where:
             A = [steeringVec(theta)]  is a nRxAnt by numAngles matrix
             Rn is the covariance matrix of the antenna signal
             x is the input signal vector
    */

    doaSpectrum  	=   (float *) &scratchPad[0];
	
//	tempInPtr		=	(__float2_t *) sigIn;
//	inputPower		=	0.f;
//	for (i = 0; i < numAnt; i++ )
//	{
//		input0		=	_amem8_f2(tempInPtr++);
//		input0		=	_dmpysp(input0, input0);
//		inputPower	+=	_hif2(input0) + _lof2(input0);
//	}

    /* Compute the spectrum and record the max */
    //totPower 	= 	0.f;
	maxPow 		=	0.f;
	//maxInd		=	-1;
	if ( numAnt == 4 )
	{
		__float2_t input1, input2, input3, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			//totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				//maxInd 	=	i;
			}
		}
	}
	else if ( numAnt == 8 )
	{
		__float2_t input1, input2, input3, input4, input5, input6, input7, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		input4 			=	_amem8_f2(&sigIn[4]);
		input5 			=	_amem8_f2(&sigIn[5]);
		input6 			=	_amem8_f2(&sigIn[6]);
		input7 			=	_amem8_f2(&sigIn[7]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input4));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input5));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input6));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input7));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			//totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				//maxInd 	=	i;
			}
		}
	}
#if 0
	else
	{
		int32_t j;
		__float2_t input1, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		tempSteerVecPtr =	(__float2_t *) steeringVec;
		
		for (i = 0; i < steeringVecSize; i++ )
		{
			input1	=	_amem8_f2(&sigIn[1]);
			f2temp1	=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
			tempSteerVecPtr++;
			for (j = 2; j < numAnt; j++ )
			{
				input1	=	_amem8_f2(&sigIn[j]);
				f2temp1	=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr), input1));
				tempSteerVecPtr++;
			}
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	=	tempPow;
			totPower	+=	tempPow;
			if (tempPow > maxPow)
			{
				maxPow	=	tempPow;
				maxInd 	=	i;
			}
		}
		
	}
#endif
	*peakVal = maxPow;
//	thre3dB		=	maxPow * 0.5f;
//	signalPower	=	0.f;
//	leftInd     =	maxInd - 1;
//	while ((doaSpectrum[leftInd] >= thre3dB) && (leftInd != maxInd))
//	{
//		signalPower 	+= 	doaSpectrum[leftInd];
//		leftInd--;
//		if (leftInd < 0)
//			leftInd		=	steeringVecSize - 1;
//	}
//	
//	rightInd     =	maxInd + 1;
//	while ((doaSpectrum[rightInd] >= thre3dB) && (rightInd != maxInd))
//	{
//		signalPower 	+= 	doaSpectrum[rightInd];
//		rightInd++;
//		if (rightInd == steeringVecSize)
//			rightInd	=	0;
//	}
//    temp3dBSpan			=	rightInd - 1 - (leftInd + 1) + 1;
//	if (temp3dBSpan <= 0)
//		temp3dBSpan		+=  steeringVecSize;
//	tempVarSqrInv		=	2.f * (float)RADARDEMO_AOAESTBF_VAREST_CONST * (float)RADARDEMO_AOAESTBF_VAREST_CONST * inputPower * (float) numAnt * signalPower;
//	tempInterpol		=	_rcpsp(noise * totPower);
//	tempInterpol		=	tempInterpol * (2.f - noise * totPower * tempInterpol);
//	tempInterpol		=	tempInterpol * (2.f - noise * totPower * tempInterpol);

//	tempVarSqrInv		*=	tempInterpol;	
//	tempVar 			= 	estResolution * (float) temp3dBSpan;
//	tempInterpol		=	_rsqrsp(tempVarSqrInv);
//    tempInterpol		=   tempInterpol * (1.5f - (0.5f * tempVarSqrInv) * tempInterpol * tempInterpol);
//    tempInterpol		=   tempInterpol * (1.5f - (0.5f * tempVarSqrInv) * tempInterpol * tempInterpol);
//	tempVar				*=	tempInterpol;
//	
//	*estVar 			=  	tempVar;
//	*angleEst 			= 	maxInd;
    return(1);
}

