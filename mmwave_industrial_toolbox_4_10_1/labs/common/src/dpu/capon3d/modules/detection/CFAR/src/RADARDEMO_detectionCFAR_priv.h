/*! 
 *  \file   RADARDEMO_detectionCFAR_priv.h
 *
 *  \brief   Header file for RADARDEMO_detectionCFAR module's internal functions
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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


#ifndef RADARDEMO_DOPPLERPROC_RPIV_H
#define RADARDEMO_DOPPLERPROC_RPIV_H

#include <common/src/utils/swpform.h>
#ifndef _TMS320C6600
#include <common/src/dpu/capon3d/modules/utilities/radar_c674x.h>
#endif

#include <common/src/dpu/capon3d/modules/utilities/radar_commonMath.h>

extern double ti_math_logtable[];


//#define USE_TABLE_FOR_K0


#ifdef USE_TABLE_FOR_K0
extern float rltvThr_CFARCA[];
extern float rltvThr_CFAROS_8[];
extern float rltvThr_CFAROS_16[];
extern float rltvThr_CFAROS_24[];
extern float rltvThr_CFAROS_32[];
extern float rltvThr_CFAROS_40[];
extern float rltvThr_CFAROS_48[];
extern float rltvThr_CFAROS_56[];
extern float rltvThr_CFAROS_64[];
extern float * rltvThr_CFAROS[];
#endif
/**
 *  \enum   
 *   {
 *	RADARDEMO_DETECTIONCFAR_CFAR_CAVG = 0,	
 *  RADARDEMO_DETECTIONCFAR_CFAR_CACC,
 *  RADARDEMO_DETECTIONCFAR_CFAR_CASO,
 *	RADARDEMO_DETECTIONCFAR_CFAR_CAGO
 *   }   RADARDEMO_detectionCACFAR_Type;
 *
 *  \brief   enum for CFAR types.
 *
 *
 */

typedef enum
{
	RADARDEMO_DETECTIONCFAR_CFAR_CAVG = 0,					/**< CA-CFAR type: cell average*/
	RADARDEMO_DETECTIONCFAR_CFAR_CACC,					    /**< CA-CFAR type: cell accumulation*/
	RADARDEMO_DETECTIONCFAR_CFAR_CASO,					    /**< CA-CFAR type: smaller of*/
	RADARDEMO_DETECTIONCFAR_CFAR_CAGO,					    /**< CA-CFAR type: greater of*/
	RADARDEMO_DETECTIONCFAR_RA_CFAR_CASO				/**< CFAR type: cell average CFAR, smaller of the 2 windows for both 2 passes, and for range-azimuth*/
} RADARDEMO_detectionCACFAR_Type;


/** 
 *  \struct   _RADARDEMO_detectionCFAR_handle_
 *   {
 *	uint32_t     fft1DSize;  	
 *	uint32_t     fft2DSize;  
 *  int32_t 	 *scratchPad; 
 *	uin8_t       cfarType;		
 *	float        relThr;  	   
 *  float        rangeRes;  	
 *  float        dopplerRes;  	
 *  uint8_t      searchWinSizeRange;	    			
 *  uint8_t      guardSizeRange;	    			   
 *  uint8_t      searchWinSizeDoppler;	    			
 *  uint8_t      guardSizeDoppler;	    			   
 *      uint8_t      leftSkipSize;	    			
 *      uint8_t      rightSkipSize;	  
 *  uint8_t      enableSecondPassSearch;
 *   }   _RADARDEMO_detectionCFAR_handle_;
 *
 *  \brief   Structure element of the list of descriptors for UL allocations.
 *
 *
 */

typedef struct _RADARDEMO_detectionCFAR_handle_
{
	uint32_t     fft1DSize;  					/**< 1D FFT size*/
	uint32_t     fft2DSize;  					/**< 2D FFT size*/
	int32_t 	 *scratchPad;                   /**< Pointer to scratch pad, size of (fft1DSize * (sizeof(int16_t)+sizeof(float)) + 100*sizeof(float)) bytes*/
	uint8_t      cfarType;						/**< Type of CFAR.*/
	float        relThr;  	    				/**< Input relative threshold.*/
	float        dopplerSearchRelThr;  	        /**< Doppler search relative threshold.*/
	float        rangeRes;  	    			/**< Range resolution.*/
	float        dopplerRes;  	    			/**< Doppler resolution.*/
	uint8_t      searchWinSizeRange;	    	/**< Search window size for range domain search.*/
	uint8_t      guardSizeRange;	    		/**< Number of guard samples for range domain search.*/
	uint8_t      searchWinSizeDoppler;	    	/**< Search window size for Doppler domain search.*/
	uint8_t      guardSizeDoppler;	    		/**< Number of guard samples for Doppler domain search.*/
	uint16_t	 maxNumDetObj;	    			/**< Maximum number of objects to detect.*/
	uint8_t      leftSkipSize;                  /**< number of samples to be skipped on the left side in range domain. */
	uint8_t      rightSkipSize;                 /**< number of samples to be skipped on the right side in range domain. */
	uint8_t      leftSkipSizeAzimuth;            /**< number of samples to be skipped on the left side in azimuth domain. */
	uint8_t      rightSkipSizeAzimuth;           /**< number of samples to be skipped on the right side in azimuth domain. */
	uint8_t      enableSecondPassSearch;    	/**< Flag for enabling second pass search, if set to 1. If set to 0, no second pass search*/
	uint32_t    log2MagFlag; 					/**<use log2(mag) as input*/
	uint32_t    shortened1DInput; 				/**<Flag if set to 1, to indicate that the heatmap is already trimmed by skip left sample at 1D dimmension*/
	uint32_t	angleDim1;						/*Dim1 of angle, for removing non-local max side peaks in angle domain*/
	uint32_t	angleDim2;						/*Dim2 of angle, for removing non-local max side peaks in angle domain*/
	RADARDEMO_detectionCACFAR_Type caCfarType;  /**< RADARDEMO_detectionCACFAR_Type */
} RADARDEMO_detectionCFAR_handle;




/*! 
   \fn     RADARDEMO_detectionCFAR_CA
 
   \brief   Performs peak search and calculation of range and speed of detected object using CA-CFAR.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output SNR estimation to detected objects. 
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */
extern int32_t	RADARDEMO_detectionCFAR_CA(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * snrEst,
							OUT  float    * noise);
							
/*!
   \fn     RADARDEMO_detectionCFAR_OS

   \brief   Performs peak search and calculation of range and speed of detected object using CFAROS. All parameters hardcoded for cycle performance.
            search window length = 2 * 16 and K = 24. 

   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output SNR estimation to detected objects. 
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_detectionCFAR_OS(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * snrEst,
							OUT  float    * noise);							


/*! 
   \fn     RADARDEMO_detectionCFAR_CAAll
 
   \brief   Performs peak search and calculation of range and speed of detected object using CASO-CFAR.
  
	\param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    dopplerInd
               Pointer to the output Doppler indices to detected objects. 
			   
   \param[out]    rangeEst
               Pointer to the output range estimation to detected objects. 
			   
   \param[out]    dopplerEst
               Pointer to the output Doppler estimation to detected objects. 
			   
   \param[out]    rangeVar
               Pointer to the output variance for range estimation to detected objects. 
			   
   \param[out]    dopplerVar
               Pointer to the output variance for Doppler estimation to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output SNR estimation to detected objects. 
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_detectionCFAR_CAAll(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * dopplerInd,
							OUT  float    * rangeEst,
							OUT  float    * dopplerEst,
							OUT  float    * rangeVar,
							OUT  float    * dopplerVar,
							OUT  float    * snrEst,
							OUT  float    * noise);



/*! 
   \fn     RADARDEMO_detectionCFAR_raCAAll
 
   \brief   Performs peak search and calculation of range and azimuth of detected object using CASO-CFAR.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    azimuthInd
               Pointer to the output azimuth indices to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_detectionCFAR_raCAAll(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * azimuthInd,
							OUT  float    * snrEst,
							OUT  float    * noise,
							IN	 float	  * azMaxPerRangeBin,  
							IN   float		sidelobeThr, 
							IN   uint8_t	enableSecondPass,
							IN	 uint8_t	enable_neighbour_check);

/*! 
   \fn     RADARDEMO_detectionCFAR_raCAAll_ver2
 
   \brief   Performs peak search and calculation of range using CASO-CFAR, and angle index using local max.
  
   \param[in]    InputPower
               Input power profile from integration.
 
   \param[in]    detectionCFARInst
               Pointer to input detection handle.
 
   \param[out]    rangeInd
               Pointer to the output range indices to detected objects. 

   \param[out]    azimuthInd
               Pointer to the output azimuth indices to detected objects. 
			   
   \param[out]    snrEst
               Pointer to the output linear SNR estimation to detected objects.
			   
   \param[out]    noise
               Pointer to the output noise estimation detected objects. 
			   
   \ret       number of objects detected.
   
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_detectionCFAR_raCAAll_ver2(
							IN float   **InputPower,
                            IN  RADARDEMO_detectionCFAR_handle *detectionCFARInst,
							OUT  uint16_t * rangeInd, 
							OUT  uint16_t * azimuthInd,
							OUT  float    * snrEst,
							OUT  float    * noise,
							IN	 float	  * azMaxPerRangeBin,  
							IN   float		sidelobeThr, 
							IN   uint8_t	enableSecondPass,
							IN	 uint8_t	enable_neighbour_check);
#ifndef _WIN32
static inline float log2sp_i(float a)
{
  double  ln2  =  0.693147180559945;
  double  base =  1.4426950408890;
  float   c1   = -0.2302894f;
  float   c2   =  0.1908169f;
  float   c3   = -0.2505905f;
  float   c4   =  0.3333164f;
  float   c5   = -0.5000002f;
  float   MAXe =  3.402823466E+38;
  float   pol, r1, r2, r3, r4, res;
  double  dr, frcpax, rcp, T;
  int     N, T_index;

  /* r = x * frcpa(x) -1 */
  rcp = _rcpdp((double) a);
  frcpax = _itod(_clr(_hi(rcp),0,16), 0);
  dr = frcpax * (double) a  - 1.0;

  /* Polynomial p(r) that approximates ln(1+r) - r */
  r1 = (float) dr;
  r2 = r1*r1;
  r3 = r1*r2;
  r4 = r2*r2;

  pol  = c5*r2 + ((c4*r3) + ((c2*r1 + c3) + c1*r2)*r4);
  pol *= (float) base;

  /* Reconstruction: result = T + r + p(r) */
  N       = _extu(_hi(frcpax),  1, 21) - 1023;
  T_index = _extu(_hi(frcpax), 12, 29);
  T       = (ti_math_logtable[T_index] - ln2 * (double) N) * base;
  res     = (dr * base  + T) + (pol);

  if (a <= 0.0f) {
    res = _itof(0xFF800000);
  }
  if (a > MAXe) {
    res = 1024.0;
  }

  return (res);
}
#endif

#endif //RADARDEMO_DOPPLERPROC_RPIV_H

