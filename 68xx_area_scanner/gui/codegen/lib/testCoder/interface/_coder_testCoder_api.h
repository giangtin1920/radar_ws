/*
 * File: _coder_testCoder_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 01-Dec-2021 16:24:37
 */

#ifndef _CODER_TESTCODER_API_H
#define _CODER_TESTCODER_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T numDetectedObj;
  real_T x[17];
  real_T y[17];
  real_T z[17];
  real_T doppler[17];
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void testCoder(real_T payload[272], struct0_T *ptCloud);
  void testCoder_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
  void testCoder_atexit(void);
  void testCoder_initialize(void);
  void testCoder_terminate(void);
  void testCoder_xil_shutdown(void);
  void testCoder_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_testCoder_api.h
 *
 * [EOF]
 */
