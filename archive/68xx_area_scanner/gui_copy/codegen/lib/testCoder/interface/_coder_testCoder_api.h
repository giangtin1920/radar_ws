/*
 * File: _coder_testCoder_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2021 07:14:25
 */

#ifndef _CODER_TESTCODER_API_H
#define _CODER_TESTCODER_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void testCoder(real_T b, real_T c, real_T *a, char_T magicWord[8]);
  void testCoder_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray *
                     plhs[2]);
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
