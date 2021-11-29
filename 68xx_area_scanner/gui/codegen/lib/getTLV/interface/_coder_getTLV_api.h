/*
 * File: _coder_getTLV_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Nov-2021 11:15:16
 */

#ifndef _CODER_GETTLV_API_H
#define _CODER_GETTLV_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_13x1
#define struct_emxArray_real_T_13x1

struct emxArray_real_T_13x1
{
  real_T data[13];
  int32_T size[2];
};

#endif                                 /*struct_emxArray_real_T_13x1*/

#ifndef typedef_emxArray_real_T_13x1
#define typedef_emxArray_real_T_13x1

typedef struct emxArray_real_T_13x1 emxArray_real_T_13x1;

#endif                                 /*typedef_emxArray_real_T_13x1*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T type;
  real_T length;
  emxArray_real_T_13x1 payload;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void getTLV(real_T framePacket[128], real_T *idx, struct0_T *tlv);
  void getTLV_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray
                  *plhs[2]);
  void getTLV_atexit(void);
  void getTLV_initialize(void);
  void getTLV_terminate(void);
  void getTLV_xil_shutdown(void);
  void getTLV_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_getTLV_api.h
 *
 * [EOF]
 */
