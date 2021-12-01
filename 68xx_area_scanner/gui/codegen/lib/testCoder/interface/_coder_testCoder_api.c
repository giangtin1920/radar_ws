/*
 * File: _coder_testCoder_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 01-Dec-2021 16:24:37
 */

/* Include Files */
#include "_coder_testCoder_api.h"
#include "_coder_testCoder_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "testCoder",                         /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[272];
static const mxArray *b_emlrt_marshallOut(const real_T u[17]);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[272];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *payload,
  const char_T *identifier))[272];
static const mxArray *emlrt_marshallOut(const struct0_T *u);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[272]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[272]
{
  real_T (*y)[272];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u[17]
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u[17])
{
  static const int32_T iv[2] = { 1, 17 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 17; b_i++) {
    pData[i] = u[b_i];
    i++;
  }

  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[272]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[272]
{
  static const int32_T dims[1] = { 272 };

  real_T (*ret)[272];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[272])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *payload
 *                const char_T *identifier
 * Return Type  : real_T (*)[272]
 */
  static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *payload,
  const char_T *identifier))[272]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[272];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(payload), &thisId);
  emlrtDestroyArray(&payload);
  return y;
}

/*
 * Arguments    : const struct0_T *u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const struct0_T *u)
{
  static const char_T *sv[5] = { "numDetectedObj", "x", "y", "z", "doppler" };

  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 5, sv));
  b_y = NULL;
  m = emlrtCreateDoubleScalar(u->numDetectedObj);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "numDetectedObj", b_y, 0);
  emlrtSetFieldR2017b(y, 0, "x", b_emlrt_marshallOut(u->x), 1);
  emlrtSetFieldR2017b(y, 0, "y", b_emlrt_marshallOut(u->y), 2);
  emlrtSetFieldR2017b(y, 0, "z", b_emlrt_marshallOut(u->z), 3);
  emlrtSetFieldR2017b(y, 0, "doppler", b_emlrt_marshallOut(u->doppler), 4);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void testCoder_api(const mxArray * const prhs[1], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  struct0_T ptCloud;
  real_T (*payload)[272];
  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  payload = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "payload");

  /* Invoke the target function */
  testCoder(*payload, &ptCloud);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(&ptCloud);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void testCoder_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  testCoder_xil_terminate();
  testCoder_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void testCoder_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void testCoder_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_testCoder_api.c
 *
 * [EOF]
 */
