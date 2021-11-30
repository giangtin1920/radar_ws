//
//  _coder_testCoder_api.cpp
//
//  Code generation for function '_coder_testCoder_api'
//


// Include files
#include "_coder_testCoder_api.h"
#include "rt_nonfinite.h"
#include "testCoder.h"
#include "testCoder_data.h"

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *b, const
  char_T *identifier);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const real_T u);
static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const char_T u[8]);

// Function Definitions
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *b, const
  char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b), &thisId);
  emlrtDestroyArray(&b);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const char_T u[8])
{
  static const int32_T iv[2] = { 1, 8 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a(sp, 8, m, &u[0]);
  emlrtAssign(&y, m);
  return y;
}

void testCoder_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray
                   *plhs[2])
{
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  real_T a;
  real_T b;
  real_T c;
  char_T magicWord[8];
  st.tls = emlrtRootTLSGlobal;

  // Marshall function inputs
  b = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "b");
  c = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "c");

  // Invoke the target function
  testCoder(&st, b, c, &a, magicWord);

  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(a);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(&st, magicWord);
  }
}

// End of code generation (_coder_testCoder_api.cpp)
