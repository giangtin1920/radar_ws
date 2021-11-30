//
//  _coder_testCoder_mex.cpp
//
//  Code generation for function '_coder_testCoder_mex'
//


// Include files
#include "_coder_testCoder_mex.h"
#include "_coder_testCoder_api.h"
#include "rt_nonfinite.h"
#include "testCoder_data.h"
#include "testCoder_initialize.h"
#include "testCoder_terminate.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&testCoder_atexit);

  // Module initialization.
  testCoder_initialize();
  try {
    emlrtShouldCleanupOnError(emlrtRootTLSGlobal, false);

    // Dispatch the entry-point.
    testCoder_mexFunction(nlhs, plhs, nrhs, prhs);

    // Module termination.
    testCoder_terminate();
  } catch (...) {
    emlrtCleanupOnException(emlrtRootTLSGlobal);
    throw;
  }
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

void testCoder_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const
  mxArray *prhs[2])
{
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  const mxArray *outputs[2];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;

  // Check for proper number of arguments.
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4, 9,
                        "testCoder");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "testCoder");
  }

  // Call the function.
  testCoder_api(prhs, nlhs, outputs);

  // Copy over outputs to the caller.
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, &outputs[0]);
}

// End of code generation (_coder_testCoder_mex.cpp)
