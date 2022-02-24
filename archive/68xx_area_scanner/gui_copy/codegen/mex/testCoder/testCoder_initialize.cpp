//
//  testCoder_initialize.cpp
//
//  Code generation for function 'testCoder_initialize'
//


// Include files
#include "testCoder_initialize.h"
#include "_coder_testCoder_mex.h"
#include "rt_nonfinite.h"
#include "testCoder_data.h"

// Variable Definitions
static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

// Function Declarations
static void testCoder_once();

// Function Definitions
static void testCoder_once()
{
  mex_InitInfAndNan();

  // Allocate instance data
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  // Initialize Coverage Information
  covrtScriptInit(&emlrtCoverageInstance,
                  "G:\\My Drive\\AUTONOMOUS_EV\\Radar_ws\\radar_ws\\68xx_area_scanner\\gui_copy\\testCoder.m",
                  0U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);

  // Initialize Function Information
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "testCoder", 0, -1, 1393);

  // Initialize Basic Block Information
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 44, -1, 1386);

  // Initialize If Information
  // Initialize MCDC Information
  // Initialize For Information
  // Initialize While Information
  // Initialize Switch Information
  // Start callback for coverage engine
  covrtScriptStart(&emlrtCoverageInstance, 0U);
}

void testCoder_initialize()
{
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    testCoder_once();
  }
}

// End of code generation (testCoder_initialize.cpp)
