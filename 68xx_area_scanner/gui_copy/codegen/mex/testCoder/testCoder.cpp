//
//  testCoder.cpp
//
//  Code generation for function 'testCoder'
//


// Include files
#include "testCoder.h"
#include "rt_nonfinite.h"
#include "testCoder_data.h"

// Function Definitions
void testCoder(const emlrtStack *, real_T b, real_T c, real_T *a, char_T
               magicWord[8])
{
  static const char_T cv[8] = { '\x02', '\x01', '\x04', '\x03', '\x06', '\x05',
    '\x08', '\x07' };

  for (int32_T i = 0; i < 8; i++) {
    magicWord[i] = cv[i];
  }

  covrtLogFcn(&emlrtCoverageInstance, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0, 0);
  *a = b + c;

  // MagicWord = strfind(bytesBufferStr, char([2 1 4 3 6 5 8 7]));
}

// End of code generation (testCoder.cpp)
