//
// File: testCoder.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 30-Nov-2021 07:14:25
//

// Include Files
#include "testCoder.h"

// Function Definitions
//
// Arguments    : double b
//                double c
//                double *a
//                char magicWord[8]
// Return Type  : void
//
void testCoder(double b, double c, double *a, char magicWord[8])
{
  static const char cv[8] = { '\x02', '\x01', '\x04', '\x03', '\x06', '\x05',
    '\x08', '\x07' };

  for (int i = 0; i < 8; i++) {
    magicWord[i] = cv[i];
  }

  *a = b + c;

  // MagicWord = strfind(bytesBufferStr, char([2 1 4 3 6 5 8 7]));
}

//
// File trailer for testCoder.cpp
//
// [EOF]
//
