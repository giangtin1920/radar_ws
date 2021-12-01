//
// File: testCoder.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 01-Dec-2021 16:24:37
//

// Include Files
#include "testCoder.h"
#include "testCoder_types.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions
//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// UNTITLED3 Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const double payload[272]
//                struct0_T *ptCloud
// Return Type  : void
//
void testCoder(const double payload[272], struct0_T *ptCloud)
{
  float p[68];
  float f;
  int i;
  unsigned char x[272];

  //  % Range, in m
  //  % Angel, in rad
  // , ... % Doplper, in m/s
  ptCloud->numDetectedObj = 17.0;
  for (i = 0; i < 272; i++) {
    double d;
    unsigned char u;
    d = rt_roundd_snf(payload[i]);
    if (d < 256.0) {
      if (d >= 0.0) {
        u = static_cast<unsigned char>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 256.0) {
      u = MAX_uint8_T;
    } else {
      u = 0U;
    }

    x[i] = u;
  }

  memcpy((void *)&p[0], (void *)&x[0], (unsigned int)((size_t)68 * sizeof(float)));
  for (i = 0; i < 17; i++) {
    float f1;
    int b_i;
    b_i = i << 2;
    f = p[b_i];
    f1 = p[b_i + 2];
    ptCloud->z[i] = f * static_cast<float>(sin(static_cast<double>(f1)));
    f *= static_cast<float>(cos(static_cast<double>(f1)));
    f1 = p[b_i + 1];
    ptCloud->y[i] = f * static_cast<float>(cos(static_cast<double>(f1)));
    ptCloud->x[i] = f * static_cast<float>(sin(static_cast<double>(f1)));
    ptCloud->doppler[i] = p[b_i + 3];
  }
}

//
// File trailer for testCoder.cpp
//
// [EOF]
//
