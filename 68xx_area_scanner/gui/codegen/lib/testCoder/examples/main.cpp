//
// File: main.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 01-Dec-2021 16:24:37
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "testCoder.h"
#include "testCoder_terminate.h"
#include "testCoder_types.h"

// Function Declarations
static void argInit_272x1_real_T(double result[272]);
static double argInit_real_T();
static void main_testCoder();

// Function Definitions
//
// Arguments    : double result[272]
// Return Type  : void
//
static void argInit_272x1_real_T(double result[272])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 272; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_testCoder()
{
  struct0_T ptCloud;
  double dv[272];

  // Initialize function 'testCoder' input arguments.
  // Initialize function input argument 'payload'.
  // Call the entry-point 'testCoder'.
  argInit_272x1_real_T(dv);
  testCoder(dv, &ptCloud);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_testCoder();

  // Terminate the application.
  // You do not need to do this more than one time.
  testCoder_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
