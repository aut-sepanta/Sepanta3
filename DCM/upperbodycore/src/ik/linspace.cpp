//
// File: linspace.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 25-May-2016 21:40:53
//

// Include Files
#include "ik/rt_nonfinite.h"
#include "ik/simple_IK.h"
#include "ik/linspace.h"

// Function Definitions

//
// Arguments    : double d1
//                double d2
//                double y[91]
// Return Type  : void
//
void linspace(double d1, double d2, double y[91])
{
  double delta1;
  double delta2;
  int k;
  y[90] = d2;
  y[0] = d1;
  if (((d1 < 0.0) != (d2 < 0.0)) && ((fabs(d1) > 8.9884656743115785E+307) ||
       (fabs(d2) > 8.9884656743115785E+307))) {
    delta1 = d1 / 90.0;
    delta2 = d2 / 90.0;
    for (k = 0; k < 89; k++) {
      y[1 + k] = (d1 + delta2 * (1.0 + (double)k)) - delta1 * (1.0 + (double)k);
    }
  } else {
    delta1 = (d2 - d1) / 90.0;
    for (k = 0; k < 89; k++) {
      y[1 + k] = d1 + (1.0 + (double)k) * delta1;
    }
  }
}

//
// File trailer for linspace.cpp
//
// [EOF]
//
