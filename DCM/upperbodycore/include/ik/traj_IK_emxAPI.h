//
// File: traj_IK_emxAPI.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 20-Jun-2016 19:33:14
//
#ifndef __TRAJ_IK_EMXAPI_H__
#define __TRAJ_IK_EMXAPI_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "ik/rt_defines.h"
#include "ik/rt_nonfinite.h"
#include "ik/rtwtypes.h"
#include "ik/traj_IK_types.h"

// Function Declarations
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for traj_IK_emxAPI.h
//
// [EOF]
//
