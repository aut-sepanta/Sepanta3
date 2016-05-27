//
// File: traj_IK_emxutil.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 26-May-2016 04:27:53
//
#ifndef __TRAJ_IK_EMXUTIL_H__
#define __TRAJ_IK_EMXUTIL_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "traj_IK_types.h"

// Function Declarations
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T1(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T2(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for traj_IK_emxutil.h
//
// [EOF]
//
