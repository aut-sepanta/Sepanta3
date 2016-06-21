//
// File: traj_IK_emxutil.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 20-Jun-2016 19:33:14
//
#ifndef __TRAJ_IK_EMXUTIL_H__
#define __TRAJ_IK_EMXUTIL_H__

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
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T1(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for traj_IK_emxutil.h
//
// [EOF]
//
