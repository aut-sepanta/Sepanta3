//
// File: traj_IK_types.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 20-Jun-2016 19:33:14
//
#ifndef __TRAJ_IK_TYPES_H__
#define __TRAJ_IK_TYPES_H__

// Include Files
#include "ik/rtwtypes.h"

// Type Definitions
#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray__common

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T
#endif

//
// File trailer for traj_IK_types.h
//
// [EOF]
//
