//
// File: traj_IK_emxAPI.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 26-May-2016 04:27:53
//

// Include Files
#include "ik/rt_nonfinite.h"
#include "ik/traj_IK.h"
#include "ik/traj_IK_emxAPI.h"
#include "ik/traj_IK_emxutil.h"

// Function Definitions

//
// Arguments    : int numDimensions
//                int *size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  emxInit_real_T2(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : double *data
//                int numDimensions
//                int *size
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapperND_real_T(double *data, int numDimensions, int *
  size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  emxInit_real_T2(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : double *data
//                int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols)
{
  emxArray_real_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real_T2(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

//
// Arguments    : int rows
//                int cols
// Return Type  : emxArray_real_T *
//
emxArray_real_T *emxCreate_real_T(int rows, int cols)
{
  emxArray_real_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real_T2(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

//
// Arguments    : emxArray_real_T *emxArray
// Return Type  : void
//
void emxDestroyArray_real_T(emxArray_real_T *emxArray)
{
  emxFree_real_T(&emxArray);
}

//
// Arguments    : emxArray_real_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxInit_real_T2(pEmxArray, numDimensions);
}

//
// File trailer for traj_IK_emxAPI.cpp
//
// [EOF]
//
