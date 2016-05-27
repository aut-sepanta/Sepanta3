//
// File: traj_IK.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 26-May-2016 23:28:29
//
#ifndef __TRAJ_IK_H__
#define __TRAJ_IK_H__

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
extern bool traj_IK(const double q_init[3], const double final[2], double TE,
                    double f, emxArray_real_T *Q, emxArray_real_T *v1,
                    emxArray_real_T *v2, emxArray_real_T *v3);

#endif

//
// File trailer for traj_IK.h
//
// [EOF]
//
