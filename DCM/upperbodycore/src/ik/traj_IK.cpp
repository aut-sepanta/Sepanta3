//
// File: traj_IK.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 26-May-2016 23:28:29
//

// Include Files
#include "ik/rt_nonfinite.h"
#include "ik/traj_IK.h"
#include "ik/traj_IK_emxutil.h"
#include <iostream>
#include <stdio.h>
// Function Declarations
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : const double q_init[3]
//                const double final[2]
//                double TE
//                double f
//                emxArray_real_T *Q
//                emxArray_real_T *v1
//                emxArray_real_T *v2
//                emxArray_real_T *v3
// Return Type  : void
//
bool traj_IK(const double q_init[3], const double final[2], double TE, double f,
             emxArray_real_T *Q, emxArray_real_T *v1, emxArray_real_T *v2,
             emxArray_real_T *v3)
{
  double num_dot;
  double X1;
  double Y1;
  double bb;
  double aa;
  double delta1;
  emxArray_real_T *step;
  int i0;
  int k;
  emxArray_real_T *X;
  emxArray_real_T *Y;
  int i;
  emxArray_real_T *T;
  static const signed char iv0[4] = { 0, 0, 1, 0 };

  double newq[3];
  double E;
  double q1;
  double q2;
  double c;
  double d;
  double er1;
  double er2;
  double x;
  emxArray_real_T *v;
  int m;

  //  ==================== generating path =======================
  //  --- calculating the number of points ---
  num_dot = f * TE;

  //  --- path points ---
  //  --- partial fk ---
  X1 = ((8.5 + 22.0 * cos(q_init[0])) + 24.8 * cos(q_init[0] + q_init[1])) + 8.0
    * cos((q_init[0] + q_init[1]) + q_init[2]);
  Y1 = (22.0 * sin(q_init[0]) + 24.8 * sin(q_init[0] + q_init[1])) + 8.0 * sin
    ((q_init[0] + q_init[1]) + q_init[2]);

  //  ---
  //  -----------------------------
  bb = 3.0 * (final[0] - X1) / (TE * TE);
  aa = -2.0 * bb / (3.0 * TE);

  //  --- generating path ---
  delta1 = num_dot;
  if (num_dot < 0.0) {
    delta1 = 0.0;
  }

  emxInit_real_T(&step, 2);
  i0 = step->size[0] * step->size[1];
  step->size[0] = 1;
  step->size[1] = (int)floor(delta1);
  emxEnsureCapacity((emxArray__common *)step, i0, (int)sizeof(double));
  if (step->size[1] >= 1) {
    step->data[step->size[1] - 1] = TE;
    if (step->size[1] >= 2) {
      step->data[0] = 0.0;
      if (step->size[1] >= 3) {
        if ((TE < 0.0) && (fabs(TE) > 8.9884656743115785E+307)) {
          delta1 = TE / ((double)step->size[1] - 1.0);
          i0 = step->size[1];
          for (k = 0; k <= i0 - 3; k++) {
            step->data[1 + k] = delta1 * (1.0 + (double)k);
          }
        } else {
          delta1 = TE / ((double)step->size[1] - 1.0);
          i0 = step->size[1];
          for (k = 0; k <= i0 - 3; k++) {
            step->data[1 + k] = (1.0 + (double)k) * delta1;
          }
        }
      }
    }
  }

  emxInit_real_T(&X, 2);
  i0 = X->size[0] * X->size[1];
  X->size[0] = 1;
  X->size[1] = step->size[1];
  emxEnsureCapacity((emxArray__common *)X, i0, (int)sizeof(double));
  k = step->size[1];
  for (i0 = 0; i0 < k; i0++) {
    X->data[i0] = 0.0;
  }

  emxInit_real_T(&Y, 2);
  i0 = Y->size[0] * Y->size[1];
  Y->size[0] = 1;
  Y->size[1] = step->size[1];
  emxEnsureCapacity((emxArray__common *)Y, i0, (int)sizeof(double));
  k = step->size[1];
  for (i0 = 0; i0 < k; i0++) {
    Y->data[i0] = 0.0;
  }

  for (i = 0; i < step->size[1]; i++) {
    X->data[i] = (aa * rt_powd_snf(step->data[i], 3.0) + bb * (step->data[i] *
      step->data[i])) + X1;
    Y->data[i] = (final[1] - Y1) / (final[0] - X1) * (X->data[i] - X1) + Y1;
  }

  emxFree_real_T(&step);
  emxInit_real_T1(&T, 3);
  i0 = T->size[0] * T->size[1] * T->size[2];
  T->size[0] = 3;
  T->size[1] = 4;
  T->size[2] = (int)num_dot;
  emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
  k = 12 * (int)num_dot;
  for (i0 = 0; i0 < k; i0++) {
    T->data[i0] = 0.0;
  }

  for (k = 0; k < X->size[1]; k++) {
    T->data[T->size[0] * T->size[1] * k] = 0.0;
    T->data[T->size[0] + T->size[0] * T->size[1] * k] = -1.0;
    T->data[(T->size[0] << 1) + T->size[0] * T->size[1] * k] = 0.0;
    T->data[T->size[0] * 3 + T->size[0] * T->size[1] * k] = X->data[k];
    T->data[1 + T->size[0] * T->size[1] * k] = 1.0;
    T->data[(T->size[0] + T->size[0] * T->size[1] * k) + 1] = 0.0;
    T->data[((T->size[0] << 1) + T->size[0] * T->size[1] * k) + 1] = 0.0;
    T->data[(T->size[0] * 3 + T->size[0] * T->size[1] * k) + 1] = Y->data[k];
    for (i0 = 0; i0 < 4; i0++) {
      T->data[(T->size[0] * i0 + T->size[0] * T->size[1] * k) + 2] = iv0[i0];
    }
  }

  emxFree_real_T(&Y);
  emxFree_real_T(&X);

  //  ==================== generating q =====================
  //  --- starting point ---
  for (i0 = 0; i0 < 3; i0++) {
    newq[i0] = q_init[i0];
  }

  i0 = Q->size[0] * Q->size[1];
  Q->size[0] = (int)num_dot;
  Q->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)Q, i0, (int)sizeof(double));
  k = (int)num_dot * 3;
  for (i0 = 0; i0 < k; i0++) {
    Q->data[i0] = 0.0;
  }

  for (i = 0; i < (int)num_dot; i++) {
    //  --- newq = sepanta_ik(q0,T(:,:,i),L) ---
    //  -----------------------
    E = 1.0;

    //  ----------------------
    q1 = newq[0];
    q2 = newq[1];

    //  ----------------------
    //  ----------------------
    c = (T->data[T->size[0] * 3 + T->size[0] * T->size[1] * i] - 8.5) - 8.0 *
      T->data[T->size[0] * T->size[1] * i];
    d = T->data[(T->size[0] * 3 + T->size[0] * T->size[1] * i) + 1] - 8.0 *
      T->data[1 + T->size[0] * T->size[1] * i];

    //  ----------------------
    int iteration = 1000000;
    while (E > 1.0E-8 && iteration > 0) 
    {
      iteration--;
      er1 = (22.0 * cos(q1) + 24.8 * cos(q1 + q2)) - c;
      er2 = (22.0 * sin(q1) + 24.8 * sin(q1 + q2)) - d;
      E = er1 * er1 + er2 * er2;
      delta1 = q1 + q2;
      x = q1 + q2;
      q1 += -0.0001 * (2.0 * er1 * (-22.0 * sin(q1) - 24.8 * sin(q1 + q2)) + 2.0
                       * er2 * (22.0 * cos(q1) + 24.8 * cos(q1 + q2)));
      q2 += -0.0001 * (2.0 * er1 * (-24.8 * sin(delta1)) + 2.0 * er2 * (24.8 *
        cos(x)));
    }

    if ( iteration <= 0 )
    {
    	return false;
    }

    //  ----------------------
    newq[0] = q1;
    newq[1] = q2;
    newq[2] = (rt_atan2d_snf(T->data[1 + T->size[0] * T->size[1] * i], T->data
                [T->size[0] * T->size[1] * i]) - q1) - q2;
    for (i0 = 0; i0 < 3; i0++) {
      Q->data[i + Q->size[0] * i0] = newq[i0];
    }
  }

  

  emxFree_real_T(&T);
  emxInit_real_T(&v, 2);

  //  --- target ---
  //  --- velocities ---
  //  ==================== generating v ======================
  i0 = v->size[0] * v->size[1];
  v->size[0] = (int)num_dot;
  v->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)v, i0, (int)sizeof(double));
  k = (int)num_dot * 3;
  for (i0 = 0; i0 < k; i0++) {
    v->data[i0] = 0.0;
  }

  for (m = 0; m < 3; m++) {
    for (i = 0; i < (int)(num_dot - 1.0); i++) {
      v->data[((int)((1.0 + (double)i) + 1.0) + v->size[0] * m) - 1] = (Q->data
        [((int)((1.0 + (double)i) + 1.0) + Q->size[0] * m) - 1] - Q->data[i +
        Q->size[0] * m]) / (1.0 / f);
    }

    v->data[v->size[0] * m] = v->data[1 + v->size[0] * m] / 2.0;
  }

  k = v->size[0];
  i0 = v1->size[0];
  v1->size[0] = k;
  emxEnsureCapacity((emxArray__common *)v1, i0, (int)sizeof(double));
  for (i0 = 0; i0 < k; i0++) {
    v1->data[i0] = v->data[i0];
  }

  k = v->size[0];
  i0 = v2->size[0];
  v2->size[0] = k;
  emxEnsureCapacity((emxArray__common *)v2, i0, (int)sizeof(double));
  for (i0 = 0; i0 < k; i0++) {
    v2->data[i0] = v->data[i0 + v->size[0]];
  }

  k = v->size[0];
  i0 = v3->size[0];
  v3->size[0] = k;
  emxEnsureCapacity((emxArray__common *)v3, i0, (int)sizeof(double));
  for (i0 = 0; i0 < k; i0++) {
    v3->data[i0] = v->data[i0 + (v->size[0] << 1)];
  }

  emxFree_real_T(&v);

  return true;
}

//
// File trailer for traj_IK.cpp
//
// [EOF]
//
