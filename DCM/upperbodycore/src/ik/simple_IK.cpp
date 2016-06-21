//
// File: simple_IK.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 25-May-2016 21:40:53
//

// Include Files
#include "ik/rt_nonfinite.h"
#include "ik/simple_IK.h"
#include "ik/linspace.h"

#include "stdio.h"
#include "iostream"
// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

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
// Arguments    : const double q_init[3]
//                const double final[2]
//                double q[3]
// Return Type  : void
//
bool simple_IK(const double q_init[3], const double final[2], double q[3])
{
  std::cout<<"in simple ik"<<std::endl;
  std::cout<<q_init[0]<<" "<<q_init[1]<<" "<<q_init[2]<<std::endl;
  std::cout<<final[0]<<" "<<final[1]<<std::endl;
  std::cout<<q[0]<<" "<<q[1]<<" "<<q[2]<<std::endl;

  double X_temp[91];
  double Y_temp[91];
  double T[1080];
  int k;
  int i0;
  static const signed char iv0[4] = { 0, 0, 1, 0 };

  int i;
  double E;
  double q1;
  double q2;
  double c;
  double d;
  double er1;
  double er2;
  double x;
  double b_x;

  //  ==================== generating path =======================
  //  --- calculating the number of points ---
  //  --- path points ---
  //  --- partial fk ---
  //  ---
  //  -----------------------------
  linspace(((19.5 + 21.8 * cos(q_init[0])) + 17.2 * cos(q_init[0] + q_init[1])) +
           23.5 * cos((q_init[0] + q_init[1]) + q_init[2]), final[0], X_temp);
  linspace((21.8 * sin(q_init[0]) + 17.2 * sin(q_init[0] + q_init[1])) + 23.5 *
           sin((q_init[0] + q_init[1]) + q_init[2]), final[1], Y_temp);
  for (k = 0; k < 90; k++) {
    T[12 * k] = 0.0;
    T[3 + 12 * k] = -1.0;
    T[6 + 12 * k] = 0.0;
    T[9 + 12 * k] = X_temp[1 + k];
    T[1 + 12 * k] = 1.0;
    T[4 + 12 * k] = 0.0;
    T[7 + 12 * k] = 0.0;
    T[10 + 12 * k] = Y_temp[1 + k];
    for (i0 = 0; i0 < 4; i0++) {
      T[2 + (3 * i0 + 12 * k)] = iv0[i0];
    }
  }

  //  ==================== generating q =====================
  //  --- starting point ---
  for (i0 = 0; i0 < 3; i0++) {
    q[i0] = q_init[i0];
  }

  for (i = 0; i < 90; i++) {
    //  --- newq = sepanta_ik(q0,T(:,:,i),L) ---
    //  -----------------------
    E = 1.0;

    //  ----------------------
    q1 = q[0];
    q2 = q[1];
  
    //  ----------------------
    //  ----------------------
    c = (T[9 + 12 * i] - 19.5) - 23.5 * T[12 * i];
    d = T[10 + 12 * i] - 23.5 * T[1 + 12 * i];

    //  ----------------------
    int iteration = 1000000;
    while (E > 1.0E-8 && iteration > 0) {

      iteration--;

      //std::cout<<"I : "<<iteration<<" E : "<<E<<std::endl;
      er1 = (21.8 * cos(q1) + 17.2 * cos(q1 + q2)) - c;
      er2 = (21.8 * sin(q1) + 17.2 * sin(q1 + q2)) - d;
      E = er1 * er1 + er2 * er2;
      x = q1 + q2;
      b_x = q1 + q2;
      q1 += -0.0001 * (2.0 * er1 * (-21.8 * sin(q1) - 17.2 * sin(q1 + q2)) + 2.0
                       * er2 * (21.8 * cos(q1) + 17.2 * cos(q1 + q2)));
      q2 += -0.0001 * (2.0 * er1 * (-17.2 * sin(x)) + 2.0 * er2 * (17.2 * cos
        (b_x)));
    }

    if ( iteration <= 0 )
    {
    	return false;
    }

    //  ----------------------
    // Q(i,:) = newq;
    q[0] = q1;
    q[1] = q2;
    q[2] = (rt_atan2d_snf(T[1 + 12 * i], T[12 * i]) - q1) - q2;

     std::cout<<q[0]<<" "<<q[1]<<" "<<q[2]<<std::endl;

   
  }
   return true;
}

//
// File trailer for simple_IK.cpp
//
// [EOF]
//
