#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  pError = iError = dError = 0.0;
}

void PID::UpdateError(double cte) {
  dError = cte - pError;
  pError = cte;
  iError += cte;
}

double PID::TotalError() {
  return - Kp * pError - Kd * dError - Ki * iError;
}

