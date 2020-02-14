#include <usv_gazebo_plugins/impeller.h>

#include <algorithm> // min/max
#include <math.h>

using namespace gazebo;

Impeller::Impeller(double maxForceFwd, double maxForceRev,
                   ThrustMapping thrustMapping)
    : maxForceFwd_{maxForceFwd}, maxForceRev_{std::abs(maxForceRev)},
      thrustMapping_{thrustMapping} {}

Impeller::Impeller(double maxForceFwd, double maxForceRev)
    : maxForceFwd_{maxForceFwd}, maxForceRev_{std::abs(maxForceRev)},
      thrustMapping_{ThrustMapping::linear} {}

double Impeller::scaleThrust(double percentage) {
  switch (thrustMapping_) {
  case ThrustMapping::linear:
    return scaleThrustLinear(percentage);
  case ThrustMapping::GLF:
    return scaleThrustGLF(percentage);
  default:
    return scaleThrustLinear(percentage);
  }
}

double Impeller::scaleThrustLinear(double percentage) {
  double result = 0.0;
  if (percentage < 0.0) {
    result = maxForceRev_ * percentage;
    result = std::max(result, -1 * maxForceRev_);
  } else {
    result = maxForceFwd_ * percentage;
    result = std::min(result, maxForceFwd_);
  }
  return result;
}

double Impeller::scaleThrustGLF(double percentage) {
  double val = 0.0;
  if (percentage > 0.01) {
    val = glf(percentage, 0.01, 59.82, 5.0, 0.38, 0.56, 0.28);
    val = std::min(val, maxForceFwd_);
  } else if (percentage < 0.01) {
    val = glf(percentage, -199.13, -0.09, 8.84, 5.34, 0.99, -0.57);
    val = std::max(val, maxForceRev_);
  } else {
    val = 0.0;
  }
  return val;
}

double Impeller::glf(double x, float A, float K, float B, float v, float C,
                     float M) {
  return A + (K - A) / (pow(C + exp(-B * (x - M)), 1.0 / v));
}

double Impeller::getMaxForceRev() { return maxForceRev_; }

double Impeller::getMaxForceFwd() { return maxForceFwd_; }

ThrustMapping Impeller::getThrustMapping() { return thrustMapping_; }

void Impeller::setMaxForceFwd(double maxForce) { maxForceFwd_ = maxForce; }

void Impeller::setMaxForceRev(double maxForce) {
  maxForceRev_ = std::abs(maxForce);
}

void Impeller::setThrustMapping(ThrustMapping thrustMapping) {
  thrustMapping_ = thrustMapping;
}
