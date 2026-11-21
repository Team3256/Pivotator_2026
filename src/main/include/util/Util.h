// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

/** Basic functions that are used often. */
namespace Util {

inline constexpr double kEpsilon = 1e-12;

inline int Gcd(int a, int b) {
  while (b != 0) {
    int temp = b;
    b = a % b;
    a = temp;
  }
  return std::abs(a);
}

inline int Lcm(int a, int b) {
  return std::abs(a * b) / Gcd(a, b);
}

inline double Limit(double v, double min, double max) {
  return std::min(max, std::max(min, v));
}

/** Limits the given input to the given magnitude. */
inline double Limit(double v, double maxMagnitude) {
  return Limit(v, -maxMagnitude, maxMagnitude);
}

inline int Limit(int v, int min, int max) {
  return std::min(max, std::max(min, v));
}

/** Checks if the given input is within the range (min, max), both exclusive. */
inline bool InRange(double v, double min, double max) {
  return v > min && v < max;
}

inline bool InRange(double v, double maxMagnitude) {
  return InRange(v, -maxMagnitude, maxMagnitude);
}

inline double Interpolate(double a, double b, double x) {
  x = Limit(x, 0.0, 1.0);
  return a + (b - a) * x;
}

inline std::string JoinStrings(const std::string& delim, const std::vector<std::string>& strings) {
  std::ostringstream sb;
  for (size_t i = 0; i < strings.size(); ++i) {
    sb << strings[i];
    if (i < strings.size() - 1) {
      sb << delim;
    }
  }
  return sb.str();
}

inline bool EpsilonEquals(double a, double b, double epsilon = kEpsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}

inline bool EpsilonEquals(int a, int b, int epsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}

inline bool AllCloseTo(const std::vector<double>& list, double value, double epsilon) {
  bool result = true;
  for (double v : list) {
    result &= EpsilonEquals(v, value, epsilon);
  }
  return result;
}

inline double SnapToClosest(const std::vector<double>& list, double value) {
  double closest = std::numeric_limits<double>::max();
  double minDiff = std::numeric_limits<double>::max();
  for (double v : list) {
    double diff = std::abs(v - value);
    if (diff < minDiff) {
      minDiff = diff;
      closest = v;
    }
  }
  return closest;
}

inline double SnapToZone(const std::vector<double>& list, double value, double epsilon) {
  double closest = std::numeric_limits<double>::max();
  double minDiff = std::numeric_limits<double>::max();
  for (double v : list) {
    double diff = std::abs(v - value);
    if (diff < minDiff) {
      minDiff = diff;
      closest = v;
    }
  }
  return minDiff < epsilon ? closest : value;
}

inline double HandleDeadband(double value, double deadband) {
  deadband = std::abs(deadband);
  if (deadband == 1) {
    return 0;
  }
  double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
  return (std::abs(value) > std::abs(deadband)) ? scaledValue : 0;
}

}  // namespace Util
