#pragma once
#include <cmath>

double deg2rad(double deg);

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

template<class T>
constexpr const T sign(const T& val) {
    return (T(0) < val) - (val < T(0));
}