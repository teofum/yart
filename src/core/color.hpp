#ifndef YART_COLOR_HPP
#define YART_COLOR_HPP

#include "spectrum.hpp"

namespace yart::color {

class XYZ {
public:
  float X = 0, Y = 0, Z = 0;

//  static XYZ fromSpectrum(const Spectrum& s) {
//    return XYZ(
//
//    ) / Spectra::CIE_Y_integral;
//  }

  constexpr XYZ(float X, float Y, float Z) noexcept: X(X), Y(Y), Z(Z) {}
};

}

#endif //YART_COLOR_HPP
