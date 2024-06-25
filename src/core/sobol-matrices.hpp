#ifndef YART_SOBOL_MATRICES_HPP
#define YART_SOBOL_MATRICES_HPP

#include <math/math.hpp>

namespace yart::sobol {

constexpr size_t nSobolDimensions = 1024;
constexpr size_t sobolMatrixSize = 52;

extern const uint32_t matrices[nSobolDimensions * sobolMatrixSize];

}

#endif //YART_SOBOL_MATRICES_HPP
