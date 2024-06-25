#include "sobol-matrices.hpp"

namespace yart::sobol {

const uint32_t matrices[nSobolDimensions * sobolMatrixSize] = {
  // Defined in a file with a non-standard extension so clang-format doesn't
  // choke on 8,000+ lines of data
#include "sobol.tables"
};

}