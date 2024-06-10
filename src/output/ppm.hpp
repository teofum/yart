#ifndef YART_PPM_HPP
#define YART_PPM_HPP

#include "core/buffer.hpp"

namespace yart::output {

void writePPM(std::ostream& ostream, const Buffer& buffer);

}

#endif //YART_PPM_HPP
