#ifndef YART_METAL_UTILS_HPP
#define YART_METAL_UTILS_HPP

#include <Foundation/Foundation.hpp>

namespace yart::frontend::metal {

[[nodiscard]] constexpr NS::String* operator ""_ns(
  const char* cStr,
  size_t len
) noexcept {
  return NS::String::string(cStr, NS::UTF8StringEncoding);
}

}

#endif //YART_METAL_UTILS_HPP
