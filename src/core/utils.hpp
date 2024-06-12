#ifndef YART_UTILS_HPP
#define YART_UTILS_HPP

#include <chrono>

namespace yart {

template<typename Rep, typename Period>
[[nodiscard]] constexpr auto toMillis(std::chrono::duration<Rep, Period> t) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(t);
}

}


#endif //YART_UTILS_HPP
