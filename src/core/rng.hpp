#ifndef YART_RNG_HPP
#define YART_RNG_HPP

#include <random>
#include <xoshiro.hpp>

namespace yart {

class RNG {
public:
  RNG() noexcept = default;

  explicit RNG(uint64_t seed) noexcept;

  [[nodiscard]] float uniform() noexcept;

  void seed(uint64_t seed) noexcept;

private:
  Xoshiro::Xoshiro256PP m_rng;
  std::uniform_real_distribution<float> m_dist;
};

// https://github.com/explosion/murmurhash/blob/master/murmurhash/MurmurHash2.cpp
inline uint64_t murmurHash64A(const uint8_t* key, size_t len, uint64_t seed) {
  const uint64_t m = 0xc6a4a7935bd1e995ull;
  const int r = 47;

  uint64_t h = seed ^ (len * m);

  const uint8_t* end = key + 8 * (len / 8);

  while (key != end) {
    uint64_t k;
    std::memcpy(&k, key, sizeof(uint64_t));
    key += 8;

    k *= m;
    k ^= k >> r;
    k *= m;

    h ^= k;
    h *= m;
  }

  switch (len & 7) {
    case 7:
      h ^= uint64_t(key[6]) << 48;
    case 6:
      h ^= uint64_t(key[5]) << 40;
    case 5:
      h ^= uint64_t(key[4]) << 32;
    case 4:
      h ^= uint64_t(key[3]) << 24;
    case 3:
      h ^= uint64_t(key[2]) << 16;
    case 2:
      h ^= uint64_t(key[1]) << 8;
    case 1:
      h ^= uint64_t(key[0]);
      h *= m;
  }

  h ^= h >> r;
  h *= m;
  h ^= h >> r;

  return h;
}

// Hash util functions yoinked from pbrt
template<typename... Args>
inline void hashRecursiveCopy(char* buf, Args...);

template<>
inline void hashRecursiveCopy(char* buf) {}

template<typename T, typename... Args>
inline void hashRecursiveCopy(char* buf, T v, Args... args) {
  memcpy(buf, &v, sizeof(T));
  hashRecursiveCopy(buf + sizeof(T), args...);
}

template<typename... Args>
inline uint64_t hash(Args... args) {
  constexpr size_t sz = (sizeof(Args) + ... + 0);
  constexpr size_t n = (sz + 7) / 8;
  uint64_t buf[n];
  hashRecursiveCopy((char*) buf, args...);
  return murmurHash64A((const unsigned char*) buf, sz, 0);
}

constexpr uint32_t permel(uint32_t i, uint32_t l, uint32_t p) {
  uint32_t w = l - 1;
  w |= w >> 1;
  w |= w >> 2;
  w |= w >> 4;
  w |= w >> 8;
  w |= w >> 16;

  do {
    i ^= p;
    i *= 0xe170893d;
    i ^= p >> 16;
    i ^= (i & w) >> 4;
    i ^= p >> 8;
    i *= 0x0929eb3f;
    i ^= p >> 23;
    i ^= (i & w) >> 1;
    i *= 1 | p >> 27;
    i *= 0x6935fa69;
    i ^= (i & w) >> 11;
    i *= 0x74dcb303;
    i ^= (i & w) >> 2;
    i *= 0x9e501cc3;
    i ^= (i & w) >> 2;
    i *= 0xc860a3df;
    i &= w;
    i ^= i >> 5;
  } while (i >= l);

  return (i + p) % l;
}

}

#endif //YART_RNG_HPP
