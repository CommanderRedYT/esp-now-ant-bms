#pragma once

// system includes
#include <string>
#include <bit>
#include <type_traits>
#include <vector>

template<typename T> constexpr T convert_big_endian(T val) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
return std::byteswap(val);
#else
return val;
#endif
}

/// Format the byte array \p data of length \p len in pretty-printed, human-readable hex.
std::string format_hex_pretty(const uint8_t *data, size_t length);
/// Format the word array \p data of length \p len in pretty-printed, human-readable hex.
std::string format_hex_pretty(const uint16_t *data, size_t length);
/// Format the vector \p data in pretty-printed, human-readable hex.
std::string format_hex_pretty(const std::vector<uint8_t> &data);
/// Format the vector \p data in pretty-printed, human-readable hex.
std::string format_hex_pretty(const std::vector<uint16_t> &data);
/// Format an unsigned integer in pretty-printed, human-readable hex, starting with the most significant byte.
template<typename T, std::enable_if_t<std::is_unsigned<T>::value, int> = 0> std::string format_hex_pretty(T val) {
val = convert_big_endian(val);
return format_hex_pretty(reinterpret_cast<uint8_t *>(&val), sizeof(T));
}
