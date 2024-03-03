// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_COMMON_STRING_H_
#define LINE_FOLLOWER_BLOCKS_COMMON_STRING_H_

#include <memory>
#include <string>
#include <utility>

namespace line_follower {

template <typename... Args>
std::string string_format(const std::string& format, Args&&... args) {
    size_t size = std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args)...) +
                  1;  // Extra space for '\0'
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), std::forward<Args>(args)...);
    return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_COMMON_STRING_H_
