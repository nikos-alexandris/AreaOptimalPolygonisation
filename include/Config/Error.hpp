#pragma once

#include <exception>

namespace project {
namespace config {

class Error : public std::exception {};

} // namespace config
} // namespace project