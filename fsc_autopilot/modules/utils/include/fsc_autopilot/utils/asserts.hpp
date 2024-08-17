// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This code is adapted from the Microsoft Guidelines Support Library
// (https://github.com/microsoft/FSC)'s `assert` header

#ifndef FSC_AUTOPILOT_UTILS_ASSERTS_HPP_
#define FSC_AUTOPILOT_UTILS_ASSERTS_HPP_

#if defined(_MSC_VER)
#error This library does not support MSVC/windows

#endif

#include <exception>  // IWYU pragma: export

// Deleted GSL_SUPPRESS definition

#if defined(__clang__) || defined(__GNUC__)
#define FSC_LIKELY(x) __builtin_expect(!!(x), 1)
#define FSC_UNLIKELY(x) __builtin_expect(!!(x), 0)

#else

#define FSC_LIKELY(x) (!!(x))
#define FSC_UNLIKELY(x) (!!(x))
#endif  // defined(__clang__) || defined(__GNUC__)

//
// FSC_ASSUME(cond)
//
// Tell the optimizer that the predicate cond must hold. It is unspecified
// whether or not cond is actually evaluated.
//
#if defined(__GNUC__)
#define FSC_ASSUME(cond) \
  ((cond) ? static_cast<void>(0) : __builtin_unreachable())
#else
#define FSC_ASSUME(cond) static_cast<void>((cond) ? 0 : 0)
#endif

//
// FSC.assert: assertions
//

#define FSC_CONTRACT_CHECK(type, cond) \
  (FSC_LIKELY(cond) ? static_cast<void>(0) : std::terminate())

#define Expects(cond) FSC_CONTRACT_CHECK("Precondition", cond)
#define Ensures(cond) FSC_CONTRACT_CHECK("Postcondition", cond)

#endif  // FSC_AUTOPILOT_UTILS_ASSERTS_HPP_
