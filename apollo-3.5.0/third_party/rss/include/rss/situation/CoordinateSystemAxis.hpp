// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (c) 2018 Intel Corporation
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//	  and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//    IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//    INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//    OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//    POSSIBILITY OF SUCH DAMAGE.
//
// ----------------- END LICENSE BLOCK -----------------------------------


/**
 * @file
 *
 */

#pragma once
#include <limits>

#include <cstdint>
#include <string>
#include <unordered_map>
namespace rss {
namespace situation {
/*!
 * \brief Enum CoordinateSystemAxis
 *
 * Enumeration defining the axis of the situation coordinate system.
 */
enum class CoordinateSystemAxis : int32_t
{
  Longitudinal = 0, /*!< longitudinal axis: ego vehicle is driving in positive direction */
  Lateral = 1       /*!< lateral axis: ego vehicle left to right defines the positive direction */
};

} // namespace situation
} // namespace rss
/*!
 * @brief Conversion of ::rss::situation::CoordinateSystemAxis to std::string helper.
 *
 */
std::string toString(::rss::situation::CoordinateSystemAxis const e);

/*!
 * @brief Conversion from std::string to enum type T helper.
 *
 * @param [in] str - a fully qualified string name of enum class type
 *
 * @return T enum value
 *
 * @throws std::out_of_range exception if the given string does not match any enum type
 *
 * Example usage:
 * @code
 *   auto value = fromString<SomeEnumType>("SomeEnumType::eValue");
 *   assert(value == SomeEnumType::eValue);
 *   // Or:
 *   auto value = fromString<SomeEnumType>("eValue");
 *   assert(value == SomeEnumType::eValue);
 * @endcode
 */
template <typename EnumType> EnumType fromString(std::string const &str);

template <>::rss::situation::CoordinateSystemAxis fromString(std::string const &str);
