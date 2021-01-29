// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tuts:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef TUTS__MSG__DETAIL__NUM__TRAITS_HPP_
#define TUTS__MSG__DETAIL__NUM__TRAITS_HPP_

#include "tuts/msg/detail/num__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tuts::msg::Num>()
{
  return "tuts::msg::Num";
}

template<>
inline const char * name<tuts::msg::Num>()
{
  return "tuts/msg/Num";
}

template<>
struct has_fixed_size<tuts::msg::Num>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tuts::msg::Num>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tuts::msg::Num>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TUTS__MSG__DETAIL__NUM__TRAITS_HPP_
