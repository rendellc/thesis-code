// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__TRAITS_HPP_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__TRAITS_HPP_

#include "simulator/msg/detail/float_array__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<simulator::msg::FloatArray>()
{
  return "simulator::msg::FloatArray";
}

template<>
inline const char * name<simulator::msg::FloatArray>()
{
  return "simulator/msg/FloatArray";
}

template<>
struct has_fixed_size<simulator::msg::FloatArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<simulator::msg::FloatArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<simulator::msg::FloatArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__TRAITS_HPP_
