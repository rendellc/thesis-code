// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tuts:srv/AddInts.idl
// generated code does not contain a copyright notice

#ifndef TUTS__SRV__DETAIL__ADD_INTS__TRAITS_HPP_
#define TUTS__SRV__DETAIL__ADD_INTS__TRAITS_HPP_

#include "tuts/srv/detail/add_ints__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tuts::srv::AddInts_Request>()
{
  return "tuts::srv::AddInts_Request";
}

template<>
inline const char * name<tuts::srv::AddInts_Request>()
{
  return "tuts/srv/AddInts_Request";
}

template<>
struct has_fixed_size<tuts::srv::AddInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tuts::srv::AddInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tuts::srv::AddInts_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tuts::srv::AddInts_Response>()
{
  return "tuts::srv::AddInts_Response";
}

template<>
inline const char * name<tuts::srv::AddInts_Response>()
{
  return "tuts/srv/AddInts_Response";
}

template<>
struct has_fixed_size<tuts::srv::AddInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tuts::srv::AddInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tuts::srv::AddInts_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tuts::srv::AddInts>()
{
  return "tuts::srv::AddInts";
}

template<>
inline const char * name<tuts::srv::AddInts>()
{
  return "tuts/srv/AddInts";
}

template<>
struct has_fixed_size<tuts::srv::AddInts>
  : std::integral_constant<
    bool,
    has_fixed_size<tuts::srv::AddInts_Request>::value &&
    has_fixed_size<tuts::srv::AddInts_Response>::value
  >
{
};

template<>
struct has_bounded_size<tuts::srv::AddInts>
  : std::integral_constant<
    bool,
    has_bounded_size<tuts::srv::AddInts_Request>::value &&
    has_bounded_size<tuts::srv::AddInts_Response>::value
  >
{
};

template<>
struct is_service<tuts::srv::AddInts>
  : std::true_type
{
};

template<>
struct is_service_request<tuts::srv::AddInts_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tuts::srv::AddInts_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TUTS__SRV__DETAIL__ADD_INTS__TRAITS_HPP_
