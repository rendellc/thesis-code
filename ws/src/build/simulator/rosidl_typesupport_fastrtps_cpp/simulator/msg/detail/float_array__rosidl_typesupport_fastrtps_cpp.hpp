// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "simulator/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "simulator/msg/detail/float_array__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace simulator
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_simulator
cdr_serialize(
  const simulator::msg::FloatArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_simulator
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  simulator::msg::FloatArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_simulator
get_serialized_size(
  const simulator::msg::FloatArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_simulator
max_serialized_size_FloatArray(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace simulator

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_simulator
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, simulator, msg, FloatArray)();

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
