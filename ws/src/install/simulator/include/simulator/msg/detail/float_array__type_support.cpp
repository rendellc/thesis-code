// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "simulator/msg/detail/float_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace simulator
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FloatArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) simulator::msg::FloatArray(_init);
}

void FloatArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<simulator::msg::FloatArray *>(message_memory);
  typed_message->~FloatArray();
}

size_t size_function__FloatArray__steer_torques(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FloatArray__steer_torques(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__FloatArray__steer_torques(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__FloatArray__steer_torques(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__FloatArray__drive_torques(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FloatArray__drive_torques(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__FloatArray__drive_torques(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__FloatArray__drive_torques(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FloatArray_message_member_array[2] = {
  {
    "steer_torques",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator::msg::FloatArray, steer_torques),  // bytes offset in struct
    nullptr,  // default value
    size_function__FloatArray__steer_torques,  // size() function pointer
    get_const_function__FloatArray__steer_torques,  // get_const(index) function pointer
    get_function__FloatArray__steer_torques,  // get(index) function pointer
    resize_function__FloatArray__steer_torques  // resize(index) function pointer
  },
  {
    "drive_torques",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator::msg::FloatArray, drive_torques),  // bytes offset in struct
    nullptr,  // default value
    size_function__FloatArray__drive_torques,  // size() function pointer
    get_const_function__FloatArray__drive_torques,  // get_const(index) function pointer
    get_function__FloatArray__drive_torques,  // get(index) function pointer
    resize_function__FloatArray__drive_torques  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FloatArray_message_members = {
  "simulator::msg",  // message namespace
  "FloatArray",  // message name
  2,  // number of fields
  sizeof(simulator::msg::FloatArray),
  FloatArray_message_member_array,  // message members
  FloatArray_init_function,  // function to initialize message memory (memory has to be allocated)
  FloatArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FloatArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FloatArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace simulator


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<simulator::msg::FloatArray>()
{
  return &::simulator::msg::rosidl_typesupport_introspection_cpp::FloatArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, simulator, msg, FloatArray)() {
  return &::simulator::msg::rosidl_typesupport_introspection_cpp::FloatArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
