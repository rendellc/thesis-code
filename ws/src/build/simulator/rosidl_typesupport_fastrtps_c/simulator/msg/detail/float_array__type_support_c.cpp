// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice
#include "simulator/msg/detail/float_array__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "simulator/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "simulator/msg/detail/float_array__struct.h"
#include "simulator/msg/detail/float_array__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // drive_torques, steer_torques
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // drive_torques, steer_torques

// forward declare type support functions


using _FloatArray__ros_msg_type = simulator__msg__FloatArray;

static bool _FloatArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FloatArray__ros_msg_type * ros_message = static_cast<const _FloatArray__ros_msg_type *>(untyped_ros_message);
  // Field name: steer_torques
  {
    size_t size = ros_message->steer_torques.size;
    auto array_ptr = ros_message->steer_torques.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: drive_torques
  {
    size_t size = ros_message->drive_torques.size;
    auto array_ptr = ros_message->drive_torques.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _FloatArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FloatArray__ros_msg_type * ros_message = static_cast<_FloatArray__ros_msg_type *>(untyped_ros_message);
  // Field name: steer_torques
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->steer_torques.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->steer_torques);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->steer_torques, size)) {
      return "failed to create array for field 'steer_torques'";
    }
    auto array_ptr = ros_message->steer_torques.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: drive_torques
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->drive_torques.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->drive_torques);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->drive_torques, size)) {
      return "failed to create array for field 'drive_torques'";
    }
    auto array_ptr = ros_message->drive_torques.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator
size_t get_serialized_size_simulator__msg__FloatArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FloatArray__ros_msg_type * ros_message = static_cast<const _FloatArray__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name steer_torques
  {
    size_t array_size = ros_message->steer_torques.size;
    auto array_ptr = ros_message->steer_torques.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drive_torques
  {
    size_t array_size = ros_message->drive_torques.size;
    auto array_ptr = ros_message->drive_torques.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FloatArray__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_simulator__msg__FloatArray(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator
size_t max_serialized_size_simulator__msg__FloatArray(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: steer_torques
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: drive_torques
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _FloatArray__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_simulator__msg__FloatArray(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_FloatArray = {
  "simulator::msg",
  "FloatArray",
  _FloatArray__cdr_serialize,
  _FloatArray__cdr_deserialize,
  _FloatArray__get_serialized_size,
  _FloatArray__max_serialized_size
};

static rosidl_message_type_support_t _FloatArray__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FloatArray,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, simulator, msg, FloatArray)() {
  return &_FloatArray__type_support;
}

#if defined(__cplusplus)
}
#endif
