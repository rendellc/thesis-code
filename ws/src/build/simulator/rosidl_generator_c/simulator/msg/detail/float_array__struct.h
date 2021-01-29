// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_H_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'steer_torques'
// Member 'drive_torques'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/FloatArray in the package simulator.
typedef struct simulator__msg__FloatArray
{
  rosidl_runtime_c__double__Sequence steer_torques;
  rosidl_runtime_c__double__Sequence drive_torques;
} simulator__msg__FloatArray;

// Struct for a sequence of simulator__msg__FloatArray.
typedef struct simulator__msg__FloatArray__Sequence
{
  simulator__msg__FloatArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulator__msg__FloatArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_H_
