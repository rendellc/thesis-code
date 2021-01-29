// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tuts:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef TUTS__MSG__DETAIL__NUM__STRUCT_H_
#define TUTS__MSG__DETAIL__NUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Num in the package tuts.
typedef struct tuts__msg__Num
{
  int64_t value;
} tuts__msg__Num;

// Struct for a sequence of tuts__msg__Num.
typedef struct tuts__msg__Num__Sequence
{
  tuts__msg__Num * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tuts__msg__Num__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TUTS__MSG__DETAIL__NUM__STRUCT_H_
