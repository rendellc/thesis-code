// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tuts:srv/AddInts.idl
// generated code does not contain a copyright notice

#ifndef TUTS__SRV__DETAIL__ADD_INTS__STRUCT_H_
#define TUTS__SRV__DETAIL__ADD_INTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/AddInts in the package tuts.
typedef struct tuts__srv__AddInts_Request
{
  int64_t a;
  int64_t b;
  int64_t c;
} tuts__srv__AddInts_Request;

// Struct for a sequence of tuts__srv__AddInts_Request.
typedef struct tuts__srv__AddInts_Request__Sequence
{
  tuts__srv__AddInts_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tuts__srv__AddInts_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/AddInts in the package tuts.
typedef struct tuts__srv__AddInts_Response
{
  int64_t sum;
} tuts__srv__AddInts_Response;

// Struct for a sequence of tuts__srv__AddInts_Response.
typedef struct tuts__srv__AddInts_Response__Sequence
{
  tuts__srv__AddInts_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tuts__srv__AddInts_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TUTS__SRV__DETAIL__ADD_INTS__STRUCT_H_
