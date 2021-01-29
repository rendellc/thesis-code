// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice
#include "simulator/msg/detail/float_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `steer_torques`
// Member `drive_torques`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
simulator__msg__FloatArray__init(simulator__msg__FloatArray * msg)
{
  if (!msg) {
    return false;
  }
  // steer_torques
  if (!rosidl_runtime_c__double__Sequence__init(&msg->steer_torques, 0)) {
    simulator__msg__FloatArray__fini(msg);
    return false;
  }
  // drive_torques
  if (!rosidl_runtime_c__double__Sequence__init(&msg->drive_torques, 0)) {
    simulator__msg__FloatArray__fini(msg);
    return false;
  }
  return true;
}

void
simulator__msg__FloatArray__fini(simulator__msg__FloatArray * msg)
{
  if (!msg) {
    return;
  }
  // steer_torques
  rosidl_runtime_c__double__Sequence__fini(&msg->steer_torques);
  // drive_torques
  rosidl_runtime_c__double__Sequence__fini(&msg->drive_torques);
}

simulator__msg__FloatArray *
simulator__msg__FloatArray__create()
{
  simulator__msg__FloatArray * msg = (simulator__msg__FloatArray *)malloc(sizeof(simulator__msg__FloatArray));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulator__msg__FloatArray));
  bool success = simulator__msg__FloatArray__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
simulator__msg__FloatArray__destroy(simulator__msg__FloatArray * msg)
{
  if (msg) {
    simulator__msg__FloatArray__fini(msg);
  }
  free(msg);
}


bool
simulator__msg__FloatArray__Sequence__init(simulator__msg__FloatArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  simulator__msg__FloatArray * data = NULL;
  if (size) {
    data = (simulator__msg__FloatArray *)calloc(size, sizeof(simulator__msg__FloatArray));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulator__msg__FloatArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulator__msg__FloatArray__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
simulator__msg__FloatArray__Sequence__fini(simulator__msg__FloatArray__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      simulator__msg__FloatArray__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

simulator__msg__FloatArray__Sequence *
simulator__msg__FloatArray__Sequence__create(size_t size)
{
  simulator__msg__FloatArray__Sequence * array = (simulator__msg__FloatArray__Sequence *)malloc(sizeof(simulator__msg__FloatArray__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = simulator__msg__FloatArray__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
simulator__msg__FloatArray__Sequence__destroy(simulator__msg__FloatArray__Sequence * array)
{
  if (array) {
    simulator__msg__FloatArray__Sequence__fini(array);
  }
  free(array);
}
