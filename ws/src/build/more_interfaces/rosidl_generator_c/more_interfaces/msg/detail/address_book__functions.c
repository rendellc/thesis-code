// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from more_interfaces:msg/AddressBook.idl
// generated code does not contain a copyright notice
#include "more_interfaces/msg/detail/address_book__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `first_name`
// Member `last_name`
// Member `address`
#include "rosidl_runtime_c/string_functions.h"

bool
more_interfaces__msg__AddressBook__init(more_interfaces__msg__AddressBook * msg)
{
  if (!msg) {
    return false;
  }
  // first_name
  if (!rosidl_runtime_c__String__init(&msg->first_name)) {
    more_interfaces__msg__AddressBook__fini(msg);
    return false;
  }
  // last_name
  if (!rosidl_runtime_c__String__init(&msg->last_name)) {
    more_interfaces__msg__AddressBook__fini(msg);
    return false;
  }
  // gender
  // age
  // address
  if (!rosidl_runtime_c__String__init(&msg->address)) {
    more_interfaces__msg__AddressBook__fini(msg);
    return false;
  }
  return true;
}

void
more_interfaces__msg__AddressBook__fini(more_interfaces__msg__AddressBook * msg)
{
  if (!msg) {
    return;
  }
  // first_name
  rosidl_runtime_c__String__fini(&msg->first_name);
  // last_name
  rosidl_runtime_c__String__fini(&msg->last_name);
  // gender
  // age
  // address
  rosidl_runtime_c__String__fini(&msg->address);
}

more_interfaces__msg__AddressBook *
more_interfaces__msg__AddressBook__create()
{
  more_interfaces__msg__AddressBook * msg = (more_interfaces__msg__AddressBook *)malloc(sizeof(more_interfaces__msg__AddressBook));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(more_interfaces__msg__AddressBook));
  bool success = more_interfaces__msg__AddressBook__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
more_interfaces__msg__AddressBook__destroy(more_interfaces__msg__AddressBook * msg)
{
  if (msg) {
    more_interfaces__msg__AddressBook__fini(msg);
  }
  free(msg);
}


bool
more_interfaces__msg__AddressBook__Sequence__init(more_interfaces__msg__AddressBook__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  more_interfaces__msg__AddressBook * data = NULL;
  if (size) {
    data = (more_interfaces__msg__AddressBook *)calloc(size, sizeof(more_interfaces__msg__AddressBook));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = more_interfaces__msg__AddressBook__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        more_interfaces__msg__AddressBook__fini(&data[i - 1]);
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
more_interfaces__msg__AddressBook__Sequence__fini(more_interfaces__msg__AddressBook__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      more_interfaces__msg__AddressBook__fini(&array->data[i]);
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

more_interfaces__msg__AddressBook__Sequence *
more_interfaces__msg__AddressBook__Sequence__create(size_t size)
{
  more_interfaces__msg__AddressBook__Sequence * array = (more_interfaces__msg__AddressBook__Sequence *)malloc(sizeof(more_interfaces__msg__AddressBook__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = more_interfaces__msg__AddressBook__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
more_interfaces__msg__AddressBook__Sequence__destroy(more_interfaces__msg__AddressBook__Sequence * array)
{
  if (array) {
    more_interfaces__msg__AddressBook__Sequence__fini(array);
  }
  free(array);
}
