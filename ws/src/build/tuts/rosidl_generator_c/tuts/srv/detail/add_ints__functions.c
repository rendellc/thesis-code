// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tuts:srv/AddInts.idl
// generated code does not contain a copyright notice
#include "tuts/srv/detail/add_ints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
tuts__srv__AddInts_Request__init(tuts__srv__AddInts_Request * msg)
{
  if (!msg) {
    return false;
  }
  // a
  // b
  // c
  return true;
}

void
tuts__srv__AddInts_Request__fini(tuts__srv__AddInts_Request * msg)
{
  if (!msg) {
    return;
  }
  // a
  // b
  // c
}

tuts__srv__AddInts_Request *
tuts__srv__AddInts_Request__create()
{
  tuts__srv__AddInts_Request * msg = (tuts__srv__AddInts_Request *)malloc(sizeof(tuts__srv__AddInts_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tuts__srv__AddInts_Request));
  bool success = tuts__srv__AddInts_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
tuts__srv__AddInts_Request__destroy(tuts__srv__AddInts_Request * msg)
{
  if (msg) {
    tuts__srv__AddInts_Request__fini(msg);
  }
  free(msg);
}


bool
tuts__srv__AddInts_Request__Sequence__init(tuts__srv__AddInts_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  tuts__srv__AddInts_Request * data = NULL;
  if (size) {
    data = (tuts__srv__AddInts_Request *)calloc(size, sizeof(tuts__srv__AddInts_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tuts__srv__AddInts_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tuts__srv__AddInts_Request__fini(&data[i - 1]);
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
tuts__srv__AddInts_Request__Sequence__fini(tuts__srv__AddInts_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tuts__srv__AddInts_Request__fini(&array->data[i]);
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

tuts__srv__AddInts_Request__Sequence *
tuts__srv__AddInts_Request__Sequence__create(size_t size)
{
  tuts__srv__AddInts_Request__Sequence * array = (tuts__srv__AddInts_Request__Sequence *)malloc(sizeof(tuts__srv__AddInts_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = tuts__srv__AddInts_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
tuts__srv__AddInts_Request__Sequence__destroy(tuts__srv__AddInts_Request__Sequence * array)
{
  if (array) {
    tuts__srv__AddInts_Request__Sequence__fini(array);
  }
  free(array);
}


bool
tuts__srv__AddInts_Response__init(tuts__srv__AddInts_Response * msg)
{
  if (!msg) {
    return false;
  }
  // sum
  return true;
}

void
tuts__srv__AddInts_Response__fini(tuts__srv__AddInts_Response * msg)
{
  if (!msg) {
    return;
  }
  // sum
}

tuts__srv__AddInts_Response *
tuts__srv__AddInts_Response__create()
{
  tuts__srv__AddInts_Response * msg = (tuts__srv__AddInts_Response *)malloc(sizeof(tuts__srv__AddInts_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tuts__srv__AddInts_Response));
  bool success = tuts__srv__AddInts_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
tuts__srv__AddInts_Response__destroy(tuts__srv__AddInts_Response * msg)
{
  if (msg) {
    tuts__srv__AddInts_Response__fini(msg);
  }
  free(msg);
}


bool
tuts__srv__AddInts_Response__Sequence__init(tuts__srv__AddInts_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  tuts__srv__AddInts_Response * data = NULL;
  if (size) {
    data = (tuts__srv__AddInts_Response *)calloc(size, sizeof(tuts__srv__AddInts_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tuts__srv__AddInts_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tuts__srv__AddInts_Response__fini(&data[i - 1]);
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
tuts__srv__AddInts_Response__Sequence__fini(tuts__srv__AddInts_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tuts__srv__AddInts_Response__fini(&array->data[i]);
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

tuts__srv__AddInts_Response__Sequence *
tuts__srv__AddInts_Response__Sequence__create(size_t size)
{
  tuts__srv__AddInts_Response__Sequence * array = (tuts__srv__AddInts_Response__Sequence *)malloc(sizeof(tuts__srv__AddInts_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = tuts__srv__AddInts_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
tuts__srv__AddInts_Response__Sequence__destroy(tuts__srv__AddInts_Response__Sequence * array)
{
  if (array) {
    tuts__srv__AddInts_Response__Sequence__fini(array);
  }
  free(array);
}
