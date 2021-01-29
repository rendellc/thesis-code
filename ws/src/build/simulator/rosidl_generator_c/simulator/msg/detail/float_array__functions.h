// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__FUNCTIONS_H_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "simulator/msg/rosidl_generator_c__visibility_control.h"

#include "simulator/msg/detail/float_array__struct.h"

/// Initialize msg/FloatArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * simulator__msg__FloatArray
 * )) before or use
 * simulator__msg__FloatArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
bool
simulator__msg__FloatArray__init(simulator__msg__FloatArray * msg);

/// Finalize msg/FloatArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
void
simulator__msg__FloatArray__fini(simulator__msg__FloatArray * msg);

/// Create msg/FloatArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * simulator__msg__FloatArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
simulator__msg__FloatArray *
simulator__msg__FloatArray__create();

/// Destroy msg/FloatArray message.
/**
 * It calls
 * simulator__msg__FloatArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
void
simulator__msg__FloatArray__destroy(simulator__msg__FloatArray * msg);


/// Initialize array of msg/FloatArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * simulator__msg__FloatArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
bool
simulator__msg__FloatArray__Sequence__init(simulator__msg__FloatArray__Sequence * array, size_t size);

/// Finalize array of msg/FloatArray messages.
/**
 * It calls
 * simulator__msg__FloatArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
void
simulator__msg__FloatArray__Sequence__fini(simulator__msg__FloatArray__Sequence * array);

/// Create array of msg/FloatArray messages.
/**
 * It allocates the memory for the array and calls
 * simulator__msg__FloatArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
simulator__msg__FloatArray__Sequence *
simulator__msg__FloatArray__Sequence__create(size_t size);

/// Destroy array of msg/FloatArray messages.
/**
 * It calls
 * simulator__msg__FloatArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator
void
simulator__msg__FloatArray__Sequence__destroy(simulator__msg__FloatArray__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__FUNCTIONS_H_
