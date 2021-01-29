// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_HPP_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__simulator__msg__FloatArray __attribute__((deprecated))
#else
# define DEPRECATED__simulator__msg__FloatArray __declspec(deprecated)
#endif

namespace simulator
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FloatArray_
{
  using Type = FloatArray_<ContainerAllocator>;

  explicit FloatArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FloatArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _steer_torques_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _steer_torques_type steer_torques;
  using _drive_torques_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _drive_torques_type drive_torques;

  // setters for named parameter idiom
  Type & set__steer_torques(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->steer_torques = _arg;
    return *this;
  }
  Type & set__drive_torques(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->drive_torques = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulator::msg::FloatArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulator::msg::FloatArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulator::msg::FloatArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulator::msg::FloatArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulator::msg::FloatArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulator::msg::FloatArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulator::msg::FloatArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulator::msg::FloatArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulator::msg::FloatArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulator::msg::FloatArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulator__msg__FloatArray
    std::shared_ptr<simulator::msg::FloatArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulator__msg__FloatArray
    std::shared_ptr<simulator::msg::FloatArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FloatArray_ & other) const
  {
    if (this->steer_torques != other.steer_torques) {
      return false;
    }
    if (this->drive_torques != other.drive_torques) {
      return false;
    }
    return true;
  }
  bool operator!=(const FloatArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FloatArray_

// alias to use template instance with default allocator
using FloatArray =
  simulator::msg::FloatArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulator

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__STRUCT_HPP_
