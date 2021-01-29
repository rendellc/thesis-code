// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulator:msg/FloatArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__BUILDER_HPP_
#define SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__BUILDER_HPP_

#include "simulator/msg/detail/float_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace simulator
{

namespace msg
{

namespace builder
{

class Init_FloatArray_drive_torques
{
public:
  explicit Init_FloatArray_drive_torques(::simulator::msg::FloatArray & msg)
  : msg_(msg)
  {}
  ::simulator::msg::FloatArray drive_torques(::simulator::msg::FloatArray::_drive_torques_type arg)
  {
    msg_.drive_torques = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulator::msg::FloatArray msg_;
};

class Init_FloatArray_steer_torques
{
public:
  Init_FloatArray_steer_torques()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FloatArray_drive_torques steer_torques(::simulator::msg::FloatArray::_steer_torques_type arg)
  {
    msg_.steer_torques = std::move(arg);
    return Init_FloatArray_drive_torques(msg_);
  }

private:
  ::simulator::msg::FloatArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulator::msg::FloatArray>()
{
  return simulator::msg::builder::Init_FloatArray_steer_torques();
}

}  // namespace simulator

#endif  // SIMULATOR__MSG__DETAIL__FLOAT_ARRAY__BUILDER_HPP_
