// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tuts:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef TUTS__MSG__DETAIL__NUM__BUILDER_HPP_
#define TUTS__MSG__DETAIL__NUM__BUILDER_HPP_

#include "tuts/msg/detail/num__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace tuts
{

namespace msg
{

namespace builder
{

class Init_Num_value
{
public:
  Init_Num_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tuts::msg::Num value(::tuts::msg::Num::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tuts::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tuts::msg::Num>()
{
  return tuts::msg::builder::Init_Num_value();
}

}  // namespace tuts

#endif  // TUTS__MSG__DETAIL__NUM__BUILDER_HPP_
