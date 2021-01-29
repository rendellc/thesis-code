// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tuts:srv/AddInts.idl
// generated code does not contain a copyright notice

#ifndef TUTS__SRV__DETAIL__ADD_INTS__BUILDER_HPP_
#define TUTS__SRV__DETAIL__ADD_INTS__BUILDER_HPP_

#include "tuts/srv/detail/add_ints__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace tuts
{

namespace srv
{

namespace builder
{

class Init_AddInts_Request_c
{
public:
  explicit Init_AddInts_Request_c(::tuts::srv::AddInts_Request & msg)
  : msg_(msg)
  {}
  ::tuts::srv::AddInts_Request c(::tuts::srv::AddInts_Request::_c_type arg)
  {
    msg_.c = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tuts::srv::AddInts_Request msg_;
};

class Init_AddInts_Request_b
{
public:
  explicit Init_AddInts_Request_b(::tuts::srv::AddInts_Request & msg)
  : msg_(msg)
  {}
  Init_AddInts_Request_c b(::tuts::srv::AddInts_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_AddInts_Request_c(msg_);
  }

private:
  ::tuts::srv::AddInts_Request msg_;
};

class Init_AddInts_Request_a
{
public:
  Init_AddInts_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddInts_Request_b a(::tuts::srv::AddInts_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_AddInts_Request_b(msg_);
  }

private:
  ::tuts::srv::AddInts_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tuts::srv::AddInts_Request>()
{
  return tuts::srv::builder::Init_AddInts_Request_a();
}

}  // namespace tuts


namespace tuts
{

namespace srv
{

namespace builder
{

class Init_AddInts_Response_sum
{
public:
  Init_AddInts_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tuts::srv::AddInts_Response sum(::tuts::srv::AddInts_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tuts::srv::AddInts_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tuts::srv::AddInts_Response>()
{
  return tuts::srv::builder::Init_AddInts_Response_sum();
}

}  // namespace tuts

#endif  // TUTS__SRV__DETAIL__ADD_INTS__BUILDER_HPP_
