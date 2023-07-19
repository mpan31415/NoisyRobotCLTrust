// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Falconpos.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__FALCONPOS__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__FALCONPOS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/falconpos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Falconpos_z
{
public:
  explicit Init_Falconpos_z(::tutorial_interfaces::msg::Falconpos & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Falconpos z(::tutorial_interfaces::msg::Falconpos::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Falconpos msg_;
};

class Init_Falconpos_y
{
public:
  explicit Init_Falconpos_y(::tutorial_interfaces::msg::Falconpos & msg)
  : msg_(msg)
  {}
  Init_Falconpos_z y(::tutorial_interfaces::msg::Falconpos::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Falconpos_z(msg_);
  }

private:
  ::tutorial_interfaces::msg::Falconpos msg_;
};

class Init_Falconpos_x
{
public:
  Init_Falconpos_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Falconpos_y x(::tutorial_interfaces::msg::Falconpos::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Falconpos_y(msg_);
  }

private:
  ::tutorial_interfaces::msg::Falconpos msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Falconpos>()
{
  return tutorial_interfaces::msg::builder::Init_Falconpos_x();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__FALCONPOS__BUILDER_HPP_
