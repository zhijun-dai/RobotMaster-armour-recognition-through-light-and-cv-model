// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hikvision_interface/msg/hik_image_info.hpp"


#ifndef HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__BUILDER_HPP_
#define HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hikvision_interface/msg/detail/hik_image_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hikvision_interface
{

namespace msg
{

namespace builder
{

class Init_HikImageInfo_blue
{
public:
  explicit Init_HikImageInfo_blue(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  ::hikvision_interface::msg::HikImageInfo blue(::hikvision_interface::msg::HikImageInfo::_blue_type arg)
  {
    msg_.blue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_green
{
public:
  explicit Init_HikImageInfo_green(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_blue green(::hikvision_interface::msg::HikImageInfo::_green_type arg)
  {
    msg_.green = std::move(arg);
    return Init_HikImageInfo_blue(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_red
{
public:
  explicit Init_HikImageInfo_red(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_green red(::hikvision_interface::msg::HikImageInfo::_red_type arg)
  {
    msg_.red = std::move(arg);
    return Init_HikImageInfo_green(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_exposure
{
public:
  explicit Init_HikImageInfo_exposure(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_red exposure(::hikvision_interface::msg::HikImageInfo::_exposure_type arg)
  {
    msg_.exposure = std::move(arg);
    return Init_HikImageInfo_red(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_gain
{
public:
  explicit Init_HikImageInfo_gain(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_exposure gain(::hikvision_interface::msg::HikImageInfo::_gain_type arg)
  {
    msg_.gain = std::move(arg);
    return Init_HikImageInfo_exposure(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_frame_num
{
public:
  explicit Init_HikImageInfo_frame_num(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_gain frame_num(::hikvision_interface::msg::HikImageInfo::_frame_num_type arg)
  {
    msg_.frame_num = std::move(arg);
    return Init_HikImageInfo_gain(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_dev_stamp
{
public:
  explicit Init_HikImageInfo_dev_stamp(::hikvision_interface::msg::HikImageInfo & msg)
  : msg_(msg)
  {}
  Init_HikImageInfo_frame_num dev_stamp(::hikvision_interface::msg::HikImageInfo::_dev_stamp_type arg)
  {
    msg_.dev_stamp = std::move(arg);
    return Init_HikImageInfo_frame_num(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

class Init_HikImageInfo_header
{
public:
  Init_HikImageInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HikImageInfo_dev_stamp header(::hikvision_interface::msg::HikImageInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HikImageInfo_dev_stamp(msg_);
  }

private:
  ::hikvision_interface::msg::HikImageInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hikvision_interface::msg::HikImageInfo>()
{
  return hikvision_interface::msg::builder::Init_HikImageInfo_header();
}

}  // namespace hikvision_interface

#endif  // HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__BUILDER_HPP_
