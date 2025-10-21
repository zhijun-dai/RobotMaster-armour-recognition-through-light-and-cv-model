// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hikvision_interface/msg/hik_image_info.hpp"


#ifndef HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_HPP_
#define HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'dev_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hikvision_interface__msg__HikImageInfo __attribute__((deprecated))
#else
# define DEPRECATED__hikvision_interface__msg__HikImageInfo __declspec(deprecated)
#endif

namespace hikvision_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HikImageInfo_
{
  using Type = HikImageInfo_<ContainerAllocator>;

  explicit HikImageInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    dev_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_num = 0ul;
      this->gain = 0.0f;
      this->exposure = 0.0f;
      this->red = 0ul;
      this->green = 0ul;
      this->blue = 0ul;
    }
  }

  explicit HikImageInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    dev_stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_num = 0ul;
      this->gain = 0.0f;
      this->exposure = 0.0f;
      this->red = 0ul;
      this->green = 0ul;
      this->blue = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _dev_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _dev_stamp_type dev_stamp;
  using _frame_num_type =
    uint32_t;
  _frame_num_type frame_num;
  using _gain_type =
    float;
  _gain_type gain;
  using _exposure_type =
    float;
  _exposure_type exposure;
  using _red_type =
    uint32_t;
  _red_type red;
  using _green_type =
    uint32_t;
  _green_type green;
  using _blue_type =
    uint32_t;
  _blue_type blue;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__dev_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->dev_stamp = _arg;
    return *this;
  }
  Type & set__frame_num(
    const uint32_t & _arg)
  {
    this->frame_num = _arg;
    return *this;
  }
  Type & set__gain(
    const float & _arg)
  {
    this->gain = _arg;
    return *this;
  }
  Type & set__exposure(
    const float & _arg)
  {
    this->exposure = _arg;
    return *this;
  }
  Type & set__red(
    const uint32_t & _arg)
  {
    this->red = _arg;
    return *this;
  }
  Type & set__green(
    const uint32_t & _arg)
  {
    this->green = _arg;
    return *this;
  }
  Type & set__blue(
    const uint32_t & _arg)
  {
    this->blue = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hikvision_interface::msg::HikImageInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const hikvision_interface::msg::HikImageInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hikvision_interface::msg::HikImageInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hikvision_interface::msg::HikImageInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hikvision_interface__msg__HikImageInfo
    std::shared_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hikvision_interface__msg__HikImageInfo
    std::shared_ptr<hikvision_interface::msg::HikImageInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HikImageInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->dev_stamp != other.dev_stamp) {
      return false;
    }
    if (this->frame_num != other.frame_num) {
      return false;
    }
    if (this->gain != other.gain) {
      return false;
    }
    if (this->exposure != other.exposure) {
      return false;
    }
    if (this->red != other.red) {
      return false;
    }
    if (this->green != other.green) {
      return false;
    }
    if (this->blue != other.blue) {
      return false;
    }
    return true;
  }
  bool operator!=(const HikImageInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HikImageInfo_

// alias to use template instance with default allocator
using HikImageInfo =
  hikvision_interface::msg::HikImageInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hikvision_interface

#endif  // HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_HPP_
