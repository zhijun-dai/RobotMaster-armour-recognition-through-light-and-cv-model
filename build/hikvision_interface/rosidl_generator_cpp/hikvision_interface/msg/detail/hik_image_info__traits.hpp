// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hikvision_interface/msg/hik_image_info.hpp"


#ifndef HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__TRAITS_HPP_
#define HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hikvision_interface/msg/detail/hik_image_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'dev_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace hikvision_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const HikImageInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: dev_stamp
  {
    out << "dev_stamp: ";
    to_flow_style_yaml(msg.dev_stamp, out);
    out << ", ";
  }

  // member: frame_num
  {
    out << "frame_num: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_num, out);
    out << ", ";
  }

  // member: gain
  {
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
    out << ", ";
  }

  // member: exposure
  {
    out << "exposure: ";
    rosidl_generator_traits::value_to_yaml(msg.exposure, out);
    out << ", ";
  }

  // member: red
  {
    out << "red: ";
    rosidl_generator_traits::value_to_yaml(msg.red, out);
    out << ", ";
  }

  // member: green
  {
    out << "green: ";
    rosidl_generator_traits::value_to_yaml(msg.green, out);
    out << ", ";
  }

  // member: blue
  {
    out << "blue: ";
    rosidl_generator_traits::value_to_yaml(msg.blue, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HikImageInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: dev_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dev_stamp:\n";
    to_block_style_yaml(msg.dev_stamp, out, indentation + 2);
  }

  // member: frame_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_num: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_num, out);
    out << "\n";
  }

  // member: gain
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
    out << "\n";
  }

  // member: exposure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exposure: ";
    rosidl_generator_traits::value_to_yaml(msg.exposure, out);
    out << "\n";
  }

  // member: red
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "red: ";
    rosidl_generator_traits::value_to_yaml(msg.red, out);
    out << "\n";
  }

  // member: green
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "green: ";
    rosidl_generator_traits::value_to_yaml(msg.green, out);
    out << "\n";
  }

  // member: blue
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blue: ";
    rosidl_generator_traits::value_to_yaml(msg.blue, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HikImageInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hikvision_interface

namespace rosidl_generator_traits
{

[[deprecated("use hikvision_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hikvision_interface::msg::HikImageInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  hikvision_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hikvision_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const hikvision_interface::msg::HikImageInfo & msg)
{
  return hikvision_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hikvision_interface::msg::HikImageInfo>()
{
  return "hikvision_interface::msg::HikImageInfo";
}

template<>
inline const char * name<hikvision_interface::msg::HikImageInfo>()
{
  return "hikvision_interface/msg/HikImageInfo";
}

template<>
struct has_fixed_size<hikvision_interface::msg::HikImageInfo>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hikvision_interface::msg::HikImageInfo>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hikvision_interface::msg::HikImageInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__TRAITS_HPP_
