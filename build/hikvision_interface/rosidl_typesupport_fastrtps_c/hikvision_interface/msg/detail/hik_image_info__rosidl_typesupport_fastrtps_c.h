// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice
#ifndef HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hikvision_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hikvision_interface/msg/detail/hik_image_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
bool cdr_serialize_hikvision_interface__msg__HikImageInfo(
  const hikvision_interface__msg__HikImageInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
bool cdr_deserialize_hikvision_interface__msg__HikImageInfo(
  eprosima::fastcdr::Cdr &,
  hikvision_interface__msg__HikImageInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
size_t get_serialized_size_hikvision_interface__msg__HikImageInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
size_t max_serialized_size_hikvision_interface__msg__HikImageInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
bool cdr_serialize_key_hikvision_interface__msg__HikImageInfo(
  const hikvision_interface__msg__HikImageInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
size_t get_serialized_size_key_hikvision_interface__msg__HikImageInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
size_t max_serialized_size_key_hikvision_interface__msg__HikImageInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hikvision_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hikvision_interface, msg, HikImageInfo)();

#ifdef __cplusplus
}
#endif

#endif  // HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
