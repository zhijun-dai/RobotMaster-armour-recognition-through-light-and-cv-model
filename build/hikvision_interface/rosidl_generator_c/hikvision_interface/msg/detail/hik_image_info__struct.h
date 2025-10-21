// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hikvision_interface/msg/hik_image_info.h"


#ifndef HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_H_
#define HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'dev_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/HikImageInfo in the package hikvision_interface.
/**
  * header
 */
typedef struct hikvision_interface__msg__HikImageInfo
{
  std_msgs__msg__Header header;
  /// sensor stamp
  builtin_interfaces__msg__Time dev_stamp;
  uint32_t frame_num;
  /// acqusistion
  float gain;
  float exposure;
  /// white balance
  uint32_t red;
  uint32_t green;
  uint32_t blue;
} hikvision_interface__msg__HikImageInfo;

// Struct for a sequence of hikvision_interface__msg__HikImageInfo.
typedef struct hikvision_interface__msg__HikImageInfo__Sequence
{
  hikvision_interface__msg__HikImageInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hikvision_interface__msg__HikImageInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HIKVISION_INTERFACE__MSG__DETAIL__HIK_IMAGE_INFO__STRUCT_H_
