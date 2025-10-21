// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "hikvision_interface/msg/detail/hik_image_info__struct.h"
#include "hikvision_interface/msg/detail/hik_image_info__type_support.h"
#include "hikvision_interface/msg/detail/hik_image_info__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace hikvision_interface
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _HikImageInfo_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _HikImageInfo_type_support_ids_t;

static const _HikImageInfo_type_support_ids_t _HikImageInfo_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _HikImageInfo_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _HikImageInfo_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _HikImageInfo_type_support_symbol_names_t _HikImageInfo_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hikvision_interface, msg, HikImageInfo)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hikvision_interface, msg, HikImageInfo)),
  }
};

typedef struct _HikImageInfo_type_support_data_t
{
  void * data[2];
} _HikImageInfo_type_support_data_t;

static _HikImageInfo_type_support_data_t _HikImageInfo_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _HikImageInfo_message_typesupport_map = {
  2,
  "hikvision_interface",
  &_HikImageInfo_message_typesupport_ids.typesupport_identifier[0],
  &_HikImageInfo_message_typesupport_symbol_names.symbol_name[0],
  &_HikImageInfo_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t HikImageInfo_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_HikImageInfo_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &hikvision_interface__msg__HikImageInfo__get_type_hash,
  &hikvision_interface__msg__HikImageInfo__get_type_description,
  &hikvision_interface__msg__HikImageInfo__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace hikvision_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, hikvision_interface, msg, HikImageInfo)() {
  return &::hikvision_interface::msg::rosidl_typesupport_c::HikImageInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
