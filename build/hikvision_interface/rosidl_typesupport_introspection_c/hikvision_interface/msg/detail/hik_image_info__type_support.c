// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hikvision_interface/msg/detail/hik_image_info__rosidl_typesupport_introspection_c.h"
#include "hikvision_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hikvision_interface/msg/detail/hik_image_info__functions.h"
#include "hikvision_interface/msg/detail/hik_image_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `dev_stamp`
#include "builtin_interfaces/msg/time.h"
// Member `dev_stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hikvision_interface__msg__HikImageInfo__init(message_memory);
}

void hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_fini_function(void * message_memory)
{
  hikvision_interface__msg__HikImageInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dev_stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, dev_stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, frame_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gain",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, gain),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "exposure",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, exposure),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "red",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, red),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "green",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, green),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "blue",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hikvision_interface__msg__HikImageInfo, blue),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_members = {
  "hikvision_interface__msg",  // message namespace
  "HikImageInfo",  // message name
  8,  // number of fields
  sizeof(hikvision_interface__msg__HikImageInfo),
  false,  // has_any_key_member_
  hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_member_array,  // message members
  hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_type_support_handle = {
  0,
  &hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_members,
  get_message_typesupport_handle_function,
  &hikvision_interface__msg__HikImageInfo__get_type_hash,
  &hikvision_interface__msg__HikImageInfo__get_type_description,
  &hikvision_interface__msg__HikImageInfo__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hikvision_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hikvision_interface, msg, HikImageInfo)() {
  hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_type_support_handle.typesupport_identifier) {
    hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hikvision_interface__msg__HikImageInfo__rosidl_typesupport_introspection_c__HikImageInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
