// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice

#include "hikvision_interface/msg/detail/hik_image_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hikvision_interface
const rosidl_type_hash_t *
hikvision_interface__msg__HikImageInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x1f, 0x1e, 0x4d, 0xd5, 0x2f, 0xff, 0x64,
      0x04, 0x03, 0x7c, 0xb6, 0xf3, 0xfd, 0x2d, 0x6b,
      0x37, 0xf1, 0x87, 0x34, 0x6b, 0x8b, 0xf6, 0xd2,
      0x76, 0xec, 0xe7, 0xdf, 0x3f, 0x23, 0xa3, 0xe0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char hikvision_interface__msg__HikImageInfo__TYPE_NAME[] = "hikvision_interface/msg/HikImageInfo";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__header[] = "header";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__dev_stamp[] = "dev_stamp";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__frame_num[] = "frame_num";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__gain[] = "gain";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__exposure[] = "exposure";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__red[] = "red";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__green[] = "green";
static char hikvision_interface__msg__HikImageInfo__FIELD_NAME__blue[] = "blue";

static rosidl_runtime_c__type_description__Field hikvision_interface__msg__HikImageInfo__FIELDS[] = {
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__dev_stamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__frame_num, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__gain, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__exposure, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__red, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__green, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hikvision_interface__msg__HikImageInfo__FIELD_NAME__blue, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription hikvision_interface__msg__HikImageInfo__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hikvision_interface__msg__HikImageInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hikvision_interface__msg__HikImageInfo__TYPE_NAME, 36, 36},
      {hikvision_interface__msg__HikImageInfo__FIELDS, 8, 8},
    },
    {hikvision_interface__msg__HikImageInfo__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# header\n"
  "std_msgs/Header header\n"
  "# sensor stamp\n"
  "builtin_interfaces/Time dev_stamp\n"
  "uint32 frame_num\n"
  "# acqusistion\n"
  "float32 gain\n"
  "float32 exposure\n"
  "# white balance\n"
  "uint32 red\n"
  "uint32 green\n"
  "uint32 blue";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hikvision_interface__msg__HikImageInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hikvision_interface__msg__HikImageInfo__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 193, 193},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hikvision_interface__msg__HikImageInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hikvision_interface__msg__HikImageInfo__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
