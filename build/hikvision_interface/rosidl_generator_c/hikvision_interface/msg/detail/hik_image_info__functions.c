// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice
#include "hikvision_interface/msg/detail/hik_image_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `dev_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
hikvision_interface__msg__HikImageInfo__init(hikvision_interface__msg__HikImageInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hikvision_interface__msg__HikImageInfo__fini(msg);
    return false;
  }
  // dev_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->dev_stamp)) {
    hikvision_interface__msg__HikImageInfo__fini(msg);
    return false;
  }
  // frame_num
  // gain
  // exposure
  // red
  // green
  // blue
  return true;
}

void
hikvision_interface__msg__HikImageInfo__fini(hikvision_interface__msg__HikImageInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // dev_stamp
  builtin_interfaces__msg__Time__fini(&msg->dev_stamp);
  // frame_num
  // gain
  // exposure
  // red
  // green
  // blue
}

bool
hikvision_interface__msg__HikImageInfo__are_equal(const hikvision_interface__msg__HikImageInfo * lhs, const hikvision_interface__msg__HikImageInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // dev_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->dev_stamp), &(rhs->dev_stamp)))
  {
    return false;
  }
  // frame_num
  if (lhs->frame_num != rhs->frame_num) {
    return false;
  }
  // gain
  if (lhs->gain != rhs->gain) {
    return false;
  }
  // exposure
  if (lhs->exposure != rhs->exposure) {
    return false;
  }
  // red
  if (lhs->red != rhs->red) {
    return false;
  }
  // green
  if (lhs->green != rhs->green) {
    return false;
  }
  // blue
  if (lhs->blue != rhs->blue) {
    return false;
  }
  return true;
}

bool
hikvision_interface__msg__HikImageInfo__copy(
  const hikvision_interface__msg__HikImageInfo * input,
  hikvision_interface__msg__HikImageInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // dev_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->dev_stamp), &(output->dev_stamp)))
  {
    return false;
  }
  // frame_num
  output->frame_num = input->frame_num;
  // gain
  output->gain = input->gain;
  // exposure
  output->exposure = input->exposure;
  // red
  output->red = input->red;
  // green
  output->green = input->green;
  // blue
  output->blue = input->blue;
  return true;
}

hikvision_interface__msg__HikImageInfo *
hikvision_interface__msg__HikImageInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hikvision_interface__msg__HikImageInfo * msg = (hikvision_interface__msg__HikImageInfo *)allocator.allocate(sizeof(hikvision_interface__msg__HikImageInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hikvision_interface__msg__HikImageInfo));
  bool success = hikvision_interface__msg__HikImageInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hikvision_interface__msg__HikImageInfo__destroy(hikvision_interface__msg__HikImageInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hikvision_interface__msg__HikImageInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hikvision_interface__msg__HikImageInfo__Sequence__init(hikvision_interface__msg__HikImageInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hikvision_interface__msg__HikImageInfo * data = NULL;

  if (size) {
    data = (hikvision_interface__msg__HikImageInfo *)allocator.zero_allocate(size, sizeof(hikvision_interface__msg__HikImageInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hikvision_interface__msg__HikImageInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hikvision_interface__msg__HikImageInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hikvision_interface__msg__HikImageInfo__Sequence__fini(hikvision_interface__msg__HikImageInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hikvision_interface__msg__HikImageInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hikvision_interface__msg__HikImageInfo__Sequence *
hikvision_interface__msg__HikImageInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hikvision_interface__msg__HikImageInfo__Sequence * array = (hikvision_interface__msg__HikImageInfo__Sequence *)allocator.allocate(sizeof(hikvision_interface__msg__HikImageInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hikvision_interface__msg__HikImageInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hikvision_interface__msg__HikImageInfo__Sequence__destroy(hikvision_interface__msg__HikImageInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hikvision_interface__msg__HikImageInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hikvision_interface__msg__HikImageInfo__Sequence__are_equal(const hikvision_interface__msg__HikImageInfo__Sequence * lhs, const hikvision_interface__msg__HikImageInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hikvision_interface__msg__HikImageInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hikvision_interface__msg__HikImageInfo__Sequence__copy(
  const hikvision_interface__msg__HikImageInfo__Sequence * input,
  hikvision_interface__msg__HikImageInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hikvision_interface__msg__HikImageInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hikvision_interface__msg__HikImageInfo * data =
      (hikvision_interface__msg__HikImageInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hikvision_interface__msg__HikImageInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hikvision_interface__msg__HikImageInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hikvision_interface__msg__HikImageInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
