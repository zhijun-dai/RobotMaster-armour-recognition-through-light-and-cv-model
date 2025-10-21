// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hikvision_interface:msg/HikImageInfo.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "hikvision_interface/msg/detail/hik_image_info__struct.h"
#include "hikvision_interface/msg/detail/hik_image_info__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool hikvision_interface__msg__hik_image_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("hikvision_interface.msg._hik_image_info.HikImageInfo", full_classname_dest, 52) == 0);
  }
  hikvision_interface__msg__HikImageInfo * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // dev_stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "dev_stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->dev_stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // frame_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "frame_num");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frame_num = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // gain
    PyObject * field = PyObject_GetAttrString(_pymsg, "gain");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gain = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // exposure
    PyObject * field = PyObject_GetAttrString(_pymsg, "exposure");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->exposure = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // red
    PyObject * field = PyObject_GetAttrString(_pymsg, "red");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->red = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // green
    PyObject * field = PyObject_GetAttrString(_pymsg, "green");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->green = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // blue
    PyObject * field = PyObject_GetAttrString(_pymsg, "blue");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->blue = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hikvision_interface__msg__hik_image_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HikImageInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hikvision_interface.msg._hik_image_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HikImageInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hikvision_interface__msg__HikImageInfo * ros_message = (hikvision_interface__msg__HikImageInfo *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dev_stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->dev_stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "dev_stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frame_num
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->frame_num);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frame_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gain
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gain);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gain", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // exposure
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->exposure);
    {
      int rc = PyObject_SetAttrString(_pymessage, "exposure", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // red
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->red);
    {
      int rc = PyObject_SetAttrString(_pymessage, "red", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // green
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->green);
    {
      int rc = PyObject_SetAttrString(_pymessage, "green", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // blue
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->blue);
    {
      int rc = PyObject_SetAttrString(_pymessage, "blue", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
