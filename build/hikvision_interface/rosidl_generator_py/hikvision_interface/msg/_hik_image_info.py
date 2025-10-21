# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hikvision_interface:msg/HikImageInfo.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HikImageInfo(type):
    """Metaclass of message 'HikImageInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hikvision_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hikvision_interface.msg.HikImageInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hik_image_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hik_image_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hik_image_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hik_image_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hik_image_info

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HikImageInfo(metaclass=Metaclass_HikImageInfo):
    """Message class 'HikImageInfo'."""

    __slots__ = [
        '_header',
        '_dev_stamp',
        '_frame_num',
        '_gain',
        '_exposure',
        '_red',
        '_green',
        '_blue',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'dev_stamp': 'builtin_interfaces/Time',
        'frame_num': 'uint32',
        'gain': 'float',
        'exposure': 'float',
        'red': 'uint32',
        'green': 'uint32',
        'blue': 'uint32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from builtin_interfaces.msg import Time
        self.dev_stamp = kwargs.get('dev_stamp', Time())
        self.frame_num = kwargs.get('frame_num', int())
        self.gain = kwargs.get('gain', float())
        self.exposure = kwargs.get('exposure', float())
        self.red = kwargs.get('red', int())
        self.green = kwargs.get('green', int())
        self.blue = kwargs.get('blue', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.dev_stamp != other.dev_stamp:
            return False
        if self.frame_num != other.frame_num:
            return False
        if self.gain != other.gain:
            return False
        if self.exposure != other.exposure:
            return False
        if self.red != other.red:
            return False
        if self.green != other.green:
            return False
        if self.blue != other.blue:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def dev_stamp(self):
        """Message field 'dev_stamp'."""
        return self._dev_stamp

    @dev_stamp.setter
    def dev_stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'dev_stamp' field must be a sub message of type 'Time'"
        self._dev_stamp = value

    @builtins.property
    def frame_num(self):
        """Message field 'frame_num'."""
        return self._frame_num

    @frame_num.setter
    def frame_num(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'frame_num' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'frame_num' field must be an unsigned integer in [0, 4294967295]"
        self._frame_num = value

    @builtins.property
    def gain(self):
        """Message field 'gain'."""
        return self._gain

    @gain.setter
    def gain(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'gain' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'gain' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._gain = value

    @builtins.property
    def exposure(self):
        """Message field 'exposure'."""
        return self._exposure

    @exposure.setter
    def exposure(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'exposure' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'exposure' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._exposure = value

    @builtins.property
    def red(self):
        """Message field 'red'."""
        return self._red

    @red.setter
    def red(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'red' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'red' field must be an unsigned integer in [0, 4294967295]"
        self._red = value

    @builtins.property
    def green(self):
        """Message field 'green'."""
        return self._green

    @green.setter
    def green(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'green' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'green' field must be an unsigned integer in [0, 4294967295]"
        self._green = value

    @builtins.property
    def blue(self):
        """Message field 'blue'."""
        return self._blue

    @blue.setter
    def blue(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'blue' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'blue' field must be an unsigned integer in [0, 4294967295]"
        self._blue = value
