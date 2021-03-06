"""autogenerated by genpy from arm_navigation_msgs/PositionConstraint.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import arm_navigation_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class PositionConstraint(genpy.Message):
  _md5sum = "7e3d9697e64b346b9d3cb7311bb88ccb"
  _type = "arm_navigation_msgs/PositionConstraint"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# This message contains the definition of a position constraint.
Header header

# The robot link this constraint refers to
string link_name

# The offset (in the link frame) for the target point on the link we are planning for
geometry_msgs/Point target_point_offset

# The nominal/target position for the point we are planning for
geometry_msgs/Point position

# The shape of the bounded region that constrains the position of the end-effector
# This region is always centered at the position defined above
arm_navigation_msgs/Shape constraint_region_shape

# The orientation of the bounded region that constrains the position of the end-effector. 
# This allows the specification of non-axis aligned constraints
geometry_msgs/Quaternion constraint_region_orientation

# Constraint weighting factor - a weight for this constraint
float64 weight

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: arm_navigation_msgs/Shape
byte SPHERE=0
byte BOX=1
byte CYLINDER=2
byte MESH=3

byte type


#### define sphere, box, cylinder ####
# the origin of each shape is considered at the shape's center

# for sphere
# radius := dimensions[0]

# for cylinder
# radius := dimensions[0]
# length := dimensions[1]
# the length is along the Z axis

# for box
# size_x := dimensions[0]
# size_y := dimensions[1]
# size_z := dimensions[2]
float64[] dimensions


#### define mesh ####

# list of triangles; triangle k is defined by tre vertices located
# at indices triangles[3k], triangles[3k+1], triangles[3k+2]
int32[] triangles
geometry_msgs/Point[] vertices

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['header','link_name','target_point_offset','position','constraint_region_shape','constraint_region_orientation','weight']
  _slot_types = ['std_msgs/Header','string','geometry_msgs/Point','geometry_msgs/Point','arm_navigation_msgs/Shape','geometry_msgs/Quaternion','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,link_name,target_point_offset,position,constraint_region_shape,constraint_region_orientation,weight

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PositionConstraint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.link_name is None:
        self.link_name = ''
      if self.target_point_offset is None:
        self.target_point_offset = geometry_msgs.msg.Point()
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.constraint_region_shape is None:
        self.constraint_region_shape = arm_navigation_msgs.msg.Shape()
      if self.constraint_region_orientation is None:
        self.constraint_region_orientation = geometry_msgs.msg.Quaternion()
      if self.weight is None:
        self.weight = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.link_name = ''
      self.target_point_offset = geometry_msgs.msg.Point()
      self.position = geometry_msgs.msg.Point()
      self.constraint_region_shape = arm_navigation_msgs.msg.Shape()
      self.constraint_region_orientation = geometry_msgs.msg.Quaternion()
      self.weight = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.link_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6db.pack(_x.target_point_offset.x, _x.target_point_offset.y, _x.target_point_offset.z, _x.position.x, _x.position.y, _x.position.z, _x.constraint_region_shape.type))
      length = len(self.constraint_region_shape.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.constraint_region_shape.dimensions))
      length = len(self.constraint_region_shape.triangles)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.constraint_region_shape.triangles))
      length = len(self.constraint_region_shape.vertices)
      buff.write(_struct_I.pack(length))
      for val1 in self.constraint_region_shape.vertices:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_5d.pack(_x.constraint_region_orientation.x, _x.constraint_region_orientation.y, _x.constraint_region_orientation.z, _x.constraint_region_orientation.w, _x.weight))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.target_point_offset is None:
        self.target_point_offset = geometry_msgs.msg.Point()
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.constraint_region_shape is None:
        self.constraint_region_shape = arm_navigation_msgs.msg.Shape()
      if self.constraint_region_orientation is None:
        self.constraint_region_orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.link_name = str[start:end].decode('utf-8')
      else:
        self.link_name = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.target_point_offset.x, _x.target_point_offset.y, _x.target_point_offset.z, _x.position.x, _x.position.y, _x.position.z, _x.constraint_region_shape.type,) = _struct_6db.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.constraint_region_shape.dimensions = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.constraint_region_shape.triangles = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.constraint_region_shape.vertices = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.constraint_region_shape.vertices.append(val1)
      _x = self
      start = end
      end += 40
      (_x.constraint_region_orientation.x, _x.constraint_region_orientation.y, _x.constraint_region_orientation.z, _x.constraint_region_orientation.w, _x.weight,) = _struct_5d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.link_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6db.pack(_x.target_point_offset.x, _x.target_point_offset.y, _x.target_point_offset.z, _x.position.x, _x.position.y, _x.position.z, _x.constraint_region_shape.type))
      length = len(self.constraint_region_shape.dimensions)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.constraint_region_shape.dimensions.tostring())
      length = len(self.constraint_region_shape.triangles)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.constraint_region_shape.triangles.tostring())
      length = len(self.constraint_region_shape.vertices)
      buff.write(_struct_I.pack(length))
      for val1 in self.constraint_region_shape.vertices:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_5d.pack(_x.constraint_region_orientation.x, _x.constraint_region_orientation.y, _x.constraint_region_orientation.z, _x.constraint_region_orientation.w, _x.weight))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.target_point_offset is None:
        self.target_point_offset = geometry_msgs.msg.Point()
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.constraint_region_shape is None:
        self.constraint_region_shape = arm_navigation_msgs.msg.Shape()
      if self.constraint_region_orientation is None:
        self.constraint_region_orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.link_name = str[start:end].decode('utf-8')
      else:
        self.link_name = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.target_point_offset.x, _x.target_point_offset.y, _x.target_point_offset.z, _x.position.x, _x.position.y, _x.position.z, _x.constraint_region_shape.type,) = _struct_6db.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.constraint_region_shape.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.constraint_region_shape.triangles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.constraint_region_shape.vertices = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.constraint_region_shape.vertices.append(val1)
      _x = self
      start = end
      end += 40
      (_x.constraint_region_orientation.x, _x.constraint_region_orientation.y, _x.constraint_region_orientation.z, _x.constraint_region_orientation.w, _x.weight,) = _struct_5d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5d = struct.Struct("<5d")
_struct_3I = struct.Struct("<3I")
_struct_6db = struct.Struct("<6db")
_struct_3d = struct.Struct("<3d")
