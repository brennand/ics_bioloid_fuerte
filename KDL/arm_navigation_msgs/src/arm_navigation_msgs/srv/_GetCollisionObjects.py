"""autogenerated by genpy from arm_navigation_msgs/GetCollisionObjectsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetCollisionObjectsRequest(genpy.Message):
  _md5sum = "3ae7288b23c84452d229e442c1673708"
  _type = "arm_navigation_msgs/GetCollisionObjectsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """


bool include_points

"""
  __slots__ = ['include_points']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       include_points

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetCollisionObjectsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.include_points is None:
        self.include_points = False
    else:
      self.include_points = False

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
      buff.write(_struct_B.pack(self.include_points))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.include_points,) = _struct_B.unpack(str[start:end])
      self.include_points = bool(self.include_points)
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
      buff.write(_struct_B.pack(self.include_points))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.include_points,) = _struct_B.unpack(str[start:end])
      self.include_points = bool(self.include_points)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
"""autogenerated by genpy from arm_navigation_msgs/GetCollisionObjectsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import arm_navigation_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class GetCollisionObjectsResponse(genpy.Message):
  _md5sum = "c361b849f4eb74ea667a930b0b9dc801"
  _type = "arm_navigation_msgs/GetCollisionObjectsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
arm_navigation_msgs/CollisionMap points

arm_navigation_msgs/CollisionObject[] collision_objects

arm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects


================================================================================
MSG: arm_navigation_msgs/CollisionMap
#header for interpreting box positions
Header header

#boxes for use in collision testing
OrientedBoundingBox[] boxes

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
MSG: arm_navigation_msgs/OrientedBoundingBox
#the center of the box
geometry_msgs/Point32 center

#the extents of the box, assuming the center is at the point
geometry_msgs/Point32 extents

#the axis of the box
geometry_msgs/Point32 axis

#the angle of rotation around the axis
float32 angle

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: arm_navigation_msgs/CollisionObject
# a header, used for interpreting the poses
Header header

# the id of the object
string id

# The padding used for filtering points near the object.
# This does not affect collision checking for the object.  
# Set to negative to get zero padding.
float32 padding

#This contains what is to be done with the object
CollisionObjectOperation operation

#the shapes associated with the object
arm_navigation_msgs/Shape[] shapes

#the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses

================================================================================
MSG: arm_navigation_msgs/CollisionObjectOperation
#Puts the object into the environment
#or updates the object if already added
byte ADD=0

#Removes the object from the environment entirely
byte REMOVE=1

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes an attached object, detaches from the attached link
#But adds back in as regular object
byte DETACH_AND_ADD_AS_OBJECT=2

#Only valid within the context of a CollisionAttachedObject message
#Will be ignored if sent with an CollisionObject message
#Takes current object in the environment and removes it as
#a regular object
byte ATTACH_AND_REMOVE_AS_OBJECT=3

# Byte code for operation
byte operation

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: arm_navigation_msgs/AttachedCollisionObject
# The CollisionObject will be attached with a fixed joint to this link
# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation 
# is set to REMOVE will remove all attached bodies attached to any object
string link_name

#Reserved for indicating that all attached objects should be removed
string REMOVE_ALL_ATTACHED_OBJECTS = "all"

#This contains the actual shapes and poses for the CollisionObject
#to be attached to the link
#If action is remove and no object.id is set, all objects
#attached to the link indicated by link_name will be removed
CollisionObject object

# The set of links that the attached objects are allowed to touch
# by default - the link_name is included by default
string[] touch_links

"""
  __slots__ = ['points','collision_objects','attached_collision_objects']
  _slot_types = ['arm_navigation_msgs/CollisionMap','arm_navigation_msgs/CollisionObject[]','arm_navigation_msgs/AttachedCollisionObject[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       points,collision_objects,attached_collision_objects

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetCollisionObjectsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.points is None:
        self.points = arm_navigation_msgs.msg.CollisionMap()
      if self.collision_objects is None:
        self.collision_objects = []
      if self.attached_collision_objects is None:
        self.attached_collision_objects = []
    else:
      self.points = arm_navigation_msgs.msg.CollisionMap()
      self.collision_objects = []
      self.attached_collision_objects = []

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
      buff.write(_struct_3I.pack(_x.points.header.seq, _x.points.header.stamp.secs, _x.points.header.stamp.nsecs))
      _x = self.points.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.points.boxes)
      buff.write(_struct_I.pack(length))
      for val1 in self.points.boxes:
        _v1 = val1.center
        _x = _v1
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        _v2 = val1.extents
        _x = _v2
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        _v3 = val1.axis
        _x = _v3
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_f.pack(val1.angle))
      length = len(self.collision_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.collision_objects:
        _v4 = val1.header
        buff.write(_struct_I.pack(_v4.seq))
        _v5 = _v4.stamp
        _x = _v5
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v4.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.padding))
        _v6 = val1.operation
        buff.write(_struct_b.pack(_v6.operation))
        length = len(val1.shapes)
        buff.write(_struct_I.pack(length))
        for val2 in val1.shapes:
          buff.write(_struct_b.pack(val2.type))
          length = len(val2.dimensions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.pack(pattern, *val2.dimensions))
          length = len(val2.triangles)
          buff.write(_struct_I.pack(length))
          pattern = '<%si'%length
          buff.write(struct.pack(pattern, *val2.triangles))
          length = len(val2.vertices)
          buff.write(_struct_I.pack(length))
          for val3 in val2.vertices:
            _x = val3
            buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.poses:
          _v7 = val2.position
          _x = _v7
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v8 = val2.orientation
          _x = _v8
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.attached_collision_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.attached_collision_objects:
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v9 = val1.object
        _v10 = _v9.header
        buff.write(_struct_I.pack(_v10.seq))
        _v11 = _v10.stamp
        _x = _v11
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v10.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v9.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(_v9.padding))
        _v12 = _v9.operation
        buff.write(_struct_b.pack(_v12.operation))
        length = len(_v9.shapes)
        buff.write(_struct_I.pack(length))
        for val3 in _v9.shapes:
          buff.write(_struct_b.pack(val3.type))
          length = len(val3.dimensions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.pack(pattern, *val3.dimensions))
          length = len(val3.triangles)
          buff.write(_struct_I.pack(length))
          pattern = '<%si'%length
          buff.write(struct.pack(pattern, *val3.triangles))
          length = len(val3.vertices)
          buff.write(_struct_I.pack(length))
          for val4 in val3.vertices:
            _x = val4
            buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(_v9.poses)
        buff.write(_struct_I.pack(length))
        for val3 in _v9.poses:
          _v13 = val3.position
          _x = _v13
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v14 = val3.orientation
          _x = _v14
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.touch_links)
        buff.write(_struct_I.pack(length))
        for val2 in val1.touch_links:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.points is None:
        self.points = arm_navigation_msgs.msg.CollisionMap()
      if self.collision_objects is None:
        self.collision_objects = None
      if self.attached_collision_objects is None:
        self.attached_collision_objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.points.header.seq, _x.points.header.stamp.secs, _x.points.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.points.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.points.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points.boxes = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.OrientedBoundingBox()
        _v15 = val1.center
        _x = _v15
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        _v16 = val1.extents
        _x = _v16
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        _v17 = val1.axis
        _x = _v17
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        start = end
        end += 4
        (val1.angle,) = _struct_f.unpack(str[start:end])
        self.points.boxes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.collision_objects = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.CollisionObject()
        _v18 = val1.header
        start = end
        end += 4
        (_v18.seq,) = _struct_I.unpack(str[start:end])
        _v19 = _v18.stamp
        _x = _v19
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v18.frame_id = str[start:end].decode('utf-8')
        else:
          _v18.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.id = str[start:end].decode('utf-8')
        else:
          val1.id = str[start:end]
        start = end
        end += 4
        (val1.padding,) = _struct_f.unpack(str[start:end])
        _v20 = val1.operation
        start = end
        end += 1
        (_v20.operation,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.shapes = []
        for i in range(0, length):
          val2 = arm_navigation_msgs.msg.Shape()
          start = end
          end += 1
          (val2.type,) = _struct_b.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.dimensions = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%si'%length
          start = end
          end += struct.calcsize(pattern)
          val2.triangles = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val2.vertices = []
          for i in range(0, length):
            val3 = geometry_msgs.msg.Point()
            _x = val3
            start = end
            end += 24
            (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
            val2.vertices.append(val3)
          val1.shapes.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.poses = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Pose()
          _v21 = val2.position
          _x = _v21
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v22 = val2.orientation
          _x = _v22
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.poses.append(val2)
        self.collision_objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.attached_collision_objects = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.AttachedCollisionObject()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        _v23 = val1.object
        _v24 = _v23.header
        start = end
        end += 4
        (_v24.seq,) = _struct_I.unpack(str[start:end])
        _v25 = _v24.stamp
        _x = _v25
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v24.frame_id = str[start:end].decode('utf-8')
        else:
          _v24.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v23.id = str[start:end].decode('utf-8')
        else:
          _v23.id = str[start:end]
        start = end
        end += 4
        (_v23.padding,) = _struct_f.unpack(str[start:end])
        _v26 = _v23.operation
        start = end
        end += 1
        (_v26.operation,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v23.shapes = []
        for i in range(0, length):
          val3 = arm_navigation_msgs.msg.Shape()
          start = end
          end += 1
          (val3.type,) = _struct_b.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val3.dimensions = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%si'%length
          start = end
          end += struct.calcsize(pattern)
          val3.triangles = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val3.vertices = []
          for i in range(0, length):
            val4 = geometry_msgs.msg.Point()
            _x = val4
            start = end
            end += 24
            (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
            val3.vertices.append(val4)
          _v23.shapes.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v23.poses = []
        for i in range(0, length):
          val3 = geometry_msgs.msg.Pose()
          _v27 = val3.position
          _x = _v27
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v28 = val3.orientation
          _x = _v28
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          _v23.poses.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.touch_links = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8')
          else:
            val2 = str[start:end]
          val1.touch_links.append(val2)
        self.attached_collision_objects.append(val1)
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
      buff.write(_struct_3I.pack(_x.points.header.seq, _x.points.header.stamp.secs, _x.points.header.stamp.nsecs))
      _x = self.points.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.points.boxes)
      buff.write(_struct_I.pack(length))
      for val1 in self.points.boxes:
        _v29 = val1.center
        _x = _v29
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        _v30 = val1.extents
        _x = _v30
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        _v31 = val1.axis
        _x = _v31
        buff.write(_struct_3f.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_f.pack(val1.angle))
      length = len(self.collision_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.collision_objects:
        _v32 = val1.header
        buff.write(_struct_I.pack(_v32.seq))
        _v33 = _v32.stamp
        _x = _v33
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v32.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(val1.padding))
        _v34 = val1.operation
        buff.write(_struct_b.pack(_v34.operation))
        length = len(val1.shapes)
        buff.write(_struct_I.pack(length))
        for val2 in val1.shapes:
          buff.write(_struct_b.pack(val2.type))
          length = len(val2.dimensions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.dimensions.tostring())
          length = len(val2.triangles)
          buff.write(_struct_I.pack(length))
          pattern = '<%si'%length
          buff.write(val2.triangles.tostring())
          length = len(val2.vertices)
          buff.write(_struct_I.pack(length))
          for val3 in val2.vertices:
            _x = val3
            buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.poses:
          _v35 = val2.position
          _x = _v35
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v36 = val2.orientation
          _x = _v36
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.attached_collision_objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.attached_collision_objects:
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v37 = val1.object
        _v38 = _v37.header
        buff.write(_struct_I.pack(_v38.seq))
        _v39 = _v38.stamp
        _x = _v39
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v38.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v37.id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_f.pack(_v37.padding))
        _v40 = _v37.operation
        buff.write(_struct_b.pack(_v40.operation))
        length = len(_v37.shapes)
        buff.write(_struct_I.pack(length))
        for val3 in _v37.shapes:
          buff.write(_struct_b.pack(val3.type))
          length = len(val3.dimensions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val3.dimensions.tostring())
          length = len(val3.triangles)
          buff.write(_struct_I.pack(length))
          pattern = '<%si'%length
          buff.write(val3.triangles.tostring())
          length = len(val3.vertices)
          buff.write(_struct_I.pack(length))
          for val4 in val3.vertices:
            _x = val4
            buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(_v37.poses)
        buff.write(_struct_I.pack(length))
        for val3 in _v37.poses:
          _v41 = val3.position
          _x = _v41
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v42 = val3.orientation
          _x = _v42
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.touch_links)
        buff.write(_struct_I.pack(length))
        for val2 in val1.touch_links:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.points is None:
        self.points = arm_navigation_msgs.msg.CollisionMap()
      if self.collision_objects is None:
        self.collision_objects = None
      if self.attached_collision_objects is None:
        self.attached_collision_objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.points.header.seq, _x.points.header.stamp.secs, _x.points.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.points.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.points.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points.boxes = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.OrientedBoundingBox()
        _v43 = val1.center
        _x = _v43
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        _v44 = val1.extents
        _x = _v44
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        _v45 = val1.axis
        _x = _v45
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _struct_3f.unpack(str[start:end])
        start = end
        end += 4
        (val1.angle,) = _struct_f.unpack(str[start:end])
        self.points.boxes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.collision_objects = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.CollisionObject()
        _v46 = val1.header
        start = end
        end += 4
        (_v46.seq,) = _struct_I.unpack(str[start:end])
        _v47 = _v46.stamp
        _x = _v47
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v46.frame_id = str[start:end].decode('utf-8')
        else:
          _v46.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.id = str[start:end].decode('utf-8')
        else:
          val1.id = str[start:end]
        start = end
        end += 4
        (val1.padding,) = _struct_f.unpack(str[start:end])
        _v48 = val1.operation
        start = end
        end += 1
        (_v48.operation,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.shapes = []
        for i in range(0, length):
          val2 = arm_navigation_msgs.msg.Shape()
          start = end
          end += 1
          (val2.type,) = _struct_b.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%si'%length
          start = end
          end += struct.calcsize(pattern)
          val2.triangles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val2.vertices = []
          for i in range(0, length):
            val3 = geometry_msgs.msg.Point()
            _x = val3
            start = end
            end += 24
            (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
            val2.vertices.append(val3)
          val1.shapes.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.poses = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Pose()
          _v49 = val2.position
          _x = _v49
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v50 = val2.orientation
          _x = _v50
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.poses.append(val2)
        self.collision_objects.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.attached_collision_objects = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.AttachedCollisionObject()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        _v51 = val1.object
        _v52 = _v51.header
        start = end
        end += 4
        (_v52.seq,) = _struct_I.unpack(str[start:end])
        _v53 = _v52.stamp
        _x = _v53
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v52.frame_id = str[start:end].decode('utf-8')
        else:
          _v52.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v51.id = str[start:end].decode('utf-8')
        else:
          _v51.id = str[start:end]
        start = end
        end += 4
        (_v51.padding,) = _struct_f.unpack(str[start:end])
        _v54 = _v51.operation
        start = end
        end += 1
        (_v54.operation,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v51.shapes = []
        for i in range(0, length):
          val3 = arm_navigation_msgs.msg.Shape()
          start = end
          end += 1
          (val3.type,) = _struct_b.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val3.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%si'%length
          start = end
          end += struct.calcsize(pattern)
          val3.triangles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val3.vertices = []
          for i in range(0, length):
            val4 = geometry_msgs.msg.Point()
            _x = val4
            start = end
            end += 24
            (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
            val3.vertices.append(val4)
          _v51.shapes.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v51.poses = []
        for i in range(0, length):
          val3 = geometry_msgs.msg.Pose()
          _v55 = val3.position
          _x = _v55
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v56 = val3.orientation
          _x = _v56
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          _v51.poses.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.touch_links = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8')
          else:
            val2 = str[start:end]
          val1.touch_links.append(val2)
        self.attached_collision_objects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
_struct_f = struct.Struct("<f")
_struct_2I = struct.Struct("<2I")
_struct_3I = struct.Struct("<3I")
_struct_4d = struct.Struct("<4d")
_struct_3f = struct.Struct("<3f")
_struct_3d = struct.Struct("<3d")
class GetCollisionObjects(object):
  _type          = 'arm_navigation_msgs/GetCollisionObjects'
  _md5sum = '8a4f57995c7be09c22ca01de6eb557ac'
  _request_class  = GetCollisionObjectsRequest
  _response_class = GetCollisionObjectsResponse
