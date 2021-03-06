"""autogenerated by genpy from arm_navigation_msgs/Constraints.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import arm_navigation_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class Constraints(genpy.Message):
  _md5sum = "fe6b6f09c687fd46c05a2de4ca18378a"
  _type = "arm_navigation_msgs/Constraints"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This message contains a list of motion planning constraints.

arm_navigation_msgs/JointConstraint[] joint_constraints
arm_navigation_msgs/PositionConstraint[] position_constraints
arm_navigation_msgs/OrientationConstraint[] orientation_constraints
arm_navigation_msgs/VisibilityConstraint[] visibility_constraints

================================================================================
MSG: arm_navigation_msgs/JointConstraint
# Constrain the position of a joint to be within a certain bound
string joint_name

# the bound to be achieved is [position - tolerance_below, position + tolerance_above]
float64 position
float64 tolerance_above
float64 tolerance_below

# A weighting factor for this constraint
float64 weight
================================================================================
MSG: arm_navigation_msgs/PositionConstraint
# This message contains the definition of a position constraint.
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

================================================================================
MSG: arm_navigation_msgs/OrientationConstraint
# This message contains the definition of an orientation constraint.
Header header

# The robot link this constraint refers to
string link_name

# The type of the constraint
int32 type
int32 LINK_FRAME=0
int32 HEADER_FRAME=1

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/Quaternion orientation

# optional RPY error tolerances specified if 
float64 absolute_roll_tolerance
float64 absolute_pitch_tolerance
float64 absolute_yaw_tolerance

# Constraint weighting factor - a weight for this constraint
float64 weight

================================================================================
MSG: arm_navigation_msgs/VisibilityConstraint
# This message contains the definition of a visibility constraint.
Header header

# The point stamped target that needs to be kept within view of the sensor
geometry_msgs/PointStamped target

# The local pose of the frame in which visibility is to be maintained
# The frame id should represent the robot link to which the sensor is attached
# The visual axis of the sensor is assumed to be along the X axis of this frame
geometry_msgs/PoseStamped sensor_pose

# The deviation (in radians) that will be tolerated
# Constraint error will be measured as the solid angle between the 
# X axis of the frame defined above and the vector between the origin 
# of the frame defined above and the target location
float64 absolute_tolerance


================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

"""
  __slots__ = ['joint_constraints','position_constraints','orientation_constraints','visibility_constraints']
  _slot_types = ['arm_navigation_msgs/JointConstraint[]','arm_navigation_msgs/PositionConstraint[]','arm_navigation_msgs/OrientationConstraint[]','arm_navigation_msgs/VisibilityConstraint[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint_constraints,position_constraints,orientation_constraints,visibility_constraints

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Constraints, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.joint_constraints is None:
        self.joint_constraints = []
      if self.position_constraints is None:
        self.position_constraints = []
      if self.orientation_constraints is None:
        self.orientation_constraints = []
      if self.visibility_constraints is None:
        self.visibility_constraints = []
    else:
      self.joint_constraints = []
      self.position_constraints = []
      self.orientation_constraints = []
      self.visibility_constraints = []

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
      length = len(self.joint_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_constraints:
        _x = val1.joint_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_4d.pack(_x.position, _x.tolerance_above, _x.tolerance_below, _x.weight))
      length = len(self.position_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.position_constraints:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.target_point_offset
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v4 = val1.position
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = val1.constraint_region_shape
        buff.write(_struct_b.pack(_v5.type))
        length = len(_v5.dimensions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.pack(pattern, *_v5.dimensions))
        length = len(_v5.triangles)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.pack(pattern, *_v5.triangles))
        length = len(_v5.vertices)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.vertices:
          _x = val3
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = val1.constraint_region_orientation
        _x = _v6
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.weight))
      length = len(self.orientation_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.orientation_constraints:
        _v7 = val1.header
        buff.write(_struct_I.pack(_v7.seq))
        _v8 = _v7.stamp
        _x = _v8
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v7.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_i.pack(val1.type))
        _v9 = val1.orientation
        _x = _v9
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _x = val1
        buff.write(_struct_4d.pack(_x.absolute_roll_tolerance, _x.absolute_pitch_tolerance, _x.absolute_yaw_tolerance, _x.weight))
      length = len(self.visibility_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.visibility_constraints:
        _v10 = val1.header
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
        _v12 = val1.target
        _v13 = _v12.header
        buff.write(_struct_I.pack(_v13.seq))
        _v14 = _v13.stamp
        _x = _v14
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v13.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v15 = _v12.point
        _x = _v15
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v16 = val1.sensor_pose
        _v17 = _v16.header
        buff.write(_struct_I.pack(_v17.seq))
        _v18 = _v17.stamp
        _x = _v18
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v17.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v19 = _v16.pose
        _v20 = _v19.position
        _x = _v20
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v21 = _v19.orientation
        _x = _v21
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.absolute_tolerance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.joint_constraints is None:
        self.joint_constraints = None
      if self.position_constraints is None:
        self.position_constraints = None
      if self.orientation_constraints is None:
        self.orientation_constraints = None
      if self.visibility_constraints is None:
        self.visibility_constraints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.JointConstraint()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.joint_name = str[start:end].decode('utf-8')
        else:
          val1.joint_name = str[start:end]
        _x = val1
        start = end
        end += 32
        (_x.position, _x.tolerance_above, _x.tolerance_below, _x.weight,) = _struct_4d.unpack(str[start:end])
        self.joint_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.PositionConstraint()
        _v22 = val1.header
        start = end
        end += 4
        (_v22.seq,) = _struct_I.unpack(str[start:end])
        _v23 = _v22.stamp
        _x = _v23
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v22.frame_id = str[start:end].decode('utf-8')
        else:
          _v22.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        _v24 = val1.target_point_offset
        _x = _v24
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v25 = val1.position
        _x = _v25
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v26 = val1.constraint_region_shape
        start = end
        end += 1
        (_v26.type,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        _v26.dimensions = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        _v26.triangles = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v26.vertices = []
        for i in range(0, length):
          val3 = geometry_msgs.msg.Point()
          _x = val3
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v26.vertices.append(val3)
        _v27 = val1.constraint_region_orientation
        _x = _v27
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.weight,) = _struct_d.unpack(str[start:end])
        self.position_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.orientation_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.OrientationConstraint()
        _v28 = val1.header
        start = end
        end += 4
        (_v28.seq,) = _struct_I.unpack(str[start:end])
        _v29 = _v28.stamp
        _x = _v29
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v28.frame_id = str[start:end].decode('utf-8')
        else:
          _v28.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        start = end
        end += 4
        (val1.type,) = _struct_i.unpack(str[start:end])
        _v30 = val1.orientation
        _x = _v30
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _x = val1
        start = end
        end += 32
        (_x.absolute_roll_tolerance, _x.absolute_pitch_tolerance, _x.absolute_yaw_tolerance, _x.weight,) = _struct_4d.unpack(str[start:end])
        self.orientation_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.visibility_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.VisibilityConstraint()
        _v31 = val1.header
        start = end
        end += 4
        (_v31.seq,) = _struct_I.unpack(str[start:end])
        _v32 = _v31.stamp
        _x = _v32
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v31.frame_id = str[start:end].decode('utf-8')
        else:
          _v31.frame_id = str[start:end]
        _v33 = val1.target
        _v34 = _v33.header
        start = end
        end += 4
        (_v34.seq,) = _struct_I.unpack(str[start:end])
        _v35 = _v34.stamp
        _x = _v35
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v34.frame_id = str[start:end].decode('utf-8')
        else:
          _v34.frame_id = str[start:end]
        _v36 = _v33.point
        _x = _v36
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v37 = val1.sensor_pose
        _v38 = _v37.header
        start = end
        end += 4
        (_v38.seq,) = _struct_I.unpack(str[start:end])
        _v39 = _v38.stamp
        _x = _v39
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v38.frame_id = str[start:end].decode('utf-8')
        else:
          _v38.frame_id = str[start:end]
        _v40 = _v37.pose
        _v41 = _v40.position
        _x = _v41
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v42 = _v40.orientation
        _x = _v42
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.absolute_tolerance,) = _struct_d.unpack(str[start:end])
        self.visibility_constraints.append(val1)
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
      length = len(self.joint_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_constraints:
        _x = val1.joint_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_4d.pack(_x.position, _x.tolerance_above, _x.tolerance_below, _x.weight))
      length = len(self.position_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.position_constraints:
        _v43 = val1.header
        buff.write(_struct_I.pack(_v43.seq))
        _v44 = _v43.stamp
        _x = _v44
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v43.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v45 = val1.target_point_offset
        _x = _v45
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v46 = val1.position
        _x = _v46
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v47 = val1.constraint_region_shape
        buff.write(_struct_b.pack(_v47.type))
        length = len(_v47.dimensions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(_v47.dimensions.tostring())
        length = len(_v47.triangles)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(_v47.triangles.tostring())
        length = len(_v47.vertices)
        buff.write(_struct_I.pack(length))
        for val3 in _v47.vertices:
          _x = val3
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v48 = val1.constraint_region_orientation
        _x = _v48
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.weight))
      length = len(self.orientation_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.orientation_constraints:
        _v49 = val1.header
        buff.write(_struct_I.pack(_v49.seq))
        _v50 = _v49.stamp
        _x = _v50
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v49.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.link_name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_i.pack(val1.type))
        _v51 = val1.orientation
        _x = _v51
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _x = val1
        buff.write(_struct_4d.pack(_x.absolute_roll_tolerance, _x.absolute_pitch_tolerance, _x.absolute_yaw_tolerance, _x.weight))
      length = len(self.visibility_constraints)
      buff.write(_struct_I.pack(length))
      for val1 in self.visibility_constraints:
        _v52 = val1.header
        buff.write(_struct_I.pack(_v52.seq))
        _v53 = _v52.stamp
        _x = _v53
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v52.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v54 = val1.target
        _v55 = _v54.header
        buff.write(_struct_I.pack(_v55.seq))
        _v56 = _v55.stamp
        _x = _v56
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v55.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v57 = _v54.point
        _x = _v57
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v58 = val1.sensor_pose
        _v59 = _v58.header
        buff.write(_struct_I.pack(_v59.seq))
        _v60 = _v59.stamp
        _x = _v60
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v59.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v61 = _v58.pose
        _v62 = _v61.position
        _x = _v62
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v63 = _v61.orientation
        _x = _v63
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.absolute_tolerance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.joint_constraints is None:
        self.joint_constraints = None
      if self.position_constraints is None:
        self.position_constraints = None
      if self.orientation_constraints is None:
        self.orientation_constraints = None
      if self.visibility_constraints is None:
        self.visibility_constraints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.JointConstraint()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.joint_name = str[start:end].decode('utf-8')
        else:
          val1.joint_name = str[start:end]
        _x = val1
        start = end
        end += 32
        (_x.position, _x.tolerance_above, _x.tolerance_below, _x.weight,) = _struct_4d.unpack(str[start:end])
        self.joint_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.PositionConstraint()
        _v64 = val1.header
        start = end
        end += 4
        (_v64.seq,) = _struct_I.unpack(str[start:end])
        _v65 = _v64.stamp
        _x = _v65
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v64.frame_id = str[start:end].decode('utf-8')
        else:
          _v64.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        _v66 = val1.target_point_offset
        _x = _v66
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v67 = val1.position
        _x = _v67
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v68 = val1.constraint_region_shape
        start = end
        end += 1
        (_v68.type,) = _struct_b.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        _v68.dimensions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        end += struct.calcsize(pattern)
        _v68.triangles = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v68.vertices = []
        for i in range(0, length):
          val3 = geometry_msgs.msg.Point()
          _x = val3
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v68.vertices.append(val3)
        _v69 = val1.constraint_region_orientation
        _x = _v69
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.weight,) = _struct_d.unpack(str[start:end])
        self.position_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.orientation_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.OrientationConstraint()
        _v70 = val1.header
        start = end
        end += 4
        (_v70.seq,) = _struct_I.unpack(str[start:end])
        _v71 = _v70.stamp
        _x = _v71
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v70.frame_id = str[start:end].decode('utf-8')
        else:
          _v70.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.link_name = str[start:end].decode('utf-8')
        else:
          val1.link_name = str[start:end]
        start = end
        end += 4
        (val1.type,) = _struct_i.unpack(str[start:end])
        _v72 = val1.orientation
        _x = _v72
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _x = val1
        start = end
        end += 32
        (_x.absolute_roll_tolerance, _x.absolute_pitch_tolerance, _x.absolute_yaw_tolerance, _x.weight,) = _struct_4d.unpack(str[start:end])
        self.orientation_constraints.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.visibility_constraints = []
      for i in range(0, length):
        val1 = arm_navigation_msgs.msg.VisibilityConstraint()
        _v73 = val1.header
        start = end
        end += 4
        (_v73.seq,) = _struct_I.unpack(str[start:end])
        _v74 = _v73.stamp
        _x = _v74
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v73.frame_id = str[start:end].decode('utf-8')
        else:
          _v73.frame_id = str[start:end]
        _v75 = val1.target
        _v76 = _v75.header
        start = end
        end += 4
        (_v76.seq,) = _struct_I.unpack(str[start:end])
        _v77 = _v76.stamp
        _x = _v77
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v76.frame_id = str[start:end].decode('utf-8')
        else:
          _v76.frame_id = str[start:end]
        _v78 = _v75.point
        _x = _v78
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v79 = val1.sensor_pose
        _v80 = _v79.header
        start = end
        end += 4
        (_v80.seq,) = _struct_I.unpack(str[start:end])
        _v81 = _v80.stamp
        _x = _v81
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v80.frame_id = str[start:end].decode('utf-8')
        else:
          _v80.frame_id = str[start:end]
        _v82 = _v79.pose
        _v83 = _v82.position
        _x = _v83
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v84 = _v82.orientation
        _x = _v84
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.absolute_tolerance,) = _struct_d.unpack(str[start:end])
        self.visibility_constraints.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
_struct_d = struct.Struct("<d")
_struct_i = struct.Struct("<i")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
