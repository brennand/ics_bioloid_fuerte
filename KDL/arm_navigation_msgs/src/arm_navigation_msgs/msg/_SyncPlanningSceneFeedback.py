"""autogenerated by genpy from arm_navigation_msgs/SyncPlanningSceneFeedback.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SyncPlanningSceneFeedback(genpy.Message):
  _md5sum = "5470cffcd2540df5b10d2ed9ddfde7e4"
  _type = "arm_navigation_msgs/SyncPlanningSceneFeedback"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
bool client_processing
bool ready



"""
  __slots__ = ['client_processing','ready']
  _slot_types = ['bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       client_processing,ready

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SyncPlanningSceneFeedback, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.client_processing is None:
        self.client_processing = False
      if self.ready is None:
        self.ready = False
    else:
      self.client_processing = False
      self.ready = False

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
      buff.write(_struct_2B.pack(_x.client_processing, _x.ready))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.client_processing, _x.ready,) = _struct_2B.unpack(str[start:end])
      self.client_processing = bool(self.client_processing)
      self.ready = bool(self.ready)
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
      buff.write(_struct_2B.pack(_x.client_processing, _x.ready))
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
      _x = self
      start = end
      end += 2
      (_x.client_processing, _x.ready,) = _struct_2B.unpack(str[start:end])
      self.client_processing = bool(self.client_processing)
      self.ready = bool(self.ready)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2B = struct.Struct("<2B")
