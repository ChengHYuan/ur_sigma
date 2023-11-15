# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dmp/GetDMPPlanRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetDMPPlanRequest(genpy.Message):
  _md5sum = "bae6b051e2f7b80225be1fd25b5b705a"
  _type = "dmp/GetDMPPlanRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# A starting state to begin planning from
float64[] x_0

# A starting instantaneous change of state to begin planning from
float64[] x_dot_0

# The time in seconds at which to begin the planning segment. 
# Should only be nonzero when doing a partial segment plan that does not start at beginning of DMP
float64 t_0

# The goal of the plan
float64[] goal

# For dimensions with a value greater than zero, planning will continue until 
# the predicted state is within the specified distance of the goal in all such dimensions.
# Dimensions with values less than or equal to zero will be ignored.
# Trajectory plan will always be at least tau seconds long.
float64[] goal_thresh

# The length of the requested plan segment in seconds. Set to -1 if plan until goal is desired.
float64 seg_length

# A time constant to set the length of DMP replay in seconds until 95% phase convergence
float64 tau

# The time resolution, in seconds, at which to plan
float64 dt

# Number of times to loop in numerical integration.  This can generally be 1, unless dt is large (>1 second)
int32 integrate_iter

"""
  __slots__ = ['x_0','x_dot_0','t_0','goal','goal_thresh','seg_length','tau','dt','integrate_iter']
  _slot_types = ['float64[]','float64[]','float64','float64[]','float64[]','float64','float64','float64','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x_0,x_dot_0,t_0,goal,goal_thresh,seg_length,tau,dt,integrate_iter

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetDMPPlanRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x_0 is None:
        self.x_0 = []
      if self.x_dot_0 is None:
        self.x_dot_0 = []
      if self.t_0 is None:
        self.t_0 = 0.
      if self.goal is None:
        self.goal = []
      if self.goal_thresh is None:
        self.goal_thresh = []
      if self.seg_length is None:
        self.seg_length = 0.
      if self.tau is None:
        self.tau = 0.
      if self.dt is None:
        self.dt = 0.
      if self.integrate_iter is None:
        self.integrate_iter = 0
    else:
      self.x_0 = []
      self.x_dot_0 = []
      self.t_0 = 0.
      self.goal = []
      self.goal_thresh = []
      self.seg_length = 0.
      self.tau = 0.
      self.dt = 0.
      self.integrate_iter = 0

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
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.x_0))
      length = len(self.x_dot_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.x_dot_0))
      _x = self.t_0
      buff.write(_get_struct_d().pack(_x))
      length = len(self.goal)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.goal))
      length = len(self.goal_thresh)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.goal_thresh))
      _x = self
      buff.write(_get_struct_3di().pack(_x.seg_length, _x.tau, _x.dt, _x.integrate_iter))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.x_0 = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.x_dot_0 = s.unpack(str[start:end])
      start = end
      end += 8
      (self.t_0,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.goal = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.goal_thresh = s.unpack(str[start:end])
      _x = self
      start = end
      end += 28
      (_x.seg_length, _x.tau, _x.dt, _x.integrate_iter,) = _get_struct_3di().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.x_0.tostring())
      length = len(self.x_dot_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.x_dot_0.tostring())
      _x = self.t_0
      buff.write(_get_struct_d().pack(_x))
      length = len(self.goal)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.goal.tostring())
      length = len(self.goal_thresh)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.goal_thresh.tostring())
      _x = self
      buff.write(_get_struct_3di().pack(_x.seg_length, _x.tau, _x.dt, _x.integrate_iter))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.x_0 = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.x_dot_0 = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 8
      (self.t_0,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.goal = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.goal_thresh = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 28
      (_x.seg_length, _x.tau, _x.dt, _x.integrate_iter,) = _get_struct_3di().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3di = None
def _get_struct_3di():
    global _struct_3di
    if _struct_3di is None:
        _struct_3di = struct.Struct("<3di")
    return _struct_3di
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dmp/GetDMPPlanResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import dmp.msg

class GetDMPPlanResponse(genpy.Message):
  _md5sum = "dcf9f84a71b2617cf4bbc301a97c05cd"
  _type = "dmp/GetDMPPlanResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
# Returns a planned trajectory. 
DMPTraj plan

#1 if the final time is greater than tau AND the planned position is within goal_thresh of the goal, 0 otherwise
uint8 at_goal






================================================================================
MSG: dmp/DMPTraj
# points and times should be the same length
DMPPoint[] points

# Times of observations, in seconds, starting at zero
float64[] times



================================================================================
MSG: dmp/DMPPoint
# Positions and velocities of DOFs
#Velocity is only used for movement plans, not for giving demonstrations.
float64[] positions
float64[] velocities


"""
  __slots__ = ['plan','at_goal']
  _slot_types = ['dmp/DMPTraj','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       plan,at_goal

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetDMPPlanResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.plan is None:
        self.plan = dmp.msg.DMPTraj()
      if self.at_goal is None:
        self.at_goal = 0
    else:
      self.plan = dmp.msg.DMPTraj()
      self.at_goal = 0

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
      length = len(self.plan.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.points:
        length = len(val1.positions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.Struct(pattern).pack(*val1.positions))
        length = len(val1.velocities)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.Struct(pattern).pack(*val1.velocities))
      length = len(self.plan.times)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.plan.times))
      _x = self.at_goal
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.plan is None:
        self.plan = dmp.msg.DMPTraj()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.points = []
      for i in range(0, length):
        val1 = dmp.msg.DMPPoint()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.positions = s.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.velocities = s.unpack(str[start:end])
        self.plan.points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.plan.times = s.unpack(str[start:end])
      start = end
      end += 1
      (self.at_goal,) = _get_struct_B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.plan.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.points:
        length = len(val1.positions)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.positions.tostring())
        length = len(val1.velocities)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.velocities.tostring())
      length = len(self.plan.times)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.plan.times.tostring())
      _x = self.at_goal
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.plan is None:
        self.plan = dmp.msg.DMPTraj()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.points = []
      for i in range(0, length):
        val1 = dmp.msg.DMPPoint()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.positions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.velocities = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        self.plan.points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.plan.times = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 1
      (self.at_goal,) = _get_struct_B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class GetDMPPlan(object):
  _type          = 'dmp/GetDMPPlan'
  _md5sum = '5cd79fd80676a4f8f062c5472a3190b1'
  _request_class  = GetDMPPlanRequest
  _response_class = GetDMPPlanResponse
