; Auto-generated. Do not edit!


(cl:in-package dmp-srv)


;//! \htmlinclude GoalToPath-request.msg.html

(cl:defclass <GoalToPath-request> (roslisp-msg-protocol:ros-message)
  ((Start
    :reader Start
    :initarg :Start
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (GoalIndex
    :reader GoalIndex
    :initarg :GoalIndex
    :type cl:integer
    :initform 0))
)

(cl:defclass GoalToPath-request (<GoalToPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalToPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalToPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GoalToPath-request> is deprecated: use dmp-srv:GoalToPath-request instead.")))

(cl:ensure-generic-function 'Start-val :lambda-list '(m))
(cl:defmethod Start-val ((m <GoalToPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:Start-val is deprecated.  Use dmp-srv:Start instead.")
  (Start m))

(cl:ensure-generic-function 'GoalIndex-val :lambda-list '(m))
(cl:defmethod GoalIndex-val ((m <GoalToPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:GoalIndex-val is deprecated.  Use dmp-srv:GoalIndex instead.")
  (GoalIndex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalToPath-request>) ostream)
  "Serializes a message object of type '<GoalToPath-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'Start))
  (cl:let* ((signed (cl:slot-value msg 'GoalIndex)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalToPath-request>) istream)
  "Deserializes a message object of type '<GoalToPath-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Start) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Start)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'GoalIndex) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalToPath-request>)))
  "Returns string type for a service object of type '<GoalToPath-request>"
  "dmp/GoalToPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalToPath-request)))
  "Returns string type for a service object of type 'GoalToPath-request"
  "dmp/GoalToPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalToPath-request>)))
  "Returns md5sum for a message object of type '<GoalToPath-request>"
  "61b7759cacf0fb0b224df3e235dda576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalToPath-request)))
  "Returns md5sum for a message object of type 'GoalToPath-request"
  "61b7759cacf0fb0b224df3e235dda576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalToPath-request>)))
  "Returns full string definition for message of type '<GoalToPath-request>"
  (cl:format cl:nil "float64[] Start~%~%int32 GoalIndex~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalToPath-request)))
  "Returns full string definition for message of type 'GoalToPath-request"
  (cl:format cl:nil "float64[] Start~%~%int32 GoalIndex~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalToPath-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Start) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalToPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalToPath-request
    (cl:cons ':Start (Start msg))
    (cl:cons ':GoalIndex (GoalIndex msg))
))
;//! \htmlinclude GoalToPath-response.msg.html

(cl:defclass <GoalToPath-response> (roslisp-msg-protocol:ros-message)
  ((Path
    :reader Path
    :initarg :Path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass GoalToPath-response (<GoalToPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalToPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalToPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GoalToPath-response> is deprecated: use dmp-srv:GoalToPath-response instead.")))

(cl:ensure-generic-function 'Path-val :lambda-list '(m))
(cl:defmethod Path-val ((m <GoalToPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:Path-val is deprecated.  Use dmp-srv:Path instead.")
  (Path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalToPath-response>) ostream)
  "Serializes a message object of type '<GoalToPath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalToPath-response>) istream)
  "Deserializes a message object of type '<GoalToPath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalToPath-response>)))
  "Returns string type for a service object of type '<GoalToPath-response>"
  "dmp/GoalToPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalToPath-response)))
  "Returns string type for a service object of type 'GoalToPath-response"
  "dmp/GoalToPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalToPath-response>)))
  "Returns md5sum for a message object of type '<GoalToPath-response>"
  "61b7759cacf0fb0b224df3e235dda576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalToPath-response)))
  "Returns md5sum for a message object of type 'GoalToPath-response"
  "61b7759cacf0fb0b224df3e235dda576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalToPath-response>)))
  "Returns full string definition for message of type '<GoalToPath-response>"
  (cl:format cl:nil "~%nav_msgs/Path Path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalToPath-response)))
  "Returns full string definition for message of type 'GoalToPath-response"
  (cl:format cl:nil "~%nav_msgs/Path Path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalToPath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalToPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalToPath-response
    (cl:cons ':Path (Path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoalToPath)))
  'GoalToPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoalToPath)))
  'GoalToPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalToPath)))
  "Returns string type for a service object of type '<GoalToPath>"
  "dmp/GoalToPath")