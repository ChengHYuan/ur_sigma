; Auto-generated. Do not edit!


(cl:in-package epos_msgs-msg)


;//! \htmlinclude MotorStates.msg.html

(cl:defclass <MotorStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (states
    :reader states
    :initarg :states
    :type (cl:vector epos_msgs-msg:MotorState)
   :initform (cl:make-array 0 :element-type 'epos_msgs-msg:MotorState :initial-element (cl:make-instance 'epos_msgs-msg:MotorState))))
)

(cl:defclass MotorStates (<MotorStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name epos_msgs-msg:<MotorStates> is deprecated: use epos_msgs-msg:MotorStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotorStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos_msgs-msg:header-val is deprecated.  Use epos_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <MotorStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos_msgs-msg:states-val is deprecated.  Use epos_msgs-msg:states instead.")
  (states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorStates>) ostream)
  "Serializes a message object of type '<MotorStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorStates>) istream)
  "Deserializes a message object of type '<MotorStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'epos_msgs-msg:MotorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorStates>)))
  "Returns string type for a message object of type '<MotorStates>"
  "epos_msgs/MotorStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorStates)))
  "Returns string type for a message object of type 'MotorStates"
  "epos_msgs/MotorStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorStates>)))
  "Returns md5sum for a message object of type '<MotorStates>"
  "3ad89420af0daf166bb737efa2555173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorStates)))
  "Returns md5sum for a message object of type 'MotorStates"
  "3ad89420af0daf166bb737efa2555173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorStates>)))
  "Returns full string definition for message of type '<MotorStates>"
  (cl:format cl:nil "~%Header header~%~%MotorState[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: epos_msgs/MotorState~%~%Header header~%~%string motor_name~%~%float64 position~%float64 velocity~%float64 current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorStates)))
  "Returns full string definition for message of type 'MotorStates"
  (cl:format cl:nil "~%Header header~%~%MotorState[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: epos_msgs/MotorState~%~%Header header~%~%string motor_name~%~%float64 position~%float64 velocity~%float64 current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorStates>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorStates
    (cl:cons ':header (header msg))
    (cl:cons ':states (states msg))
))
