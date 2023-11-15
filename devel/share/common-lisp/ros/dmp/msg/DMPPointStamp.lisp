; Auto-generated. Do not edit!


(cl:in-package dmp-msg)


;//! \htmlinclude DMPPointStamp.msg.html

(cl:defclass <DMPPointStamp> (roslisp-msg-protocol:ros-message)
  ((head
    :reader head
    :initarg :head
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (positions
    :reader positions
    :initarg :positions
    :type cl:float
    :initform 0.0))
)

(cl:defclass DMPPointStamp (<DMPPointStamp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DMPPointStamp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DMPPointStamp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-msg:<DMPPointStamp> is deprecated: use dmp-msg:DMPPointStamp instead.")))

(cl:ensure-generic-function 'head-val :lambda-list '(m))
(cl:defmethod head-val ((m <DMPPointStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-msg:head-val is deprecated.  Use dmp-msg:head instead.")
  (head m))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <DMPPointStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-msg:positions-val is deprecated.  Use dmp-msg:positions instead.")
  (positions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DMPPointStamp>) ostream)
  "Serializes a message object of type '<DMPPointStamp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'head) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DMPPointStamp>) istream)
  "Deserializes a message object of type '<DMPPointStamp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'head) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'positions) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DMPPointStamp>)))
  "Returns string type for a message object of type '<DMPPointStamp>"
  "dmp/DMPPointStamp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DMPPointStamp)))
  "Returns string type for a message object of type 'DMPPointStamp"
  "dmp/DMPPointStamp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DMPPointStamp>)))
  "Returns md5sum for a message object of type '<DMPPointStamp>"
  "db08163e791135724278324162c51a67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DMPPointStamp)))
  "Returns md5sum for a message object of type 'DMPPointStamp"
  "db08163e791135724278324162c51a67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DMPPointStamp>)))
  "Returns full string definition for message of type '<DMPPointStamp>"
  (cl:format cl:nil "std_msgs/Header head~%float64 positions~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DMPPointStamp)))
  "Returns full string definition for message of type 'DMPPointStamp"
  (cl:format cl:nil "std_msgs/Header head~%float64 positions~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DMPPointStamp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'head))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DMPPointStamp>))
  "Converts a ROS message object to a list"
  (cl:list 'DMPPointStamp
    (cl:cons ':head (head msg))
    (cl:cons ':positions (positions msg))
))
