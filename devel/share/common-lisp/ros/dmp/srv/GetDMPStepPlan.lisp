; Auto-generated. Do not edit!


(cl:in-package dmp-srv)


;//! \htmlinclude GetDMPStepPlan-request.msg.html

(cl:defclass <GetDMPStepPlan-request> (roslisp-msg-protocol:ros-message)
  ((x_0
    :reader x_0
    :initarg :x_0
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (current
    :reader current
    :initarg :current
    :type dmp-msg:DMPPoint
    :initform (cl:make-instance 'dmp-msg:DMPPoint))
   (t_0
    :reader t_0
    :initarg :t_0
    :type cl:float
    :initform 0.0)
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (goal_thresh
    :reader goal_thresh
    :initarg :goal_thresh
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (seg_length
    :reader seg_length
    :initarg :seg_length
    :type cl:float
    :initform 0.0)
   (tau
    :reader tau
    :initarg :tau
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0)
   (integrate_iter
    :reader integrate_iter
    :initarg :integrate_iter
    :type cl:integer
    :initform 0))
)

(cl:defclass GetDMPStepPlan-request (<GetDMPStepPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDMPStepPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDMPStepPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GetDMPStepPlan-request> is deprecated: use dmp-srv:GetDMPStepPlan-request instead.")))

(cl:ensure-generic-function 'x_0-val :lambda-list '(m))
(cl:defmethod x_0-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:x_0-val is deprecated.  Use dmp-srv:x_0 instead.")
  (x_0 m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:current-val is deprecated.  Use dmp-srv:current instead.")
  (current m))

(cl:ensure-generic-function 't_0-val :lambda-list '(m))
(cl:defmethod t_0-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:t_0-val is deprecated.  Use dmp-srv:t_0 instead.")
  (t_0 m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:goal-val is deprecated.  Use dmp-srv:goal instead.")
  (goal m))

(cl:ensure-generic-function 'goal_thresh-val :lambda-list '(m))
(cl:defmethod goal_thresh-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:goal_thresh-val is deprecated.  Use dmp-srv:goal_thresh instead.")
  (goal_thresh m))

(cl:ensure-generic-function 'seg_length-val :lambda-list '(m))
(cl:defmethod seg_length-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:seg_length-val is deprecated.  Use dmp-srv:seg_length instead.")
  (seg_length m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:tau-val is deprecated.  Use dmp-srv:tau instead.")
  (tau m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:dt-val is deprecated.  Use dmp-srv:dt instead.")
  (dt m))

(cl:ensure-generic-function 'integrate_iter-val :lambda-list '(m))
(cl:defmethod integrate_iter-val ((m <GetDMPStepPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:integrate_iter-val is deprecated.  Use dmp-srv:integrate_iter instead.")
  (integrate_iter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDMPStepPlan-request>) ostream)
  "Serializes a message object of type '<GetDMPStepPlan-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_0))))
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
   (cl:slot-value msg 'x_0))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal))))
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
   (cl:slot-value msg 'goal))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal_thresh))))
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
   (cl:slot-value msg 'goal_thresh))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'seg_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tau))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'integrate_iter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDMPStepPlan-request>) istream)
  "Deserializes a message object of type '<GetDMPStepPlan-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_0) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_0)))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_0) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal)))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal_thresh) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal_thresh)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'seg_length) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tau) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'integrate_iter) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDMPStepPlan-request>)))
  "Returns string type for a service object of type '<GetDMPStepPlan-request>"
  "dmp/GetDMPStepPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPStepPlan-request)))
  "Returns string type for a service object of type 'GetDMPStepPlan-request"
  "dmp/GetDMPStepPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDMPStepPlan-request>)))
  "Returns md5sum for a message object of type '<GetDMPStepPlan-request>"
  "5cb36999dd0228bf3a5de0dcd217f277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDMPStepPlan-request)))
  "Returns md5sum for a message object of type 'GetDMPStepPlan-request"
  "5cb36999dd0228bf3a5de0dcd217f277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDMPStepPlan-request>)))
  "Returns full string definition for message of type '<GetDMPStepPlan-request>"
  (cl:format cl:nil "# A starting state to begin planning from~%float64[] x_0~%~%# current status~%DMPPoint current~%~%# The time in seconds at which to begin the planning segment. ~%# Should only be nonzero when doing a partial segment plan that does not start at beginning of DMP~%float64 t_0~%~%# The goal of the plan~%float64[] goal~%~%# For dimensions with a value greater than zero, planning will continue until ~%# the predicted state is within the specified distance of the goal in all such dimensions.~%# Dimensions with values less than or equal to zero will be ignored.~%# Trajectory plan will always be at least tau seconds long.~%float64[] goal_thresh~%~%# The length of the requested plan segment in seconds. Set to -1 if plan until goal is desired.~%float64 seg_length~%~%# A time constant to set the length of DMP replay in seconds until 95% phase convergence~%float64 tau~%~%# The time resolution, in seconds, at which to plan~%float64 dt~%~%# Number of times to loop in numerical integration.  This can generally be 1, unless dt is large (>1 second)~%int32 integrate_iter~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDMPStepPlan-request)))
  "Returns full string definition for message of type 'GetDMPStepPlan-request"
  (cl:format cl:nil "# A starting state to begin planning from~%float64[] x_0~%~%# current status~%DMPPoint current~%~%# The time in seconds at which to begin the planning segment. ~%# Should only be nonzero when doing a partial segment plan that does not start at beginning of DMP~%float64 t_0~%~%# The goal of the plan~%float64[] goal~%~%# For dimensions with a value greater than zero, planning will continue until ~%# the predicted state is within the specified distance of the goal in all such dimensions.~%# Dimensions with values less than or equal to zero will be ignored.~%# Trajectory plan will always be at least tau seconds long.~%float64[] goal_thresh~%~%# The length of the requested plan segment in seconds. Set to -1 if plan until goal is desired.~%float64 seg_length~%~%# A time constant to set the length of DMP replay in seconds until 95% phase convergence~%float64 tau~%~%# The time resolution, in seconds, at which to plan~%float64 dt~%~%# Number of times to loop in numerical integration.  This can generally be 1, unless dt is large (>1 second)~%int32 integrate_iter~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDMPStepPlan-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal_thresh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDMPStepPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDMPStepPlan-request
    (cl:cons ':x_0 (x_0 msg))
    (cl:cons ':current (current msg))
    (cl:cons ':t_0 (t_0 msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':goal_thresh (goal_thresh msg))
    (cl:cons ':seg_length (seg_length msg))
    (cl:cons ':tau (tau msg))
    (cl:cons ':dt (dt msg))
    (cl:cons ':integrate_iter (integrate_iter msg))
))
;//! \htmlinclude GetDMPStepPlan-response.msg.html

(cl:defclass <GetDMPStepPlan-response> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type dmp-msg:DMPPoint
    :initform (cl:make-instance 'dmp-msg:DMPPoint))
   (at_goal
    :reader at_goal
    :initarg :at_goal
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetDMPStepPlan-response (<GetDMPStepPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDMPStepPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDMPStepPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GetDMPStepPlan-response> is deprecated: use dmp-srv:GetDMPStepPlan-response instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <GetDMPStepPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:plan-val is deprecated.  Use dmp-srv:plan instead.")
  (plan m))

(cl:ensure-generic-function 'at_goal-val :lambda-list '(m))
(cl:defmethod at_goal-val ((m <GetDMPStepPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:at_goal-val is deprecated.  Use dmp-srv:at_goal instead.")
  (at_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDMPStepPlan-response>) ostream)
  "Serializes a message object of type '<GetDMPStepPlan-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'at_goal)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDMPStepPlan-response>) istream)
  "Deserializes a message object of type '<GetDMPStepPlan-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'at_goal)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDMPStepPlan-response>)))
  "Returns string type for a service object of type '<GetDMPStepPlan-response>"
  "dmp/GetDMPStepPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPStepPlan-response)))
  "Returns string type for a service object of type 'GetDMPStepPlan-response"
  "dmp/GetDMPStepPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDMPStepPlan-response>)))
  "Returns md5sum for a message object of type '<GetDMPStepPlan-response>"
  "5cb36999dd0228bf3a5de0dcd217f277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDMPStepPlan-response)))
  "Returns md5sum for a message object of type 'GetDMPStepPlan-response"
  "5cb36999dd0228bf3a5de0dcd217f277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDMPStepPlan-response>)))
  "Returns full string definition for message of type '<GetDMPStepPlan-response>"
  (cl:format cl:nil "~%# Returns a planned trajectory. ~%DMPPoint plan~%~%#1 if the final time is greater than tau AND the planned position is within goal_thresh of the goal, 0 otherwise~%uint8 at_goal~%~%~%~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDMPStepPlan-response)))
  "Returns full string definition for message of type 'GetDMPStepPlan-response"
  (cl:format cl:nil "~%# Returns a planned trajectory. ~%DMPPoint plan~%~%#1 if the final time is greater than tau AND the planned position is within goal_thresh of the goal, 0 otherwise~%uint8 at_goal~%~%~%~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDMPStepPlan-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDMPStepPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDMPStepPlan-response
    (cl:cons ':plan (plan msg))
    (cl:cons ':at_goal (at_goal msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDMPStepPlan)))
  'GetDMPStepPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDMPStepPlan)))
  'GetDMPStepPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPStepPlan)))
  "Returns string type for a service object of type '<GetDMPStepPlan>"
  "dmp/GetDMPStepPlan")