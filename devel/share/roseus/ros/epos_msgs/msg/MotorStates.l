;; Auto-generated. Do not edit!


(when (boundp 'epos_msgs::MotorStates)
  (if (not (find-package "EPOS_MSGS"))
    (make-package "EPOS_MSGS"))
  (shadow 'MotorStates (find-package "EPOS_MSGS")))
(unless (find-package "EPOS_MSGS::MOTORSTATES")
  (make-package "EPOS_MSGS::MOTORSTATES"))

(in-package "ROS")
;;//! \htmlinclude MotorStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass epos_msgs::MotorStates
  :super ros::object
  :slots (_header _states ))

(defmethod epos_msgs::MotorStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:states __states) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _states __states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; epos_msgs/MotorState[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; epos_msgs/MotorState[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; epos_msgs/MotorState[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance epos_msgs::MotorState :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get epos_msgs::MotorStates :md5sum-) "3ad89420af0daf166bb737efa2555173")
(setf (get epos_msgs::MotorStates :datatype-) "epos_msgs/MotorStates")
(setf (get epos_msgs::MotorStates :definition-)
      "
Header header

MotorState[] states

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: epos_msgs/MotorState

Header header

string motor_name

float64 position
float64 velocity
float64 current

")



(provide :epos_msgs/MotorStates "3ad89420af0daf166bb737efa2555173")


