;; Auto-generated. Do not edit!


(when (boundp 'epos_msgs::MotorState)
  (if (not (find-package "EPOS_MSGS"))
    (make-package "EPOS_MSGS"))
  (shadow 'MotorState (find-package "EPOS_MSGS")))
(unless (find-package "EPOS_MSGS::MOTORSTATE")
  (make-package "EPOS_MSGS::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass epos_msgs::MotorState
  :super ros::object
  :slots (_header _motor_name _position _velocity _current ))

(defmethod epos_msgs::MotorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:motor_name __motor_name) "")
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:current __current) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _motor_name (string __motor_name))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _current (float __current))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:motor_name
   (&optional __motor_name)
   (if __motor_name (setq _motor_name __motor_name)) _motor_name)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _motor_name
    4 (length _motor_name)
    ;; float64 _position
    8
    ;; float64 _velocity
    8
    ;; float64 _current
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _motor_name
       (write-long (length _motor_name) s) (princ _motor_name s)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _motor_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _motor_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get epos_msgs::MotorState :md5sum-) "8ea0c2787c1fbaef7888f7ab841a4cbc")
(setf (get epos_msgs::MotorState :datatype-) "epos_msgs/MotorState")
(setf (get epos_msgs::MotorState :definition-)
      "
Header header

string motor_name

float64 position
float64 velocity
float64 current

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

")



(provide :epos_msgs/MotorState "8ea0c2787c1fbaef7888f7ab841a4cbc")


