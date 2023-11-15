;; Auto-generated. Do not edit!


(when (boundp 'dmp::DMPPointStamp)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'DMPPointStamp (find-package "DMP")))
(unless (find-package "DMP::DMPPOINTSTAMP")
  (make-package "DMP::DMPPOINTSTAMP"))

(in-package "ROS")
;;//! \htmlinclude DMPPointStamp.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dmp::DMPPointStamp
  :super ros::object
  :slots (_head _positions ))

(defmethod dmp::DMPPointStamp
  (:init
   (&key
    ((:head __head) (instance std_msgs::Header :init))
    ((:positions __positions) 0.0)
    )
   (send-super :init)
   (setq _head __head)
   (setq _positions (float __positions))
   self)
  (:head
   (&rest __head)
   (if (keywordp (car __head))
       (send* _head __head)
     (progn
       (if __head (setq _head (car __head)))
       _head)))
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _head
    (send _head :serialization-length)
    ;; float64 _positions
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _head
       (send _head :serialize s)
     ;; float64 _positions
       (sys::poke _positions (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _head
     (send _head :deserialize buf ptr-) (incf ptr- (send _head :serialization-length))
   ;; float64 _positions
     (setq _positions (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dmp::DMPPointStamp :md5sum-) "db08163e791135724278324162c51a67")
(setf (get dmp::DMPPointStamp :datatype-) "dmp/DMPPointStamp")
(setf (get dmp::DMPPointStamp :definition-)
      "std_msgs/Header head
float64 positions



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



(provide :dmp/DMPPointStamp "db08163e791135724278324162c51a67")


