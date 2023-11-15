;; Auto-generated. Do not edit!


(when (boundp 'dmp::GoalToPath)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'GoalToPath (find-package "DMP")))
(unless (find-package "DMP::GOALTOPATH")
  (make-package "DMP::GOALTOPATH"))
(unless (find-package "DMP::GOALTOPATHREQUEST")
  (make-package "DMP::GOALTOPATHREQUEST"))
(unless (find-package "DMP::GOALTOPATHRESPONSE")
  (make-package "DMP::GOALTOPATHRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass dmp::GoalToPathRequest
  :super ros::object
  :slots (_Start _GoalIndex ))

(defmethod dmp::GoalToPathRequest
  (:init
   (&key
    ((:Start __Start) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:GoalIndex __GoalIndex) 0)
    )
   (send-super :init)
   (setq _Start __Start)
   (setq _GoalIndex (round __GoalIndex))
   self)
  (:Start
   (&optional __Start)
   (if __Start (setq _Start __Start)) _Start)
  (:GoalIndex
   (&optional __GoalIndex)
   (if __GoalIndex (setq _GoalIndex __GoalIndex)) _GoalIndex)
  (:serialization-length
   ()
   (+
    ;; float64[] _Start
    (* 8    (length _Start)) 4
    ;; int32 _GoalIndex
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _Start
     (write-long (length _Start) s)
     (dotimes (i (length _Start))
       (sys::poke (elt _Start i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _GoalIndex
       (write-long _GoalIndex s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _Start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Start (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Start i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _GoalIndex
     (setq _GoalIndex (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dmp::GoalToPathResponse
  :super ros::object
  :slots (_Path ))

(defmethod dmp::GoalToPathResponse
  (:init
   (&key
    ((:Path __Path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _Path __Path)
   self)
  (:Path
   (&rest __Path)
   (if (keywordp (car __Path))
       (send* _Path __Path)
     (progn
       (if __Path (setq _Path (car __Path)))
       _Path)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _Path
    (send _Path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _Path
       (send _Path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _Path
     (send _Path :deserialize buf ptr-) (incf ptr- (send _Path :serialization-length))
   ;;
   self)
  )

(defclass dmp::GoalToPath
  :super ros::object
  :slots ())

(setf (get dmp::GoalToPath :md5sum-) "61b7759cacf0fb0b224df3e235dda576")
(setf (get dmp::GoalToPath :datatype-) "dmp/GoalToPath")
(setf (get dmp::GoalToPath :request) dmp::GoalToPathRequest)
(setf (get dmp::GoalToPath :response) dmp::GoalToPathResponse)

(defmethod dmp::GoalToPathRequest
  (:response () (instance dmp::GoalToPathResponse :init)))

(setf (get dmp::GoalToPathRequest :md5sum-) "61b7759cacf0fb0b224df3e235dda576")
(setf (get dmp::GoalToPathRequest :datatype-) "dmp/GoalToPathRequest")
(setf (get dmp::GoalToPathRequest :definition-)
      "float64[] Start

int32 GoalIndex
---

nav_msgs/Path Path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get dmp::GoalToPathResponse :md5sum-) "61b7759cacf0fb0b224df3e235dda576")
(setf (get dmp::GoalToPathResponse :datatype-) "dmp/GoalToPathResponse")
(setf (get dmp::GoalToPathResponse :definition-)
      "float64[] Start

int32 GoalIndex
---

nav_msgs/Path Path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :dmp/GoalToPath "61b7759cacf0fb0b224df3e235dda576")


