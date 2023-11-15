
(cl:in-package :asdf)

(defsystem "epos_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MotorState" :depends-on ("_package_MotorState"))
    (:file "_package_MotorState" :depends-on ("_package"))
    (:file "MotorStates" :depends-on ("_package_MotorStates"))
    (:file "_package_MotorStates" :depends-on ("_package"))
  ))