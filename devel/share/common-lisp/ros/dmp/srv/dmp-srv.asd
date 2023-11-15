
(cl:in-package :asdf)

(defsystem "dmp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dmp-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "GetDMPPlan" :depends-on ("_package_GetDMPPlan"))
    (:file "_package_GetDMPPlan" :depends-on ("_package"))
    (:file "GetDMPStepPlan" :depends-on ("_package_GetDMPStepPlan"))
    (:file "_package_GetDMPStepPlan" :depends-on ("_package"))
    (:file "GoalToPath" :depends-on ("_package_GoalToPath"))
    (:file "_package_GoalToPath" :depends-on ("_package"))
    (:file "LearnDMPFromDemo" :depends-on ("_package_LearnDMPFromDemo"))
    (:file "_package_LearnDMPFromDemo" :depends-on ("_package"))
    (:file "SetActiveDMP" :depends-on ("_package_SetActiveDMP"))
    (:file "_package_SetActiveDMP" :depends-on ("_package"))
  ))