
(cl:in-package :asdf)

(defsystem "Human_intention-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Skeleton" :depends-on ("_package_Skeleton"))
    (:file "_package_Skeleton" :depends-on ("_package"))
    (:file "Goal" :depends-on ("_package_Goal"))
    (:file "_package_Goal" :depends-on ("_package"))
    (:file "PerceptInfo" :depends-on ("_package_PerceptInfo"))
    (:file "_package_PerceptInfo" :depends-on ("_package"))
    (:file "EnableJointGroup" :depends-on ("_package_EnableJointGroup"))
    (:file "_package_EnableJointGroup" :depends-on ("_package"))
  ))