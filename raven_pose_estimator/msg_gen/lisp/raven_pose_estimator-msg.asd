
(cl:in-package :asdf)

(defsystem "raven_pose_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointsAndPoses" :depends-on ("_package_JointsAndPoses"))
    (:file "_package_JointsAndPoses" :depends-on ("_package"))
  ))