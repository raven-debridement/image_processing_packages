
(cl:in-package :asdf)

(defsystem "raven_pose_estimator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ThreshRed" :depends-on ("_package_ThreshRed"))
    (:file "_package_ThreshRed" :depends-on ("_package"))
  ))