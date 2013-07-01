
(cl:in-package :asdf)

(defsystem "stereo_click-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :stereo_click-msg
)
  :components ((:file "_package")
    (:file "ConvertPoint" :depends-on ("_package_ConvertPoint"))
    (:file "_package_ConvertPoint" :depends-on ("_package"))
    (:file "ConvertPoints" :depends-on ("_package_ConvertPoints"))
    (:file "_package_ConvertPoints" :depends-on ("_package"))
  ))