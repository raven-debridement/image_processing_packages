
(cl:in-package :asdf)

(defsystem "stereo_click-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ClickPoint" :depends-on ("_package_ClickPoint"))
    (:file "_package_ClickPoint" :depends-on ("_package"))
  ))