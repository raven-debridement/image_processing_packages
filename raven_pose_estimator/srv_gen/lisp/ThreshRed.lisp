; Auto-generated. Do not edit!


(cl:in-package raven_pose_estimator-srv)


;//! \htmlinclude ThreshRed-request.msg.html

(cl:defclass <ThreshRed-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ThreshRed-request (<ThreshRed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThreshRed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThreshRed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raven_pose_estimator-srv:<ThreshRed-request> is deprecated: use raven_pose_estimator-srv:ThreshRed-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ThreshRed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raven_pose_estimator-srv:input-val is deprecated.  Use raven_pose_estimator-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThreshRed-request>) ostream)
  "Serializes a message object of type '<ThreshRed-request>"
  (cl:let* ((signed (cl:slot-value msg 'input)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThreshRed-request>) istream)
  "Deserializes a message object of type '<ThreshRed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThreshRed-request>)))
  "Returns string type for a service object of type '<ThreshRed-request>"
  "raven_pose_estimator/ThreshRedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThreshRed-request)))
  "Returns string type for a service object of type 'ThreshRed-request"
  "raven_pose_estimator/ThreshRedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThreshRed-request>)))
  "Returns md5sum for a message object of type '<ThreshRed-request>"
  "4ba1d3acdddc600edccce3683403edb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThreshRed-request)))
  "Returns md5sum for a message object of type 'ThreshRed-request"
  "4ba1d3acdddc600edccce3683403edb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThreshRed-request>)))
  "Returns full string definition for message of type '<ThreshRed-request>"
  (cl:format cl:nil "int8 input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThreshRed-request)))
  "Returns full string definition for message of type 'ThreshRed-request"
  (cl:format cl:nil "int8 input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThreshRed-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThreshRed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ThreshRed-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ThreshRed-response.msg.html

(cl:defclass <ThreshRed-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ThreshRed-response (<ThreshRed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThreshRed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThreshRed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raven_pose_estimator-srv:<ThreshRed-response> is deprecated: use raven_pose_estimator-srv:ThreshRed-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ThreshRed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raven_pose_estimator-srv:output-val is deprecated.  Use raven_pose_estimator-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThreshRed-response>) ostream)
  "Serializes a message object of type '<ThreshRed-response>"
  (cl:let* ((signed (cl:slot-value msg 'output)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThreshRed-response>) istream)
  "Deserializes a message object of type '<ThreshRed-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThreshRed-response>)))
  "Returns string type for a service object of type '<ThreshRed-response>"
  "raven_pose_estimator/ThreshRedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThreshRed-response)))
  "Returns string type for a service object of type 'ThreshRed-response"
  "raven_pose_estimator/ThreshRedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThreshRed-response>)))
  "Returns md5sum for a message object of type '<ThreshRed-response>"
  "4ba1d3acdddc600edccce3683403edb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThreshRed-response)))
  "Returns md5sum for a message object of type 'ThreshRed-response"
  "4ba1d3acdddc600edccce3683403edb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThreshRed-response>)))
  "Returns full string definition for message of type '<ThreshRed-response>"
  (cl:format cl:nil "int8 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThreshRed-response)))
  "Returns full string definition for message of type 'ThreshRed-response"
  (cl:format cl:nil "int8 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThreshRed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThreshRed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ThreshRed-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ThreshRed)))
  'ThreshRed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ThreshRed)))
  'ThreshRed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThreshRed)))
  "Returns string type for a service object of type '<ThreshRed>"
  "raven_pose_estimator/ThreshRed")