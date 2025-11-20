; Auto-generated. Do not edit!


(cl:in-package robot_communication-msg)


;//! \htmlinclude vision_pattern.msg.html

(cl:defclass <vision_pattern> (roslisp-msg-protocol:ros-message)
  ((curvature
    :reader curvature
    :initarg :curvature
    :type cl:float
    :initform 0.0)
   (offset
    :reader offset
    :initarg :offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass vision_pattern (<vision_pattern>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vision_pattern>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vision_pattern)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_communication-msg:<vision_pattern> is deprecated: use robot_communication-msg:vision_pattern instead.")))

(cl:ensure-generic-function 'curvature-val :lambda-list '(m))
(cl:defmethod curvature-val ((m <vision_pattern>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:curvature-val is deprecated.  Use robot_communication-msg:curvature instead.")
  (curvature m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <vision_pattern>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:offset-val is deprecated.  Use robot_communication-msg:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vision_pattern>) ostream)
  "Serializes a message object of type '<vision_pattern>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'curvature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vision_pattern>) istream)
  "Deserializes a message object of type '<vision_pattern>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curvature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vision_pattern>)))
  "Returns string type for a message object of type '<vision_pattern>"
  "robot_communication/vision_pattern")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vision_pattern)))
  "Returns string type for a message object of type 'vision_pattern"
  "robot_communication/vision_pattern")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vision_pattern>)))
  "Returns md5sum for a message object of type '<vision_pattern>"
  "6f7fe8c9b73d40487b6bbeb43e558374")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vision_pattern)))
  "Returns md5sum for a message object of type 'vision_pattern"
  "6f7fe8c9b73d40487b6bbeb43e558374")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vision_pattern>)))
  "Returns full string definition for message of type '<vision_pattern>"
  (cl:format cl:nil "# vision_pattern.msg~%float32 curvature~%float32 offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vision_pattern)))
  "Returns full string definition for message of type 'vision_pattern"
  (cl:format cl:nil "# vision_pattern.msg~%float32 curvature~%float32 offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vision_pattern>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vision_pattern>))
  "Converts a ROS message object to a list"
  (cl:list 'vision_pattern
    (cl:cons ':curvature (curvature msg))
    (cl:cons ':offset (offset msg))
))
