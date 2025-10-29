; Auto-generated. Do not edit!


(cl:in-package robot_communication-msg)


;//! \htmlinclude velocity_comm.msg.html

(cl:defclass <velocity_comm> (roslisp-msg-protocol:ros-message)
  ((left_vel
    :reader left_vel
    :initarg :left_vel
    :type cl:float
    :initform 0.0)
   (right_vel
    :reader right_vel
    :initarg :right_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass velocity_comm (<velocity_comm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocity_comm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocity_comm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_communication-msg:<velocity_comm> is deprecated: use robot_communication-msg:velocity_comm instead.")))

(cl:ensure-generic-function 'left_vel-val :lambda-list '(m))
(cl:defmethod left_vel-val ((m <velocity_comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:left_vel-val is deprecated.  Use robot_communication-msg:left_vel instead.")
  (left_vel m))

(cl:ensure-generic-function 'right_vel-val :lambda-list '(m))
(cl:defmethod right_vel-val ((m <velocity_comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:right_vel-val is deprecated.  Use robot_communication-msg:right_vel instead.")
  (right_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocity_comm>) ostream)
  "Serializes a message object of type '<velocity_comm>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocity_comm>) istream)
  "Deserializes a message object of type '<velocity_comm>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocity_comm>)))
  "Returns string type for a message object of type '<velocity_comm>"
  "robot_communication/velocity_comm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocity_comm)))
  "Returns string type for a message object of type 'velocity_comm"
  "robot_communication/velocity_comm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocity_comm>)))
  "Returns md5sum for a message object of type '<velocity_comm>"
  "6276768e5df1d6b78b96e63931fb03fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocity_comm)))
  "Returns md5sum for a message object of type 'velocity_comm"
  "6276768e5df1d6b78b96e63931fb03fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocity_comm>)))
  "Returns full string definition for message of type '<velocity_comm>"
  (cl:format cl:nil "# velocity_comm.msg~%float64 left_vel~%float64 right_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocity_comm)))
  "Returns full string definition for message of type 'velocity_comm"
  (cl:format cl:nil "# velocity_comm.msg~%float64 left_vel~%float64 right_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocity_comm>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocity_comm>))
  "Converts a ROS message object to a list"
  (cl:list 'velocity_comm
    (cl:cons ':left_vel (left_vel msg))
    (cl:cons ':right_vel (right_vel msg))
))
