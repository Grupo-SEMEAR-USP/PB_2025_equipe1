; Auto-generated. Do not edit!


(cl:in-package robot_communication-msg)


;//! \htmlinclude encoder_comm.msg.html

(cl:defclass <encoder_comm> (roslisp-msg-protocol:ros-message)
  ((left_enc
    :reader left_enc
    :initarg :left_enc
    :type cl:float
    :initform 0.0)
   (right_enc
    :reader right_enc
    :initarg :right_enc
    :type cl:float
    :initform 0.0))
)

(cl:defclass encoder_comm (<encoder_comm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <encoder_comm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'encoder_comm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_communication-msg:<encoder_comm> is deprecated: use robot_communication-msg:encoder_comm instead.")))

(cl:ensure-generic-function 'left_enc-val :lambda-list '(m))
(cl:defmethod left_enc-val ((m <encoder_comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:left_enc-val is deprecated.  Use robot_communication-msg:left_enc instead.")
  (left_enc m))

(cl:ensure-generic-function 'right_enc-val :lambda-list '(m))
(cl:defmethod right_enc-val ((m <encoder_comm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_communication-msg:right_enc-val is deprecated.  Use robot_communication-msg:right_enc instead.")
  (right_enc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <encoder_comm>) ostream)
  "Serializes a message object of type '<encoder_comm>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_enc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_enc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <encoder_comm>) istream)
  "Deserializes a message object of type '<encoder_comm>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_enc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_enc) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<encoder_comm>)))
  "Returns string type for a message object of type '<encoder_comm>"
  "robot_communication/encoder_comm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'encoder_comm)))
  "Returns string type for a message object of type 'encoder_comm"
  "robot_communication/encoder_comm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<encoder_comm>)))
  "Returns md5sum for a message object of type '<encoder_comm>"
  "62d5c2afe8311d58c8b7b83a4571d468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'encoder_comm)))
  "Returns md5sum for a message object of type 'encoder_comm"
  "62d5c2afe8311d58c8b7b83a4571d468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<encoder_comm>)))
  "Returns full string definition for message of type '<encoder_comm>"
  (cl:format cl:nil "# encoder_comm.msg~%float32 left_enc~%float32 right_enc~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'encoder_comm)))
  "Returns full string definition for message of type 'encoder_comm"
  (cl:format cl:nil "# encoder_comm.msg~%float32 left_enc~%float32 right_enc~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <encoder_comm>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <encoder_comm>))
  "Converts a ROS message object to a list"
  (cl:list 'encoder_comm
    (cl:cons ':left_enc (left_enc msg))
    (cl:cons ':right_enc (right_enc msg))
))
