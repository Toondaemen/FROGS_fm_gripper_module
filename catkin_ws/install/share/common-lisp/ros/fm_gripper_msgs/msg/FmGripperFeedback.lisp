; Auto-generated. Do not edit!


(cl:in-package fm_gripper_msgs-msg)


;//! \htmlinclude FmGripperFeedback.msg.html

(cl:defclass <FmGripperFeedback> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:float
    :initform 0.0))
)

(cl:defclass FmGripperFeedback (<FmGripperFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FmGripperFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FmGripperFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fm_gripper_msgs-msg:<FmGripperFeedback> is deprecated: use fm_gripper_msgs-msg:FmGripperFeedback instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <FmGripperFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:status-val is deprecated.  Use fm_gripper_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FmGripperFeedback>) ostream)
  "Serializes a message object of type '<FmGripperFeedback>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FmGripperFeedback>) istream)
  "Deserializes a message object of type '<FmGripperFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'status) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FmGripperFeedback>)))
  "Returns string type for a message object of type '<FmGripperFeedback>"
  "fm_gripper_msgs/FmGripperFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FmGripperFeedback)))
  "Returns string type for a message object of type 'FmGripperFeedback"
  "fm_gripper_msgs/FmGripperFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FmGripperFeedback>)))
  "Returns md5sum for a message object of type '<FmGripperFeedback>"
  "7e1ff3a25d34c3fba4479917ce302971")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FmGripperFeedback)))
  "Returns md5sum for a message object of type 'FmGripperFeedback"
  "7e1ff3a25d34c3fba4479917ce302971")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FmGripperFeedback>)))
  "Returns full string definition for message of type '<FmGripperFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float64 status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FmGripperFeedback)))
  "Returns full string definition for message of type 'FmGripperFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float64 status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FmGripperFeedback>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FmGripperFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'FmGripperFeedback
    (cl:cons ':status (status msg))
))
