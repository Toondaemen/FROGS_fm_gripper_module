; Auto-generated. Do not edit!


(cl:in-package fm_gripper_msgs-msg)


;//! \htmlinclude FmGripperResult.msg.html

(cl:defclass <FmGripperResult> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass FmGripperResult (<FmGripperResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FmGripperResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FmGripperResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fm_gripper_msgs-msg:<FmGripperResult> is deprecated: use fm_gripper_msgs-msg:FmGripperResult instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <FmGripperResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:result-val is deprecated.  Use fm_gripper_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FmGripperResult>) ostream)
  "Serializes a message object of type '<FmGripperResult>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FmGripperResult>) istream)
  "Deserializes a message object of type '<FmGripperResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FmGripperResult>)))
  "Returns string type for a message object of type '<FmGripperResult>"
  "fm_gripper_msgs/FmGripperResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FmGripperResult)))
  "Returns string type for a message object of type 'FmGripperResult"
  "fm_gripper_msgs/FmGripperResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FmGripperResult>)))
  "Returns md5sum for a message object of type '<FmGripperResult>"
  "9b05623554ab950ed237d43d45f0b4dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FmGripperResult)))
  "Returns md5sum for a message object of type 'FmGripperResult"
  "9b05623554ab950ed237d43d45f0b4dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FmGripperResult>)))
  "Returns full string definition for message of type '<FmGripperResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%int64 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FmGripperResult)))
  "Returns full string definition for message of type 'FmGripperResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%int64 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FmGripperResult>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FmGripperResult>))
  "Converts a ROS message object to a list"
  (cl:list 'FmGripperResult
    (cl:cons ':result (result msg))
))
