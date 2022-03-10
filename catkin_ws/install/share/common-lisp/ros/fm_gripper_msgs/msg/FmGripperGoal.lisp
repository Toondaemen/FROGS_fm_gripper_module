; Auto-generated. Do not edit!


(cl:in-package fm_gripper_msgs-msg)


;//! \htmlinclude FmGripperGoal.msg.html

(cl:defclass <FmGripperGoal> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0)
   (force
    :reader force
    :initarg :force
    :type cl:integer
    :initform 0)
   (calibrate
    :reader calibrate
    :initarg :calibrate
    :type cl:boolean
    :initform cl:nil)
   (reset
    :reader reset
    :initarg :reset
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FmGripperGoal (<FmGripperGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FmGripperGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FmGripperGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fm_gripper_msgs-msg:<FmGripperGoal> is deprecated: use fm_gripper_msgs-msg:FmGripperGoal instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <FmGripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:distance-val is deprecated.  Use fm_gripper_msgs-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <FmGripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:velocity-val is deprecated.  Use fm_gripper_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <FmGripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:force-val is deprecated.  Use fm_gripper_msgs-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'calibrate-val :lambda-list '(m))
(cl:defmethod calibrate-val ((m <FmGripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:calibrate-val is deprecated.  Use fm_gripper_msgs-msg:calibrate instead.")
  (calibrate m))

(cl:ensure-generic-function 'reset-val :lambda-list '(m))
(cl:defmethod reset-val ((m <FmGripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fm_gripper_msgs-msg:reset-val is deprecated.  Use fm_gripper_msgs-msg:reset instead.")
  (reset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FmGripperGoal>) ostream)
  "Serializes a message object of type '<FmGripperGoal>"
  (cl:let* ((signed (cl:slot-value msg 'distance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'force)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calibrate) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FmGripperGoal>) istream)
  "Deserializes a message object of type '<FmGripperGoal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'force) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'calibrate) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reset) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FmGripperGoal>)))
  "Returns string type for a message object of type '<FmGripperGoal>"
  "fm_gripper_msgs/FmGripperGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FmGripperGoal)))
  "Returns string type for a message object of type 'FmGripperGoal"
  "fm_gripper_msgs/FmGripperGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FmGripperGoal>)))
  "Returns md5sum for a message object of type '<FmGripperGoal>"
  "fff58c2b7a2066e0d202de176f11f4bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FmGripperGoal)))
  "Returns md5sum for a message object of type 'FmGripperGoal"
  "fff58c2b7a2066e0d202de176f11f4bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FmGripperGoal>)))
  "Returns full string definition for message of type '<FmGripperGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal         ~%int64 distance~%int64 velocity~%int64 force~%bool calibrate~%bool reset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FmGripperGoal)))
  "Returns full string definition for message of type 'FmGripperGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal         ~%int64 distance~%int64 velocity~%int64 force~%bool calibrate~%bool reset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FmGripperGoal>))
  (cl:+ 0
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FmGripperGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FmGripperGoal
    (cl:cons ':distance (distance msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':force (force msg))
    (cl:cons ':calibrate (calibrate msg))
    (cl:cons ':reset (reset msg))
))
