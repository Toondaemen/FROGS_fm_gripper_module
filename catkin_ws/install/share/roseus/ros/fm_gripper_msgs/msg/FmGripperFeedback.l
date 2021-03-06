;; Auto-generated. Do not edit!


(when (boundp 'fm_gripper_msgs::FmGripperFeedback)
  (if (not (find-package "FM_GRIPPER_MSGS"))
    (make-package "FM_GRIPPER_MSGS"))
  (shadow 'FmGripperFeedback (find-package "FM_GRIPPER_MSGS")))
(unless (find-package "FM_GRIPPER_MSGS::FMGRIPPERFEEDBACK")
  (make-package "FM_GRIPPER_MSGS::FMGRIPPERFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude FmGripperFeedback.msg.html


(defclass fm_gripper_msgs::FmGripperFeedback
  :super ros::object
  :slots (_status ))

(defmethod fm_gripper_msgs::FmGripperFeedback
  (:init
   (&key
    ((:status __status) 0.0)
    )
   (send-super :init)
   (setq _status (float __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; float64 _status
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _status
       (sys::poke _status (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _status
     (setq _status (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fm_gripper_msgs::FmGripperFeedback :md5sum-) "7e1ff3a25d34c3fba4479917ce302971")
(setf (get fm_gripper_msgs::FmGripperFeedback :datatype-) "fm_gripper_msgs/FmGripperFeedback")
(setf (get fm_gripper_msgs::FmGripperFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#feedback
float64 status



")



(provide :fm_gripper_msgs/FmGripperFeedback "7e1ff3a25d34c3fba4479917ce302971")


