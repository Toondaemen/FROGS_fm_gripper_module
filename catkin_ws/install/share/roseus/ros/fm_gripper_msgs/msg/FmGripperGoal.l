;; Auto-generated. Do not edit!


(when (boundp 'fm_gripper_msgs::FmGripperGoal)
  (if (not (find-package "FM_GRIPPER_MSGS"))
    (make-package "FM_GRIPPER_MSGS"))
  (shadow 'FmGripperGoal (find-package "FM_GRIPPER_MSGS")))
(unless (find-package "FM_GRIPPER_MSGS::FMGRIPPERGOAL")
  (make-package "FM_GRIPPER_MSGS::FMGRIPPERGOAL"))

(in-package "ROS")
;;//! \htmlinclude FmGripperGoal.msg.html


(defclass fm_gripper_msgs::FmGripperGoal
  :super ros::object
  :slots (_distance _velocity _force _calibrate _reset ))

(defmethod fm_gripper_msgs::FmGripperGoal
  (:init
   (&key
    ((:distance __distance) 0)
    ((:velocity __velocity) 0)
    ((:force __force) 0)
    ((:calibrate __calibrate) nil)
    ((:reset __reset) nil)
    )
   (send-super :init)
   (setq _distance (round __distance))
   (setq _velocity (round __velocity))
   (setq _force (round __force))
   (setq _calibrate __calibrate)
   (setq _reset __reset)
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:calibrate
   (&optional __calibrate)
   (if __calibrate (setq _calibrate __calibrate)) _calibrate)
  (:reset
   (&optional __reset)
   (if __reset (setq _reset __reset)) _reset)
  (:serialization-length
   ()
   (+
    ;; int64 _distance
    8
    ;; int64 _velocity
    8
    ;; int64 _force
    8
    ;; bool _calibrate
    1
    ;; bool _reset
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _distance (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _distance) (= (length (_distance . bv)) 2)) ;; bignum
              (write-long (ash (elt (_distance . bv) 0) 0) s)
              (write-long (ash (elt (_distance . bv) 1) -1) s))
             ((and (class _distance) (= (length (_distance . bv)) 1)) ;; big1
              (write-long (elt (_distance . bv) 0) s)
              (write-long (if (>= _distance 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _distance s)(write-long (if (>= _distance 0) 0 #xffffffff) s)))
     ;; int64 _velocity
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _velocity (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _velocity) (= (length (_velocity . bv)) 2)) ;; bignum
              (write-long (ash (elt (_velocity . bv) 0) 0) s)
              (write-long (ash (elt (_velocity . bv) 1) -1) s))
             ((and (class _velocity) (= (length (_velocity . bv)) 1)) ;; big1
              (write-long (elt (_velocity . bv) 0) s)
              (write-long (if (>= _velocity 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _velocity s)(write-long (if (>= _velocity 0) 0 #xffffffff) s)))
     ;; int64 _force
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _force (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _force) (= (length (_force . bv)) 2)) ;; bignum
              (write-long (ash (elt (_force . bv) 0) 0) s)
              (write-long (ash (elt (_force . bv) 1) -1) s))
             ((and (class _force) (= (length (_force . bv)) 1)) ;; big1
              (write-long (elt (_force . bv) 0) s)
              (write-long (if (>= _force 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _force s)(write-long (if (>= _force 0) 0 #xffffffff) s)))
     ;; bool _calibrate
       (if _calibrate (write-byte -1 s) (write-byte 0 s))
     ;; bool _reset
       (if _reset (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
      (setf _distance (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _distance (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _velocity
#+(or :alpha :irix6 :x86_64)
      (setf _velocity (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _velocity (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _force
#+(or :alpha :irix6 :x86_64)
      (setf _force (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _force (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _calibrate
     (setq _calibrate (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reset
     (setq _reset (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fm_gripper_msgs::FmGripperGoal :md5sum-) "fff58c2b7a2066e0d202de176f11f4bd")
(setf (get fm_gripper_msgs::FmGripperGoal :datatype-) "fm_gripper_msgs/FmGripperGoal")
(setf (get fm_gripper_msgs::FmGripperGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#goal         
int64 distance
int64 velocity
int64 force
bool calibrate
bool reset

")



(provide :fm_gripper_msgs/FmGripperGoal "fff58c2b7a2066e0d202de176f11f4bd")


