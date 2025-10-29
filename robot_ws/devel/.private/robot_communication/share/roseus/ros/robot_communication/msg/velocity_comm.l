;; Auto-generated. Do not edit!


(when (boundp 'robot_communication::velocity_comm)
  (if (not (find-package "ROBOT_COMMUNICATION"))
    (make-package "ROBOT_COMMUNICATION"))
  (shadow 'velocity_comm (find-package "ROBOT_COMMUNICATION")))
(unless (find-package "ROBOT_COMMUNICATION::VELOCITY_COMM")
  (make-package "ROBOT_COMMUNICATION::VELOCITY_COMM"))

(in-package "ROS")
;;//! \htmlinclude velocity_comm.msg.html


(defclass robot_communication::velocity_comm
  :super ros::object
  :slots (_left_vel _right_vel ))

(defmethod robot_communication::velocity_comm
  (:init
   (&key
    ((:left_vel __left_vel) 0.0)
    ((:right_vel __right_vel) 0.0)
    )
   (send-super :init)
   (setq _left_vel (float __left_vel))
   (setq _right_vel (float __right_vel))
   self)
  (:left_vel
   (&optional __left_vel)
   (if __left_vel (setq _left_vel __left_vel)) _left_vel)
  (:right_vel
   (&optional __right_vel)
   (if __right_vel (setq _right_vel __right_vel)) _right_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _left_vel
    8
    ;; float64 _right_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _left_vel
       (sys::poke _left_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_vel
       (sys::poke _right_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _left_vel
     (setq _left_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_vel
     (setq _right_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robot_communication::velocity_comm :md5sum-) "6276768e5df1d6b78b96e63931fb03fc")
(setf (get robot_communication::velocity_comm :datatype-) "robot_communication/velocity_comm")
(setf (get robot_communication::velocity_comm :definition-)
      "# velocity_comm.msg
float64 left_vel
float64 right_vel
")



(provide :robot_communication/velocity_comm "6276768e5df1d6b78b96e63931fb03fc")


