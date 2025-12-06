;; Auto-generated. Do not edit!


(when (boundp 'robot_communication::encoder_comm)
  (if (not (find-package "ROBOT_COMMUNICATION"))
    (make-package "ROBOT_COMMUNICATION"))
  (shadow 'encoder_comm (find-package "ROBOT_COMMUNICATION")))
(unless (find-package "ROBOT_COMMUNICATION::ENCODER_COMM")
  (make-package "ROBOT_COMMUNICATION::ENCODER_COMM"))

(in-package "ROS")
;;//! \htmlinclude encoder_comm.msg.html


(defclass robot_communication::encoder_comm
  :super ros::object
  :slots (_left_enc _right_enc ))

(defmethod robot_communication::encoder_comm
  (:init
   (&key
    ((:left_enc __left_enc) 0.0)
    ((:right_enc __right_enc) 0.0)
    )
   (send-super :init)
   (setq _left_enc (float __left_enc))
   (setq _right_enc (float __right_enc))
   self)
  (:left_enc
   (&optional __left_enc)
   (if __left_enc (setq _left_enc __left_enc)) _left_enc)
  (:right_enc
   (&optional __right_enc)
   (if __right_enc (setq _right_enc __right_enc)) _right_enc)
  (:serialization-length
   ()
   (+
    ;; float32 _left_enc
    4
    ;; float32 _right_enc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_enc
       (sys::poke _left_enc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_enc
       (sys::poke _right_enc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_enc
     (setq _left_enc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_enc
     (setq _right_enc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_communication::encoder_comm :md5sum-) "62d5c2afe8311d58c8b7b83a4571d468")
(setf (get robot_communication::encoder_comm :datatype-) "robot_communication/encoder_comm")
(setf (get robot_communication::encoder_comm :definition-)
      "# encoder_comm.msg
float32 left_enc
float32 right_enc
")



(provide :robot_communication/encoder_comm "62d5c2afe8311d58c8b7b83a4571d468")


