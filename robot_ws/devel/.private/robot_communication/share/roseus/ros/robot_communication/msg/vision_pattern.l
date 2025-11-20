;; Auto-generated. Do not edit!


(when (boundp 'robot_communication::vision_pattern)
  (if (not (find-package "ROBOT_COMMUNICATION"))
    (make-package "ROBOT_COMMUNICATION"))
  (shadow 'vision_pattern (find-package "ROBOT_COMMUNICATION")))
(unless (find-package "ROBOT_COMMUNICATION::VISION_PATTERN")
  (make-package "ROBOT_COMMUNICATION::VISION_PATTERN"))

(in-package "ROS")
;;//! \htmlinclude vision_pattern.msg.html


(defclass robot_communication::vision_pattern
  :super ros::object
  :slots (_curvature _offset ))

(defmethod robot_communication::vision_pattern
  (:init
   (&key
    ((:curvature __curvature) 0.0)
    ((:offset __offset) 0.0)
    )
   (send-super :init)
   (setq _curvature (float __curvature))
   (setq _offset (float __offset))
   self)
  (:curvature
   (&optional __curvature)
   (if __curvature (setq _curvature __curvature)) _curvature)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:serialization-length
   ()
   (+
    ;; float32 _curvature
    4
    ;; float32 _offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _curvature
       (sys::poke _curvature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset
       (sys::poke _offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _curvature
     (setq _curvature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset
     (setq _offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_communication::vision_pattern :md5sum-) "6f7fe8c9b73d40487b6bbeb43e558374")
(setf (get robot_communication::vision_pattern :datatype-) "robot_communication/vision_pattern")
(setf (get robot_communication::vision_pattern :definition-)
      "# vision_pattern.msg
float32 curvature
float32 offset
")



(provide :robot_communication/vision_pattern "6f7fe8c9b73d40487b6bbeb43e558374")


