
(cl:in-package :asdf)

(defsystem "robot_communication-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "encoder_comm" :depends-on ("_package_encoder_comm"))
    (:file "_package_encoder_comm" :depends-on ("_package"))
    (:file "velocity_comm" :depends-on ("_package_velocity_comm"))
    (:file "_package_velocity_comm" :depends-on ("_package"))
  ))