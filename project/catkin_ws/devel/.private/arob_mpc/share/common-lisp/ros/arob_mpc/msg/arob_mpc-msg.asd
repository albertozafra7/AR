
(cl:in-package :asdf)

(defsystem "arob_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "vector_poses" :depends-on ("_package_vector_poses"))
    (:file "_package_vector_poses" :depends-on ("_package"))
  ))