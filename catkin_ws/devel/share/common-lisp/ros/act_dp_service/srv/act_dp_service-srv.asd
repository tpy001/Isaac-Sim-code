
(cl:in-package :asdf)

(defsystem "act_dp_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :act_dp_service-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "get_action" :depends-on ("_package_get_action"))
    (:file "_package_get_action" :depends-on ("_package"))
  ))