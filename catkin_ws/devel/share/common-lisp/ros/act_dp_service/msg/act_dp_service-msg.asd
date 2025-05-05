
(cl:in-package :asdf)

(defsystem "act_dp_service-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RawData" :depends-on ("_package_RawData"))
    (:file "_package_RawData" :depends-on ("_package"))
  ))