
(cl:in-package :asdf)

(defsystem "track_drive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "laneinfo" :depends-on ("_package_laneinfo"))
    (:file "_package_laneinfo" :depends-on ("_package"))
  ))