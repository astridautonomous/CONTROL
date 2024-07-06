
(cl:in-package :asdf)

(defsystem "st_pid-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Pat" :depends-on ("_package_Pat"))
    (:file "_package_Pat" :depends-on ("_package"))
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
  ))