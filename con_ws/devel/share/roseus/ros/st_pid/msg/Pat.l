;; Auto-generated. Do not edit!


(when (boundp 'st_pid::Pat)
  (if (not (find-package "ST_PID"))
    (make-package "ST_PID"))
  (shadow 'Pat (find-package "ST_PID")))
(unless (find-package "ST_PID::PAT")
  (make-package "ST_PID::PAT"))

(in-package "ROS")
;;//! \htmlinclude Pat.msg.html


(defclass st_pid::Pat
  :super ros::object
  :slots (_path_ ))

(defmethod st_pid::Pat
  (:init
   (&key
    ((:path_ __path_) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _path_ __path_)
   self)
  (:path_
   (&optional __path_)
   (if __path_ (setq _path_ __path_)) _path_)
  (:serialization-length
   ()
   (+
    ;; uint8[] _path_
    (* 1    (length _path_)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _path_
     (write-long (length _path_) s)
     (princ _path_ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _path_
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _path_ (make-array n :element-type :char))
     (replace _path_ buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get st_pid::Pat :md5sum-) "7e7e4cb758a1a3e10b6dab1976b351ef")
(setf (get st_pid::Pat :datatype-) "st_pid/Pat")
(setf (get st_pid::Pat :definition-)
      "uint8[] path_
")



(provide :st_pid/Pat "7e7e4cb758a1a3e10b6dab1976b351ef")


