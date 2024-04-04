;; Auto-generated. Do not edit!


(when (boundp 'st_pid::Path)
  (if (not (find-package "ST_PID"))
    (make-package "ST_PID"))
  (shadow 'Path (find-package "ST_PID")))
(unless (find-package "ST_PID::PATH")
  (make-package "ST_PID::PATH"))

(in-package "ROS")
;;//! \htmlinclude Path.msg.html


(defclass st_pid::Path
  :super ros::object
  :slots (_path_ ))

(defmethod st_pid::Path
  (:init
   (&key
    ((:path_ __path_) (make-array 0 :initial-element 0 :element-type :integer))
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
    ;; int32[] _path_
    (* 4    (length _path_)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _path_
     (write-long (length _path_) s)
     (dotimes (i (length _path_))
       (write-long (elt _path_ i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _path_
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_ (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _path_ i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get st_pid::Path :md5sum-) "a4482bbc958240ef8af7049ce9e00f0e")
(setf (get st_pid::Path :datatype-) "st_pid/Path")
(setf (get st_pid::Path :definition-)
      "int32[] path_
")



(provide :st_pid/Path "a4482bbc958240ef8af7049ce9e00f0e")


