;; Auto-generated. Do not edit!


(when (boundp 'track_drive::laneinfo)
  (if (not (find-package "TRACK_DRIVE"))
    (make-package "TRACK_DRIVE"))
  (shadow 'laneinfo (find-package "TRACK_DRIVE")))
(unless (find-package "TRACK_DRIVE::LANEINFO")
  (make-package "TRACK_DRIVE::LANEINFO"))

(in-package "ROS")
;;//! \htmlinclude laneinfo.msg.html


(defclass track_drive::laneinfo
  :super ros::object
  :slots (_left_x _left_y _left_slope _right_x _right_y _right_slope ))

(defmethod track_drive::laneinfo
  (:init
   (&key
    ((:left_x __left_x) 0.0)
    ((:left_y __left_y) 0.0)
    ((:left_slope __left_slope) 0.0)
    ((:right_x __right_x) 0.0)
    ((:right_y __right_y) 0.0)
    ((:right_slope __right_slope) 0.0)
    )
   (send-super :init)
   (setq _left_x (float __left_x))
   (setq _left_y (float __left_y))
   (setq _left_slope (float __left_slope))
   (setq _right_x (float __right_x))
   (setq _right_y (float __right_y))
   (setq _right_slope (float __right_slope))
   self)
  (:left_x
   (&optional __left_x)
   (if __left_x (setq _left_x __left_x)) _left_x)
  (:left_y
   (&optional __left_y)
   (if __left_y (setq _left_y __left_y)) _left_y)
  (:left_slope
   (&optional __left_slope)
   (if __left_slope (setq _left_slope __left_slope)) _left_slope)
  (:right_x
   (&optional __right_x)
   (if __right_x (setq _right_x __right_x)) _right_x)
  (:right_y
   (&optional __right_y)
   (if __right_y (setq _right_y __right_y)) _right_y)
  (:right_slope
   (&optional __right_slope)
   (if __right_slope (setq _right_slope __right_slope)) _right_slope)
  (:serialization-length
   ()
   (+
    ;; float32 _left_x
    4
    ;; float32 _left_y
    4
    ;; float32 _left_slope
    4
    ;; float32 _right_x
    4
    ;; float32 _right_y
    4
    ;; float32 _right_slope
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_x
       (sys::poke _left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_y
       (sys::poke _left_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_slope
       (sys::poke _left_slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_x
       (sys::poke _right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_y
       (sys::poke _right_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_slope
       (sys::poke _right_slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_x
     (setq _left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_y
     (setq _left_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_slope
     (setq _left_slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_x
     (setq _right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_y
     (setq _right_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_slope
     (setq _right_slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get track_drive::laneinfo :md5sum-) "44a42243e734addbc7ddd0e00184e0ab")
(setf (get track_drive::laneinfo :datatype-) "track_drive/laneinfo")
(setf (get track_drive::laneinfo :definition-)
      "float32 left_x
float32 left_y
float32 left_slope
float32 right_x
float32 right_y
float32 right_slope
")



(provide :track_drive/laneinfo "44a42243e734addbc7ddd0e00184e0ab")


