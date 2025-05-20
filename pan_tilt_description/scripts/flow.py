#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OpticalFlowTracker:
    def __init__(self):
        rospy.init_node('optical_flow_tracker')
        self.bridge = CvBridge()
        self.frame = None
        self.old_gray = None
        self.mask = None
        self.roi_box = None
        self.initialized = False
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        cv2.namedWindow("Optical Flow")
        cv2.setMouseCallback("Optical Flow", self.select_roi)

        rospy.Subscriber("/datavideo/video", Image, self.image_callback)
        rospy.loginfo("📌 Esperando imagen... Haz una selección con el mouse.")

        self.run()

    def select_roi(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.frame is not None:
            r = cv2.selectROI("Optical Flow", self.frame, fromCenter=False, showCrosshair=True)
            x, y, w, h = map(int, r)
            if w > 0 and h > 0:
                self.roi_box = (x, y, w, h)
                self.old_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                roi = self.old_gray[y:y+h, x:x+w]
                self.p0 = cv2.goodFeaturesToTrack(roi, mask=None, maxCorners=100, qualityLevel=0.3, minDistance=7)
                if self.p0 is not None:
                    self.p0 += np.array([[x, y]], dtype=np.float32)  # mover al sistema global
                    self.mask = np.zeros_like(self.frame)
                    self.initialized = True
                    rospy.loginfo("🎯 ROI seleccionada, comenzando seguimiento.")

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"⚠️ Error al convertir imagen: {e}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            frame_disp = self.frame.copy()

            if self.initialized and self.old_gray is not None:
                frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)

                if p1 is not None and st is not None:
                    good_new = p1[st == 1]
                    good_old = self.p0[st == 1]

                    # Dibujar puntos y líneas
                    for new, old in zip(good_new, good_old):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                        frame_disp = cv2.circle(frame_disp, (int(a), int(b)), 5, (0, 0, 255), -1)

                    frame_disp = cv2.add(frame_disp, self.mask)

                    # Bounding box del nuevo conjunto de puntos
                    x_min, y_min = np.min(good_new, axis=0).astype(int)
                    x_max, y_max = np.max(good_new, axis=0).astype(int)
                    w = x_max - x_min
                    h = y_max - y_min
                    cx = x_min + w // 2
                    cy = y_min + h // 2
                    cv2.rectangle(frame_disp, (x_min, y_min), (x_max, y_max), (255, 0, 255), 2)
                    cv2.circle(frame_disp, (cx, cy), 4, (255, 255, 0), -1)
                    cv2.putText(frame_disp, f"{w}x{h}px", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    rospy.loginfo_throttle(1.0, f"📍 ROI center: ({cx}, {cy}) | Size: {w}x{h}px")

                    # Actualizar
                    self.old_gray = frame_gray.copy()
                    self.p0 = good_new.reshape(-1, 1, 2)

            cv2.imshow("Optical Flow", frame_disp)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        OpticalFlowTracker()
    except rospy.ROSInterruptException:
        pass
