#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ROIMedidor:
    def __init__(self):
        rospy.init_node('roi_en_vivo')
        self.bridge = CvBridge()
        self.image = None
        self.start_point = None
        self.current_point = None
        self.drawing = False
        self.roi = None

        self.IMAGE_WIDTH = 1280
        self.IMAGE_HEIGHT = 720
        self.CENTER_X_IMAGE = 640
        self.CENTER_Y_IMAGE = 360
        self.CENTER_X_CAM = 643
        self.CENTER_Y_CAM = 415
        self.FOV_H = 63.7
        self.FOV_V = 35.84
        self.punto_extra_x = 643
        self.punto_extra_y = int(387.5)


        cv2.namedWindow("ROI en vivo")
        cv2.resizeWindow("ROI en vivo", 1920, 1080)
        cv2.setMouseCallback("ROI en vivo", self.mouse_callback)

        rospy.Subscriber('/datavideo/video', Image, self.image_callback)
        rospy.loginfo("🟢 Nodo iniciado. Esperando imagen del topic...")

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except:
            rospy.logwarn("⚠️ No se pudo convertir la imagen.")
            return

        display = self.image.copy()

        # 🟢 Centro de la imagen
        cv2.circle(display, (self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE), 3, (0, 255, 0), -1)
        cv2.line(display, (self.CENTER_X_IMAGE - 10, self.CENTER_Y_IMAGE),
                 (self.CENTER_X_IMAGE + 10, self.CENTER_Y_IMAGE), (0, 255, 0), 1)
        cv2.line(display, (self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE - 10),
                 (self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE + 10), (0, 255, 0), 1)

        # 🔵 Centro real de la cámara
        cv2.circle(display, (self.CENTER_X_CAM, self.CENTER_Y_CAM), 3, (255, 0, 0), -1)
        cv2.line(display, (self.CENTER_X_CAM - 10, self.CENTER_Y_CAM),
                 (self.CENTER_X_CAM + 10, self.CENTER_Y_CAM), (255, 0, 0), 1)
        cv2.line(display, (self.CENTER_X_CAM, self.CENTER_Y_CAM - 10),
                 (self.CENTER_X_CAM, self.CENTER_Y_CAM + 10), (255, 0, 0), 1)

        # 🟣 Punto de interés adicional (magenta)
        punto_extra_x = 643
        punto_extra_y = int(387.5)
        cv2.circle(display, (punto_extra_x, punto_extra_y), 4, (255, 0, 255), -1)
        cv2.putText(display, "Punto extra", (punto_extra_x + 10, punto_extra_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # ROI en curso
        if self.drawing and self.start_point and self.current_point:
            cv2.rectangle(display, self.start_point, self.current_point, (255, 0, 0), 1)
            

        # ROI finalizada
        if self.roi:
            x, y, w, h = self.roi
            cx_roi = x + w // 2
            cy_roi = y + h // 2
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(display, (cx_roi, cy_roi), 4, (0, 0, 255), -1)
            #cv2.putText(display, f"{w} x {h} px", (x, y - 10),
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("ROI en vivo", display)
        key = cv2.waitKey(1)

        # 🔘 Guardar imagen recortada al presionar 's'
        if key == ord('s') and self.roi and self.image is not None:
            x, y, w, h = self.roi
            recorte = self.image[y:y+h, x:x+w]
            ruta_salida = "/home/hugo/ros_ws/src/pan_tilt_ros/images/roi_recortada.png"
            cv2.imwrite(ruta_salida, recorte)
            rospy.loginfo(f"💾 ROI recortada guardada en: {ruta_salida}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start_point = (x, y)
            self.drawing = True
            self.current_point = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.current_point = (x, y)
            
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            x0, y0 = self.start_point
            x1, y1 = x, y
            x, y = min(x0, x1), min(y0, y1)
            w, h = abs(x1 - x0), abs(y1 - y0)
            self.roi = (x, y, w, h)

            # Centro de la ROI (actual después del zoom)
            cx_roi = x + w // 2
            cy_roi = y + h // 2

            # ================================
            # Corrección respecto al punto extra
            dx_extra = self.punto_extra_x - cx_roi
            dy_extra = self.punto_extra_y - cy_roi

            # Corrección respecto al centro de la imagen
            dx_img = self.CENTER_X_IMAGE - cx_roi
            dy_img = self.CENTER_Y_IMAGE - cy_roi

            # Corrección respecto al centro óptico real de la cámara
            dx_cam = self.CENTER_X_CAM - cx_roi
            dy_cam = self.CENTER_Y_CAM - cy_roi

            # Píxeles por grado
            px_per_deg_x = self.IMAGE_WIDTH / self.FOV_H
            px_per_deg_y = self.IMAGE_HEIGHT / self.FOV_V

            # Cálculos de corrección angular
            delta_yaw_extra = dx_extra / px_per_deg_x
            delta_pitch_extra = -dy_extra / px_per_deg_y

            delta_yaw_img = dx_img / px_per_deg_x
            delta_pitch_img = -dy_img / px_per_deg_y

            delta_yaw_cam = dx_cam / px_per_deg_x
            delta_pitch_cam = -dy_cam / px_per_deg_y

            # ================================
            # Mostrar resultados
            rospy.loginfo(f"📐 ROI: {w} x {h} px")
            rospy.loginfo(f"📍 Centro ROI actual: ({cx_roi}, {cy_roi})\n")

            rospy.loginfo(f"📌 Corrección para alinear con CENTRO DE IMAGEN ({self.CENTER_X_IMAGE},{self.CENTER_Y_IMAGE}):")
            rospy.loginfo(f"   ➤ Δx = {dx_img}px, Δy = {dy_img}px")
            rospy.loginfo(f"   ➤ Yaw   = {delta_yaw_img:.3f}°")
            rospy.loginfo(f"   ➤ Pitch = {delta_pitch_img:.3f}°\n")

            rospy.loginfo(f"📌 Corrección para alinear con CENTRO DE CÁMARA ({self.CENTER_X_CAM},{self.CENTER_Y_CAM}):")
            rospy.loginfo(f"   ➤ Δx = {dx_cam}px, Δy = {dy_cam}px")
            rospy.loginfo(f"   ➤ Yaw   = {delta_yaw_cam:.3f}°")
            rospy.loginfo(f"   ➤ Pitch = {delta_pitch_cam:.3f}°\n")

            rospy.loginfo(f"📌 Corrección para alinear con PUNTO EXTRA ({self.punto_extra_x},{self.punto_extra_y}):")
            rospy.loginfo(f"   ➤ Δx = {dx_extra}px, Δy = {dy_extra}px")
            rospy.loginfo(f"   ➤ Yaw   = {delta_yaw_extra:.3f}°")
            rospy.loginfo(f"   ➤ Pitch = {delta_pitch_extra:.3f}°")


if __name__ == '__main__':
    try:
        ROIMedidor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
