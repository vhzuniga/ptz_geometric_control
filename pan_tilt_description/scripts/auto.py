#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
import subprocess
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from pan_tilt_msgs.msg import PanTiltCmdDeg


CORRECCIONES_EXTRA = {
    1.0: (0.000, 0.000), 2.0: (-0.027, -0.585), 3.0: (-0.051, -0.514),
    4.0: (-0.090, -0.444), 5.0: (-0.126, -0.331), 6.0: (-0.152, -0.264),
    7.0: (-0.171, -0.171), 8.0: (-0.220, -0.088), 9.0: (-0.248, -0.035),
    10.0: (-0.248, -0.030), 11.0: (-0.250, 0.039), 12.0: (-0.250, 0.069),
    13.0: (-0.257, 0.123), 14.0: (-0.262, 0.132), 15.0: (-0.290, 0.185),
    16.0: (-0.291, 0.215), 17.0: (-0.307, 0.225), 18.0: (-0.313, 0.230),
    19.0: (-0.313, 0.230), 20.0: (-0.313, 0.230)
}

ZOOM_FOVS = {
    1.0: (63.7, 35.84), 2.0: (56.9, 31.2), 3.0: (50.7, 27.3), 4.0: (45.9, 24.5),
    5.0: (40.5, 21.6), 6.0: (37.4, 19.6), 7.0: (32.2, 17.2), 8.0: (29.1, 15.2),
    9.0: (25.3, 13.0), 10.0: (21.7, 11.1), 11.0: (18.3, 9.3), 12.0: (15.2, 7.7),
    13.0: (10.0, 6.2), 14.0: (7.8, 4.8), 15.0: (6.2, 3.6), 16.0: (5.2, 2.9),
    17.0: (4.1, 2.3), 18.0: (3.5, 1.9), 19.0: (2.9, 1.7), 20.0: (2.3, 1.3)
}

ZOOM_FACTORES = {
    1.0: (1.00, 1.00), 2.0: (1.12, 1.10), 3.0: (1.28, 1.25), 4.0: (1.39, 1.33),
    5.0: (1.61, 1.53), 6.0: (1.74, 1.67), 7.0: (1.98, 1.89), 8.0: (2.19, 2.10),
    9.0: (2.59, 2.43), 10.0: (3.09, 2.84), 11.0: (3.64, 3.25), 12.0: (4.37, 3.79),
    13.0: (6.29, 5.15), 14.0: (7.95, 6.48), 15.0: (9.89, 7.95), 16.0: (12.16, 9.35),
    17.0: (14.77, 10.86), 18.0: (16.82, 12.10), 19.0: (18.23, 12.85), 20.0: (18.56, 13.00),
}

class PTZAuto:
    def __init__(self, debug=False):
        rospy.init_node("ptz_auto_zoom", anonymous=True)
        self.image = None
        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher("/pan_tilt_cmd_deg", PanTiltCmdDeg, queue_size=1)
        self.image_dir = os.path.expanduser("~/ros_ws/src/pan_tilt_ros/images")
        os.makedirs(self.image_dir, exist_ok=True)
        rospy.Subscriber("/datavideo/video", Image, self.image_callback)
        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback)
        self.debug = debug
        print("🟢 Esperando punto publicado...")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.debug:
            img = self.image.copy()
            # Solo dibujar el cursor extra (fucsia)
            cx, cy = 643, int(387.5)
            cv2.drawMarker(img, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Punto extra", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            cv2.imshow("Vista en vivo", img)
            cv2.waitKey(1)


    def leer_posicion_actual(self):
        output = subprocess.getoutput("rostopic echo -n 1 /pan_tilt_status")
        yaw = pitch = 0.0
        for line in output.splitlines():
            if 'yaw_now:' in line: yaw = float(line.split(':')[-1])
            if 'pitch_now:' in line: pitch = float(line.split(':')[-1])
        return yaw, pitch

    def calcular_zoom(self, obj_ancho_px, obj_alto_px):
        img_w, img_h = 1280, 720
        for zoom in sorted(ZOOM_FACTORES.keys(), reverse=True):
            fx, fy = ZOOM_FACTORES[zoom]
            ancho_zoom = obj_ancho_px * fx
            alto_zoom = obj_alto_px * fy
            if ancho_zoom <= img_w * 0.9 and alto_zoom <= img_h * 0.9:
                return zoom
        return 1.0


    def aplicar_zoom(self, nivel):
        subprocess.call(f"rosservice call /set_zoom \"level: {nivel}\"", shell=True)
        time.sleep(0.5)

    def aplicar_correccion(self, zoom):
        delta_yaw, delta_pitch = CORRECCIONES_EXTRA[zoom]
        yaw, pitch = self.leer_posicion_actual()
        print(f"📍 Pose actual: yaw = {yaw:.2f}°, pitch = {pitch:.2f}°")
        nuevo_yaw = yaw + delta_yaw
        nuevo_pitch = pitch + delta_pitch
        cmd = PanTiltCmdDeg()
        cmd.yaw = nuevo_yaw
        cmd.pitch = nuevo_pitch
        cmd.speed = 20
        self.pub_cmd.publish(cmd)
        print(f"🔧 Corrección aplicada (cursor extra): Δyaw={delta_yaw:+.2f}°, Δpitch={delta_pitch:+.2f}°")
        print(f"📍 Pose final esperada: yaw = {nuevo_yaw:.2f}°, pitch = {nuevo_pitch:.2f}°")
        time.sleep(1.0)

    def calcular_error_visual(self, zoom):
        ref_path = os.path.join(self.image_dir, "roi_recortada.png")
        vista_path = os.path.join(self.image_dir, "vista_final.jpg")
        if not os.path.exists(ref_path):
            print("❌ roi_recortada.png no encontrada.")
            return

        img_ref = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
        img_actual = cv2.imread(vista_path)
        if img_ref is None or img_actual is None:
            print("❌ No se pudo cargar una de las imágenes.")
            return

        # === Detección y matching con SIFT ===
        sift = cv2.SIFT_create()
        kp1, des1 = sift.detectAndCompute(img_ref, None)
        kp2, des2 = sift.detectAndCompute(cv2.cvtColor(img_actual, cv2.COLOR_BGR2GRAY), None)
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]
        if len(good) < 4:
            print("❌ No hay suficientes matches.")
            return

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if H is None:
            print("❌ No se pudo calcular la homografía.")
            return

        # === Bounding box proyectado ===
        h, w = img_ref.shape
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(corners, H).reshape(-1, 2)

        # === Cálculo de centros
        cx_obj = int(np.mean(projected[:, 0]))
        cy_obj = int(np.mean(projected[:, 1]))
        cx_img, cy_img = 640, 360  # Centro imagen estándar 1280x720

        dx, dy = cx_obj - cx_img, cy_obj - cy_img

        # === Reporte de error y márgenes
        print(f"\n📏 Error visual: dx = {dx}px, dy = {dy}px")
        print(f"🖼️ Margen izq: {int(min(projected[:,0]))} px")
        print(f"🖼️ Margen der: {1280 - int(max(projected[:,0]))} px")
        print(f"🖼️ Margen sup: {int(min(projected[:,1]))} px")
        print(f"🖼️ Margen inf: {720 - int(max(projected[:,1]))} px")

        # === Dibujar bounding box
        for i in range(4):
            pt1 = tuple(np.int32(projected[i]))
            pt2 = tuple(np.int32(projected[(i + 1) % 4]))
            cv2.line(img_actual, pt1, pt2, (0, 0, 255), 2)

        # === Centro imagen (verde)
        cv2.drawMarker(img_actual, (cx_img, cy_img), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img_actual, "Centro imagen", (cx_img + 10, cy_img - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # === Centro objeto detectado (azul)
        cv2.drawMarker(img_actual, (cx_obj, cy_obj), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img_actual, "Centro objeto", (cx_obj + 10, cy_obj - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # === Guardar imagen de resultado
        cv2.imwrite(os.path.join(self.image_dir, "resultado.jpg"), img_actual)

        # === Cierre del nodo
        rospy.signal_shutdown("Proceso completado")

    def estimar_pixeles_proyectados(self, ancho_m, alto_m, distancia_m, zoom):
        fov_h_deg, fov_v_deg = ZOOM_FOVS[zoom]
        fov_h_rad = np.deg2rad(fov_h_deg)
        fov_v_rad = np.deg2rad(fov_v_deg)

        img_w, img_h = 1280, 720

        pix_ancho = (ancho_m / (2 * distancia_m * np.tan(fov_h_rad / 2))) * img_w
        pix_alto = (alto_m / (2 * distancia_m * np.tan(fov_v_rad / 2))) * img_h

        return int(pix_ancho), int(pix_alto)


    def point_callback(self, msg):
        if self.image is None:
            print("⚠️ Imagen aún no recibida.")
            return

        alto_m = float(input("📐 Altura del objeto (m): "))
        ancho_m = float(input("📐 Ancho del objeto (m): "))
        distancia_m = float(input("📏 Distancia estimada al objeto (m): "))

        img_w, img_h = 1280, 720

        # === Paso 1: estimar tamaño proyectado en zoom x1.0
        pix_ancho_1x, pix_alto_1x = self.estimar_pixeles_proyectados(ancho_m, alto_m, distancia_m, 1.0)
        print(f"\n📏 Proyección estimada en zoom x1.0: {pix_ancho_1x}px x {pix_alto_1x}px")

        # === Paso 2: buscar el mayor zoom donde el objeto aún cabe en pantalla
        for zoom in sorted(ZOOM_FACTORES.keys(), reverse=True):
            fx, fy = ZOOM_FACTORES[zoom]
            ancho_zoom = pix_ancho_1x * fx
            alto_zoom = pix_alto_1x * fy
            if ancho_zoom <= img_w * 0.9 and alto_zoom <= img_h * 0.9:
                print(f"📏 Proyección estimada en zoom x{zoom}: {int(ancho_zoom)}px x {int(alto_zoom)}px")
                break
        else:
            zoom = 1.0
            print("⚠️ Solo es visible completamente en zoom x1.0.")

        print(f"🔍 Zoom recomendado: x{zoom}")

        self.aplicar_zoom(zoom)
        self.aplicar_correccion(zoom)

        # Guardar la imagen final SIN dibujar el cursor fucsia
        img_final = self.image.copy()
        cv2.imwrite(os.path.join(self.image_dir, "vista_final.jpg"), img_final)

        self.calcular_error_visual(zoom)

if __name__ == "__main__":
    try:
        PTZAuto(debug=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass