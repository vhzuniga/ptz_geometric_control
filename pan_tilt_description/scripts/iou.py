#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2

# === Rutas de las imágenes ===
ruta_ref = "/home/hugo/ros_ws/src/pan_tilt_ros/images/recuadro.jpg"
ruta_actual = "/home/hugo/ros_ws/src/pan_tilt_ros/images/final.jpg"

# === Cargar imágenes ===
img_ref = cv2.imread(ruta_ref, cv2.IMREAD_GRAYSCALE)
img_actual = cv2.imread(ruta_actual)
img_actual_gray = cv2.cvtColor(img_actual, cv2.COLOR_BGR2GRAY)

if img_ref is None or img_actual is None:
    raise FileNotFoundError("❌ No se pudo cargar una de las imágenes.")

# === SIFT ===
sift = cv2.SIFT_create()
kp_ref, des_ref = sift.detectAndCompute(img_ref, None)
kp_act, des_act = sift.detectAndCompute(img_actual_gray, None)

# === Matching con Lowe ===
bf = cv2.BFMatcher()
matches = bf.knnMatch(des_ref, des_act, k=2)
good = [m for m, n in matches if m.distance < 0.75 * n.distance]

if len(good) < 4:
    raise ValueError("❌ No hay suficientes matches buenos.")

# === Homografía con RANSAC ===
src_pts = np.float32([kp_ref[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
dst_pts = np.float32([kp_act[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
if H is None:
    raise ValueError("❌ No se pudo estimar la homografía.")

# === Proyección del contorno de la imagen de referencia ===
h_ref, w_ref = img_ref.shape
corners = np.float32([[0, 0], [w_ref, 0], [w_ref, h_ref], [0, h_ref]]).reshape(-1, 1, 2)
proyectado = cv2.perspectiveTransform(corners, H).reshape(-1, 2)

# Validación del área del box proyectado
area = cv2.contourArea(proyectado.astype(np.int32))
if area < 300:
    cv2.putText(img_actual, "Area proyectada pequena", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

# === Dibujo del bounding box ===
for i in range(4):
    pt1 = tuple(np.int32(proyectado[i]))
    pt2 = tuple(np.int32(proyectado[(i + 1) % 4]))
    cv2.line(img_actual, pt1, pt2, (0, 0, 255), 2)

# === Centro del objeto ===
cx_obj = int(np.mean(proyectado[:, 0]))
cy_obj = int(np.mean(proyectado[:, 1]))

# === Centro de la imagen ===
h_img, w_img, _ = img_actual.shape
cx_img = w_img // 2
cy_img = h_img // 2

# === Dibujo de los centros ===
cv2.drawMarker(img_actual, (cx_img, cy_img), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)  # verde
cv2.drawMarker(img_actual, (cx_obj, cy_obj), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)  # azul

# === Error entre centros ===
dx = cx_obj - cx_img
dy = cy_obj - cy_img
print(f"\n🎯 Error entre centros: Δx = {dx}px, Δy = {dy}px")

# === Márgenes ===
x_coords = proyectado[:, 0]
y_coords = proyectado[:, 1]

margen_izq = int(x_coords.min())
margen_der = int(w_img - x_coords.max())
margen_sup = int(y_coords.min())
margen_inf = int(h_img - y_coords.max())


print("\n📏 Márgenes extra respecto a la imagen:")
print(f"   🔹 Izquierda : {margen_izq} px")
print(f"   🔹 Derecha   : {margen_der} px")
print(f"   🔹 Superior  : {margen_sup} px")
print(f"   🔹 Inferior  : {margen_inf} px")

# === Mostrar resultado ===
cv2.imshow("Resultado", img_actual)
cv2.waitKey(0)
cv2.destroyAllWindows()
