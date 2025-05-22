#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
import subprocess
import threading
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pan_tilt_msgs.msg import PanTiltCmdDeg
import rospkg

# Factores de zoom para cálculos
ZOOM_FACTORES = {
    1.0: (1.00, 1.00), 2.0: (1.12, 1.10), 3.0: (1.28, 1.25), 4.0: (1.39, 1.33),
    5.0: (1.61, 1.53), 6.0: (1.74, 1.67), 7.0: (1.98, 1.89), 8.0: (2.19, 2.10),
    9.0: (2.59, 2.43), 10.0: (3.09, 2.84), 11.0: (3.64, 3.25), 12.0: (4.37, 3.79),
    13.0: (6.29, 5.15), 14.0: (7.95, 6.48), 15.0: (9.89, 7.95), 16.0: (12.16, 9.35),
    17.0: (14.77, 10.86), 18.0: (16.82, 12.10), 19.0: (18.23, 12.85), 20.0: (18.56, 13.00),
}
ZOOM_FOVS = {
    1.0: (63.7, 35.84), 2.0: (56.9, 31.2), 3.0: (50.7, 27.3), 4.0: (45.9, 24.5),
    5.0: (40.5, 21.6), 6.0: (37.4, 19.6), 7.0: (32.2, 17.2), 8.0: (29.1, 15.2),
    9.0: (25.3, 13.0), 10.0: (21.7, 11.1), 11.0: (18.3, 9.3), 12.0: (15.2, 7.7),
    13.0: (10.0, 6.2), 14.0: (7.8, 4.8), 15.0: (6.2, 3.6), 16.0: (5.2, 2.9),
    17.0: (4.1, 2.3), 18.0: (3.5, 1.9), 19.0: (2.9, 1.7), 20.0: (2.3, 1.3)
}
# Corrección si estás usando el CENTRO DE LA IMAGEN (cursor = 1)
CORRECCION_CURSOR_IMAGEN = {
    1.0: (0.000, 0.000), 2.0: (-0.027, -0.100), 3.0: (-0.051, -0.171), 4.0: (-0.090, -0.241),
    5.0: (-0.126, -0.354), 6.0: (-0.152, -0.421), 7.0: (-0.171, -0.514), 8.0: (-0.220, -0.597),
    9.0: (-0.248, -0.650), 10.0: (-0.248, -0.655), 11.0: (-0.250, -0.724), 12.0: (-0.250, -0.754),
    13.0: (-0.257, -0.808), 14.0: (-0.262, -0.817), 15.0: (-0.290, -0.870), 16.0: (-0.291, -0.900),
    17.0: (-0.307, -0.910), 18.0: (-0.313, -0.915), 19.0: (-0.313, -0.915), 20.0: (-0.313, -0.915)
}

# Corrección si estás usando el CENTRO DE LA CÁMARA (cursor = 2)
CORRECCION_CURSOR_CAMARA = {
    1.0: (0.000, 0.000), 2.0: (-0.027, 0.100), 3.0: (-0.051, 0.171), 4.0: (-0.090, 0.241),
    5.0: (-0.126, 0.354), 6.0: (-0.152, 0.421), 7.0: (-0.171, 0.514), 8.0: (-0.220, 0.597),
    9.0: (-0.248, 0.650), 10.0: (-0.248, 0.655), 11.0: (-0.250, 0.724), 12.0: (-0.250, 0.754),
    13.0: (-0.257, 0.808), 14.0: (-0.262, 0.817), 15.0: (-0.290, 0.870), 16.0: (-0.291, 0.900),
    17.0: (-0.307, 0.910), 18.0: (-0.313, 0.915), 19.0: (-0.313, 0.915), 20.0: (-0.313, 0.915)
}

# Corrección si estás usando el PUNTO EXTRA (cursor = 3)
CORRECCION_CURSOR_EXTRA = {
    1.0: (0.000, 0.000), 2.0: (-0.027, -0.585), 3.0: (-0.051, -0.514), 4.0: (-0.090, -0.444),
    5.0: (-0.126, -0.331), 6.0: (-0.152, -0.264), 7.0: (-0.171, -0.171), 8.0: (-0.220, -0.088),
    9.0: (-0.248, -0.035), 10.0: (-0.248, -0.030), 11.0: (-0.250, 0.039), 12.0: (-0.250, 0.069),
    13.0: (-0.257, 0.123), 14.0: (-0.262, 0.132), 15.0: (-0.290, -0.185), 16.0: (-0.291, 0.215),
    17.0: (-0.307, 0.225), 18.0: (-0.313, 0.230), 19.0: (-0.313, 0.230), 20.0: (-0.313, -0.230)
}



class ZoomCorrector:
    def __init__(self):
        # Inicializar el nodo ROS
        rospy.init_node('zoom_corrector', anonymous=True)
        
        # Puente para convertir mensajes ROS a imágenes OpenCV
        self.bridge = CvBridge()
        
        # Variables para almacenar la imagen y el ROI
        self.image = None
        self.drawing = False
        self.start_x, self.start_y = -1, -1
        self.current_x, self.current_y = -1, -1
        self.roi = None
        self.roi_selected = False
        self.roi_image = None
        self.optimal_zoom = 1.0
        self.roi_saved = False
        self.status_message = ""
        self.is_zooming = False
        self.current_zoom = 1.0
        self.error_dx = 0
        self.error_dy = 0
        self.margen_izq = 0
        self.margen_der = 0
        self.margen_sup = 0
        self.margen_inf = 0
        self.muestra_resultado = False
        self.resultado_imagen = None
        self.CENTER_X_IMAGE = 640
        self.CENTER_Y_IMAGE = 360
        self.CENTER_X_CAM = 643
        self.CENTER_Y_CAM = 415
        self.EXTRA_X = 643
        self.EXTRA_Y = int(387.5)
        self.cursor_seleccionado = 1  # Solo centro de imagen por defecto

                
        # Directorio para guardar imágenes
        rospack = rospkg.RosPack()
        ruta_pkg = rospack.get_path("ptz_geometric_control")
        self.image_dir = os.path.join(ruta_pkg, "images")
        os.makedirs(self.image_dir, exist_ok=True)
        
        # Publicadores
        self.pub_cmd = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=1)
        
        # Suscripción al tema de la cámara
        rospy.Subscriber('/datavideo/video', Image, self.image_callback)
        
        # Crear ventana y configurar callback del mouse
        cv2.namedWindow("Seleccion de ROI y Zoom")
        cv2.setMouseCallback("Seleccion de ROI y Zoom", self.mouse_callback)
        
        print("\n🟢 Nodo inicializado.")
        print("📝 Instrucciones:")
        print("   g: guardar ROI de referencia (en zoom alto)")
        print("   s: guardar vista inicial (en zoom x1)")
        print("   z: aplicar zoom recomendado")
        print("   c: calcular error visual con ROI de referencia")
        print("   r: reiniciar ROI")
        print("   0-3: cambiar cursor activo")
        print("   ESC: salir del programa\n")

        # Iniciar el bucle principal
        self.main_loop()
    
    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"⚠️ Error al convertir la imagen: {e}")
    
    def mouse_callback(self, event, x, y, flags, param):
        # No permitir dibujar ROI si estamos haciendo zoom
        if self.is_zooming:
            return
            
        # Control de eventos del mouse para selección de ROI
        if event == cv2.EVENT_LBUTTONDOWN:
            # Inicio de dibujo
            self.drawing = True
            self.start_x, self.start_y = x, y
            self.current_x, self.current_y = x, y
            self.roi = None
            self.roi_selected = False
            self.roi_image = None
            self.roi_saved = False
            self.status_message = "Dibujando ROI..."
            
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # Actualizar posición actual mientras se arrastra
            self.current_x, self.current_y = x, y
            
        elif event == cv2.EVENT_LBUTTONUP:
            # Fin de dibujo
            self.drawing = False
            # Calcular ancho y alto del ROI
            w = abs(self.current_x - self.start_x)
            h = abs(self.current_y - self.start_y)
            # Asegurar que las coordenadas estén en orden correcto (x0,y0) es la esquina superior izquierda
            x0 = min(self.start_x, self.current_x)
            y0 = min(self.start_y, self.current_y)
            
            if w > 10 and h > 10:  # ROI mínimo
                self.roi = (x0, y0, w, h)
                self.roi_selected = True
                if self.image is not None:
                    self.roi_image = self.image[y0:y0+h, x0:x0+w].copy()
                
                # Calcular zoom óptimo
                self.optimal_zoom = self.calcular_zoom_optimo(w, h)
                print(f"📐 ROI seleccionado: {x0},{y0} {w}x{h}px")
                print(f"🔍 Zoom recomendado: x{self.optimal_zoom}")
                print("💾 Pulsa 's' para guardar el ROI")
                self.status_message = f"ROI seleccionado. Zoom: x{self.optimal_zoom}. Pulsa 's' para guardar vista inicial"
    
    def calcular_zoom_optimo(self, w, h):
        """Calcula el nivel de zoom optimo para el ROI seleccionado"""
        # Obtener dimensiones de la imagen
        if self.image is None:
            return 1.0
            
        img_h, img_w = self.image.shape[:2]
        
        # Buscar el zoom máximo que no haga que el objeto sea demasiado grande
        for zoom in sorted(ZOOM_FACTORES.keys(), reverse=True):
            factor_x, factor_y = ZOOM_FACTORES[zoom]
            
            # Verificar si el objeto será visible con este zoom
            if w * factor_x <= img_w * 0.9 and h * factor_y <= img_h * 0.9:
                return zoom
                
        return 1.0  # Zoom mínimo por defecto
    
    def guardar_roi(self):
        """Guarda la imagen del ROI seleccionado"""
        if self.roi_selected and self.roi_image is not None:
            print("📸 ROI no se guarda como referencia (solo usado para cálculo de zoom).")
            self.guardar_imagen("vista_inicial.jpg")
            self.roi_saved = True
            self.status_message = "ROI guardado. Pulsa 'z' para aplicar zoom"
            return True
        else:
            print("⚠️ No hay ROI seleccionado para guardar")
            return False

    def guardar_roi_manual(self):
        """Guarda manualmente un ROI en cualquier zoom como referencia"""
        if self.roi_selected and self.roi_image is not None:
            ruta = os.path.join(self.image_dir, "roi_recortada.png")
            cv2.imwrite(ruta, self.roi_image)
            print(f"📸 ROI manual de referencia guardado como: {ruta}")
            self.status_message = "ROI de referencia guardado. Continua con zoom x1 para el flujo normal."
            return True
        else:
            print("⚠️ No hay ROI seleccionado para guardar manualmente")
            return False

        
    def dibujar_cursor_seleccionado(self, img):
        """Dibuja el cursor activo en la imagen"""
        if self.cursor_seleccionado in [0, 1]:
            cv2.drawMarker(img, (self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro imagen", (self.CENTER_X_IMAGE + 10, self.CENTER_Y_IMAGE - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.cursor_seleccionado in [0, 2]:
            cv2.drawMarker(img, (self.CENTER_X_CAM, self.CENTER_Y_CAM), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro camara", (self.CENTER_X_CAM + 10, self.CENTER_Y_CAM - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if self.cursor_seleccionado in [0, 3]:
            cv2.drawMarker(img, (self.EXTRA_X, self.EXTRA_Y), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Punto extra", (self.EXTRA_X + 10, self.EXTRA_Y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        return img


    def guardar_imagen(self, nombre, dibujar_cursores=True):
        """Guarda la imagen actual. Si dibujar_cursores=False, guarda sin anotaciones."""
        if self.image is not None:
            img = self.image.copy()

            if dibujar_cursores:
                # === Dibujar centro de la imagen (siempre) ===
                cx_img, cy_img = self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE
                cv2.drawMarker(img, (cx_img, cy_img), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(img, "Centro imagen", (cx_img + 10, cy_img - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # === Dibujar cursor activo SOLO si es diferente al centro imagen ===
                if self.cursor_seleccionado == 2:
                    cv2.drawMarker(img, (self.CENTER_X_CAM, self.CENTER_Y_CAM), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(img, "Centro camara", (self.CENTER_X_CAM + 10, self.CENTER_Y_CAM - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                elif self.cursor_seleccionado == 3:
                    cv2.drawMarker(img, (self.EXTRA_X, self.EXTRA_Y), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(img, "Punto extra", (self.EXTRA_X + 10, self.EXTRA_Y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            ruta = os.path.join(self.image_dir, nombre)
            cv2.imwrite(ruta, img)
            print(f"📸 Imagen guardada como: {ruta}")
            return ruta
        return None



        
    def aplicar_zoom(self, nivel_final):
        """Inicia el proceso de zoom en un hilo separado"""
        if self.is_zooming:
            print("⚠️ Ya se está aplicando zoom. Espera a que termine.")
            return False
            
        self.is_zooming = True
        
        # Iniciar un hilo para aplicar el zoom
        threading.Thread(target=self.zoom_en_hilo, args=(nivel_final,), daemon=True).start()
        return True
    
    def zoom_en_hilo(self, nivel_final):
        """Aplica el zoom directamente en un hilo separado"""
        try:
            # Indicar que estamos aplicando zoom
            self.status_message = f"Aplicando zoom: x{nivel_final}..."
            print(f"🔍 Aplicando zoom a x{nivel_final}...")
                      
            # Aplicar zoom directamente
            cmd = f"rosservice call /set_zoom \"level: {nivel_final}\""
            subprocess.call(cmd, shell=True)
            
            # Actualizar zoom actual
            self.current_zoom = nivel_final
            
            # Esperar a que se estabilice la imagen
            time.sleep(0.5)
            
            # 🔧 Aplicar ajuste automático por nivel de zoom
            self.aplicar_ajuste_zoom()

            # Esperar después de la corrección también
            time.sleep(1.5)
            
            # Guardar imagen con zoom
            self.guardar_imagen("vista.jpg", dibujar_cursores=False)
            self.guardar_imagen("vista2.jpg", dibujar_cursores=True)

            
            # Actualizar mensaje de estado
            print(f"✅ Zoom aplicado: x{nivel_final}")
            self.status_message = f"Zoom aplicado: x{nivel_final}. Pulsa 'c' para calcular correccion"
            
        except Exception as e:
            print(f"❌ Error al aplicar zoom: {e}")
            self.status_message = f"Error al aplicar zoom: {str(e)}"
        
        finally:
            # Marcar como terminado
            self.is_zooming = False
    
    def calcular_error_visual(self):
        ruta_ref = os.path.join(self.image_dir, "roi_recortada.png")
        ruta_actual = os.path.join(self.image_dir, "vista.jpg")
        """Calcula el error visual entre el centro del ROI y el centro de la imagen con SIFT"""
        if not os.path.exists(ruta_ref):
            print("❌ No se encontró roi_recortada.png. Guarda un ROI válido con 'g' antes de calcular el error.")
            return False        
        try:
            # === Cargar imágenes ===
            img_ref = cv2.imread(ruta_ref, cv2.IMREAD_GRAYSCALE)
            img_actual = cv2.imread(ruta_actual)
            if img_ref is None or img_actual is None:
                self.status_message = "Error: No se pudieron cargar las imagenes"
                print("❌ No se pudo cargar una de las imagenes.")
                return False
            
            img_actual_gray = cv2.cvtColor(img_actual, cv2.COLOR_BGR2GRAY)
            
            # === SIFT ===
            sift = cv2.SIFT_create()
            kp_ref, des_ref = sift.detectAndCompute(img_ref, None)
            kp_act, des_act = sift.detectAndCompute(img_actual_gray, None)
            
            # === Matching con Lowe ===
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(des_ref, des_act, k=2)
            good = [m for m, n in matches if m.distance < 0.75 * n.distance]
            
            if len(good) < 4:
                self.status_message = "Error: No hay suficientes coincidencias entre imagenes"
                print("❌ No hay suficientes matches buenos.")
                return False
            
            # === Homografía con RANSAC ===
            src_pts = np.float32([kp_ref[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_act[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            
            if H is None:
                self.status_message = "Error: No se pudo estimar la homografia"
                print("❌ No se pudo estimar la homografía.")
                return False
            
            # === Proyección del contorno de la imagen de referencia ===
            h_ref, w_ref = img_ref.shape
            corners = np.float32([[0, 0], [w_ref, 0], [w_ref, h_ref], [0, h_ref]]).reshape(-1, 1, 2)
            proyectado = cv2.perspectiveTransform(corners, H).reshape(-1, 2)
            
            # Validación del área del box proyectado
            area = cv2.contourArea(proyectado.astype(np.int32))
            if area < 1000:
                self.status_message = "Error: El area proyectada es muy pequena"
                print("⚠️ El área proyectada es muy pequeña. Revisa la referencia.")
                return False
            
            # === Dibujo del bounding box ===
            img_resultado = img_actual.copy()
            for i in range(4):
                pt1 = tuple(np.int32(proyectado[i]))
                pt2 = tuple(np.int32(proyectado[(i + 1) % 4]))
                cv2.line(img_resultado, pt1, pt2, (0, 0, 255), 2)
            
            h_img, w_img = img_resultado.shape[:2]
            # === Centro del objeto ===
            cx_obj = int(np.mean(proyectado[:, 0]))
            cy_obj = int(np.mean(proyectado[:, 1]))
            
            # === Centro del cursor activo
            if self.cursor_seleccionado == 1:
                cx_ref, cy_ref = self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE
            elif self.cursor_seleccionado == 2:
                cx_ref, cy_ref = self.CENTER_X_CAM, self.CENTER_Y_CAM
            elif self.cursor_seleccionado == 3:
                cx_ref, cy_ref = self.EXTRA_X, self.EXTRA_Y
            else:
                cx_ref, cy_ref = self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE  # por defecto

            # Dibujar cursor de referencia con etiqueta según tipo
            color_cursor = (0, 255, 0)  # Verde por defecto

            if self.cursor_seleccionado == 1:
                label = "Centro imagen"
            elif self.cursor_seleccionado == 2:
                label = "Centro camara"
                color_cursor = (255, 0, 0)  # Azul
            elif self.cursor_seleccionado == 3:
                label = "Punto extra"
                color_cursor = (255, 0, 255)  # Fucsia
            else:
                label = "Cursor activo"  # Caso combinado u otro

            cv2.drawMarker(img_resultado, (cx_ref, cy_ref), color_cursor, cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img_resultado, label, (cx_ref + 10, cy_ref - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_cursor, 1)

            # Dibujo del objeto
            cv2.drawMarker(img_resultado, (cx_obj, cy_obj), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img_resultado, "Centro ROI", (cx_obj + 10, cy_obj - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # === Error entre centros
            self.error_dx = cx_obj - cx_ref
            self.error_dy = cy_obj - cy_ref

            # === Márgenes ===
            x_coords = proyectado[:, 0]
            y_coords = proyectado[:, 1]
            self.margen_izq = int(x_coords.min())
            self.margen_der = int(w_img - x_coords.max())
            self.margen_sup = int(y_coords.min())
            self.margen_inf = int(h_img - y_coords.max())

            
            # Mostrar resultados
            print(f"\n🎯 Error entre centros: Δx = {self.error_dx}px, Δy = {self.error_dy}px")
            print("\n📏 Márgenes extra respecto a la imagen:")
            print(f"   🔹 Izquierda : {self.margen_izq} px")
            print(f"   🔹 Derecha   : {self.margen_der} px")
            print(f"   🔹 Superior  : {self.margen_sup} px")
            print(f"   🔹 Inferior  : {self.margen_inf} px")
            
            # Convertir error de píxeles a ángulos
            fov_h, fov_v = ZOOM_FOVS[self.current_zoom]
            px_per_deg_x = w_img / fov_h
            px_per_deg_y = h_img / fov_v
            delta_yaw = self.error_dx / px_per_deg_x
            delta_pitch = -self.error_dy / px_per_deg_y
            
            # Guardar imagen resultado
            cv2.imwrite(os.path.join(self.image_dir, "resultado.jpg"), img_resultado)
            self.resultado_imagen = img_resultado
            self.muestra_resultado = True
            
            # Actualizar mensaje de estado
            self.status_message = f"Error calculado: Δx={self.error_dx}px, Δy={self.error_dy}px"
            
            # Leer posición actual
            yaw_now, pitch_now = self.leer_posicion_actual()
            nuevo_yaw = yaw_now - delta_yaw
            nuevo_pitch = pitch_now - delta_pitch
            
            return True
            
        except Exception as e:
            print(f"❌ Error al calcular la correccion visual: {e}")
            self.status_message = f"Error en cálculo: {str(e)}"
            return False
    

    def leer_posicion_actual(self):
        """Lee la posicion actual de la cámara con precision"""
        try:
            # Usar subprocess con stdout más preciso
            cmd = "rostopic echo -n 1 /pan_tilt_status"
            output = subprocess.getoutput(cmd)
            
            # Buscar las líneas exactas de yaw_now y pitch_now
            lines = output.splitlines()
            yaw_line = ""
            pitch_line = ""
            
            for i, line in enumerate(lines):
                if 'yaw_now:' in line and 'yaw_now_' not in line:
                    yaw_line = line.strip()
                if 'pitch_now:' in line and 'pitch_now_' not in line:
                    pitch_line = line.strip()
            
            # Extraer valores con precisión
            if yaw_line and pitch_line:
                yaw_now = float(yaw_line.split('yaw_now:')[1].strip())
                pitch_now = float(pitch_line.split('pitch_now:')[1].strip())
                
                # Mostrar valores exactos sin redondeo
                return yaw_now, pitch_now
            else:
                raise ValueError("No se encontraron las líneas de yaw_now o pitch_now en la salida")
                
        except Exception as e:
            print(f"❌ Error al leer posicion actual: {e}")
            print(f"❌ Salida del comando: {output[:200]}...")  # Mostrar parte de la salida para debug
            return 0.0, 0.0
        
 
    def dibujar_info(self, img):
        # Código existente para dibujar el rectángulo del ROI
        if self.drawing and self.current_x != -1:
            cv2.rectangle(img, 
                        (self.start_x, self.start_y), 
                        (self.current_x, self.current_y), 
                        (0, 255, 255), 2)
            
            # Mostrar dimensiones
            w = abs(self.current_x - self.start_x)
            h = abs(self.current_y - self.start_y)
            text = f"{w}x{h}"
            x = min(self.start_x, self.current_x)
            y = min(self.start_y, self.current_y)
            cv2.putText(img, text, (x, y-10 if y>20 else y+h+20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Mostrar cursores dinámicamente según tecla
        if self.cursor_seleccionado in [0, 1]:
            # 🟢 Centro de la imagen
            cv2.drawMarker(img, (self.CENTER_X_IMAGE, self.CENTER_Y_IMAGE), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro imagen", (self.CENTER_X_IMAGE + 10, self.CENTER_Y_IMAGE - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.cursor_seleccionado in [0, 2]:
            # 🔵 Centro real de la cámara
            cv2.drawMarker(img, (self.CENTER_X_CAM, self.CENTER_Y_CAM), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro camara", (self.CENTER_X_CAM + 10, self.CENTER_Y_CAM - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if self.cursor_seleccionado in [0, 3]:
            # 🟣 Punto extra
            cv2.drawMarker(img, (self.EXTRA_X, self.EXTRA_Y), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Punto extra", (self.EXTRA_X + 10, self.EXTRA_Y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # Dibujar ROI seleccionado y su centro
        if self.roi_selected and self.roi:
            x, y, w, h = self.roi
            # Color verde si está guardado, amarillo si no
            color = (0, 255, 0) if self.roi_saved else (0, 255, 255)
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            
            # Calcular y dibujar el centro del ROI (azul)
            cx_roi = x + w // 2
            cy_roi = y + h // 2
            cv2.drawMarker(img, (cx_roi, cy_roi), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro ROI", (cx_roi + 10, cy_roi - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Mostrar información del ROI
            roi_text = f"ROI: {w}x{h}"
            if self.roi_saved:
                roi_text += " (Guardado)"
            
            cv2.putText(img, roi_text, (x, y-10 if y>20 else y+h+20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Mostrar zoom recomendado
            zoom_text = f"Zoom recomendado: x{self.optimal_zoom}"
            cv2.putText(img, zoom_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Resto del código existente...
        if self.status_message:
            cv2.putText(img, self.status_message, (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.putText(img, f"Zoom actual: x{self.current_zoom}", (img.shape[1] - 250, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.putText(img, "s: guarda vista | z: zoom | c: calcula error | r: reset", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return img
    

    def aplicar_ajuste_zoom(self):
        """Aplica correcciones angulares específicas por nivel de zoom y cursor activo"""
        try:
            zoom = self.current_zoom

            if self.cursor_seleccionado == 1:
                delta_yaw, delta_pitch = CORRECCION_CURSOR_IMAGEN.get(zoom, (0.0, 0.0))
                tipo_cursor = "Centro imagen"
            elif self.cursor_seleccionado == 2:
                delta_yaw, delta_pitch = CORRECCION_CURSOR_CAMARA.get(zoom, (0.0, 0.0))
                tipo_cursor = "Centro camara"
            elif self.cursor_seleccionado == 3:
                delta_yaw, delta_pitch = CORRECCION_CURSOR_EXTRA.get(zoom, (0.0, 0.0))
                tipo_cursor = "Punto extra"
            else:
                print("⚠️ No se puede aplicar corrección con múltiples cursores activos (modo 0). Usa solo 1, 2 o 3.")
                self.status_message = "Error: múltiples cursores activos (modo 0)"
                return

            # Leer posicion actual
            yaw_now, pitch_now = self.leer_posicion_actual()
            nuevo_yaw = yaw_now + delta_yaw
            nuevo_pitch = pitch_now + delta_pitch

            # Crear mensaje
            cmd = PanTiltCmdDeg()
            cmd.yaw = nuevo_yaw
            cmd.pitch = nuevo_pitch
            cmd.speed = 20

            self.pub_cmd.publish(cmd)

            print(f"\n🛠️ Ajuste por zoom aplicado para x{zoom} usando {tipo_cursor}:")
            print(f"   🔹 Correccion: Δyaw={delta_yaw}°, Δpitch={delta_pitch}°")
            print(f"   🔹 Nueva posicion: yaw={nuevo_yaw}°, pitch={nuevo_pitch}°")
            self.status_message = f"Ajuste aplicado (x{zoom}) con {tipo_cursor.lower()}"

        except Exception as e:
            print(f"❌ Error al aplicar ajuste por zoom: {e}")
            self.status_message = f"Error al aplicar ajuste por zoom: {str(e)}"


    def main_loop(self):
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            # Si tenemos una imagen, procesarla y mostrarla
            if self.image is not None:
                # Crear copia para dibujar
                vis = self.image.copy()
                
                # Dibujar información
                vis = self.dibujar_info(vis)
                
                # Mostrar la imagen
                cv2.imshow("Seleccion de ROI y Zoom", vis)
                
                # Mostrar resultado si está disponible
                if self.muestra_resultado and self.resultado_imagen is not None:
                    cv2.imshow("Resultado de Deteccion", self.resultado_imagen)
                
                # Control de teclado
                key = cv2.waitKey(1) & 0xFF
                
                # Si se presiona ESC, salir
                if key == 27:
                    break
                # Si se presiona 'r', resetear ROI (solo si no estamos haciendo zoom)
                elif key == ord('r') and not self.is_zooming:
                    self.roi = None
                    self.roi_selected = False
                    self.roi_image = None
                    self.roi_saved = False
                    self.error_dx = 0
                    self.error_dy = 0
                    self.muestra_resultado = False
                    if self.resultado_imagen is not None:
                        cv2.destroyWindow("Resultado de Deteccion")
                        self.resultado_imagen = None
                    self.status_message = "ROI reiniciado"
                    print("🔄 ROI reiniciado")
                # Si se presiona 's', guardar vista actual (solo si no estamos haciendo zoom)
                elif key == ord('s') and self.roi_selected and not self.is_zooming:
                    self.guardar_roi()
                # Si se presiona 'z', aplicar zoom (solo si el ROI está guardado y no estamos haciendo zoom)
                elif key == ord('z') and self.roi_saved and not self.is_zooming:
                    self.aplicar_zoom(self.optimal_zoom)
                # Si se presiona 'c', calcular error visual (después de hacer zoom)
                elif key == ord('c') and self.roi_saved and not self.is_zooming:
                    self.calcular_error_visual()
                # Cambiar cursor mostrado en tiempo real
                elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
                    self.cursor_seleccionado = int(chr(key))
                    cursor_nombres = {
                        0: "Todos los cursores",
                        1: "Centro de la imagen",
                        2: "Centro real de la cámara",
                        3: "Punto extra"
                    }
                    print(f"🎯 Cursor cambiado a: {cursor_nombres.get(self.cursor_seleccionado, 'Desconocido')}")
                elif key == ord('g') and self.roi_selected and not self.is_zooming:
                    self.guardar_roi_manual()


            
            rate.sleep()
        
        # Cerrar todas las ventanas al terminar
        cv2.destroyAllWindows()
        print("👋 Programa terminado.")

if __name__ == '__main__':
    try:
        ZoomCorrector()
    except rospy.ROSInterruptException:
        pass