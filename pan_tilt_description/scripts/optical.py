#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import os
from datetime import datetime

class SinglePointTracker:
    def __init__(self):
        rospy.init_node('single_point_tracker')
        self.bridge = CvBridge()
        self.image = None
        self.prev_image = None
        
        # Configuración de cámara
        self.center_x = 643  # Centro óptico de la cámara
        self.center_y = 415
        self.zoom_level = 1.0  # Zoom inicial
        self.prev_zoom_level = 1.0  # Para detectar cambios de zoom
        
        # Diccionario FOV por nivel de zoom
        self.FOVS = {
            1.0: (63.7, 35.84), 2.0: (56.9, 31.2), 3.0: (50.7, 27.3), 4.0: (45.9, 24.5),
            5.0: (40.5, 21.6), 6.0: (37.4, 19.6), 7.0: (32.2, 17.2), 8.0: (29.1, 15.2),
            9.0: (25.3, 13.0), 10.0: (21.7, 11.1), 11.0: (18.3, 9.3), 12.0: (15.2, 7.7),
            13.0: (10.0, 6.2), 14.0: (7.8, 4.8), 15.0: (6.2, 3.6), 16.0: (5.2, 2.9),
            17.0: (4.1, 2.3), 18.0: (3.5, 1.9), 19.0: (2.9, 1.7), 20.0: (2.3, 1.3)
        }
        
        # Variables para tracking
        self.patron_size = (7, 7)  # Tamaño del tablero
        self.todos_los_puntos = None  # Todos los puntos detectados
        self.punto_activo = None  # Punto activo para seguimiento
        self.punto_inicial_zoom1 = None  # Punto en zoom x1 (referencia)
        self.resultados = defaultdict(list)  # Resultados por nivel de zoom
        self.modo_seleccion = True  # Si estamos en modo selección o seguimiento
        
        # Parámetros para optical flow
        self.lk_params = dict(
            winSize=(25, 25),
            maxLevel=4,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        # Creación de ventanas
        cv2.namedWindow("Seguimiento de Punto Único")
        cv2.setMouseCallback("Seguimiento de Punto Único", self.seleccionar_punto)
        
        # Suscripción al tópico de la cámara
        rospy.Subscriber('/datavideo/video', Image, self.image_callback)
        
        rospy.loginfo("=== SEGUIMIENTO DE PUNTO ÚNICO ===")
        rospy.loginfo("1. Presiona 'd' para detectar las esquinas del tablero")
        rospy.loginfo("2. Haz clic en UNA ESQUINA para seguir solo ese punto")
        rospy.loginfo("3. El punto se seguirá automáticamente al cambiar el zoom")
        rospy.loginfo("4. El offset se calculará automáticamente al cambiar el zoom")
        rospy.loginfo("5. Presiona 'r' para volver al modo de selección (todos los puntos)")
        rospy.loginfo("6. Presiona 'p' para ver resultados o 'g' para guardar")
        
        self.run()
    
    def image_callback(self, msg):
        """Callback para recibir imágenes de la cámara"""
        try:
            # Guardar imagen anterior si existe
            if self.image is not None:
                self.prev_image = self.image.copy()
            
            # Actualizar imagen actual
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Si estamos en modo seguimiento, actualizar posición automáticamente
            if not self.modo_seleccion and self.prev_image is not None and self.punto_activo is not None:
                self.seguir_punto()
                
                # Si hubo cambio de zoom, calcular offset automáticamente
                '''if self.zoom_level != self.prev_zoom_level:
                    self.calcular_offset()
                    self.prev_zoom_level = self.zoom_level'''
            
        except Exception as e:
            rospy.logwarn(f"Error al procesar la imagen: {e}")
    
    def seleccionar_punto(self, event, x, y, flags, param):
        """Callback para seleccionar un punto con el mouse"""
        if not self.modo_seleccion or self.todos_los_puntos is None:
            return
            
        if event == cv2.EVENT_LBUTTONDOWN:
            # Encontrar el punto más cercano al clic
            distancias = [np.sqrt((x - punto[0])**2 + (y - punto[1])**2) for punto in self.todos_los_puntos]
            idx_min = np.argmin(distancias)
            punto_seleccionado = self.todos_los_puntos[idx_min]
            
            # Guardar el punto seleccionado como punto activo
            self.punto_activo = punto_seleccionado
            self.punto_inicial_zoom1 = punto_seleccionado  # Guardar como referencia inicial
            
            # Cambiar a modo seguimiento
            self.modo_seleccion = False
            
            # Resetear zoom previo
            self.prev_zoom_level = self.zoom_level
            
            rospy.loginfo(f"Punto seleccionado: ({punto_seleccionado[0]:.1f}, {punto_seleccionado[1]:.1f})")
            rospy.loginfo("Modo de seguimiento activado - solo se seguirá este punto")
    
    def detectar_esquinas(self):
        """Detecta las esquinas del tablero de ajedrez"""
        if self.image is None:
            rospy.logwarn("No hay imagen disponible")
            return False
        
        # Cambiar a modo selección
        self.modo_seleccion = True
        self.punto_activo = None
        
        # Convertir a escala de grises
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        
        # Detectar esquinas del tablero
        ret, esquinas = cv2.findChessboardCorners(gray, self.patron_size, None)
        
        if not ret:
            # Intentar con detección adaptativa si falla el método estándar
            ret, esquinas = cv2.findChessboardCorners(gray, self.patron_size, 
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                    cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            # Refinar detección para precisión subpixel
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            refined_corners = cv2.cornerSubPix(gray, esquinas, (11, 11), (-1, -1), criteria)
            
            # Convertir a lista de puntos (x, y)
            esquinas_refinadas = [punto[0] for punto in refined_corners]
            
            # Guardar puntos
            self.todos_los_puntos = esquinas_refinadas
            
            rospy.loginfo(f"Tablero detectado con {len(self.todos_los_puntos)} esquinas")
            rospy.loginfo("Haz clic en una esquina para seguir solo ese punto")
            return True
        else:
            rospy.logwarn("No se pudo detectar el tablero de ajedrez")
            return False
    
    def seguir_punto(self):
        """Sigue el punto activo usando optical flow"""
        if self.prev_image is None or self.image is None or self.punto_activo is None:
            return False
        
        # Convertir imágenes a escala de grises
        prev_gray = cv2.cvtColor(self.prev_image, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        
        # Preparar punto para optical flow
        p0 = np.array([self.punto_activo], dtype=np.float32).reshape(-1, 1, 2)
        
        # Calcular optical flow
        p1, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, p0, None, **self.lk_params)
        
        if p1 is None or status[0] == 0:
            rospy.logwarn("Falló el seguimiento del punto")
            return False
        
        # Actualizar posición del punto
        self.punto_activo = (p1[0][0][0], p1[0][0][1])
        
        return True
    
    def calcular_offset(self):
        """Calcula el offset entre el punto inicial y el punto actual"""
        if self.punto_activo is None or self.punto_inicial_zoom1 is None:
            return
        
        # Obtener FOVs actuales
        fov_h_actual, fov_v_actual = self.FOVS.get(self.zoom_level, (None, None))
        fov_h_inicial, fov_v_inicial = self.FOVS.get(1.0, (None, None))
        
        if fov_h_actual is None or fov_v_actual is None or fov_h_inicial is None or fov_v_inicial is None:
            rospy.logwarn(f"FOV no definido para zoom {self.zoom_level}")
            return
        
        # Calcular vectores desde el centro (en píxeles)
        vector_inicial_px = (self.punto_inicial_zoom1[0] - self.center_x, 
                            self.punto_inicial_zoom1[1] - self.center_y)
        
        vector_actual_px = (self.punto_activo[0] - self.center_x, 
                            self.punto_activo[1] - self.center_y)
        
        # Convertir a ángulos (en el zoom inicial)
        altura_inicial, anchura_inicial = self.image.shape[:2]
        px_per_deg_x_inicial = anchura_inicial / fov_h_inicial
        px_per_deg_y_inicial = altura_inicial / fov_v_inicial
        
        angulo_inicial_x = vector_inicial_px[0] / px_per_deg_x_inicial
        angulo_inicial_y = vector_inicial_px[1] / px_per_deg_y_inicial
        
        # Convertir a ángulos (en el zoom actual)
        altura_actual, anchura_actual = self.image.shape[:2]
        px_per_deg_x_actual = anchura_actual / fov_h_actual
        px_per_deg_y_actual = altura_actual / fov_v_actual
        
        angulo_actual_x = vector_actual_px[0] / px_per_deg_x_actual
        angulo_actual_y = vector_actual_px[1] / px_per_deg_y_actual
        
        # Calcular offset angular (esta es la clave)
        delta_yaw = angulo_actual_x - angulo_inicial_x
        delta_pitch = angulo_actual_y - angulo_inicial_y
        
        # Calcular offset en píxeles en el zoom actual
        offset_x = delta_yaw * px_per_deg_x_actual
        offset_y = delta_pitch * px_per_deg_y_actual
        
        # Guardar resultados
        self.resultados[self.zoom_level].append({
            'offset_x': offset_x,
            'offset_y': offset_y,
            'delta_yaw': delta_yaw,
            'delta_pitch': delta_pitch,
            'punto_x': self.punto_activo[0],
            'punto_y': self.punto_activo[1],
            'angulo_x': angulo_actual_x,
            'angulo_y': angulo_actual_y
        })
        
        #rospy.loginfo(f"[Zoom x{self.zoom_level}] Vector inicial: ({vector_inicial_px[0]:.1f}, {vector_inicial_px[1]:.1f}) px")
        #rospy.loginfo(f"[Zoom x{self.zoom_level}] Vector actual: ({vector_actual_px[0]:.1f}, {vector_actual_px[1]:.1f}) px")
        #rospy.loginfo(f"[Zoom x{self.zoom_level}] Ángulo inicial: ({angulo_inicial_x:.4f}, {angulo_inicial_y:.4f}) grados")
        #rospy.loginfo(f"[Zoom x{self.zoom_level}] Ángulo actual: ({angulo_actual_x:.4f}, {angulo_actual_y:.4f}) grados")
        rospy.loginfo(f"[Zoom x{self.zoom_level}] Offset: ({offset_x:.1f}, {offset_y:.1f}) píxeles")
        #rospy.loginfo(f"[Zoom x{self.zoom_level}] Δyaw={delta_yaw:.4f}°, Δpitch={delta_pitch:.4f}°")
        
    def mostrar_resultados(self):
        """Muestra gráficamente los resultados de la calibración"""
        if not self.resultados:
            rospy.logwarn("No hay resultados para mostrar")
            return
        
        # Preparar datos para gráficas
        zoom_levels = sorted(self.resultados.keys())
        delta_yaws = []
        delta_pitchs = []
        
        for zoom in zoom_levels:
            if self.resultados[zoom]:
                # Calcular promedio si hay varias mediciones
                avg_yaw = np.mean([r['delta_yaw'] for r in self.resultados[zoom]])
                avg_pitch = np.mean([r['delta_pitch'] for r in self.resultados[zoom]])
                delta_yaws.append(avg_yaw)
                delta_pitchs.append(avg_pitch)
        
        # Crear figura
        plt.figure(figsize=(12, 6))
        
        # Gráfica delta yaw
        plt.subplot(121)
        plt.plot(zoom_levels, delta_yaws, 'ro-')
        plt.title('Delta Yaw vs Zoom')
        plt.xlabel('Nivel de Zoom')
        plt.ylabel('Delta Yaw (grados)')
        plt.grid(True)
        
        # Gráfica delta pitch
        plt.subplot(122)
        plt.plot(zoom_levels, delta_pitchs, 'bo-')
        plt.title('Delta Pitch vs Zoom')
        plt.xlabel('Nivel de Zoom')
        plt.ylabel('Delta Pitch (grados)')
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # Imprimir tabla de resultados
        print("\n=== RESULTADOS DE CALIBRACIÓN ===")
        print("Zoom\tDelta Yaw\tDelta Pitch")
        for zoom, yaw, pitch in zip(zoom_levels, delta_yaws, delta_pitchs):
            print(f"{zoom}\t{yaw:.4f}\t{pitch:.4f}")
    
    def guardar_resultados(self):
        """Guarda los resultados en un archivo CSV"""
        if not self.resultados:
            rospy.logwarn("No hay resultados para guardar")
            return
        
        try:
            # Crear directorio si no existe
            os.makedirs("calibracion", exist_ok=True)
            
            # Generar nombre de archivo con timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"calibracion/zoom_offset_{timestamp}.csv"
            
            # Preparar datos
            data = []
            for zoom in sorted(self.resultados.keys()):
                if self.resultados[zoom]:
                    # Calcular promedio si hay varias mediciones
                    avg_yaw = np.mean([r['delta_yaw'] for r in self.resultados[zoom]])
                    avg_pitch = np.mean([r['delta_pitch'] for r in self.resultados[zoom]])
                    data.append([zoom, avg_yaw, avg_pitch])
            
            # Guardar en CSV
            with open(filename, 'w') as f:
                f.write("zoom_level,delta_yaw,delta_pitch\n")
                for zoom, yaw, pitch in data:
                    f.write(f"{zoom},{yaw},{pitch}\n")
            
            rospy.loginfo(f"Resultados guardados en {filename}")
            
        except Exception as e:
            rospy.logerr(f"Error al guardar resultados: {e}")
    
    def dibujar_trayectoria_zoom(self, img):
        """Dibuja una trayectoria que muestra el movimiento del punto con el zoom"""
        if not self.resultados or self.punto_activo is None:
            return img
            
        # Recopilar puntos de la trayectoria del zoom
        zoom_points = []
        zoom_levels = sorted(self.resultados.keys())
        
        # Añadir punto inicial (zoom x1)
        if self.punto_inicial_zoom1 is not None:
            zoom_points.append((1.0, self.punto_inicial_zoom1))
        
        # Añadir puntos de otros niveles de zoom
        for zoom in zoom_levels:
            if self.resultados[zoom]:
                # Obtener posición más reciente para este zoom
                last_result = self.resultados[zoom][-1]
                punto = (last_result['punto_x'], last_result['punto_y'])
                zoom_points.append((zoom, punto))
        
        # Añadir punto actual si no está en la lista
        found = False
        for z, _ in zoom_points:
            if z == self.zoom_level:
                found = True
                break
        
        if not found:
            zoom_points.append((self.zoom_level, self.punto_activo))
            
        # Ordenar por nivel de zoom
        zoom_points.sort(key=lambda x: x[0])
        
        # Dibujar líneas entre puntos
        for i in range(len(zoom_points) - 1):
            pt1 = (int(zoom_points[i][1][0]), int(zoom_points[i][1][1]))
            pt2 = (int(zoom_points[i+1][1][0]), int(zoom_points[i+1][1][1]))
            cv2.line(img, pt1, pt2, (0, 255, 255), 2)
            
            # Dibujar etiqueta de zoom
            zoom_text = f"x{zoom_points[i][0]}"
            cv2.putText(img, zoom_text, pt1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Dibujar etiqueta del último punto
        if zoom_points:
            last_pt = (int(zoom_points[-1][1][0]), int(zoom_points[-1][1][1]))
            last_zoom = f"x{zoom_points[-1][0]}"
            cv2.putText(img, last_zoom, last_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return img
    
    def run(self):
        """Bucle principal"""
        while not rospy.is_shutdown():
            if self.image is None:
                rospy.sleep(0.1)
                continue
            
            # Crear imagen para visualización
            display_img = self.image.copy()
            
            # Dibujar centro de la cámara
            cv2.drawMarker(display_img, (self.center_x, self.center_y), 
                          (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            
            # Dibujar texto con nivel de zoom y estado
            cv2.putText(display_img, f"Zoom x{self.zoom_level}", (20, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Mostrar modo actual
            modo_texto = "SELECCIÓN" if self.modo_seleccion else "SEGUIMIENTO"
            cv2.putText(display_img, f"Modo: {modo_texto}", (20, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # En modo selección, mostrar todos los puntos
            if self.modo_seleccion and self.todos_los_puntos is not None:
                # Dibujar todos los puntos detectados
                for i, punto in enumerate(self.todos_los_puntos):
                    cv2.circle(display_img, (int(punto[0]), int(punto[1])), 4, (0, 0, 255), -1)
                    
                    # Mostrar números de los puntos
                    if i % 5 == 0:  # Mostrar solo algunos números para no saturar
                        cv2.putText(display_img, str(i), 
                                   (int(punto[0])+5, int(punto[1])+5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                
                # Dibujar líneas de la malla
                if len(self.todos_los_puntos) == self.patron_size[0] * self.patron_size[1]:
                    # Dibujar líneas horizontales
                    for i in range(self.patron_size[0]):
                        for j in range(self.patron_size[1]-1):
                            idx1 = i * self.patron_size[1] + j
                            idx2 = i * self.patron_size[1] + j + 1
                            pt1 = (int(self.todos_los_puntos[idx1][0]), int(self.todos_los_puntos[idx1][1]))
                            pt2 = (int(self.todos_los_puntos[idx2][0]), int(self.todos_los_puntos[idx2][1]))
                            cv2.line(display_img, pt1, pt2, (0, 255, 255), 1)
                    
                    # Dibujar líneas verticales
                    for i in range(self.patron_size[0]-1):
                        for j in range(self.patron_size[1]):
                            idx1 = i * self.patron_size[1] + j
                            idx2 = (i+1) * self.patron_size[1] + j
                            pt1 = (int(self.todos_los_puntos[idx1][0]), int(self.todos_los_puntos[idx1][1]))
                            pt2 = (int(self.todos_los_puntos[idx2][0]), int(self.todos_los_puntos[idx2][1]))
                            cv2.line(display_img, pt1, pt2, (0, 255, 255), 1)
            
            # En modo seguimiento, mostrar solo el punto activo
            elif not self.modo_seleccion and self.punto_activo is not None:
                # Dibujar punto seleccionado
                cv2.circle(display_img, (int(self.punto_activo[0]), int(self.punto_activo[1])), 
                          8, (255, 0, 0), -1)
                
                # Dibujar línea desde el centro de la cámara al punto
                cv2.line(display_img, (self.center_x, self.center_y), 
                        (int(self.punto_activo[0]), int(self.punto_activo[1])), 
                        (255, 255, 0), 1)
                
                # Mostrar información del punto
                texto_punto = f"Punto: ({self.punto_activo[0]:.1f}, {self.punto_activo[1]:.1f})"
                cv2.putText(display_img, texto_punto, (20, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Mostrar distancia al centro
                dx = self.punto_activo[0] - self.center_x
                dy = self.punto_activo[1] - self.center_y
                dist = np.sqrt(dx*dx + dy*dy)
                texto_dist = f"Distancia al centro: {dist:.1f} px"
                cv2.putText(display_img, texto_dist, (20, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Si hay punto inicial, mostrar offset
                if self.punto_inicial_zoom1 is not None:
                    # Dibujar punto inicial (referencia)
                    cv2.circle(display_img, (int(self.punto_inicial_zoom1[0]), int(self.punto_inicial_zoom1[1])), 
                              6, (0, 255, 0), 1)
                    
                    # Calcular offset
                    offset_x = (self.punto_activo[0] - self.center_x) - (self.punto_inicial_zoom1[0] - self.center_x)
                    offset_y = (self.punto_activo[1] - self.center_y) - (self.punto_inicial_zoom1[1] - self.center_y)
                    
                    # Mostrar offset
                    texto_offset = f"Offset: ({offset_x:.1f}, {offset_y:.1f}) px"
                    cv2.putText(display_img, texto_offset, (20, 150), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Dibujar trayectoria a través de los diferentes niveles de zoom
                display_img = self.dibujar_trayectoria_zoom(display_img)
            
            # Mostrar instrucciones
            h = display_img.shape[0]
            cv2.putText(display_img, "d: detectar tablero", (20, h-90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_img, "r: volver a selección", (20, h-70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_img, "s: guardar offset", (20, h-50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_img, "p: ver resultados / g: guardar", (20, h-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_img, "z: ingresar zoom actual", (20, h-110), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            
            # Mostrar imagen
            cv2.imshow("Seguimiento de Punto Único", display_img)
            key = cv2.waitKey(1) & 0xFF
            
            # Procesar teclas
            if key == ord('q'):
                break
            elif key == ord('d'):
                self.detectar_esquinas()
            elif key == ord('s'):
                if self.punto_activo is not None:
                    rospy.loginfo(f"[Zoom x{self.zoom_level}] Posición actual del punto: ({self.punto_activo[0]:.2f}, {self.punto_activo[1]:.2f})")
                else:
                    rospy.logwarn("No hay punto activo para mostrar su posición.")
            elif key == ord('p'):
                self.mostrar_resultados()
            elif key == ord('g'):
                self.guardar_resultados()
            elif key == ord('r'):
                self.modo_seleccion = True
                rospy.loginfo("Modo de selección activado - todos los puntos visibles")
            elif key == ord('f'):
                if not self.modo_seleccion and self.punto_activo is not None:
                    self.seguir_punto()
            elif key in [ord('+'), ord('=')]:
                self.zoom_level = min(20.0, self.zoom_level + 1.0)
                rospy.loginfo(f"Ajusta manualmente el zoom a x{self.zoom_level}")
            elif key == ord('-'):
                self.zoom_level = max(1.0, self.zoom_level - 1.0)
                rospy.loginfo(f"Ajusta manualmente el zoom a x{self.zoom_level}")
            elif key == ord('z'):
                try:
                    entrada = input("🔍 Ingresar nivel de zoom actual (1.0 a 20.0): ")
                    nuevo_zoom = float(entrada)
                    if 1.0 <= nuevo_zoom <= 20.0:
                        # Guardar ángulo antes del cambio de zoom
                        if not self.modo_seleccion and self.punto_activo is not None and self.punto_inicial_zoom1 is not None:
                            fov_h_anterior, fov_v_anterior = self.FOVS.get(self.zoom_level, (None, None))
                            if fov_h_anterior and fov_v_anterior:
                                altura, anchura = self.image.shape[:2]
                                px_per_deg_x = anchura / fov_h_anterior
                                px_per_deg_y = altura / fov_v_anterior
                                
                                # Convertir posición actual a ángulos
                                vector_actual = (self.punto_activo[0] - self.center_x, 
                                                self.punto_activo[1] - self.center_y)
                                angulo_x = vector_actual[0] / px_per_deg_x
                                angulo_y = vector_actual[1] / px_per_deg_y
                                
                                # Actualizar zoom
                                self.zoom_level = nuevo_zoom
                                rospy.loginfo(f"🔧 Nivel de zoom actualizado manualmente: {self.zoom_level}x")
                                
                                # Obtener nuevo FOV
                                fov_h_nuevo, fov_v_nuevo = self.FOVS.get(nuevo_zoom, (None, None))
                                if fov_h_nuevo and fov_v_nuevo:
                                    # Calcular nuevos píxeles por grado
                                    px_per_deg_x_nuevo = anchura / fov_h_nuevo
                                    px_per_deg_y_nuevo = altura / fov_v_nuevo
                                    
                                    # Calcular nueva posición manteniendo el mismo ángulo
                                    nuevo_vector_x = angulo_x * px_per_deg_x_nuevo
                                    nuevo_vector_y = angulo_y * px_per_deg_y_nuevo
                                    
                                    nueva_x = self.center_x + nuevo_vector_x
                                    nueva_y = self.center_y + nuevo_vector_y
                                    
                                    # Actualizar posición del punto
                                    self.punto_activo = (nueva_x, nueva_y)
                                    rospy.loginfo(f"Punto actualizado según cambio de zoom: ({nueva_x:.1f}, {nueva_y:.1f})")
                                    
                                    # Calcular offset
                                    self.calcular_offset()
                            else:
                                self.zoom_level = nuevo_zoom
                                rospy.logwarn(f"FOV no definido para zoom anterior {self.zoom_level}")
                        else:
                            self.zoom_level = nuevo_zoom
                            rospy.loginfo(f"🔧 Nivel de zoom actualizado manualmente: {self.zoom_level}x")
                    else:
                        rospy.logwarn("❌ Nivel de zoom fuera de rango (1.0 - 20.0)")
                except ValueError:
                    rospy.logwarn("❌ Entrada inválida para zoom")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        SinglePointTracker()
    except rospy.ROSInterruptException:
        pass