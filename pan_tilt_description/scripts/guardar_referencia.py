#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
import sys
import signal
import os
import json
import rospkg

class RayoVisualizador:
    def __init__(self):
        rospy.init_node("visualizador_puntos_rayo")
        
        # Publicador para los marcadores
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10, latch=True)
        
        # Variables para la flecha
        self.origen = None
        self.direccion = None
        self.cloud = None
        self.puntos_seleccionables = []
        self.puntos_filtrados = []
        self.ultimo_marker = None  # Para depuración
        
        # Variable para controlar si estamos en modo selección
        self.interaccion_activa = True
        
        rospack = rospkg.RosPack()
        ruta_pkg = rospack.get_path("pan_tilt_description")
        self.archivo_puntos = os.path.join(ruta_pkg, "config", "puntos_guardados.json")
        # Cargar puntos guardados anteriormente
        self.puntos_guardados = self.cargar_puntos_guardados()
        rospy.loginfo(f"Cargados {len(self.puntos_guardados)} puntos guardados previamente")
        
        # Configurar manejo de señales para Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Asegurarse de que el publicador está listo
        rospy.sleep(0.5)
        
        # Mostrar los puntos guardados al inicio
        if len(self.puntos_guardados) > 0:
            self.publicar_puntos_guardados()
            rospy.loginfo("Puntos guardados publicados en RViz")
        
        # Preguntar al usuario qué quiere hacer
        self.modo = self.seleccionar_modo()
        
        if self.modo == "cargar":
            # Si solo quiere cargar los puntos, terminar aquí
            rospy.loginfo("Modo de visualización: puntos cargados y mostrados en RViz.")
            return
        
        # Si llegamos aquí, estamos en modo "añadir"
        rospy.loginfo("Modo seleccionado: añadir nuevo punto.")
        
        # Suscribirse al marcador de la flecha celeste
        self.flecha_sub = rospy.Subscriber("/visualization_marker", Marker, self.callback_marker, queue_size=10)
        
        # Suscribirse a la nube de puntos
        self.cloud_sub = rospy.Subscriber("/cloud_pcd", PointCloud2, self.callback_nube, queue_size=1)
        
        # Esperar a que lleguen los datos necesarios
        timeout = 20  # Aumentado a 20 segundos
        start_time = time.time()
        marcador_detectado = False
        
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if self.ultimo_marker is not None and not marcador_detectado:
                rospy.loginfo(f"Marcador recibido: ns={self.ultimo_marker.ns}, id={self.ultimo_marker.id}, tipo={self.ultimo_marker.type}")
                if hasattr(self.ultimo_marker, 'color'):
                    rospy.loginfo(f"Color: R={self.ultimo_marker.color.r:.2f}, G={self.ultimo_marker.color.g:.2f}, B={self.ultimo_marker.color.b:.2f}")
                if hasattr(self.ultimo_marker, 'points') and len(self.ultimo_marker.points) > 0:
                    rospy.loginfo(f"Puntos: {len(self.ultimo_marker.points)}")
                marcador_detectado = True
            
            if self.origen is not None and self.direccion is not None and self.cloud is not None:
                # Detener las suscripciones para evitar mensajes continuos
                self.detener_suscripciones()
                
                rospy.loginfo("✅ Datos recibidos. Procesando puntos...")
                self.visualizar_puntos_rayo()
                
                # Seleccionar punto
                self.interactuar_con_usuario()
                return
            
            rospy.loginfo("Esperando datos... (flecha=%s, nube=%s)", 
                         "OK" if self.origen is not None else "Pendiente",
                         "OK" if self.cloud is not None else "Pendiente")
            rospy.sleep(1.0)
        
        # Si llega aquí, es por timeout
        rospy.logwarn("Timeout esperando datos.")
        
        if self.cloud is None:
            rospy.logerr("❌ No se recibió la nube de puntos.")
            return
        
        # Verificar si al menos recibimos algún marcador
        if self.ultimo_marker is not None:
            rospy.loginfo("El último marcador recibido fue:")
            rospy.loginfo(f"ns={self.ultimo_marker.ns}, id={self.ultimo_marker.id}, tipo={self.ultimo_marker.type}")
            if hasattr(self.ultimo_marker, 'color'):
                rospy.loginfo(f"Color: R={self.ultimo_marker.color.r:.2f}, G={self.ultimo_marker.color.g:.2f}, B={self.ultimo_marker.color.b:.2f}")
            
            # Preguntar si desea usar este marcador
            respuesta = input("\n¿Deseas usar el último marcador recibido como flecha? (s/n): ").strip().lower()
            if respuesta == 's' or respuesta == 'si':
                if len(self.ultimo_marker.points) >= 2:
                    # Extraer origen y destino
                    origen = np.array([self.ultimo_marker.points[0].x, self.ultimo_marker.points[0].y, self.ultimo_marker.points[0].z])
                    destino = np.array([self.ultimo_marker.points[1].x, self.ultimo_marker.points[1].y, self.ultimo_marker.points[1].z])
                    
                    # Calcular dirección normalizada
                    direccion = destino - origen
                    norm = np.linalg.norm(direccion)
                    if norm > 0:
                        self.origen = origen
                        self.direccion = direccion / norm
                        
                        # Visualizar puntos artificiales
                        self.puntos_seleccionables = self.generar_puntos_artificiales(max_dist=8.0, num_puntos=10)
                        self.limpiar_marcadores_temporales()
                        self.publicar_todos_los_marcadores()
                        
                        # Interactuar con el usuario
                        self.interactuar_con_usuario()
                        return
        
        # Si no tenemos flecha, crear una predeterminada
        rospy.logwarn("No se detectó la flecha celeste. Creando flecha por defecto.")
        respuesta = input("\n¿Deseas crear una flecha por defecto? (s/n): ").strip().lower()
        if respuesta != 's' and respuesta != 'si':
            rospy.loginfo("Operación cancelada por el usuario.")
            return
        
        # Crear flecha por defecto
        self.origen = np.array([0.06, 0.0, 0.12])  # Origen aproximado
        self.direccion = np.array([1.0, 0.0, 0.0])  # Dirección hacia adelante
        
        # Visualizar puntos artificiales
        self.puntos_seleccionables = self.generar_puntos_artificiales(max_dist=8.0, num_puntos=10)
        self.limpiar_marcadores_temporales()
        self.publicar_todos_los_marcadores()
        
        # Interactuar con el usuario
        self.interactuar_con_usuario()
    
    def seleccionar_modo(self):
        """Permite al usuario seleccionar si quiere cargar puntos o añadir uno nuevo"""
        if len(self.puntos_guardados) == 0:
            # Si no hay puntos guardados, solo se puede añadir
            print("\nNo hay puntos guardados. Solo se puede seleccionar el modo 'añadir'.")
            return "añadir"
        
        print("\n===== MODO DE OPERACIÓN =====")
        print("1: Cargar y mostrar los puntos guardados")
        print("2: Añadir un nuevo punto")
        
        while True:
            try:
                opcion = input("\nSelecciona el modo (1/2): ").strip()
                if opcion == "1":
                    return "cargar"
                elif opcion == "2":
                    return "añadir"
                else:
                    print("Opción no válida. Por favor, introduce 1 o 2.")
            except KeyboardInterrupt:
                print("\nOperación cancelada por el usuario.")
                sys.exit(0)
    
    def cargar_puntos_guardados(self):
        """Carga los puntos guardados desde un archivo JSON"""
        if not os.path.exists(self.archivo_puntos):
            return []
        
        try:
            with open(self.archivo_puntos, 'r') as f:
                datos = json.load(f)
                
            puntos = []
            for item in datos:
                punto = np.array(item['punto'])
                nombre = item['nombre']
                puntos.append((punto, nombre))
            
            return puntos
        except Exception as e:
            rospy.logerr(f"Error al cargar puntos guardados: {e}")
            return []
    
    def guardar_puntos_en_archivo(self):
        """Guarda los puntos en un archivo JSON"""
        try:
            datos = []
            for punto, nombre in self.puntos_guardados:
                datos.append({
                    'punto': punto.tolist(),
                    'nombre': nombre
                })
            
            with open(self.archivo_puntos, 'w') as f:
                json.dump(datos, f, indent=2)
            
            rospy.loginfo(f"Puntos guardados en {self.archivo_puntos}")
        except Exception as e:
            rospy.logerr(f"Error al guardar puntos: {e}")
    
    def signal_handler(self, sig, frame):
        rospy.loginfo("Programa terminado por el usuario (Ctrl+C)")
        sys.exit(0)
    
    def detener_suscripciones(self):
        """Detiene las suscripciones para evitar mensajes continuos"""
        if hasattr(self, 'flecha_sub') and self.flecha_sub is not None:
            self.flecha_sub.unregister()
        if hasattr(self, 'cloud_sub') and self.cloud_sub is not None:
            self.cloud_sub.unregister()
        rospy.loginfo("Suscripciones detenidas para facilitar la interacción")
    
    def callback_marker(self, marker):
        """Callback para detectar específicamente la flecha celeste"""
        # Guardar el último marker recibido para depuración
        self.ultimo_marker = marker
        
        # Buscar específicamente flecha celeste (namespace="camera_direction", id=3)
        if marker.ns == "camera_direction" and marker.id == 3:
            if len(marker.points) >= 2:
                # Verificar si es de color celeste
                if marker.color.r < 0.1 and marker.color.g > 0.9 and marker.color.b > 0.9:
                    # Extraer origen y destino
                    origen = np.array([marker.points[0].x, marker.points[0].y, marker.points[0].z])
                    destino = np.array([marker.points[1].x, marker.points[1].y, marker.points[1].z])
                    
                    # Calcular dirección normalizada
                    direccion = destino - origen
                    norm = np.linalg.norm(direccion)
                    if norm > 0:
                        self.origen = origen
                        self.direccion = direccion / norm
                        rospy.loginfo(f"✅ Flecha celeste detectada: origen={self.origen}, dirección={self.direccion}")
    
    def callback_nube(self, cloud_msg):
        """Callback para la nube de puntos"""
        if self.cloud is None:
            self.cloud = cloud_msg
            tamaño = len(list(pc2.read_points(cloud_msg, skip_nans=True)))
            rospy.loginfo(f"Nube de puntos recibida con {tamaño} puntos")
    
    def publicar_puntos_guardados(self):
        """Publica solo los puntos guardados permanentes"""
        if not self.puntos_guardados:
            return
            
        marker_array = MarkerArray()
        
        # Añadir los puntos guardados
        for i, (punto, nombre) in enumerate(self.puntos_guardados):
            # Añadir el marcador del punto
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "puntos_guardados"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(punto[0])
            marker.pose.position.y = float(punto[1])
            marker.pose.position.z = float(punto[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            
            # Color naranja para los puntos guardados
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker.lifetime = rospy.Duration(0)  # Permanente
            marker_array.markers.append(marker)
            
            # Añadir el texto con el nombre
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "nombres_guardados"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(punto[0])
            text_marker.pose.position.y = float(punto[1])
            text_marker.pose.position.z = float(punto[2]) + 0.05  # 5cm encima del punto
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.05  # Altura del texto
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = nombre
            
            text_marker.lifetime = rospy.Duration(0)  # Permanente
            marker_array.markers.append(text_marker)
        
        # Publicar los marcadores
        self.marker_pub.publish(marker_array)
    
    def generar_puntos_artificiales(self, max_dist=8.0, num_puntos=10):
        """Genera puntos artificiales equidistantes a lo largo del rayo"""
        puntos_artificiales = []
        
        # Distancia mínima desde el origen para empezar a generar puntos
        distancia_minima = 0.5
        
        # Distancia entre puntos artificiales
        paso = (max_dist - distancia_minima) / (num_puntos - 1) if num_puntos > 1 else max_dist
        
        # Generar puntos equidistantes a lo largo del rayo
        for i in range(num_puntos):
            distancia = distancia_minima + i * paso
            # Calcular posición del punto artificial
            punto = self.origen + self.direccion * distancia
            puntos_artificiales.append((punto, distancia))
        
        rospy.loginfo(f"Generados {len(puntos_artificiales)} puntos artificiales a lo largo del rayo")
        return puntos_artificiales
    
    def visualizar_puntos_rayo(self):
        """Visualiza los puntos que intersectan con el rayo o puntos artificiales"""
        if self.origen is None or self.direccion is None:
            rospy.logerr("Faltan datos de la flecha")
            return
        
        # Generar directamente puntos artificiales
        self.puntos_seleccionables = self.generar_puntos_artificiales(max_dist=8.0, num_puntos=10)
        
        # Limpiar solo los marcadores temporales (puntos amarillos y etiquetas)
        self.limpiar_marcadores_temporales()
        
        # Publicar los puntos guardados y los nuevos puntos seleccionables
        self.publicar_todos_los_marcadores()
    
    def limpiar_marcadores_temporales(self):
        """Elimina solo los marcadores temporales (puntos amarillos y etiquetas)"""
        marker_array = MarkerArray()
        
        # Crear marcadores de eliminación solo para los espacios de nombres temporales
        namespaces = ["puntos_rayo", "etiquetas"]
        
        for ns in namespaces:
            for i in range(100):  # Un número suficientemente grande
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()
                marker.ns = ns
                marker.id = i
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
        
        # Publicar marcadores de eliminación
        self.marker_pub.publish(marker_array)
        rospy.sleep(0.1)  # Dar tiempo para que RViz procese la eliminación
    
    def publicar_todos_los_marcadores(self):
        """Publica todos los marcadores: puntos guardados y puntos seleccionables"""
        marker_array = MarkerArray()
        
        # Primero añadir los puntos guardados (puntos permanentes)
        for i, (punto, nombre) in enumerate(self.puntos_guardados):
            # Añadir el marcador del punto
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "puntos_guardados"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(punto[0])
            marker.pose.position.y = float(punto[1])
            marker.pose.position.z = float(punto[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            
            # Color naranja para los puntos guardados
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker.lifetime = rospy.Duration(0)  # Permanente
            marker_array.markers.append(marker)
            
            # Añadir el texto con el nombre
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "nombres_guardados"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(punto[0])
            text_marker.pose.position.y = float(punto[1])
            text_marker.pose.position.z = float(punto[2]) + 0.05  # 5cm encima del punto
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.05  # Altura del texto
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = nombre
            
            text_marker.lifetime = rospy.Duration(0)  # Permanente
            marker_array.markers.append(text_marker)
        
        # Solo mostrar puntos seleccionables si estamos en modo selección
        if self.interaccion_activa and self.puntos_seleccionables:
            # Filtrar puntos que ya están guardados
            puntos_filtrados = []
            for i, (punto, distancia) in enumerate(self.puntos_seleccionables):
                ya_guardado = False
                for punto_guardado, _ in self.puntos_guardados:
                    if np.linalg.norm(punto - punto_guardado) < 0.01:  # 1cm de tolerancia
                        ya_guardado = True
                        break
                
                if not ya_guardado:
                    puntos_filtrados.append((punto, distancia, i))
            
            # Ahora añadir los puntos seleccionables filtrados (amarillos con etiquetas)
            tamaño_esfera = 0.03
            
            for idx, (punto, distancia, original_idx) in enumerate(puntos_filtrados):
                # Marcador para la esfera
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "puntos_rayo"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position.x = float(punto[0])
                marker.pose.position.y = float(punto[1])
                marker.pose.position.z = float(punto[2])
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = tamaño_esfera
                marker.scale.y = tamaño_esfera
                marker.scale.z = tamaño_esfera
                
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                
                marker.lifetime = rospy.Duration(0)
                marker_array.markers.append(marker)
                
                # Marcador para la etiqueta numérica
                text_marker = Marker()
                text_marker.header.frame_id = "base_link"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "etiquetas"
                text_marker.id = idx
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose.position.x = float(punto[0])
                text_marker.pose.position.y = float(punto[1])
                text_marker.pose.position.z = float(punto[2]) + 0.05  # 5cm sobre el punto
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.05  # Altura del texto
                
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                # Mostrar el índice que corresponde a la lista filtrada
                text_marker.text = str(idx)
                
                text_marker.lifetime = rospy.Duration(0)
                marker_array.markers.append(text_marker)
            
            self.puntos_filtrados = [p[:2] for p in puntos_filtrados]  # Guardar solo punto y distancia
            
            rospy.loginfo(f"Publicados {len(self.puntos_guardados)} puntos guardados y {len(self.puntos_filtrados)} puntos seleccionables")
        else:
            rospy.loginfo(f"Publicados {len(self.puntos_guardados)} puntos guardados")
        
        # Publicar todos los marcadores
        self.marker_pub.publish(marker_array)
    
    def mostrar_opciones_puntos(self):
        """Muestra en consola los puntos disponibles para seleccion"""
        # Filtrar puntos que ya están guardados
        puntos_filtrados = []
        for i, (punto, distancia) in enumerate(self.puntos_seleccionables):
            ya_guardado = False
            for punto_guardado, _ in self.puntos_guardados:
                if np.linalg.norm(punto - punto_guardado) < 0.01:  # 1cm de tolerancia
                    ya_guardado = True
                    break
            
            if not ya_guardado:
                puntos_filtrados.append((punto, distancia))
        
        if not puntos_filtrados:
            rospy.logwarn("No hay puntos nuevos para seleccionar")
            return False
        
        print("\n===== PUNTOS DISPONIBLES =====")
        for i, (punto, distancia) in enumerate(puntos_filtrados):
            print(f"{i}: Punto en ({punto[0]:.4f}, {punto[1]:.4f}, {punto[2]:.4f}) - Distancia: {distancia:.4f}m")
        
        self.puntos_filtrados = puntos_filtrados
        return True
    
    def interactuar_con_usuario(self):
        """Interactúa con el usuario para seleccionar un punto"""
        if not self.puntos_seleccionables:
            rospy.logwarn("No hay puntos para seleccionar")
            return
        
        # Mostrar opciones y verificar si hay puntos nuevos disponibles
        if not self.mostrar_opciones_puntos():
            rospy.loginfo("No hay puntos nuevos para seleccionar. Finalizando.")
            return
        
        try:
            # Limpiar cualquier entrada pendiente
            import termios, tty, fcntl, os
            fd = sys.stdin.fileno()
            oldattr = termios.tcgetattr(fd)
            newattr = oldattr[:]
            newattr[3] = newattr[3] & ~termios.ICANON
            termios.tcsetattr(fd, termios.TCSANOW, newattr)
            oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
            
            try:
                # Limpiar buffer
                while sys.stdin.read(1): pass
            except IOError:
                pass
            
            # Restaurar
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldattr)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
            
            # Solicitar índice del punto
            indice = int(input("\nIngresa el número del punto que deseas seleccionar: "))
            
            # Verificar si el índice es válido
            if indice < 0 or indice >= len(self.puntos_filtrados):
                rospy.logerr(f"Índice inválido. Debe estar entre 0 y {len(self.puntos_filtrados) - 1}")
                return
            
            # Solicitar nombre para el punto
            nombre = input("Ingresa un nombre para este punto: ")
            
            # Desactivar modo interacción después de seleccionar
            self.interaccion_activa = False
            
            # Guardar el punto seleccionado
            self.guardar_punto_seleccionado(indice, nombre)
            
        except ValueError:
            rospy.logerr("Por favor, ingresa un número válido")
        except EOFError:
            rospy.logerr("Error en la entrada. Programa terminado.")
        except KeyboardInterrupt:
            rospy.loginfo("Operacion cancelada por el usuario")
    
    def guardar_punto_seleccionado(self, indice, nombre):
        """Guarda el punto seleccionado en la lista de puntos guardados y en el archivo"""
        if indice < 0 or indice >= len(self.puntos_filtrados):
            rospy.logerr("Índice de punto inválido")
            return
        
        # Obtener el punto seleccionado de la lista filtrada
        punto, _ = self.puntos_filtrados[indice]
        
        # Añadir a la lista de puntos guardados
        self.puntos_guardados.append((punto, nombre))
        
        # Guardar en el archivo
        self.guardar_puntos_en_archivo()
        
        # Limpiar marcadores temporales
        self.limpiar_marcadores_temporales()
        
        # Publicar todos los marcadores (incluyendo el nuevo punto guardado)
        self.publicar_todos_los_marcadores()
        
        rospy.loginfo(f"Punto '{nombre}' guardado: {punto}")
        rospy.loginfo(f"Total de puntos guardados: {len(self.puntos_guardados)}")

if __name__ == '__main__':
    try:
        visualizador = RayoVisualizador()
        rospy.loginfo("Programa finalizado.")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error no manejado: {e}")
        import traceback
        traceback.print_exc()