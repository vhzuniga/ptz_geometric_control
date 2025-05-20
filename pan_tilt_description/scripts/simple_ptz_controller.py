#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import math
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from pan_tilt_msgs.msg import PanTiltCmdDeg
import tf2_geometry_msgs

class SimplePTZController:
    def __init__(self):
        rospy.init_node('simple_ptz_controller')

        # Configurar publicadores y suscriptores
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.ptz_publisher = rospy.Publisher('/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=10)
        
        # Suscribirse a los puntos seleccionados en RViz
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        # Configurar TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))  # Buffer más grande
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Límites de movimiento
        self.PAN_MIN_DEG = -60
        self.PAN_MAX_DEG = 60
        self.TILT_MIN_DEG = -60
        self.TILT_MAX_DEG = 60
        
        # Timer para publicar la flecha celeste constantemente
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_camera_direction)
        
        rospy.loginfo("Controlador PTZ simple iniciado. Usa 'Publish Point' en RViz.")
        
        # Esperar un momento para que TF se inicialice completamente
        rospy.sleep(1.0)

    def point_callback(self, msg):
        try:
            # Usar el frame_id del mensaje para la transformación
            target_frame_id = "base_link"
            source_frame_id = msg.header.frame_id
            
            # Obtener transformación y aplicarla
            transform = self.tf_buffer.lookup_transform(
                target_frame_id, source_frame_id, 
                rospy.Time(0), rospy.Duration(1.0))
            
            target_point = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            # Obtener posición de la cámara
            camera_position = self.get_camera_position()
            if camera_position is None:
                rospy.logwarn("No se pudo obtener posición de la cámara")
                return
            
            # Publicar una esfera roja en el punto seleccionado
            self.publish_sphere(target_point.point)
            
            # Mostrar flecha azul desde la cámara hacia el punto
            self.publish_arrow(camera_position, target_point.point, "direction_arrow", 1, (0.0, 0.0, 1.0))
            
            # Mostrar flecha amarilla desde la cámara hasta el punto
            self.publish_arrow(camera_position, target_point.point, "camera_to_point", 2, (1.0, 1.0, 0.0))
            
            # Calcular ángulos pan/tilt
            dx = target_point.point.x - camera_position.x
            dy = target_point.point.y - camera_position.y
            dz = target_point.point.z - camera_position.z
            
            # Calcular ángulos en grados
            pan_deg = math.degrees(math.atan2(dy, dx))
            tilt_deg = math.degrees(math.atan2(dz, math.sqrt(dx**2 + dy**2)))
            
            # Limitar ángulos
            pan_deg = max(self.PAN_MIN_DEG, min(self.PAN_MAX_DEG, pan_deg))
            tilt_deg = max(self.TILT_MIN_DEG, min(self.TILT_MAX_DEG, tilt_deg))
            
            # Enviar comando al PTZ
            self.move_ptz(pan_deg, -tilt_deg)
            
            rospy.loginfo("Moviendo a pan=%.2f°, tilt=%.2f°", pan_deg, -tilt_deg)
            
        except Exception as e:
            rospy.logerr("Error en point_callback: %s", e)

    def get_camera_position(self):
        """Obtiene la posición de la cámara desde TF"""
        try:
            # Intentar obtener transformación desde base_link a camera_visor
            trans = self.tf_buffer.lookup_transform(
                "base_link", "camera_visor", 
                rospy.Time(0), rospy.Duration(0.5))
            
            # Extraer la posición
            position = Point()
            position.x = trans.transform.translation.x
            position.y = trans.transform.translation.y
            position.z = trans.transform.translation.z
            
            return position
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            # Fallar silenciosamente después del primer intento
            if not hasattr(self, 'tf_error_reported'):
                rospy.logwarn("Error obteniendo posición de la cámara: %s", e)
                self.tf_error_reported = True
            return None

    def publish_camera_direction(self, event):
        """Publica una flecha celeste que muestra la dirección de la cámara"""
        try:
            # Obtener posición actual de la cámara
            camera_position = self.get_camera_position()
            if camera_position is None:
                return  # Salir silenciosamente si no podemos obtener la posición
            
            # Intentar obtener transformación para la dirección
            camera_forward = PointStamped()
            camera_forward.header.stamp = rospy.Time(0)
            camera_forward.header.frame_id = "camera_visor"
            camera_forward.point.x = 20.0  # 2 metros hacia adelante en X
            camera_forward.point.y = 0.0
            camera_forward.point.z = 0.0
            
            transform = self.tf_buffer.lookup_transform(
                "base_link", "camera_visor", 
                rospy.Time(0), rospy.Duration(0.1))
            
            # Transformar el punto
            forward_transformed = tf2_geometry_msgs.do_transform_point(camera_forward, transform)
            
            # Publicar flecha celeste
            self.publish_arrow(camera_position, forward_transformed.point, 
                              "camera_direction", 3, (0.0, 1.0, 1.0))
            
        except Exception:
            # No reportar errores para evitar spam en la consola
            pass

    def publish_sphere(self, point):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Posición de la esfera
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        
        # Tamaño de la esfera
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Color rojo
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Duración (0 = para siempre)
        marker.lifetime = rospy.Duration(0)
        
        self.marker_pub.publish(marker)

    def publish_arrow(self, start, end, namespace, id, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Origen y punto final de la flecha
        marker.points = [start, end]
        
        # Tamaño de la flecha
        marker.scale.x = 0.02  # diámetro del eje
        marker.scale.y = 0.05  # diámetro de la punta
        marker.scale.z = 0.1   # longitud de la punta
        
        # Color (r,g,b)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)

    def move_ptz(self, pan_deg, tilt_deg):
        cmd = PanTiltCmdDeg()
        cmd.yaw = pan_deg    # pan
        cmd.pitch = tilt_deg  # tilt
        cmd.speed = 30        # velocidad como entero
        self.ptz_publisher.publish(cmd)

if __name__ == '__main__':
    try:
        SimplePTZController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass