#!/usr/bin/env python3
import rospy
import yaml
from visualization_msgs.msg import Marker

def cargar_puntos(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
        return data.get('puntos', {})

if __name__ == '__main__':
    rospy.init_node('publicador_marcadores_con_texto')
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)

    puntos = cargar_puntos('/home/hugo/ros_ws/src/pan_tilt_ros/pan_tilt_description/config/puntos_guardados.yaml')

    for i, (nombre, coords) in enumerate(puntos.items()):
        x, y, z = coords['x'], coords['y'], coords['z']

        # 🎯 Punto visual
        punto = Marker()
        punto.header.frame_id = "base_link"
        punto.header.stamp = rospy.Time.now()
        punto.ns = "puntos"
        punto.id = i
        punto.type = Marker.SPHERE
        punto.action = Marker.ADD
        punto.pose.position.x = x
        punto.pose.position.y = y
        punto.pose.position.z = z
        punto.pose.orientation.w = 1.0
        punto.scale.x = 0.05
        punto.scale.y = 0.05
        punto.scale.z = 0.05
        punto.color.r = 1.0
        punto.color.g = 0.0
        punto.color.b = 0.0
        punto.color.a = 1.0
        punto.lifetime = rospy.Duration(0)

        pub.publish(punto)
        rospy.sleep(0.05)

        # 🏷 Etiqueta
        texto = Marker()
        texto.header.frame_id = "base_link"
        texto.header.stamp = rospy.Time.now()
        texto.ns = "etiquetas"
        texto.id = 1000 + i
        texto.type = Marker.TEXT_VIEW_FACING
        texto.action = Marker.ADD
        texto.pose.position.x = x
        texto.pose.position.y = y
        texto.pose.position.z = z + 0.08  # Un poco más alto que antes
        texto.pose.orientation.w = 1.0
        texto.scale.z = 0.07
        texto.color.r = 1.0
        texto.color.g = 1.0
        texto.color.b = 1.0
        texto.color.a = 1.0
        texto.text = nombre
        texto.lifetime = rospy.Duration(0)

        pub.publish(texto)
        rospy.loginfo(f"✅ Publicado: {nombre} en ({x:.3f}, {y:.3f}, {z:.3f})")

    rospy.spin()
