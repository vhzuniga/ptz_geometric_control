#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class SinglePhotoCapture:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('single_photo_capture', anonymous=True)
        
        # Crear puente para convertir entre imágenes ROS y OpenCV
        self.bridge = CvBridge()
        
        # Variable para almacenar la imagen
        self.image_received = False
        self.image = None
        
        # Suscribirse al tema de la cámara
        self.image_sub = rospy.Subscriber('/datavideo/video', Image, self.image_callback)
        
    def image_callback(self, data):
        # Solo procesa la primera imagen recibida
        if not self.image_received:
            try:
                self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.image_received = True
                # Desuscribirse después de recibir la imagen
                self.image_sub.unregister()
            except Exception as e:
                rospy.logerr("Error al convertir la imagen: %s", str(e))
    
    def save_photo(self, filename="captura.jpg"):
        timeout = rospy.Duration(5)
        start_time = rospy.Time.now()

        while not self.image_received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)

        if self.image_received:
            img = self.image.copy()

            # 📍 Centro de imagen (verde)
            cv2.drawMarker(img, (640, 360), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Centro imagen", (650, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 📍 Punto extra (fucsia)
            cv2.drawMarker(img, (643, int(387.5)), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, "Punto extra", (653, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            cv2.imwrite(filename, img)
            rospy.loginfo("Foto guardada como: %s", filename)
            return True
        else:
            rospy.logerr("No se pudo capturar ninguna imagen dentro del tiempo límite")
            return False


if __name__ == '__main__':
    try:
        # Crear la instancia de captura
        capture = SinglePhotoCapture()
        
        # Generar nombre de archivo con timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"captura_{timestamp}.jpg"
        
        # Guardar la foto
        if capture.save_photo(filename):
            rospy.loginfo("Proceso completado exitosamente")
        
        # No usamos rospy.spin() porque solo queremos una imagen
        
    except rospy.ROSInterruptException:
        pass