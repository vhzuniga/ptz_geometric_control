#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import urllib3
import time

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32
from video_stream.srv import SetZoomLevel, SetZoomLevelResponse

class CameraZoomController:
    def __init__(self):
        rospy.init_node("camera_zoom_controller")
        self.camera_hostname = "192.168.0.90"
        self.time_full_zoom = 15.8
        self.current_zoom_level = 1.0
        self.max_zoom_level = 20.0
        self.zoom_time_factor = self.time_full_zoom / (self.max_zoom_level - 1.0)

        self.http = urllib3.PoolManager()
        self.zoom_in = f"http://{self.camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomin&0"
        self.zoom_out = f"http://{self.camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomout&0"
        self.zoom_stop = f"http://{self.camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomstop&0"

        self.zoom_pub = rospy.Publisher('current_zoom_level', Float32, queue_size=1)

        rospy.Service('zoom_in', Trigger, self.handle_zoom_in)
        rospy.Service('zoom_out', Trigger, self.handle_zoom_out)
        rospy.Service('zoom_stop', Trigger, self.handle_zoom_stop)
        rospy.Service('zoom_calibrate', Trigger, self.handle_zoom_calibrate)
        rospy.Service('set_zoom', SetZoomLevel, self.handle_set_zoom_level)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_zoom_level)
        rospy.loginfo("Controlador de Zoom de Cámara inicializado.")
        self.zoom_callibrator()

    def publish_zoom_level(self, _):
        self.zoom_pub.publish(Float32(data=self.current_zoom_level))

    def zoom_callibrator(self):
        try:
            self.http.request('GET', self.zoom_out, timeout=2.50)
            rospy.loginfo("Calibrando zoom (mínimo)...")
            time.sleep(self.time_full_zoom)
            self.http.request('GET', self.zoom_stop, timeout=2.50)
            self.current_zoom_level = 1.0
            rospy.loginfo("Calibración completa. Nivel: 1.0x")
            return True
        except Exception as e:
            rospy.logerr(f"Error en calibración: {e}")
            return False

    def zoom_in_manual(self, seconds):
        if self.current_zoom_level >= self.max_zoom_level:
            rospy.logwarn("Ya estás en zoom máximo.")
            return False
        try:
            self.http.request('GET', self.zoom_in, timeout=2.50)
            time.sleep(seconds)
            self.http.request('GET', self.zoom_stop, timeout=2.50)
            self.current_zoom_level = min(self.current_zoom_level + seconds / self.zoom_time_factor, self.max_zoom_level)
            return True
        except Exception as e:
            rospy.logerr(f"Error zoom in: {e}")
            return False

    def zoom_out_manual(self, seconds):
        if self.current_zoom_level <= 1.0:
            rospy.logwarn("Ya estás en zoom mínimo.")
            return False
        try:
            self.http.request('GET', self.zoom_out, timeout=2.50)
            time.sleep(seconds)
            self.http.request('GET', self.zoom_stop, timeout=2.50)
            self.current_zoom_level = max(self.current_zoom_level - seconds / self.zoom_time_factor, 1.0)
            return True
        except Exception as e:
            rospy.logerr(f"Error zoom out: {e}")
            return False

    def set_zoom_level(self, target_level):
        if not 1.0 <= target_level <= self.max_zoom_level:
            rospy.logerr("Zoom fuera de rango.")
            return False
        if abs(self.current_zoom_level - target_level) < 0.1:
            return True

        time_needed = abs(target_level - self.current_zoom_level) * self.zoom_time_factor
        try:
            cmd = self.zoom_in if target_level > self.current_zoom_level else self.zoom_out
            self.http.request('GET', cmd, timeout=2.50)
            time.sleep(time_needed)
            self.http.request('GET', self.zoom_stop, timeout=2.50)
            self.current_zoom_level = target_level
            return True
        except Exception as e:
            rospy.logerr(f"Error set_zoom: {e}")
            return False

    def handle_zoom_in(self, _):
        success = self.zoom_in_manual(1.0)
        return TriggerResponse(success=success, message=f"Zoom in: {self.current_zoom_level:.2f}x")

    def handle_zoom_out(self, _):
        success = self.zoom_out_manual(1.0)
        return TriggerResponse(success=success, message=f"Zoom out: {self.current_zoom_level:.2f}x")

    def handle_zoom_stop(self, _):
        try:
            self.http.request('GET', self.zoom_stop, timeout=2.50)
            return TriggerResponse(success=True, message="Zoom detenido")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_zoom_calibrate(self, _):
        success = self.zoom_callibrator()
        return TriggerResponse(success=success, message="Calibrado" if success else "Fallo")

    def handle_set_zoom_level(self, req):
        success = self.set_zoom_level(req.level)
        return SetZoomLevelResponse(success=success, message=f"Zoom: {req.level:.2f}x" if success else "Fallo")

if __name__ == '__main__':
    try:
        CameraZoomController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
