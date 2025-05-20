#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import urllib3
import time
import sys
import signal

calibration_running = True

def signal_handler(sig, frame):
    global calibration_running
    calibration_running = False
    print("\n¡Calibración detenida!")

def calibrate_zoom_time():
    global calibration_running
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("\n=== CALIBRADOR DE TIEMPO DE ZOOM ===")
    print("Este script medirá cuánto tiempo tarda la cámara en ir de zoom mínimo a máximo.")
    print("Observa la cámara y presiona Ctrl+C cuando ya no veas cambios en el zoom.")
    
    camera_hostname = "192.168.0.90"
    http = urllib3.PoolManager()
    
    zoom_in = f"http://{camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomin&0"
    zoom_out = f"http://{camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomout&0"
    zoom_stop = f"http://{camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomstop&0"
    
    print("\nPASO 1: Llevando la cámara al zoom mínimo...")
    try:
        http.request('GET', zoom_out, timeout=2.50)
        for i in range(20, 0, -1):
            print(f"\rEsperando: {i} segundos restantes...", end="")
            time.sleep(1)
        http.request('GET', zoom_stop, timeout=2.50)
        print("\nCámara en zoom mínimo.")
    except Exception as e:
        print(f"\nError al establecer zoom mínimo: {e}")
        return
    
    print("\nPASO 2: Iniciando medición de zoom mínimo a máximo...")
    print("Presiona Ctrl+C cuando observes que el zoom ya no aumenta más.")
    
    start_time = time.time()
    
    try:
        http.request('GET', zoom_in, timeout=2.50)
        while calibration_running:
            elapsed = time.time() - start_time
            print(f"\rTiempo transcurrido: {elapsed:.2f} segundos", end="")
            time.sleep(0.1)
        elapsed_time = time.time() - start_time
        http.request('GET', zoom_stop, timeout=2.50)
        print(f"\n\nRESULTADO: El tiempo total para zoom completo es: {elapsed_time:.2f} segundos")
        print("Usa este valor como time_full_zoom en tu controlador.")
    except Exception as e:
        print(f"\nError durante la calibración: {e}")
        try:
            http.request('GET', zoom_stop, timeout=2.50)
        except:
            pass

if __name__ == '__main__':
    try:
        calibrate_zoom_time()
    except Exception as e:
        print(f"Error inesperado: {e}")
