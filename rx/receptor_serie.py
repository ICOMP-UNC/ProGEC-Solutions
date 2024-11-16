# -*- coding: utf-8 -*-

import sys
import time
import serial
import serial.tools.list_ports 
import smtplib
import send_alert
import data_base

sys.stdout.reconfigure(encoding='utf-8')

# Configuración de umbrales
UMBRAL_VIB_ALTA = 30
UMBRAL_HUM_BAJA = 10

# Parámetros de configuración del puerto
baudrate = 9600
bits = serial.EIGHTBITS
stop = serial.STOPBITS_ONE
parity = serial.PARITY_NONE
time_out = 1

def configurar_puerto_serial():
    """Busca y configura automáticamente el primer puerto serial disponible."""
    puertos_disponibles = serial.tools.list_ports.comports()
    for puerto in puertos_disponibles:
        try:
            serial_obj = serial.Serial(
                port=puerto.device,
                baudrate=baudrate,
                bytesize=bits,
                parity=parity,
                stopbits=stop,
                timeout=time_out
            )
            print(f"Conectado exitosamente al puerto {puerto.device}")
            return serial_obj 
        except serial.SerialException:
            continue  # Intenta el siguiente puerto si falla la conexion

    print("Error: No se encontro ningún puerto serial disponible.")
    return None

def recibir_datos(serial_obj, conexion):
    """Recibe y procesa datos del puerto serial."""
    buffer = bytearray()
    try:
        while True:
            if serial_obj.in_waiting > 0:
                buffer.extend(serial_obj.read(serial_obj.in_waiting))
                while len(buffer) >= 2:
                    data_bytes = [buffer.pop(0), buffer.pop(0)]
                    procesar_datos(data_bytes, conexion)    
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("Conexión cerrada por el usuario.")
    except Exception as e:  
        print(f"Error inesperado: {e}")
    finally:
        if serial_obj.is_open:
            serial_obj.close()
        conexion.close()

def procesar_datos(data_bytes, conexion):
    """Procesa los datos recibidos en un arreglo de 2 bytes."""
    try:
        hum_value = data_bytes[0]
        vib_value = data_bytes[1]

        print(f"Vibracion: {vib_value}, Humedad: {hum_value}")

        if vib_value > UMBRAL_VIB_ALTA :
            print(f"⚠️ Alerta: Vibracion fuera de rango seguro: {vib_value}")
           # send_alert("vib", vib_value)

        if hum_value < UMBRAL_HUM_BAJA :
            print(f"⚠️ Alerta: Humedad fuera de rango seguro: {hum_value}")
            #send_alert("hum", hum_value)

#        guardar_datos(conexion, "vib", vib_value)
#        guardar_datos(conexion, "hum", hum_value)

    except Exception as e:
        print(f"Error inesperado: {e}")
    
#def guardar_datos(conexion, tipo, valor):
#    cursor = conexion.cursor()
#    cursor.execute('''
#        INSERT INTO datos_sensores (tipo, valor) VALUES (?, ?)
#    ''', (tipo, valor))
#    conexion.commit()


# Configuración inicial
if __name__ == "__main__":
    serial_obj = configurar_puerto_serial()
    
    if serial_obj is None:
        print("Error: No se pudo abrir ningún puerto serial.")
        exit(1)
    
    conexion = data_base.configurar_base_datos()
    print("Esperando datos y almacenando en la base de datos...")
    recibir_datos(serial_obj, conexion)
