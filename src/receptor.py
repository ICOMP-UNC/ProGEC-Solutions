import serial
import smtplib
import send_alert
import data_base

# Configuración de umbrales
UMBRAL_VIB_ALTA = 80.0
UMBRAL_VIB_BAJA = 10.0
UMBRAL_HUM_ALTA = 90.0
UMBRAL_HUM_BAJA = 10.0

#parametros de configuracion del puerto
baudrate = 9600
bits = serial.EIGHTBITS
stop = serial.STOPBITS_ONE 
parity = serial.PARITY_NONE
time_out = 1
def configurar_puerto_serial(puerto):
    try:
        """Inicializa la conexión serial con los parámetros correctos."""
        return serial.Serial(
            port=puerto,
            baudrate=baudrate,
            bytesize=bits,
            parity=parity,
            stopbits=stop,
            timeout=time_out
        )
    except serial.SerialException as e:
        print(f"Error al configurar el puerto serial: {e}")
        return None


def recibir_datos(serial_obj, conexion):
    """Recibe y procesa datos del puerto serial."""
    try:
        while True:
            if serial_obj.in_waiting > 0:
                datos = serial_obj.readline().decode('utf-8').strip()
                procesar_datos(datos, conexion)
    except KeyboardInterrupt:
        print("Conexión cerrada por el usuario.")
    except Exception as e:
        print(f"Error inesperado: {e}")
    
    finally:
        if serial_obj.is_open:
            serial_obj.close()
        conexion.close()

def procesar_datos(serial_obj, conexion):
    """Recibe y procesa los datos enmascarados de 16 bits."""
    try:
        # Lee los 4 bytes: 2 para vibración (vib) y 2 para humedad (hum)
        if serial_obj.in_waiting >= 4:
            # Lee dos bytes de vibración
            byte_vib_high = ord(serial_obj.read(1))  # Parte alta de vib
            byte_vib_low = ord(serial_obj.read(1))   # Parte baja de vib

            # Reconstruir el valor original de vibración de 16 bits
            vib_value = (byte_vib_high << 8) | byte_vib_low

            # Leer los dos bytes de humedad
            byte_hum_high = ord(serial_obj.read(1))  # Parte alta de hum
            byte_hum_low = ord(serial_obj.read(1))   # Parte baja de hum

            # Reconstruir el valor original de humedad de 16 bits
            hum_value = (byte_hum_high << 8) | byte_hum_low

            print(f"Vibración: {vib_value}, Humedad: {hum_value}")

            # Validaciones de alerta
            if vib_value > UMBRAL_VIB_ALTA or vib_value < UMBRAL_VIB_BAJA:
                print(f"⚠️ Alerta: Vibración fuera de rango seguro: {vib_value}")
                send_alert("vib", vib_value)

            if hum_value > UMBRAL_HUM_ALTA or hum_value < UMBRAL_HUM_BAJA:
                print(f"⚠️ Alerta: Humedad fuera de rango seguro: {hum_value}")
                send_alert("hum", hum_value)

            # Guardar los datos en la base de datos independientemente de las alertas
            guardar_datos(conexion, "vib", vib_value)
            guardar_datos(conexion, "hum", hum_value)

    except ValueError:
        print("Error: formato de datos incorrecto")
    except Exception as e:
        print(f"Error inesperado: {e}")



def guardar_datos(conexion, tipo, valor):
    """Guarda el tipo y valor en la base de datos con la fecha/hora actual."""
    cursor = conexion.cursor()
    cursor.execute('''
        INSERT INTO datos_sensores (tipo, valor) VALUES (?, ?)
    ''', (tipo, valor))
    conexion.commit()

# Configuración inicial
if __name__ == "__main__":
    puerto = "COM3"  # Cambia según tu configuración
    serial_obj = configurar_puerto_serial(puerto)
    
    # Verificar si la conexión serial fue exitosa
    if serial_obj is None:
        print("Error: No se pudo abrir el puerto serial.")
        exit(1)  # Termina el programa si no se puede abrir el puerto serial
    
    conexion = data_base.configurar_base_datos()  # Inicia la conexión con la base de datos
    print("Esperando datos y almacenando en la base de datos...")
    recibir_datos(serial_obj, conexion)
