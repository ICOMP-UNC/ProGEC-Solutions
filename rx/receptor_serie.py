import serial
import serial.tools.list_ports 
import smtplib
import send_alert
import data_base

# Configuración de umbrales
UMBRAL_VIB_ALTA = 80.0
UMBRAL_VIB_BAJA = 10.0
UMBRAL_HUM_ALTA = 90.0
UMBRAL_HUM_BAJA = 10.0

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
            continue  # Intenta el siguiente puerto si falla la conexión

    print("Error: No se encontró ningún puerto serial disponible.")
    return None

def recibir_datos(serial_obj, conexion):
    """Recibe y procesa datos del puerto serial."""
    try:
        while True:
            if serial_obj.in_waiting >= 4:
                data_bytes = [ord(serial_obj.read(1)) for _ in range(4)]
                procesar_datos(data_bytes, conexion)
    except KeyboardInterrupt:
        print("Conexión cerrada por el usuario.")
    except Exception as e:
        print(f"Error inesperado: {e}")
    finally:
        if serial_obj.is_open:
            serial_obj.close()
        conexion.close()

def procesar_datos(data_bytes, conexion):
    """Procesa los datos recibidos en un arreglo de 4 bytes."""
    try:
        vib_value = (data_bytes[0] << 8) | data_bytes[1]
        hum_value = (data_bytes[2] << 8) | data_bytes[3]

        print(f"Vibración: {vib_value}, Humedad: {hum_value}")

        if vib_value > UMBRAL_VIB_ALTA or vib_value < UMBRAL_VIB_BAJA:
            print(f"⚠️ Alerta: Vibración fuera de rango seguro: {vib_value}")
            send_alert("vib", vib_value)

        if hum_value > UMBRAL_HUM_ALTA or hum_value < UMBRAL_HUM_BAJA:
            print(f"⚠️ Alerta: Humedad fuera de rango seguro: {hum_value}")
            send_alert("hum", hum_value)

        guardar_datos(conexion, "vib", vib_value)
        guardar_datos(conexion, "hum", hum_value)

    except ValueError:
        print("Error: formato de datos incorrecto")
    except Exception as e:
        print(f"Error inesperado: {e}")

def guardar_datos(conexion, tipo, valor):
    cursor = conexion.cursor()
    cursor.execute('''
        INSERT INTO datos_sensores (tipo, valor) VALUES (?, ?)
    ''', (tipo, valor))
    conexion.commit()

# Configuración inicial
if __name__ == "__main__":
    serial_obj = configurar_puerto_serial()
    
    if serial_obj is None:
        print("Error: No se pudo abrir ningún puerto serial.")
        exit(1)
    
    conexion = data_base.configurar_base_datos()
    print("Esperando datos y almacenando en la base de datos...")
    recibir_datos(serial_obj, conexion)
