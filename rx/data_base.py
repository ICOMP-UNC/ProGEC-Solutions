import os
from datetime import datetime

def configurar_base_datos(nombre_archivo="datos_sensores.txt"):
    """Configura el archivo de texto para almacenar los datos de sensores."""
    # Si el archivo no existe, crea el archivo y agrega una cabecera de columnas
    if not os.path.exists(nombre_archivo):
        with open(nombre_archivo, "w") as archivo:
            archivo.write("id,tipo,valor,timestamp\n")  # Cabecera del archivo
    return nombre_archivo  # Retorna el nombre del archivo como identificador

def guardar_datos(archivo, tipo, valor):
    """Guarda el tipo y valor en el archivo con un ID incremental y la fecha/hora actual."""
    # Obtener el último ID y calcular el nuevo ID
    nuevo_id = 1
    if os.path.exists(archivo):
        with open(archivo, "r") as archivo_lectura:
            lineas = archivo_lectura.readlines()
            if len(lineas) > 1:  # Si hay datos, calcula el nuevo ID
                ultimo_id = int(lineas[-1].split(",")[0])
                nuevo_id = ultimo_id + 1

    # Fecha y hora actual
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Guardar los datos en una nueva línea
    with open(archivo, "a") as archivo_escritura:
        archivo_escritura.write(f"{nuevo_id},{tipo},{valor},{timestamp}\n")

    print(f"Datos guardados: ID={nuevo_id}, Tipo={tipo}, Valor={valor}, Timestamp={timestamp}")
