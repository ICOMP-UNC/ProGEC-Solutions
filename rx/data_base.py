import sqlite3
def configurar_base_datos(nombre_bd="sensores.db"):
    """Configura la base de datos SQLite y crea la tabla si no existe."""
    conexion = sqlite3.connect(nombre_bd)
    cursor = conexion.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS datos_sensores (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            tipo TEXT NOT NULL,
            valor REAL NOT NULL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    conexion.commit()
    return conexion