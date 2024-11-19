# Guía de Instalación

En este archivo se encuentran los pasos para compilar, ejecutar y subir a la placa
STM32F1 el sistema de protección ProGEC.

## Requisitos Previos

Debes tener en tu computadora:

- [Python](https://www.python.org/downloads/) 
- [PlatformIO](https://platformio.org/install)
- [Make](https://www.gnu.org/software/make/)

## Instalación de PlatformIO

1. Descargue e instale [Visual Studio Code](https://code.visualstudio.com/).
2. Instale la extensión de PlatformIO [acá](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).

# Clonar el Repositorio de ProGEC-Solutions

Desde Github, clone el repositorio:

```
git clone https://github.com/ICOMP-UNC/ProGEC-Solutions.git
cd ProGEC-Solutions
```

# Configurar el Proyecto en VSCode

1. Abra el pryecto en Visual Studio Code.
2. Ejecute las siguientes instrucciones:

```
make clean
make build
make upload
```

# Para ejecutar la monitorización
1. ejecutar los siguientes comandos par instalar las librerias necesarias
```
pip install pyserial 
pip install sendgrid

```
Ejecute 

```
python rx/receptor_serie.py
```


