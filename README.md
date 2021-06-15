# TFG
Incorporación del código, ficheros de las pruebas y leyendas


INTRODUCCIÓN

En este repositorio se encuentran los archivos diseñados para la implementación del Trabajo de Fin de Grado .

Se trata de una estación móvil para controlar la calidad del aire que se pueda embeber en un dron.

Para su realización se ha utilizado el siguiente hardware:

Sensores: MQ-2 , MQ-135, CCS811, BME280, SDS011.

Placa: Arduino Mega 2560 + Sensor Shield v.1.0.

Pixhawk 2.4.8 + GPS + Módulo de telemetría + RTC DS3231 + módulo SD para arduino .

CONTENIDO

Código para la placa Arduino para extraer las medidas de los sensores, guardar la información y enviarla por telemetría (también se extraen los datos de la posición de Pixhawk).

Código para la estación de representación en tiempo real de los datos obtenidos por telemetría.

Código para el análisis en diferido de los ficheros CSV generados (mapa con ubicación de la medida + representación por umbral).

Ficheros CSV generados durante la implementación del trabajo.

Leyendas generadas para el código de la estación de análisis en diferido


MANUAL DE FUNCIONAMIENTO (PROXIMAMENTE)     
