#Autor: Alejandro Martinez Hermoso
#Estacion de tierra para graficar las medidas recibidas por telemetria en tiempo real 

import serial
import matplotlib.pyplot as plt
from drawnow import *
import atexit
import string
import pandas as pd
import numpy as np

#Listas para almacenar los datos que queremos
values = [] #Lista principal donde se almacenan todos los datos recibidos

#Listas que extraen los valores necesarios de la lista "values" para cada distinto tipo de variable medida
FechaHora = [] 
Humedad = []
Temperatura = []
Presion = []
CO2 = []
TVOC = []
PM10 = []
PM2_5 = []
PPM_CO = []
PPM_GLP = []
Altitud = []
Orientacion = []
SatelitesVisibles = []
Latitud = []
Longitud = []


plt.ion() #Modo interactivo ON
cnt=0

serialArduino = serial.Serial('COM4', 57600) #Comunicación con el puerto serie que recibe 
#                                             la informacion (modulo receptor de telemetria)

def plotValues(): #Para representar por medio de graficas las medidas obtenidas
    
    fig, axs = plt.subplots(3,3) #Se genera una matriz de 3x3 graficos
    fig = plt.figure(figsize=(3,10))
    fig.suptitle('Medidas de los sensores')
    
    axs[0,0].plot(FechaHora,Humedad,'.-',color = 'tab:blue') #Se construye el grafico
    axs[0,0].plot(FechaHora[999],Humedad[999],'o',color = 'tab:blue') #Se introduce el ultimo valor de la lista remarcado con un punto grande
    axs[0,0].grid(color='k', ls = '-.', lw = 0.25) #rejilla
    axs[0,0].plot(FechaHora[Humedad.index(max(Humedad))],max(Humedad),'x',color = 'k') #Para resaltar el valor maximo
    
    #Los comentarios anteriores sirven para as siguientes sentencias de representacion
    axs[0,1].plot(FechaHora,Temperatura,'.-',color = 'red')
    axs[0,1].plot(FechaHora[999],Temperatura[999],'o',color = 'red')
    axs[0,1].grid(color='k', ls = '-.', lw = 0.25)
    axs[0,1].plot(FechaHora[Temperatura.index(max(Temperatura))],max(Temperatura),'x',color = 'k')
    
    axs[0,2].plot(FechaHora,Presion,'.-',color = 'orange')
    axs[0,2].plot(FechaHora[999],Presion[999],'o',color = 'orange')
    axs[0,2].grid(color='k', ls = '-.', lw = 0.25)
    axs[0,2].plot(FechaHora[Presion.index(max(Presion))],max(Presion),'x',color = 'k')
    
    axs[1,0].plot(FechaHora,CO2,'.-',color = 'red')
    axs[1,0].plot(FechaHora,TVOC,'.-',color = 'green')
    axs[1,0].plot(FechaHora[999],CO2[999],'o',color = 'red')
    axs[1,0].plot(FechaHora[999],TVOC[999],'o',color = 'green')
    axs[1,0].grid(color='k', ls = '-.', lw = 0.25)
    axs[1,0].plot(FechaHora[TVOC.index(max(TVOC))],max(TVOC),'x',color = 'k')
    axs[1,0].plot(FechaHora[CO2.index(max(CO2))],max(CO2),'x',color = 'k')
    
    axs[1,2].plot(FechaHora,PPM_CO,'.-',color = 'pink')
    axs[1,2].plot(FechaHora,PPM_GLP,'.-',color = 'purple')
    axs[1,2].plot(FechaHora[999],PPM_CO[999],'o',color = 'pink')
    axs[1,2].plot(FechaHora[999],PPM_GLP[999],'o',color = 'purple')
    axs[1,2].grid(color='k', ls = '-.', lw = 0.25)
    axs[1,2].plot(FechaHora[PPM_CO.index(max(PPM_CO))],max(PPM_CO),'x',color = 'k')
    axs[1,2].plot(FechaHora[PPM_GLP.index(max(PPM_GLP))],max(PPM_GLP),'x',color = 'k')
    
    axs[1,1].plot(FechaHora,PM2_5,'.-',color = 'orange')
    axs[1,1].plot(FechaHora,PM10,'.-',color = 'blue')
    axs[1,1].plot(FechaHora[999],PM2_5[999],'o',color = 'orange')
    axs[1,1].plot(FechaHora[999],PM10[999],'o',color = 'blue')
    axs[1,1].grid(color='k', ls = '-.', lw = 0.25)
    axs[1,1].plot(FechaHora[PM2_5.index(max(PM2_5))],max(PM2_5),'x',color = 'k')
    axs[1,1].plot(FechaHora[PM10.index(max(PM10))],max(PM10),'x',color = 'k')
    
    axs[2,0].plot(FechaHora,Altitud,'.-',color = 'tab:brown')
    axs[2,0].plot(FechaHora[49],Altitud[49],'o',color = 'tab:brown')
    axs[2,0].grid(color='k', ls = '-.', lw = 0.25)
    axs[2,0].plot(FechaHora[Altitud.index(max(Altitud))],max(Altitud),'o',color = 'k')
    
    axs[2,1].plot(FechaHora,Orientacion,'.-',color = 'tab:green')
    axs[2,1].plot(FechaHora[999],Orientacion[999],'o',color = 'tab:green')
    axs[2,1].grid(color='k', ls = '-.', lw = 0.25)
    axs[2,1].plot(FechaHora[Orientacion.index(max(Orientacion))],max(Orientacion),'x',color = 'k')
    
    axs[2,2].plot(FechaHora,SatelitesVisibles,'.-',color ="y")
    axs[2,2].plot(FechaHora[999],SatelitesVisibles[999],'o',color ="y")
    axs[2,2].grid(color='k', ls = '-.', lw = 0.25)
    axs[2,2].plot(FechaHora[SatelitesVisibles.index(max(SatelitesVisibles))],max(SatelitesVisibles),'x',color = 'k')
    
    
    
    
    
    
    
    
    axs[0,0].title.set_text('Humedad (%)') #Titulo del grafico
    axs[0,0].title.set_color(color='tab:blue')
    axs[0,0].set_ylim(30,90) #Limites eje y
    axs[0,0].tick_params(axis='x',which='major',bottom=False,top=False,labelbottom=False) #Para no saturar con todas las marcas horarias

    axs[0,1].title.set_text('Temperatura (ºC)')
    axs[0,1].title.set_color(color='tab:red')
    axs[0,1].set_ylim(0,40)
    axs[0,1].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    
    axs[0,2].title.set_text('Presión (hPa)')
    axs[0,2].title.set_color(color='tab:orange')
    axs[0,2].set_ylim(920,1100)
    axs[0,2].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    
    axs[1,0].title.set_text('CO2-TVOC (ppm)')
    axs[1,0].title.set_color(color='tab:blue')
    axs[1,0].legend(('CO2', 'TVOC'),prop = {'size':7}, loc='upper left')
    axs[1,0].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    
    axs[1,1].title.set_text('PM 2,5-10,0 (ug/m3)')
    axs[1,1].title.set_color(color='tab:red')
    axs[1,1].set_ylim(0,40)
    axs[1,1].legend(('PM 2,5', 'PM 10'),prop = {'size': 7}, loc='upper left')
    axs[1,1].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    
    axs[1,2].title.set_text('CO-GLP (ppm)')
    axs[1,2].title.set_color(color='tab:orange')
    axs[1,2].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    axs[1,2].legend(('CO', 'GLP'),prop = {'size':7}, loc='upper left') #Leyenda para distinguir las dos lineas
    
    axs[2,0].title.set_text('Altitud (m)')
    axs[2,0].title.set_color(color='tab:blue')
    axs[2,0].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    
    
    axs[2,1].title.set_text('Orientación (deg)')
    axs[2,1].title.set_color(color='tab:red')
    axs[2,1].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False)
    axs[2,1].set_ylim(0,360)
    
    axs[2,2].title.set_text('Satélites visibles')
    axs[2,2].title.set_color(color='tab:orange')
    axs[2,2].set_ylim(0,24)
    axs[2,2].tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False,size=3)
    axs[2,2].text(0,21,FechaHora[999],fontsize=16,color='k') #Para resaltar la marca horaria de la ultima medida
    axs[2,2].text(0,20,'-----------------------------',fontsize=16,color='tab:grey')
    
    fig.tight_layout()
    plt.subplots_adjust(left=0.125, #Configuracion extra de la matriz de graficas
                        bottom=0.1, 
                        right=0.9, 
                        top=0.9, 
                        wspace=1.5, 
                        hspace=0.5)
    #plt.plot(FechaHora,Latitud,color = 'tab:blue',marker='o',linestyle = 'dotted')
    #plt.legend(loc='upper right')



def doAtExit(): #Para cuando se interrumpa la conexion
    serialArduino.close()
    print("Close serial")
    print("serialArduino.isOpen() = " + str(serialArduino.isOpen()))

    atexit.register(doAtExit)

print("serialArduino.isOpen() = " + str(serialArduino.isOpen())) #Muestra si hay conexion con el puerto de telemetria

#pre-load dummy data
for i in range(0,1000): #Se inicializan las listas, algunas con valores definidos previamente para no empezar en 0
    values.append(0)
    FechaHora.append(0)
    Humedad.append(30)
    Temperatura.append(0)
    Presion.append(925)
    CO2.append(0)
    TVOC.append(0)
    PM10.append(0)
    PM2_5.append(0)
    PPM_CO.append(0)
    PPM_GLP.append(0)
    Altitud.append(600)
    Orientacion.append(0)
    SatelitesVisibles.append(0)
    Latitud.append(0)
    Longitud.append(0)
while True:
    while (serialArduino.inWaiting()==0):
        pass
    print("readline()")
    
    
    
    
    valueRead = serialArduino.readline().decode('utf-8') #Se lee el contenido arrojado en el puerto serie
    values=valueRead.split(',') #Separa los valores cuando encuentre una coma (el String enviado esta separado por comas)
    FechaHora.append(values[0]) #Introduce la columna de la hora en su lista corespondiente
    FechaHora.pop(0) #Elimina el primero ya que se ha introducido uno nuevo en la ultima posicion
    
    Lat = float(values[1]) #Para pasarlo a float y poder trabajar con el dato 
    Latitud.append(Lat)
    Latitud.pop(0)
    
    Long = float(values[2])
    Longitud.append(Long)
    Longitud.pop(0)
    
    Alt = float(values[3])
    Altitud.append(Alt)
    Altitud.pop(0)
   
    Ori= float(values[4])
    Orientacion.append(Ori)
    Orientacion.pop(0)
   
    SatVis = float(values[5])
    SatelitesVisibles.append(SatVis)
    SatelitesVisibles.pop(0)
    
    Hum = float(values[6])
    Humedad.append(Hum)
    Humedad.pop(0)
    
    Temp = float(values[7])
    Temperatura.append(Temp)
    Temperatura.pop(0)
    
    Pres = float(values[8])
    Presion.append(Pres)
    Presion.pop(0)
   
    co2_ = float(values[9])
    CO2.append(co2_)
    CO2.pop(0)
    
    tvoc_ = float(values[10])
    TVOC.append(tvoc_)
    TVOC.pop(0)
    
    glp = float(values[11])
    PPM_GLP.append(glp)
    PPM_GLP.pop(0)
    
    co = float(values[12])
    PPM_CO.append(co)
    PPM_CO.pop(0)
    
    p25 = float(values[13])
    PM2_5.append(p25)
    PM2_5.pop(0)
    
    pm10_ = float(values[14])
    PM10.append(pm10_)
    PM10.pop(0)
    
    drawnow(plotValues) #Se ejecuta la funcion de graficar
  
      