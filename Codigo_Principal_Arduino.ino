/* Autor: ALejandro Martinez Hermoso
 * Codigo para la placa arduino para extraer datos de los sensores, ubicación y orientacion
 * Guardado en SD y enviado por telemetria
 */
#include <Arduino.h> //Librerias utilizadas
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <BMx280I2C.h>
#include "Adafruit_CCS811.h"
#include <mavlink2.h>
#include <SoftwareSerial.h>
#include <MQUnifiedsensor.h>


#define I2C_ADDRESS 0x76
//#define Lon_buf 9 --> Alternativa SDS011 

#define pm25Pin 44 //Pines de entrada PWM para la lectura de concentración de particulas
#define pm10Pin 45

#define placa "Arduino MEGA"
#define Voltage_Resolution 5
#define type "MQ-2"
#define type1 "MQ-135"
#define ADC_Bit_Resolution 10 
/***********************************************************************/
const int MQ2_PIN_A = A0; //Entrada analógica en pin 0
const int MQ2_PIN_D = 2; //Entrada digital 2
const int MQ135_PIN_A = A1; //Entrada analógica en pin 0
const int MQ135_PIN_D = 3; //Entrada digital 2
int RL_VALUE_MQ2 = 1; //Resistencia RL equivalente a 1 kohm
int RL_VALUE_MQ135 = 1; //Resistencia RL equivalente a 1 kohm
float primera=0;
int primero=0;
float RO_FACTOR_AIRE_LIMPIO =9.73; //Calibrado con otro pequeño código
float RO_FACTOR_AIRE_LIMPIO_MQ135 =3.8; //Calibrado con otro pequeño código
MQUnifiedsensor MQ2(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ2_PIN_A, type); //Objeto del sensor 
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN_A, type1); //Objeto del sensor
/**********************************************************************/
/**TOMA DE DATOS (CALIBRACION )**/
int MUESTRAS_CALIBRACION=100;                    
int TIEMPO_MUESTREO_CALIBRACION=100;                                                                      
      
/**********************************************************************/
/**Inicializacion variables de particulas**/
float PM2_5Val = 0.0;
float PM10Val = 0.0;
long PM2_5Val_A = 0;
long PM10Val_A = 0;
int lectura=0; //Para hacer que se lean los demás sensores despues de cada medida del de particulas para que todo este sincronizado

/**********************************************************************/
Adafruit_CCS811 ccs; //Objeto del sensor CCS811
BMx280I2C bmx280(I2C_ADDRESS); ///Objeto del sensor BME280
float humedad;
float temperatura;
float presion;
float CS_CO2;
float CS_TVOC;
/**********************************************************************/
float latitud=0;
float longitud=0;
float altitud=0;
float orientacion=0;
int satelites=0;
float varianza_horizontal=0;
float varianza_vertical=0;
float varianza_altitud=0;
float varianza_orientacion=0;
int a=0;
int b=0;
String datos="";
/**********************************************************************/
File registro;
File registro_csv;
RTC_DS3231 rtc;
/**********************************************************************/

void setup() {
  
  MQ2.setRegressionMethod(1); 
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configuramos segun datasheet para el GLP
  MQ2.init();
  MQ135.setRegressionMethod(1); 
  MQ135.setA(605.18); MQ135.setB(-3.937); // Configuramos segun datasheet para el CO
  MQ135.init();  
  
  pinMode(pm25Pin, INPUT); 
  pinMode(pm10Pin, INPUT);
  Serial1.begin(9600); //Puertos RXTX RX1 TX1, para que se inicien y recoja datos
  Serial.begin(57600); //Para recibir los datos del Pixhawk
  Serial2.begin(57600);
  request_datastream(); //Solicitud de datos a Pixhawk (Al final se detalla)

//INICIO DEL BME280
  while (!Serial);
  Serial.println("Conectando con el BME280");
  Wire.begin();
  if (!bmx280.begin()){Serial.println("Ha fallado la conexión con el BME280");while (1);}
  if (bmx280.isBME280()) Serial.println("BME280 conectado con EXITO\n");
  bmx280.resetToDefaults(); 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  if (bmx280.isBME280())bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);



//INICIO DEL MQ-135
Serial.print("Calibrando el sensor MQ-135...");
  float calcR0 = 0;
  for(int i = 1; i<=MUESTRAS_CALIBRACION; i ++)
  {
    MQ135.update(); //Se actualiza el sensor con su libreria
    calcR0 += MQ135.calibrate(RO_FACTOR_AIRE_LIMPIO_MQ135);
    delay(TIEMPO_MUESTREO_CALIBRACION);
  }
  MQ135.setR0(calcR0/MUESTRAS_CALIBRACION);
  Serial.println("¡CALIBRADO!   ");
  Serial.println(calcR0/MUESTRAS_CALIBRACION);
  delay(3000);
  
//INICIO DEL MQ-2
  Serial.print("Calibrando el sensor MQ-2...");
  float calR0 = 0;
  for(int i = 1; i<=MUESTRAS_CALIBRACION; i ++)
  {
    MQ2.update(); 
    calR0 += MQ2.calibrate(RO_FACTOR_AIRE_LIMPIO);
    delay(TIEMPO_MUESTREO_CALIBRACION);

  }
  MQ2.setR0(calR0/MUESTRAS_CALIBRACION);
  Serial.println("¡CALIBRADO!   ");
  Serial.println(calR0/MUESTRAS_CALIBRACION);

 // delay(3000000);//ESPERA DE 30 MINUTOS PARA QUE LOS SENSORES ANALÓGICOS SE CALIENTEN UN POCO Y PUEDAN FUNCIONAR DE UNA FORMA MÁS PRECISA

//INICIO DEL CCS811
  Serial.println("Conectando con el CCS811");
  if(!ccs.begin()){Serial.println("No se ha podido iniciar el CCS811, revisa el cableado de la conexión");}
  else {Serial.println("CCS811 conectado con EXITO...");} 
  while(!ccs.available()); //Esperamos a que el sensor esté disponible 

//INICIO DE LA SD
  Serial.print("\nIniciando SD...");
  if(!SD.begin(11)){Serial.println("Error");}
  else Serial.println("SD iniciada");

//INICIO DEL RELOJ 
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //QUITAR EL COMENTARIO CUANDO EL RELOJ NO TENGA LA HORA CORRECTA PARA CALIBRARLO
  if (!rtc.begin()) {
  Serial.println(F("Problemas con el reloj"));
  while (1);}

}

/************************************************************************************/

void loop() 
{
DateTime now = rtc.now();
  if(now.second()!=13 and now.second()!=23 and now.second()!=33 and now.second()!=43 and now.second()!=53 ){ //Para no solapar la frecuencia de MAVLink y sensor de partículas
    a=0;b=0; //Para el bloqueo de sentencias a modo de "semáforo"
    MavLink_receive(); //Para recibir los datos del Pixhawk (Al final se detalla)
    mostrar(); }//Mandar por telemetría los datos y para mandarlos también al monitor serie
    
  else {SDS();
  //Función encargada de leer los valores del sensor de partículas

 
}}


/************************************************************************************/
/*Función para SOLICITAR datos a Pixhawk*/
void request_datastream(){
  
  uint8_t _system_id = 255; //id del sistema, tiene que ser 255
  uint8_t _component_id = 2; //Es válido cualquier número excepto el del id de Pixhawk
  uint8_t _target_system = 1; //id de Pixhawk, tiene que ser 1 
  uint8_t _target_component = 0; //0=todos    (1=sensores, etc)
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //Veces por segundo que se solicitan datos (HEX)
  uint8_t _start_stop = 1; //0=stop; 1=start

  mavlink_message_t mensaje; //mensaje del tipo mavlink
  uint8_t buf[MAVLINK_MAX_PACKET_LEN]; //buffer donde cargaremos el mensaje

  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &mensaje, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop); //Funcion de mavlink para solicitar el mensaje y lo carga en la variable "mensaje"
  uint16_t len = mavlink_msg_to_send_buffer(buf, &mensaje);

  Serial1.write(buf, len); //Escribimos los valores del buffer en el puerto serie que hemos declarado en el setup()

//Ya tenemos el mensaje que contiene los datos del pixhawk utilizando el protocolo MAVLink
//Ahora tenemos que desempaquetar los datos que queremos

  
}

/************************************************************************************/
/*Función para LEER los datos proporcionados por Pixhawk*/

void MavLink_receive(){

  mavlink_message_t mensaje;
  mavlink_status_t status;

  while(Serial1.available()){
    uint8_t c=Serial1.read();

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &mensaje, &status)){

    switch(mensaje.msgid){

       case MAVLINK_MSG_ID_ATTITUDE :
      {
        mavlink_attitude_t packet1;
        mavlink_msg_attitude_decode(&mensaje,&packet1);
        orientacion=packet1.yaw*(180/3.14159265); //Conversion a grados
        if(orientacion<0) orientacion=orientacion*-1.0+180.0; 
        
      
      }
      break;

       case MAVLINK_MSG_ID_GPS_RAW_INT: 
      {
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&mensaje, &packet);
         latitud=packet.lat/10000000.0; //para pasarlo a las unidades comunes (deg)
         longitud=packet.lon/10000000.0;
         altitud=packet.alt/1000.0; //Para dejarlo en metros
         satelites=packet.satellites_visible;
       
      }
      break;
       
        
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
      {
        mavlink_ekf_status_report_t packet2;
        mavlink_msg_ekf_status_report_decode(&mensaje, &packet2); 
        varianza_horizontal=packet2.pos_horiz_variance;
        varianza_vertical=packet2.pos_vert_variance;
        varianza_altitud=packet2.terrain_alt_variance;
        varianza_orientacion=packet2.compass_variance;
        a=1;
      }
      break;

    }
  }
  }
}

/*-----------------------------------------------------------------------------*/



void mostrar(){
  
 
  
  DateTime now = rtc.now();     
  char bufferFechaHora[19];sprintf(bufferFechaHora, "%02d/%02d/%02d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());

 int modo1=now.second()%10; //Cada 10 segundos
 int modo2=now.second(); //Cada minuto
 int modo3=now.second()%5; //Cada 5 segundos
 int modo4=now.second()%30; //Cada 30 segundos
 
  if(modo1==0){

  
   if(a==1) {
    
        Serial.print("\n\n\n\n\n------------------------------------------------------------------------------------------------------ "); Serial.println(bufferFechaHora);
        Serial.println("--------------------DATOS DEL GPS--------------------");
        Serial.print("Orientación: ");Serial.print(orientacion,2);Serial.println("º    (N=0º)");
        Serial.print("Latitud: ");Serial.print(latitud,7);Serial.println(" deg"); 
        Serial.print("Longitud: ");Serial.print(longitud,7);Serial.println(" deg");
        Serial.print("Altitud: ");Serial.print(altitud,3);Serial.println(" m");
        Serial.print("Satélites visibles: ");Serial.print(satelites);Serial.println(" Satélites");
        //Serial.print("Varianza posición horizontal: ");Serial.println(varianza_horizontal);
        //Serial.print("Varianza posición vertical: ");Serial.println(varianza_vertical);
        //Serial.print("Varianza altitud: ");Serial.println(varianza_altitud);
        //Serial.print("Varianza orientación: ");Serial.print(varianza_orientacion);Serial.println("\n");   
        b=1;a=0;
        }

  
  
  if(b==1){
  
  Serial.println("--------------------VARIABLES DEL ENTORNO--------------------");
  
  
/*MQ-135* Y MQ-2*/
  
  delay(300);
  float PPM_CO=0;
  MQ135.update();
  delay(100);
  PPM_CO=MQ135.readSensor(); //Extraemos valor 
  delay(100);
  
  float PPM_GLP=0;
  MQ2.update();
  delay(100);
  if(primero==0){primera=MQ2.readSensor();primero=1;} //CORRECCIÓN DE OFFSET
  PPM_GLP=MQ2.readSensor()-primera; //Corregido el offset
  delay(100);
  bool detectado_MQ2=1;
  detectado_MQ2=digitalRead(MQ2_PIN_D); 
  if(detectado_MQ2==0){Serial.print("GLP detectado -> ");}
  else Serial.print("GLP NO detectado -> ");
  Serial.print(PPM_GLP);
  Serial.println(" ppm");
  
  bool detectado_MQ135=1;
  detectado_MQ135=digitalRead(MQ135_PIN_D);  
  if(detectado_MQ135==0){Serial.print("CO detectado -> ");}
  else Serial.print("CO NO detectado -> ");
  Serial.print(PPM_CO);
  Serial.println(" ppm");

/*BME280*/
  if(!bmx280.measure()){Serial.println("No está midiendo");} //Indica error en la toma de datos
  do{delay(100);}while(!bmx280.hasValue()); //Pequeña espera para que tome los datos
  Serial.print("Presión: "); Serial.print(bmx280.getPressure64()/100);Serial.print(" hPa  "); presion=bmx280.getPressure64()/100;
  Serial.print("Temperatura: "); Serial.print(bmx280.getTemperature());Serial.print(" ºC  ");temperatura=bmx280.getTemperature();
  Serial.print("Humedad: "); Serial.print(bmx280.getHumidity());Serial.println(" %");humedad=bmx280.getHumidity()+4.24;
delay(300);
/*CCS811*/
  if(ccs.available()){
  if(!ccs.readData()){delay(300);Serial.print("CO2: ");Serial.print(ccs.geteCO2()); CS_CO2=ccs.geteCO2()-805.96; if(CS_CO2<400){CS_CO2=400;} ;Serial.print("ppm, TVOC: ");Serial.println(ccs.getTVOC());CS_TVOC=ccs.getTVOC();}
  else{Serial.println("ERROR!");while(1);}
  lectura=0;}

/*SDS011*/
  Serial.print("PM2.5: ");
  Serial.print(PM2_5Val,2);
  Serial.print(" µg/m3  ");
  Serial.print("  PM10 : ");
  Serial.print(PM10Val,2);
  Serial.println(" µg/m3");
  lectura=1;


 /*PARA LA SD*/
  registro= SD.open("registro.txt", FILE_WRITE);
  if(registro){
        registro.print("\n\n\n\n\n------------------------------------------------------------------------------------------------------ "); registro.println(bufferFechaHora);
        registro.println("--------------------DATOS DEL GPS--------------------");
        registro.print("Orientación: ");registro.print(orientacion,2);registro.println("º    (N=0º)");
        registro.print("Latitud: ");registro.print(latitud,7);registro.println(" deg"); 
        registro.print("Longitud: ");registro.print(longitud,7);registro.println(" deg");
        registro.print("Altitud: ");registro.print(altitud,3);registro.println(" m");
        registro.print("Satélites visibles ");registro.print(satelites);registro.println(" Satélites");
        //registro.print("Varianza posición horizontal: ");registro.println(varianza_horizontal);
        //registro.print("Varianza posición vertical: ");registro.println(varianza_vertical);
        //registro.print("Varianza altitud: ");registro.println(varianza_altitud);
        //registro.print("Varianza orientación: ");registro.print(varianza_orientacion);registro.println("\n");
        registro.println("--------------------VARIABLES DEL ENTORNO--------------------");
        
        /*BME280 CCS811*/
       if(!bmx280.measure()){registro.println("No está midiendo");} //Indica error en la toma de datos
       do{delay(100);}while(!bmx280.hasValue()); //Pequeña espera para que tome los datos
       registro.print("Presión: "); registro.print(bmx280.getPressure64()/100);registro.println(" hPa  ");
       registro.print("Temperatura: "); registro.print(bmx280.getTemperature());registro.println(" ºC  ");
       registro.print("Humedad: "); registro.print(bmx280.getHumidity());registro.println(" %");
       registro.print("CO2: ");registro.print(CS_CO2);registro.println("ppm"); registro.print("TVOC: ");registro.println(CS_TVOC);
              
        /*MQ-2*/        
        if(detectado_MQ2==0){registro.print("GLP detectado -> ");}
        else registro.print("GLP NO detectado -> ");
        registro.print(PPM_GLP);
        registro.println(" ppm");

        /*MQ-135*/
        detectado_MQ135=digitalRead(MQ135_PIN_D);  
        if(detectado_MQ135==0){registro.print("CO detectado -> ");}
        else registro.print("CO NO detectado -> ");
        registro.print(PPM_CO);
        registro.println(" ppm");  
       
      /*SDS011*/  
       registro.print("PM2.5: ");
       registro.print(PM2_5Val,2);
       registro.print(" µg/m3  ");
       registro.print("  PM10 : ");
       registro.print(PM10Val,2);
       registro.println(" µg/m3");

        
registro.close();

}


//A CONTINUACIÓN GENERAMOS EL STRING PARA GUARDADO EN EL CSV (SEPARADO POR COMAS) Y ENVIARLO POR TELEMETRIA
    datos+=String (bufferFechaHora);
    datos+=",";
    datos+=String (latitud,7);
    datos+=",";
    datos+=String (longitud,7);
    datos+=",";
    datos+=String (altitud,3);
    datos+=",";
    datos+=String (orientacion);
    datos+=",";
    datos+=String (satelites);
    datos+=",";
    datos+=String (humedad);
    datos+=",";
    datos+=String (temperatura);
    datos+=",";
    datos+=String (presion);
    datos+=",";
    datos+=String (CS_CO2);
    datos+=",";
    datos+=String (CS_TVOC);
    datos+=",";
    datos+=String (PPM_GLP,2);
    datos+=",";
    datos+=String (PPM_CO,2);
    datos+=",";
    datos+=String (PM2_5Val,2);
    datos+=",";
    datos+=String (PM10Val,2);
    
registro_csv=SD.open("registro.csv", FILE_WRITE);
if(registro_csv){
    registro_csv.println(datos); //ESCRITURA EN CSV
    registro_csv.close();}
    Serial2.println(datos); //ENVIO POR TELEMETRIA
    datos="";


}
}
}

/*-----------------------------------------------------------------------------*/

void SDS(){


delay(100);
PM2_5Val_A=(pulseIn(pm25Pin, HIGH, 1500000)/ 1000 - 2 ); //Leemos el pulso PWM que nos da el valor 
delay(1000);
PM10Val_A=(pulseIn(pm10Pin, HIGH, 1500000)/ 1000 - 2 );
if((float)PM2_5Val_A<100000 and (float)PM10Val_A<100000 and (float)PM10Val_A>100 and (float)PM2_5Val_A>100){
PM10Val=((float)PM10Val_A/100.0); //dividimos entre 100 para que los valores estén en micras de gramo
PM2_5Val=((float)PM2_5Val_A/100.0);}
else{lectura=1;} //Indicar que la lectura se ha completado
Serial.println((float)PM2_5Val_A);
Serial.println((float)PM10Val_A);





}
 
/*-----------------------------------------------------------------------------*/
