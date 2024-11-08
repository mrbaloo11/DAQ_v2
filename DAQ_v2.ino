#include <Keyboard.h>
#include <I2CKeyPad.h>
#include <LiquidCrystal_I2C.h>
#include <DS3231.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "sensorDS18B20.h"

// *** Programa de adquisición de datos para colector solar ***

// * Definiciones y constantes *
#define pinTermo1 8
#define pinTermo2 9
#define BotonSerp 22
#define BotonRec 23
#define BombSerp 24
#define BombRec 25   
#define FlujoReset 29

// * Modo del Serial Monitor/Plotter
//#define modo 1    // Flujo, TSD, TID, TSS, TES, TST, TET, TSCS, TIT, TECS, TUCS, TOCS, TMCS
#define modo 2    // Flujo, TID, TES, TSS, TIT


// * Objetos y estructuras de los periféricos *
// Pantalla LCD
const int LCDcols = 20, LCDrows = 4;
LiquidCrystal_I2C lcd(0x27, LCDcols, LCDrows);

// RTC DS3231 (Reloj de Tiempo Real)
RTClib Reloj;
bool century = false;
bool h12Flag;
bool pmFlag;

// Memoria MicroSD
const int chipSelect = 53;
File dataFile;

// OneWire y DallasTemperature
OneWire oneWireObject1(pinTermo1);
OneWire oneWireObject2(pinTermo2);

DallasTemperature DS18B20a(&oneWireObject1);
DallasTemperature DS18B20b(&oneWireObject2);

// Declaración de sensores
Sensores termos[] = {
  Sensores("TSD",0x28,0xBE,0x65,0x94,0x97,0x0E,0x03,0x4A),  //T1
  Sensores("TID",0x28,0x7F,0x40,0x79,0xA2,0x00,0x03,0x2B),  //T2
  Sensores("TSS",0x28,0x04,0x0D,0x94,0x97,0x01,0x03,0xB5),  //T3
  Sensores("TES",0x28,0xA8,0xB5,0x43,0xD4,0xE1,0x3C,0x3D),  //T4
  Sensores("TST",0x28,0xBE,0xBA,0xA5,0x63,0x20,0x01,0x0E),  //T5
  Sensores("TET",0x28,0x01,0x17,0x94,0x97,0x01,0x03,0x9F),  //T6
  Sensores("TSCS",0x28,0x2E,0x61,0x94,0x97,0x0E,0x03,0xE4), //T7
  Sensores("TIT",0x28,0x92,0x53,0x94,0x97,0x13,0x03,0xFC),  //T8
  Sensores("TECS",0x28,0x42,0xC6,0xFF,0x63,0x20,0x01,0x8E), //T9
  Sensores("TUCS",0x28,0x57,0x26,0x94,0x97,0x13,0x03,0x37), //T10
  Sensores("TOCS",0x28,0xEE,0x53,0x94,0x97,0x01,0x03,0x64), //T11
  Sensores("TMCS",0x28,0xE2,0x33,0x94,0x97,0x01,0x03,0x08)  //T12
};

// Bombas
unsigned long lastDebounceTime1 = 0, lastDebounceTime2 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int BotSerpState = 0, lastSerpState = 0; //Botones de las bombas
int BotRecState = 0, lastRecState = 0;
int BombSerpState = 1; //0 = Off, 1 = Auto, 2 = Man
int BombRecState = 1; 
String BombState[3] = {"Off", "Aut", "Man"};
bool SerpCheck = false; //Enclavamiento de la bomba del serpentín

int T_min = 40, T_off = 40, T_on = 37;  //Temperaturas mínima del tanque, mínima del digestor y máxima del digestor (histéresis entre T_off y T_on)

// * Variables Globales *

// Temporizadores
uint32_t now = 0, last = 0, lastLCD = 0;
uint32_t nowP = 0, lastP = 0;

// Variables de impresión
String anio, mes, dia, hora, min, seg, Fecha, Hora, Encabezado, Sen_Fallo, Temperaturas, BS, BR, Flujo;

// Acumuladores y promediadores
uint8_t pulsos = 0;
int FalloAcum = 0, termos_tot = 0, deviceCount = 0, ciclosFlujo = 0, ciclosMin = 0;
float caudalProm = 0.0;
bool nuevo;

// * Inicio del programa *
void setup() {

  pinMode(BotonSerp, INPUT);
  pinMode(BotonRec, INPUT);
  pinMode(BombSerp,OUTPUT);
  pinMode(BombRec, OUTPUT);
  pinMode(FlujoReset, OUTPUT);
  DDRC = B00000000;

  Serial.begin(57600);    //Iniciar puerto Serial

  Wire.begin();           //Iniciar interface I2C

  lcd.begin(LCDcols, LCDrows);  //Iniciar pantalla LCD
  lcd.setBacklight(255);        //Brillo del LCD
  lcd.clear();

  DS18B20a.begin();        //Iniciar OneWire
  DS18B20b.begin();
  deviceCount = DS18B20a.getDeviceCount() + DS18B20b.getDeviceCount();
  // Serial.println(deviceCount);

  ImprimirFechaHora(); 

  if (!SD.begin(chipSelect)) {  //Iniciar y anunciar memoria SD
    lcd.setCursor(0, 1);
    lcd.print("SD:Err");
    //while (1);
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("SD:Ok!");
  }

  termos_tot = sizeof(termos)/sizeof(termos[0]);  //Saber cantidad de sensores declarados

  Encabezado = "Hora,BS,BR,Flujo";
  for(int i = 0; i < termos_tot; i++){
    Encabezado = Encabezado + "," + termos[i].nom;
  }

  Fecha = mes + "-" + dia + "-" + anio + ".csv";

  if(!SD.exists(Fecha)) dataFile = SD.open(Fecha,FILE_WRITE);

  if(dataFile){
    dataFile.println(Encabezado);
    dataFile.close();
    // Serial.println(Fecha);
    // Serial.println(Encabezado);
  }
  else // Serial.println("Error al abrir el archivo en el setup...");

  Plotter(modo,1);

  BS = String(0);
  BR = String(0);

  DateTime myDT = Reloj.now();
  last = myDT.unixtime();
  lastLCD = last;
  lastP = millis();
}

void loop(){

  int boton1 = digitalRead(BotonSerp);
  int boton2 = digitalRead(BotonRec);

  DateTime myDT = Reloj.now();
  now = myDT.unixtime();         //Guarda el momento actual
  lcd.setCursor(0, 0);    
  ImprimirFechaHora();    //Guarda e imprime en lcd la fecha y hora

  if(Hora == "00:00" && !nuevo){
    Encabezado = "Hora,BS,BR,Flujo";
    for(int i = 0; i < termos_tot; i++){
      Encabezado = Encabezado + "," + termos[i].nom;
    }

    Fecha = mes + "-" + dia + "-" + anio + ".csv";

    if(!SD.exists(Fecha)){
      dataFile = SD.open(Fecha,FILE_WRITE);
      nuevo = true;
    }

    if(dataFile){
      dataFile.println(Encabezado);
      dataFile.close();
    }
  }

  if(Hora != "00:00") nuevo = false;


  for(int i = 0; i < (sizeof(termos)/sizeof(termos[0])); i++){  //Guarda en un String los nombres de los sensores no detectados
    if(!termos[i].Estado(DS18B20a) && !termos[i].Estado(DS18B20b))
      Sen_Fallo = Sen_Fallo + termos[i].nom + ", ";
  }
  lcd.setCursor(8, 2);    
  lcd.print("Ts:");  lcd.print(deviceCount, DEC); 

  if((now - lastLCD) >= 1){     //Imprime en la LCD la lista de sensores no detectados avanzando cada 0.5 seg
    if(Sen_Fallo.length() < 20){
      ClearRow(3);
      lcd.print(Sen_Fallo);
    }
    else{
      ClearRow(3);
      lcd.print(Sen_Fallo.substring(FalloAcum, FalloAcum + (LCDcols - 1)));
      FalloAcum++;
      if(FalloAcum >= Sen_Fallo.length() - (LCDcols/2)) FalloAcum = 0;
    }
    lastLCD = myDT.unixtime();
  }

  MedirFlujo();
  
  if(now-last >= 60){                //Cada minuto (en segundos)
    Flujo = String(caudalProm/ciclosMin);
    // Serial.print("Flujo: " + Flujo);  // Verifica que se guarde el dato de flujo
    caudalProm = 0;
    ciclosMin = 0;

    DS18B20a.requestTemperatures();    //Solicita tomar lecturas 
    DS18B20b.requestTemperatures();

    Temperaturas = Hora + "," + BS + "," + BR + "," + Flujo;
    for(int i = 0; i < termos_tot; i++){
      if(termos[i].Estado(DS18B20a))
        Temperaturas = Temperaturas + "," + termos[i].Temp(DS18B20a);
      else
        Temperaturas = Temperaturas + "," + termos[i].Temp(DS18B20b);
    }

    dataFile = SD.open(Fecha,FILE_WRITE);
    if(dataFile){
      dataFile.println(Temperaturas);
      dataFile.close();
      // Serial.println(Temperaturas);
      Plotter(modo,2); 
    }
    else //Serial.println("Error al abrir el archivo...");

    last = myDT.unixtime();
  } 

  Bombas(boton1, boton2); 
  
}

// * Funciones auxiliares *

void Plotter(int mod, int ciclo){ // Ciclo: 1-Setup, 2-loop
  String temporal;
  
  switch(mod){
    case 1:
      if(ciclo == 1) Serial.println("Flujo:,TSD:,TID:,TSS:,TES:,TST:,TET:,TSCS:,TIT:,TECS:,TUCS:,TOCS:,TMCS:");
      if(ciclo == 2){
        Serial.print(Flujo.toFloat()); Serial.print(","); // Imprime el flujo
        for(int i = 0; i < termos_tot; i++){      // Recorre los sensores
          if(termos[i].Estado(DS18B20a))
            temporal = termos[i].Temp(DS18B20a);  // Guarda el dato en un string temporal
          else
            temporal = termos[i].Temp(DS18B20b);
          if(i < (termos_tot-1)){
            Serial.print(temporal.toFloat());     // Imprime cada sensor
            Serial.print(",");
          }
          else Serial.println(temporal.toFloat());// Si es el último, imprime un salto de línea
        }
      }
      break;
    case 2:
      if(ciclo == 1) Serial.println("Flujo:,TID:,TSS:,TES:,TIT:");
      if(ciclo == 2){
        Serial.print(Flujo.toFloat()); Serial.print(","); // Imprime el flujo
        for(int i = 0; i < termos_tot; i++){          // Recorre los sensores
          if(termos[i].nom == "TID" || termos[i].nom == "TSS" || termos[i].nom == "TES" || termos[i].nom == "TIT"){  // Si es alguno de los sensores seleccionados
            if(termos[i].Estado(DS18B20a))
              temporal = termos[i].Temp(DS18B20a);    //Guarda el dato en un string temporal
            else
              temporal = termos[i].Temp(DS18B20b);

            Serial.print(temporal.toFloat());
          }
          if(i < (termos_tot-1)){
              Serial.print(",");
          }
          else Serial.println();  //Si es el último, imprime el valor y un salto de línea
        }
      }
      break;
  }
}

void ImprimirFechaHora(){ //Guarda e imprime en la LCD la fecha y hora del RTC
  char buffer[5];
  DateTime myDT = Reloj.now();
  
  sprintf(buffer, "%02d",myDT.day());
  dia = buffer;
  lcd.print(dia);  lcd.print(".");
  sprintf(buffer, "%02d",myDT.month());
  mes = buffer;
  lcd.print(mes);  lcd.print(".");
  sprintf(buffer, "%02d",myDT.year());
  anio = buffer;
  anio.remove(0,2);
  lcd.print(anio);  lcd.print(" ");
  sprintf(buffer, "%02d",myDT.hour());
  hora = buffer;
  lcd.print(hora);  lcd.print(":");
  sprintf(buffer, "%02d",myDT.minute());
  min = buffer;
  lcd.print(min);  lcd.print(":");
  sprintf(buffer, "%02d",myDT.second());
  seg = buffer;
  lcd.print(seg);  lcd.print("h");
  
  Hora = hora + ":" + min;
}

void Bombas(int boton1, int boton2){
  DateTime myDT = Reloj.now();

  bool SerpSignal = false, RecSignal = false;

  String TIT, TID, TES, TSS;

  if(termos[7].Estado(DS18B20a))
    TIT = termos[7].Temp(DS18B20a);
  else
    TIT = termos[7].Temp(DS18B20b);

  if(termos[1].Temp(DS18B20a))
    TID = termos[1].Temp(DS18B20a);
  else
    TID = termos[1].Temp(DS18B20b);

  if(termos[4].Estado(DS18B20a))
    TES = termos[4].Temp(DS18B20a);
  else
    TES = termos[4].Temp(DS18B20b);

  if(termos[3].Temp(DS18B20a))
    TSS = termos[3].Temp(DS18B20a);
  else
    TSS = termos[3].Temp(DS18B20b);

  float tit = TIT.toFloat();
  float tid = TID.toFloat();
  float tes = TES.toFloat();
  float tss = TSS.toFloat();

  if(lastSerpState == 0 && boton1 == 1) lastSerpState = 1;
  if(lastRecState == 0 && boton2 == 1) lastRecState = 1;

  if(lastSerpState == 1 && boton1 == 0){
    lastSerpState = 0;
    if(BombSerpState >= 2) BombSerpState = 0;
    else BombSerpState++;
  }

  if(lastRecState == 1 && boton2 == 0){
    lastRecState = 0;
    if(BombRecState >= 2) BombRecState = 0;
    else BombRecState++;
  }

  switch(BombSerpState){
    case 0: 
      SerpSignal = false;
      break;
    case 1:
      // Para aprovechar la prog estructurada, primero revisa si mantiene el enclavamiento fuera de los 5 seg al inicio de cada hora del horario establecido
      
      if(SerpCheck && ((myDT.second() > 5 && myDT.minute() == 0) || myDT.minute() >= 1)){ //Si ya pasaron los 5 seg del min 0
        if(tes >= T_min && tes >= tid)    //y el agua que entra al serpentín está mas caliente que T_min y que la biomasa
          SerpSignal = true;                //mantiene encendida la bomba
        else{
          SerpSignal = false;             //Si no, apaga la bomba y rompe el enclavamiento hasta la siguiente hora
          SerpCheck = false;
        }
      }

      if(myDT.hour() >= 10 && myDT.hour() <= 20 && myDT.minute() == 0 && myDT.second() <= 5){ //Entre las 10 am y las 8 pm
        SerpCheck = true;                                               //Enclava la bomba cada hora por los primeros 5 seg del min 0
        SerpSignal = true;
      }
      
      if(tit >= T_min){       //Cuando el tanque está más caliente que T_min
        if(tid <= T_on)         //Si el digestor está por debajo de T_on
          SerpSignal = true;      //enciende la bomba
        if(tid >= T_off)        //Si el digestor está por arriba de T_off
          SerpSignal = false;     //apaga la bomba
      }
      break;
    case 2:
      SerpSignal = true;
      break;
  }

  switch(BombRecState){
    case 0:
      RecSignal = false;
      break;
    case 1:
      if(myDT.minute() >= 0 && myDT.minute() <= 20) //Cada hora la bomba arranca por 20 min
        RecSignal = true;
      else                                          //Los otros 40 min permanece apagada
        RecSignal = false;
      break;
    case 2:
      RecSignal = true;
      break;
  }

  if(SerpSignal){   //Manda el on/off de la bomba del serpentín y lo escribe en el datalog
    BS = String(1);
    digitalWrite(BombSerp, HIGH);
  }
  else{
    BS = String(0);
    digitalWrite(BombSerp, LOW);
  }

  if(RecSignal){    //Manda el on/off de la bomba recirculadora y lo escribe en el datalog
    BR = String(1);
    digitalWrite(BombRec, HIGH);
  }
  else{
    BR = String(0);
    digitalWrite(BombRec, LOW);
  }

  lcd.setCursor(6,1);
  lcd.print(" BS");
  if(SerpSignal) lcd.print("+");        //En la LCD, para ambas bombas,
  else lcd.print("-");                  //"+" es encendido y "-" es apagado
  lcd.print(BombState[BombSerpState]);

  lcd.print(" BR");
  if(RecSignal) lcd.print("+");
  else lcd.print("-");
  lcd.print(BombState[BombRecState]);
}

void MedirFlujo(){
  byte portCValue = PINC;

  pulsos += portCValue;
  ciclosFlujo++;
  nowP = millis();

  if(nowP - lastP >= 500){
    float factor = 5.5;
    float densidad = 1.0; // kg/L
    
    float frec = (pulsos * 1.0) / ciclosFlujo;
    float caudal_V = frec / factor;
    float caudal_m = caudal_V * densidad;

    // Serial.print(portCValue,BIN);
    // Serial.print("  ");
    // Serial.print(pulsos);
    // Serial.print("  ");
    // Serial.println(frec);

    digitalWrite(FlujoReset, HIGH);
    delay(10);
    digitalWrite(FlujoReset, LOW);

    caudalProm += caudal_m;
    ciclosMin++;
    pulsos = 0;
    ciclosFlujo = 0;

    ClearRow(2);
    lcd.setCursor(0, 2);
    lcd.print("F:"); lcd.print(caudal_m);

    lastP = millis();
  }
}

void ClearRow(int rowNum){
  char blank[LCDcols] = "";
  lcd.setCursor(0, rowNum);
  lcd.print(blank);
  lcd.setCursor(0, rowNum);
}
