#ifndef sensorDS18B20_h
#define sensorDS18B20_h
  //codigo
  class Sensores{
    private:
      uint8_t dir[8];       //Dirección del sensor
      bool estado;          //Estado: Verd=Detectado, Falso=No Detectado
      float tempC = 0.0;    //Última temperatura leída

    public:
      String nom;           //Nombre del sensor, eg. TST (Temp Salida Termotanque)
      Sensores();
      Sensores(String nombre, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
      float Temp(DallasTemperature Objeto);
      void CambiarDir(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
      bool Estado(DallasTemperature Objeto);

  };

  Sensores::Sensores(){
    nom = "NaN";
    uint8_t _dir[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
    memmove(dir, _dir, sizeof(_dir));
  }

  Sensores::Sensores(String nombre, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8){
    nom = nombre;
    dir[0] = d1;  dir[1] = d2;
    dir[2] = d3;  dir[3] = d4;
    dir[4] = d5;  dir[5] = d6;
    dir[6] = d7;  dir[7] = d8;
  }

  float Sensores::Temp(DallasTemperature Objeto){
    if(Objeto.isConnected(dir)){
      tempC = Objeto.getTempC(dir);
      estado = true;
    }
    else estado = false;
    return tempC;
  }

  void Sensores::CambiarDir(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8){
    dir[0] = d1;  dir[1] = d2;
    dir[2] = d3;  dir[3] = d4;
    dir[4] = d5;  dir[5] = d6;
    dir[6] = d7;  dir[7] = d8;
  }

  bool Sensores::Estado(DallasTemperature Objeto){
    if(Objeto.isConnected(dir)) estado = true;
    else  estado = false;
    return estado;
  }
#endif //sensorDS18B20_h 
