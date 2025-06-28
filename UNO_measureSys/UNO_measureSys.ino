#include <OneWire.h>
#include <DHT.h>
#include "HX711.h"
#import <NewPing.h>




//Declaracion de variables para pines y relojes
#define DHTTYPE DHT11             // Tipo de sensor DHT11
#define DHTPIN 2                  // Pin de datos del sensor DHT11
const int CLK_PESO=6;             // Balanza clock
const int pinPeso=5;              // Balanza digital 
const uint8_t pinLm = A5;         // Pin de lectur_ha LM35
OneWire ds(3);                    // Pin de datos del sensor DS18B20  ???
const int echoPin = 11;           // Pin del echo del sensor ultrasónico
const int trigPin = 12;           // Pin del trig del sensor ultrasónico
const uint8_t pinRELAY = 7;       // Salida de la señal del PID

// Parámetros PID
float Kp = 4.0;
float Ki = 0.6;
float Kd = 0.3;
float L_sup = 20;
float L_inf = -5;

float dt = 0.25; // intervalo de muestreo en segundos
int DT = 500;    // Intervalo de delay = 2dt

// Variables para errores
float e[3] = {0,0,0};  // errores en t0, t1, t2
float integral = 0;

DHT dht(DHTPIN, DHTTYPE);                   // Inicializa el sensor DHT
HX711 balanza;                              // Inicializa el Amplificador HX7111
NewPing sonar(trigPin, echoPin,40.0);       // Inicializa el sensor ultrasonico y pongo limite de 40 cm

void setup() {
  Serial.begin(115200);       // Inicializa la comunicación serial a 115200 bps
  dht.begin();                // Inicializa el sensor DHT
  pinMode(pinRELAY,OUTPUT);
  balanza.begin(pinPeso, CLK_PESO);   // Inicializa balanza
  balanza.set_scale(-113.f);
  balanza.tare();
}

//Declaracion de Variables
  const uint8_t mean_amount=5;
  float humidity;
  float lm_temp;
  float dht_temp;
  float distance;
  float sonda_temp;
  float peso;
  uint8_t i;
  int tol = 5;
  float pr;

void loop() {
// Actualizo el valor de las variables a 0 para que no se continuen sumando en el promedio
    humidity=0;
    lm_temp=0;
    dht_temp=0;
    distance=0;
    sonda_temp=0;
    peso=0;

  //Promedia Datos y los muestra
  for (i=0; i<mean_amount-1; i++)
  {
    // Lectura del LM35 y DHT11
    //if (isnan(humidity) || isnan(dht_temp)) {
      //Serial.println("Error al leer del sensor DHT");
    //  return;
    //}
    //Sumatoria
    lm_temp       += readTemperature(pinLm);     // Temperatura del LM35
    sonda_temp    += leerTemperaturaDS18B20();   // Temperatura de la sonda
    dht_temp      += dht.readTemperature();      // Temperatura del DHT11
    humidity      += dht.readHumidity();         // Humedad del DHT11
    //pr             = balanza.get_units(20);      //Peso de la Celda de Carga
    //peso          += 1.25*pr;//(13.1127+(1.00023*pr)-(0.0000012925*pr*pr)+(0.0000000000797*pr*pr*pr));

    delay(DT/mean_amount);
  }
  balanza.power_up();
  peso = balanza.get_units(10);
  balanza.power_down();


distance=(sonar.ping_median(10,40.0));//*(341.32/2)/(10000));       // Distancia del sensor ultrasónico en cm

  //Division segun el promedio definido
  lm_temp     = lm_temp    /mean_amount;
  humidity    = humidity   /mean_amount;
  dht_temp    = dht_temp   /mean_amount;
  sonda_temp  = sonda_temp /mean_amount;
  //peso        = peso       /mean_amount;


  // Error calculado segun temperatura de sonda:
  float setpoint = 20.0;          // temperatura deseada
  float error = setpoint - sonda_temp;
  
  // Actualizar errores desplazando valores anteriores
  e[0] = e[1];
  e[1] = e[2];
  e[2] = error;

  // Integral usando Simpson 1/3 (solo si hay al menos 3 puntos)
  if (millis() > 2 * dt * 1000) {
    float simpsonIntegral = (dt/3.0) * (e[0] + 4*e[1] + e[2]);
    integral += simpsonIntegral;  
  }

  // Derivada usando diferencias divididas de Newton
  float f01 = (e[1] - e[0]) / dt;
  float f12 = (e[2] - e[1]) / dt;
  float f012 = (f12 - f01) / (2*dt);

  float derivada = f01 + f012 * dt;  // derivada ponderada en t2

  // Control PID
  float pid_value = Kp * e[2] + Ki * integral + Kd * derivada;
  pid_value = -1*pid_value;
  // Aplicar salida al actuador (por ejemplo, calentador)
  uint8_t h_relay;
  uint8_t c_relay;

  if(pid_value>L_sup)
  {
    pid_value=L_sup;
    c_relay=10;
  }
  else if(pid_value>tol)
  {
    digitalWrite(pinRELAY, LOW);
    c_relay=10;
  }
  else if (pid_value<L_inf)
  {
    pid_value=L_inf;
    c_relay=0;
  }
  else if (pid_value<tol)
  {
    digitalWrite(pinRELAY, HIGH);
    c_relay=0;
  }

  // Debug
  Serial.print("H_RELAY:");
  Serial.print(h_relay);
  Serial.print(", ");
  Serial.print("C_RELAY:");
  Serial.print(c_relay);
  Serial.print(", ");
  Serial.print("CTRL:");
  Serial.print(pid_value);
  Serial.print(", ");
  //Serial.print("Error: "); Serial.print(e[2]);
  //Serial.print(" Integral: "); Serial.print(integral);
  //Serial.print(" Derivada: "); Serial.print(derivada);
  //Serial.print(" Output: "); Serial.println(output);

  // Mostrar los resultados de DHT11 y LM35 en el monitor serial
  Serial.print("H_DHT11:");
  Serial.print(humidity);
  Serial.print(", ");
  //Serial.print(" %\t");
  Serial.print("T_DHT11:");
  Serial.print(dht_temp);
  Serial.print(", ");
  //Serial.print(" °C\t");

  Serial.print("T_LM35:");
  Serial.print(lm_temp, 2);  // Mostrar la temperatura del LM35 con 2 decimales
  Serial.print(", ");
  //Serial.print(" °C\t");

  Serial.print("T_DS18:");
  Serial.print(sonda_temp , 3);  // Mostrar con 3 decimales
  Serial.print(", ");
  //Serial.print(" °C");

  // Mostrar la distancia en cm
  Serial.print("L_JSNSR04T:");
  Serial.print(distance, 2);  // Mostrar con 2 decimales
  Serial.print(", ");
  //Serial.print(" cm\t");

  Serial.print("M_HX711:");
  Serial.print(peso , 3);  // Mostrar con 3 decimales
  //Serial.print("");
  //Serial.print(" kg");
  Serial.print("\n");
}

// Función para leer la temperatura del LM35
float readTemperature(uint8_t PIN) {
  uint16_t adcval = analogRead(PIN);  // Leer el valor analógico del LM35

  float voltage = (float) adcval * (5.0 / 1023.0)*100;  // Convertir a voltios
  float realTemp = voltage;        // LM35: 10 mV/°C

  return realTemp;
}

float leerTemperaturaDS18B20() {
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];

  // Buscar el sensor DS18B20
  if (!ds.search(addr)) {
    ds.reset_search();
    delay(1);
    //return -999;  // Error en la búsqueda del sensor
  }

  // Verificar CRC para la integridad de los datos
  if (OneWire::crc8(addr, 7) != addr[7]) {
    return -999;  // Error de CRC
  }

  // Identificar el tipo de chip (DS18S20, DS18B20, DS1822)
  switch (addr[0]) {
    case 0x10:
      type_s = 1;  // DS18S20 NO!
      break;
    case 0x28:
    case 0x22:
      type_s = 0;  // DS18B20 o DS1822
      break;
    default:
      return -999;  // Dispositivo no reconocido
  }

  // Iniciar la conversión de temperatura
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // Iniciar conversión de temperatura

  delay(2);  // Esperar el tiempo necesario para la conversión (750ms para 12 bits de resolución)

  // Leer el valor del scratchpad (9 bytes)
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);  // Leer Scratchpad

  // Leer los 9 bytes de datos
  data[0] = ds.read();
  data[1] = ds.read();
  data[2] = ds.read();
  data[3] = ds.read();
  data[4] = ds.read();
  data[5] = ds.read();
  data[6] = ds.read();
  data[7] = ds.read();
  data[8] = ds.read();

  // Convertir los datos a temperatura
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3;  // Resolución de 9 bits por defecto
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // Resolución de 9 bits
    else if (cfg == 0x20) raw = raw & ~3; // Resolución de 10 bits
    else if (cfg == 0x40) raw = raw & ~1; // Resolución de 11 bits
  }

  // Calcular la temperatura en grados Celsius
  float celsius = (float)raw * 0.062916 + 0.954360;
  
  return celsius;
}