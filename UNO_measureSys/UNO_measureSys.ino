#include <OneWire.h>
#include <DHT.h>
#include "Adafruit_HX711.h"



//Declaracion de variables para pines y relojes
#define DHTTYPE DHT11             // Tipo de sensor DHT11
#define DHTPIN 2                  // Pin de datos del sensor DHT11
const int CLK_PESO=A0;            // Balanza clock
const int pinPeso=A1;             // Balanza digital 
const uint8_t pinHX711 = 5;       // Amplificador pin
const uint8_t pinCLK_HX711 = 6;   // Amplificador clock pin
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
int DT = 500;

// Variables para errores
float e[3] = {0,0,0};  // errores en t0, t1, t2
float integral = 0;

DHT dht(DHTPIN, DHTTYPE);                             // Inicializa el sensor DHT
Adafruit_HX711 balanza(pinHX711, pinCLK_HX711);       // Inicializa el Amplificador HX7111

void setup() {
  Serial.begin(115200);       // Inicializa la comunicación serial a 115200 bps
  dht.begin();                // Inicializa el sensor DHT
  pinMode(trigPin, OUTPUT);   // Inicializa el pin trig como salida
  pinMode(echoPin, INPUT);    // Inicializa el pin echo como entrada
  pinMode(pinRELAY,OUTPUT);
  balanza.begin();   // Inicializa balanza
  // read and toss 3 values each
  for (uint8_t t=0; t<3; t++) {
    balanza.tareA(balanza.readChannelRaw(CHAN_A_GAIN_128));
    balanza.tareA(balanza.readChannelRaw(CHAN_A_GAIN_128));
    balanza.tareB(balanza.readChannelRaw(CHAN_B_GAIN_32));
    balanza.tareB(balanza.readChannelRaw(CHAN_B_GAIN_32));
  }
}

//Declaracion de Variables
  const uint8_t mean_amount=10;
  float humidity;
  float lm_temp;
  float dht_temp;
  float distance;
  float sonda_temp;
  float peso;
  uint8_t i;
  int tol = 5;

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
    if (isnan(humidity) || isnan(dht_temp)) {
      Serial.println("Error al leer del sensor DHT");
      return;
    }
    //Sumatoria
    distance      += getUltrasonicDistance();    // Distancia del sensor ultrasónico
    lm_temp       += readTemperature(pinLm);     // Temperatura del LM35
    sonda_temp    += leerTemperaturaDS18B20();   // Temperatura de la sonda
    dht_temp      += dht.readTemperature();      // Temperatura del DHT11
    humidity      += dht.readHumidity();         // Humedad del DHT11
    peso += (float) (balanza.readChannelBlocking(CHAN_A_GAIN_64))/(2048); //Peso de la Celda de Carga

    delay(DT/mean_amount);
  }
  //Division segun el promedio definido
  lm_temp     = lm_temp    /mean_amount;
  humidity    = humidity   /mean_amount;
  dht_temp    = dht_temp   /mean_amount;
  distance    = distance   /mean_amount;
  sonda_temp  = sonda_temp /mean_amount;
  peso        = peso       /mean_amount;


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
  Serial.print("RELAY:");
  if(pid_value>L_sup)
  {
    pid_value=L_sup;
    Serial.print("10");
  }
  else if(pid_value>tol)
  {
    digitalWrite(pinRELAY, LOW);
    Serial.print("10");
  }
  else if (pid_value<L_inf)
  {
    pid_value=L_inf;
    Serial.print("0");

  }
  else if (pid_value<tol)
  {
    digitalWrite(pinRELAY, HIGH);
    Serial.print("0");
  }

  // Debug
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
  Serial.print("L_USOUND:");
  Serial.print(distance, 2);  // Mostrar con 2 decimales
  Serial.print(", ");
  //Serial.print(" cm\t");



  Serial.print("M_HX711:");
  Serial.print(peso , 3);  // Mostrar con 3 decimales
  //Serial.print("");
  //Serial.print(" kg");
  Serial.print("\n");

  //delay(2000);  // Espera 2 segundos entre lecturas
}

// Función para leer la temperatura del LM35
float readTemperature(uint8_t PIN) {
  uint16_t adcval = analogRead(PIN);  // Leer el valor analógico del LM35

  float voltage = (float) adcval * (5.0 / 1023.0)*100;  // Convertir a voltios
  float realTemp = voltage;        // LM35: 10 mV/°C

  return realTemp;
}

// Función para obtener la distancia del sensor ultrasónico
float getUltrasonicDistance() {
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(5);

  // Enviar el pulso
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Leer la duración del pulso
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia con mayor precisión
  float distance = (float) duration * 50/43 * 0.0343 / 2.0;  // Mayor precisión en el factor de velocidad

  return distance;
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

 
  




