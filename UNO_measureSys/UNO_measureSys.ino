#include <OneWire.h>
#include <DHT.h>
#include "Adafruit_HX711.h"




#define DHTTYPE DHT11     // Tipo de sensor DHT11

const int CLK=A0;             // Balanza clock
const int DOUT=A1;            // Balanza digital pin
const uint8_t pinTEMP = A5;   // Pin de lectura LM35
#define DHTPIN 2              // Pin de datos del sensor DHT11
OneWire ds(3);                // Pin de datos del sensor DS18B20
const uint8_t DATA_PIN = 5;  // Can use any pins!
const uint8_t CLOCK_PIN = 6; // Can use any pins!
const int echoPin = 11;       // Pin del echo del sensor ultrasónico
const int trigPin = 12;       // Pin del trig del sensor ultrasónico

float measured;
DHT dht(DHTPIN, DHTTYPE);    // Inicializa el sensor DHT
Adafruit_HX711 balanza(DATA_PIN, CLOCK_PIN);

void setup() {
  Serial.begin(115200);       // Inicializa la comunicación serial a 115200 bps
  dht.begin();                // Inicializa el sensor DHT
  pinMode(trigPin, OUTPUT);   // Inicializa el pin trig como salida
  pinMode(echoPin, INPUT);    // Inicializa el pin echo como entrada

  balanza.begin();   // Inicializa balanza
  // read and toss 3 values each
  for (uint8_t t=0; t<3; t++) {
    balanza.tareA(balanza.readChannelRaw(CHAN_A_GAIN_128));
    balanza.tareA(balanza.readChannelRaw(CHAN_A_GAIN_128));
    balanza.tareB(balanza.readChannelRaw(CHAN_B_GAIN_32));
    balanza.tareB(balanza.readChannelRaw(CHAN_B_GAIN_32));
  }
}

void loop() {

  const uint8_t N=10;
  float h=0;
  float t=0;
  float distance=0;
  float temperature=0;
  float peso=0;
  uint8_t i;

  for (i=0; i<N-1; i++)
  {
    // Lectura del LM35 y DHT11
    measured = readTemperature(pinTEMP);  // Temperatura del LM35
    h += dht.readHumidity();         // Humedad del DHT11
    t += dht.readTemperature();      // Temperatura del DHT11

    if (isnan(h) || isnan(t)) {
      Serial.println("Error al leer del sensor DHT");
      return;
    }

    // Lectura del sensor ultrasónico
    distance += getUltrasonicDistance();  // Distancia del sensor ultrasónico
    // Lectura del sensor sonda temp
    temperature += leerTemperaturaDS18B20(); // Temperatura °C del sensor

    peso += (float) (balanza.readChannelBlocking(CHAN_A_GAIN_64))/(2048);
    delay(2000/N);
  }
  h=h/N;
  t=t/N;
  distance=distance/N;
  temperature=temperature/N;
  peso=peso/N;




  // Mostrar los resultados de DHT11 y LM35 en el monitor serial
  Serial.print(" Humedad:");
  Serial.print(h);
  Serial.print(",");
  //Serial.print(" %\t");
  Serial.print(" Temperatura DHT11:");
  Serial.print(t);
  Serial.print(",");
  //Serial.print(" °C\t");

  Serial.print(" Temperatura LM35:");
  Serial.print(measured, 2);  // Mostrar la temperatura del LM35 con 2 decimales
  Serial.print(",");
  //Serial.print(" °C\t");

  // Mostrar la distancia en cm
  Serial.print("Distancia:");
  Serial.print(distance, 2);  // Mostrar con 2 decimales
  Serial.print(",");
  //Serial.print(" cm\t");

  Serial.print(" Temperatura DS18:");
  Serial.print(temperature , 3);  // Mostrar con 3 decimales
  Serial.print(",");
  //Serial.print(" °C");

  Serial.print(" Celda:");
  Serial.print(peso , 3);  // Mostrar con 3 decimales
  //Serial.print(",");
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
