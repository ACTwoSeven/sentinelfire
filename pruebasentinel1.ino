/*
 * Programa revisado para CubeCell HTCC-AB01 con sensores DHT22 y MQ-2
 * Envía datos de temperatura, humedad y gas mediante LoRaWAN
 * Compatible con decodificador Node-RED
 */

#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include "DHT.h"

#define LORAWAN_AT_SUPPORT ON

// Definición de pines según el diagrama
#define DHT_PIN GPIO4    // Pin 4 para DHT22
#define MQ2_PIN ADC      // Pin ADC para el sensor MQ-2 (pin 12)
#define LED_PIN GPIO2    // LED para indicar detección de gas (pin 13)

// Definición del tipo de sensor DHT
#define DHTTYPE DHT22

// Tiempo entre envíos (en milisegundos)
uint32_t TX_INTERVAL = 30000;    // 5 minutos

// Variables para almacenar las lecturas de los sensores
float temperatura;
float humedad;
int valorGas;

// Contador de mensajes enviados
static uint8_t counter = 0;

// Inicialización del sensor DHT
DHT dht(GPIO4, DHT22);

// Credenciales LoRaWAN
static uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x01, 0xB1 };
static uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 
static uint8_t appKey[] = { 0x53, 0x4B, 0x46, 0xFB, 0x1F, 0xC2, 0x6A...};

// Máscara de canales LoRaWAN
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

// Temporizador para modo de bajo consumo
TimerEvent_t sleepTimer;
bool sleepTimerExpired;

static void wakeUp()
{
  sleepTimerExpired = true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired = false;
  TimerInit(&sleepTimer, &wakeUp);
  TimerSetValue(&sleepTimer, sleeptime);
  TimerStart(&sleepTimer);
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop(&sleepTimer);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando sistema de monitoreo con DHT22 y MQ-2 con LoRaWAN");
  
  // Configurar pines
  pinMode(MQ2_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Iniciar sensor DHT
  dht.begin();
  
  // Precalentamiento del sensor MQ-2
  Serial.println("Calentando sensor MQ-2...");
  delay(20000);  // 20 segundos de calentamiento
  Serial.println("Sensor MQ-2 listo");

  // Configuración de LoRaWAN
  if (ACTIVE_REGION == LORAMAC_REGION_US915) {
    LoRaWAN.setSubBand2();
  }
 
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);

  // Conexión a red LoRaWAN
  while (1) {
    Serial.print("Conectando a la red LoRaWAN... ");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      Serial.println("CONEXIÓN FALLIDA! Esperando 30 segundos");
      lowPowerSleep(30000);
    } else {
      Serial.println("CONECTADO");
      break;
    }
  }
}

void loop() {
  counter++;
  
  // Leer datos del sensor DHT22
  humedad = dht.readHumidity();
  temperatura = dht.readTemperature();
  
  // Leer datos del sensor MQ-2
  valorGas = analogRead(MQ2_PIN);
  
  // Mostrar lecturas en serial
  Serial.println("\n--- Lecturas de sensores ---");
  
  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("Error al leer del sensor DHT22!");
  } else {
    Serial.print("Humedad: ");
    Serial.print(humedad);
    Serial.print(" % | ");
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");
  }
  
  Serial.print("Nivel de gas (analógico): ");
  Serial.println(valorGas);
  
  // Activar LED si se detecta alto nivel de gas
  if (valorGas > 700) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Preparar datos para envío LoRaWAN
  // Formato: 2 bytes para temperatura, 2 bytes para humedad, 2 bytes para valor de gas
  uint8_t buffer[6];
  
  // Convertir temperatura (-40 a 80°C con precisión de 0.1°C)
  int16_t tempInt = (int16_t)(temperatura * 10);
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 0xFF;
  
  // Convertir humedad (0-100% con precisión de 0.1%)
  uint16_t humInt = (uint16_t)(humedad * 10);
  buffer[2] = humInt >> 8;
  buffer[3] = humInt & 0xFF;
  
  // Convertir valor de gas (0-1023)
  buffer[4] = valorGas >> 8;
  buffer[5] = valorGas & 0xFF;
  
  // Enviar datos por LoRaWAN
  Serial.printf("Enviando paquete #%d con datos de sensores\n", counter);
  Serial.println("Payload en bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.printf("0x%02X ", buffer[i]);
  }
  Serial.println();
  
  bool requestack = true;  // Solicitar confirmación (ACK)
  Serial.print("Longitud a enviar: ");
  Serial.println(sizeof(buffer));
  Serial.print("Longitud a enviar: ");
  Serial.println(sizeof(buffer));
  if (LoRaWAN.send(6, buffer, 6, true)) {
    Serial.println("Envío test OK");
  }
  
  // Esperar antes de la siguiente lectura y envío
  Serial.printf("Entrando en modo de bajo consumo por %d segundos\n", TX_INTERVAL/1000);
  lowPowerSleep(TX_INTERVAL);
}

// Función para manejar mensajes recibidos desde la red LoRaWAN
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Mensaje recibido: %s, RXSIZE %d, PUERTO %d, DATOS: ", 
                mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1",
                mcpsIndication->BufferSize,
                mcpsIndication->Port);
                
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();
  
  // Interpretar comandos recibidos
  if (mcpsIndication->BufferSize > 0) {
    switch (mcpsIndication->Buffer[0]) {
      case 0x01:
        // Comando para cambiar intervalo de transmisión
        if (mcpsIndication->BufferSize >= 3) {
          uint16_t newInterval = (mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2];
          // Convertir a milisegundos (valor en segundos * 1000)
          TX_INTERVAL = 3000;
          Serial.printf("Nuevo intervalo de transmisión: %d segundos\n", newInterval);
        }
        break;
      
      case 0x02:
        // Comando para reiniciar el dispositivo
        Serial.println("Reiniciando dispositivo...");
        delay(1000);
        CySoftwareReset();
        break;
        
      default:
        Serial.println("Comando desconocido");
        break;
    }
  }
}
