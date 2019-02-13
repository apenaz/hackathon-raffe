
/**
 * Exemplo de código para plataforma
 * SaIoT usando o protocolo MQTT. Nesse exemplo foi usado a
 * biblioteca para dispositivos IoT se comunicarem com a
 * plataforma.
 * Autores:
 * Danielly Costa daniellycosta97@ufrn.edu.br
 * Patricio Oliveira patricio@bct.ect.ufrn.br
 * Ricardo Cavalcanti ricavalcanti@bct.ect.ufrn.br
 *
 * */

 #include <Arduino.h>
 #include <SaIoTDeviceLib.h>
 int aasdkasdljk;


 // Sensores
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BMP085_U.h>
 #include <Adafruit_ADXL345_U.h>
 // #include <Adafruit_L3GD20_U.h>
 //#include <DHT.h>
 #include <DHT_U.h>

#define DHTPIN            D5         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
void dhtDisplaySensorDetails(void);
void dhtValue(void);
 // Adafruit_BMP085 bmp;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
void bmpDisplaySensorDetails(void);
void temperatureValue(void);

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
void accelDisplaySensorDetails(void);
void accelValue(void);

/* Assign a unique ID to this sensor at the same time */
// Adafruit_L3GD20_Unified dht = Adafruit_L3GD20_Unified(20);
// Adafruit_L3GD20 dht;
// void dhtDisplaySensorDetails(void);
// void dhtValue(void);





SaIoTDeviceLib barril("barril-raffe","40206","apenaspatricio@gmail.com");
SaIoTSensor barril_t("t1","temperatura"," C","number");
SaIoTSensor barril_a("a1","aceleração"," m/s2","number");
SaIoTSensor barril_u("u1","umidade"," %","number");
SaIoTSensor barril_v("d1","envazamento",".","string");
SaIoTController barril_e("e1","env","button");

#define timeToSend 5000



WiFiClient espClient;
void callback(char* topic, byte* payload, unsigned int length);
String senha = "321321321321";
unsigned long tDecorrido;
String hora ="";


void setup(){
  Serial.begin(115200);
  Serial.println("INICIO");
  delay(1000);

  dht.begin();
  dhtDisplaySensorDetails();
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  }
  bmpDisplaySensorDetails();

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
  }
  accelDisplaySensorDetails();

  // /* Enable auto-ranging */
  // Serial.println("vai mano");
  // // if(!dht.begin())
  // {
  //   /* There was a problem detecting the L3GD20 ... check your connections */
  //   Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
  // }else{
  //   Serial.println("nem foii");
  // }
  // // dhtDisplaySensorDetails();


  barril.addSensor(barril_t);
  barril.addSensor(barril_a);
  barril.addSensor(barril_u);
  barril.addSensor(barril_v);
  barril.addController(barril_e);
  barril.preSetCom(espClient, callback);
  barril.startDefault(senha);

	tDecorrido = millis();
}






void loop(){

	if( ((millis() - tDecorrido)) >= timeToSend ){
    hora = SaIoTCom::getDateNow();
    temperatureValue();
    accelValue();
    dhtValue();

		tDecorrido = millis();
	}
	barril.handleLoop();
}











































void callback(char* topic, byte* payload, unsigned int length){
  String payloadS;
  Serial.print("Topic: ");
  Serial.println(topic);
  for (unsigned int i=0;i<length;i++) {
    payloadS += (char)payload[i];
  }
  if(strcmp(topic,barril.getSerial().c_str()) == 0){
    Serial.println("SerialLog: " + payloadS);
  }
  if(strcmp(topic,(barril.getSerial()+barril_e.getKey()).c_str()) == 0){

    Serial.println("SerialLog: " + payloadS);
    if (payloadS == "1") {
      String ahora = SaIoTCom::getDateNow();
      barril_v.sendData(ahora, ahora);
      Serial.println("atualizou hora:" + ahora);
      /* code */
    }
    //
  }
}











/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void bmpDisplaySensorDetails(void){
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void temperatureValue(void){
  /* Get a new sensor event */
    sensors_event_t event;
    bmp.getEvent(&event);

    /* Display the results (barometric pressure is measure in hPa) */
    if (event.pressure)
    {
      /* Display atmospheric pressue in hPa */
      Serial.print("Pressure:    ");
      Serial.print(event.pressure);
      Serial.println(" hPa");

      /* Calculating altitude with reasonable accuracy requires pressure    *
       * sea level pressure for your position at the moment the data is     *
       * converted, as well as the ambient temperature in degress           *
       * celcius.  If you don't have these values, a 'generic' value of     *
       * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
       * in sensors.h), but this isn't ideal and will give variable         *
       * results from one day to the next.                                  *
       *                                                                    *
       * You can usually find the current SLP value by looking at weather   *
       * websites or from environmental information centers near any major  *
       * airport.                                                           *
       *                                                                    *
       * For example, for Paris, France you can check the current mean      *
       * pressure and sea level at: http://bit.ly/16Au8ol                   */

      /* First we get the current temperature from the BMP085 */
      float temperature;
      bmp.getTemperature(&temperature);
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" C");
      barril_t.sendData(temperature,hora);

      /* Then convert the atmospheric pressure, and SLP to altitude         */
      /* Update this next line with the current SLP for better results      */
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      Serial.print("Altitude:    ");
      Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                          event.pressure));
      Serial.println(" m");
      Serial.println("");
    }
    else
    {
      Serial.println("Sensor error");
    }
}

void accelDisplaySensorDetails(void){
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
}
void accelValue(void){
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  float maior = event.acceleration.x;
  if (maior <= event.acceleration.y) {
    maior = event.acceleration.y;
  } else if (maior <= event.acceleration.z) {
    maior = event.acceleration.z;
  }
  double modulo = sqrtf(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));
  Serial.print("maior: "); Serial.println(maior);
  barril_a.sendData(modulo,hora);
  Serial.println("modulo: " + String(modulo));

}
//
void dhtDisplaySensorDetails(void){
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
}
void dhtValue(void){
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }


  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    barril_u.sendData(event.relative_humidity,hora);
    Serial.println("%");
  }
}
