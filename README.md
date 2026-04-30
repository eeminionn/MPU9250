#include <Wire.h>

#define MPU_ADDR 0x68

#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define GYRO_XOUT_H  0x43

// Cantidad de muestras para calcular varianza
const int NUM_MUESTRAS = 50;

// Escala del giroscopio
// ±250 grados/segundo = 131 LSB/(°/s)
const float GYRO_SCALE = 131.0;

// Offsets del giroscopio
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

// Ángulos integrados
float anguloX = 0;
float anguloY = 0;
float anguloZ = 0;

// Buffers para guardar muestras
float muestrasAnguloX[NUM_MUESTRAS];
float muestrasAnguloY[NUM_MUESTRAS];
float muestrasAnguloZ[NUM_MUESTRAS];

int indice = 0;
bool bufferLleno = false;

unsigned long tiempoAnterior = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  iniciarMPU();

  Serial.println("Calibrando giroscopio...");
  Serial.println("Deja el MPU completamente quieto.");

  calibrarGiroscopio();

  Serial.println("Calibracion lista.");
  Serial.println();

  tiempoAnterior = micros();
}

void loop() {
  int16_t rawGX, rawGY, rawGZ;

  leerGiroscopio(rawGX, rawGY, rawGZ);

  // Convertir lectura cruda a grados por segundo
  float gx = (rawGX / GYRO_SCALE) - offsetX;
  float gy = (rawGY / GYRO_SCALE) - offsetY;
  float gz = (rawGZ / GYRO_SCALE) - offsetZ;

  // Calcular delta de tiempo en segundos
  unsigned long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  tiempoAnterior = tiempoActual;

  // Integrar velocidad angular para obtener ángulo aproximado
  anguloX += gx * dt;
  anguloY += gy * dt;
  anguloZ += gz * dt;

  // Guardar muestras de ángulo
  muestrasAnguloX[indice] = anguloX;
  muestrasAnguloY[indice] = anguloY;
  muestrasAnguloZ[indice] = anguloZ;

  indice++;

  if (indice >= NUM_MUESTRAS) {
    indice = 0;
    bufferLleno = true;
  }

  if (bufferLleno) {
    float varianzaX = calcularVarianza(muestrasAnguloX, NUM_MUESTRAS);
    float varianzaY = calcularVarianza(muestrasAnguloY, NUM_MUESTRAS);
    float varianzaZ = calcularVarianza(muestrasAnguloZ, NUM_MUESTRAS);

    Serial.println("----- VARIANZA ANGULAR -----");

    Serial.print("Angulo X: ");
    Serial.print(anguloX, 2);
    Serial.print(" grados | Varianza X: ");
    Serial.println(varianzaX, 6);

    Serial.print("Angulo Y: ");
    Serial.print(anguloY, 2);
    Serial.print(" grados | Varianza Y: ");
    Serial.println(varianzaY, 6);

    Serial.print("Angulo Z: ");
    Serial.print(anguloZ, 2);
    Serial.print(" grados | Varianza Z: ");
    Serial.println(varianzaZ, 6);

    Serial.println();
  }

  delay(20);
}

void iniciarMPU() {
  // Despertar MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Configurar giroscopio en ±250 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);
}

void leerGiroscopio(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void calibrarGiroscopio() {
  const int N = 500;

  long sumaX = 0;
  long sumaY = 0;
  long sumaZ = 0;

  for (int i = 0; i < N; i++) {
    int16_t rawGX, rawGY, rawGZ;

    leerGiroscopio(rawGX, rawGY, rawGZ);

    sumaX += rawGX;
    sumaY += rawGY;
    sumaZ += rawGZ;

    delay(3);
  }

  offsetX = (sumaX / float(N)) / GYRO_SCALE;
  offsetY = (sumaY / float(N)) / GYRO_SCALE;
  offsetZ = (sumaZ / float(N)) / GYRO_SCALE;

  Serial.print("Offset X: ");
  Serial.println(offsetX, 4);

  Serial.print("Offset Y: ");
  Serial.println(offsetY, 4);

  Serial.print("Offset Z: ");
  Serial.println(offsetZ, 4);
}

float calcularVarianza(float datos[], int n) {
  float promedio = 0;
  float varianza = 0;

  for (int i = 0; i < n; i++) {
    promedio += datos[i];
  }

  promedio = promedio / n;

  for (int i = 0; i < n; i++) {
    float diferencia = datos[i] - promedio;
    varianza += diferencia * diferencia;
  }

  varianza = varianza / n;

  return varianza;
}
