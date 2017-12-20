// ================================================================
// ===       BIBLIOTECAS, CONSTANTES, VARIÁVEIS E OBJETOS       ===
// ================================================================

//Bibliotecas necessárias
#include "I2Cdev.h" //biblioteca para comunicação I2C
#include "MPU9250_9Axis_MotionApps41.h" //biblioteca para utilização do DMP do MPU-9250
#include "ResponsiveAnalogRead.h" //biblioteca pra filtragem dos sensores de flexão
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h" // biblioteca necessária para comunicação I2C
#endif

//Definição dos pinos de interrupções das IMUs
#define INTERRUPT_PIN 2  //interrupção da IMU do braço
#define INTERRUPT_PIN2 3  //interrupção da IMU da mão
#define LED_PIN 13 //pino do LED do Arduino

//variável para controle do LED
bool blinkState = false;

//Variáveis para controle das IMUs
bool dmpReady = false;  //true se inicialização do DMP1 ocorreu com sucesso
bool dmpReady2 = false;  //true se inicialização do DMP2 ocorreu com sucesso
uint8_t mpuIntStatus, mpuIntStatus2;  //contém o byte de status da interrupção do MPU-9250
uint8_t devStatus, devStatus2;     //retorna o status após cada operação no dispositivo (0=sucesso, !0=erro)
uint16_t packetSize, packetSize2;   //tamanho esperado do pacote do DMP (padrão é 42 bytes)
uint16_t fifoCount, fifoCount2;    //conta os bytes na FIFO
uint8_t fifoBuffer[64], fifoBuffer2[64]; //buffer de armazenamento da FIFO
int PolegarGraus1, IndicadorGraus1, MedioGraus1, AnelarGraus1, MindinhoGraus1; //variáveis com os ângulos de dobra dos sensores de flexão
int PolegarGraus2, IndicadorGraus2, MedioGraus2, AnelarGraus2, MindinhoGraus2; //variáveis com os ângulos de dobra dos sensores de flexão
int grausAbPolInd, grausAbIndMed, grausAbMedAne, grausAbAneMin; //variáveis com os ângulos de dobra dos sensores de flexão
String valores; //string com valores para escrever na porta serial

//Instâncias das IMUs
MPU9250 mpu(0x68); //IMU do baço
MPU9250 mpu2(0x69); //IMU da mão

//instâncias dos objetos de leitura dos pinos analógicos para cada sensor de flexão
ResponsiveAnalogRead sensorPolegar1(15,true);
ResponsiveAnalogRead sensorPolegar2(14,true);
ResponsiveAnalogRead sensorIndicador1(13,true);
ResponsiveAnalogRead sensorIndicador2(12,true);
ResponsiveAnalogRead sensorMedio1(11,true);
ResponsiveAnalogRead sensorMedio2(10,true);
ResponsiveAnalogRead sensorAnelar1(9,true);
ResponsiveAnalogRead sensorAnelar2 (8,true);
ResponsiveAnalogRead sensorMindinho1(7,true);
ResponsiveAnalogRead sensorMindinho2(6,true);
ResponsiveAnalogRead sensorAbPolInd(5, false);
ResponsiveAnalogRead sensorAbIndMed(4, false);
ResponsiveAnalogRead sensorAbMedAne(3, false);
ResponsiveAnalogRead sensorAbAneMin(2, false);

//contém os dados de orientação das IMUs
Quaternion q, q2;


// ================================================================
// ===            ROTINA DE DETECÇÃO DE INTERRUPÇÕES            ===
// ================================================================

volatile bool mpuInterrupt = false;     // indica se o pino de interrupção da IMU foi para 1
volatile bool mpuInterrupt2 = false;     // indica se o pino de interrupção da IMU foi para 1
void dmpDataReady() {
  mpuInterrupt = true;
}

void dmpDataReady2() {
  mpuInterrupt2 = true;
}

// ================================================================
// ===                   CONFIGURAÇÃO INICIAL                   ===
// ================================================================

void setup() {
  // conexão do bus I2c
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // clock de comunicação I2C 400kHz.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // inicializa comunicação serial
  Serial.begin(115200);
  while (!Serial);

  // inicializa dispositivos
  // Serial.println(F("Inicializando dispositivos I2C..."));
  mpu.initialize();
  delay(100);
  mpu2.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(INTERRUPT_PIN2, INPUT);

  // testa conexão
  // Serial.println(F("Testando conexões..."));
  //Serial.println(mpu.testConnection() ? F("MPU9250 - 1 conexão OK") : F("MPU9250 - 1 conexão falhou"));
  //delay(100);
  //Serial.println(mpu2.testConnection() ? F("MPU9250 - 2 conexão OK") : F("MPU9250 - 2 conexão falhou"));

  // carregar e configurar o DMP
  //Serial.println(F("Inicializando DMP1..."));
  devStatus = mpu.dmpInitialize(0x68);
  //Serial.println(F("Inicaalizando DMP2..."));
  devStatus2 = mpu2.dmpInitialize(0x69);

  // Testa se inicialização ocorreu sem erros
  if (devStatus == 0 && devStatus2 == 0) {
    // liga o DMP
    //Serial.println(F("Habilitando DMP1..."));
    mpu.setDMPEnabled(true);
    //Serial.println(F("Habilitando DMP2..."));
    mpu2.setDMPEnabled(true);
    // Habilita deteção de interrupções do Arduino
    //Serial.println(F("Habilitando deteção de interrupções..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    //Serial.println(F("Habilitando deteção de interrupções..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady2, RISING);
    mpuIntStatus2 = mpu2.getIntStatus();

    // configura o flag do DMP para o Loop Principal saber se pode utilizá-lo
    //Serial.println(F("DMP pronto! Esperando primeira interrupção..."));
    dmpReady = true;
    //Serial.println(F("DMP2 pronto! Esperando primeira interrupção..."));
    dmpReady2 = true;
    //recebe tamanho esperado do pacote do DMP para comparação
    packetSize = mpu.dmpGetFIFOPacketSize();
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = carregamento inicial de memória falhou
    // 2 = autalizações de connfiguração do DMP falharam
//    Serial.println(F("Algo deu errado"));
//    Serial.print(F("Inicialização do DMP1 (código "));
//    Serial.print(devStatus);
//    Serial.println(F(")"));
//    Serial.print(F("Inicialização do DMP2 (código "));
//    Serial.print(devStatus2);
//    Serial.println(F(")"));
  }
  // configura LED como saída
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                     LOOP PRINCIPAL                       ===
// ================================================================

void loop() {

  // se programação do DMP falhou, não faça nada
  if (!dmpReady || !dmpReady2) return;

  // espera interrupção do DMP ou pacotes disponíveis
  while ((!mpuInterrupt && fifoCount < packetSize) || (!mpuInterrupt2 && fifoCount2 < packetSize2)) {
    //Leitura dos valores dos sensores de flexibilidade
    sensorPolegar1.update();
    sensorPolegar2.update();
    sensorIndicador1.update();
    sensorIndicador2.update();
    sensorMedio1.update();
    sensorMedio2.update();
    sensorAnelar1.update();
    sensorAnelar2.update();
    sensorMindinho1.update();
    sensorMindinho2.update();
    sensorAbPolInd.update();
    sensorAbIndMed.update();
    sensorAbMedAne.update();
    sensorAbAneMin.update();

    //conversão dos valores brutos dos sensores para ângulos
    PolegarGraus1   = map(sensorPolegar1.getValue(),   789, 860, 0, 70);
    constrain(PolegarGraus1,0, 70);
    PolegarGraus2   = map(sensorPolegar2.getValue(),   740, 851, 0, 100);
    constrain(PolegarGraus2,0, 100);
    IndicadorGraus1 = map(sensorIndicador1.getValue(), 761, 884, 0, 80);
    constrain(IndicadorGraus1,0, 80);
    IndicadorGraus2 = map(sensorIndicador2.getValue(), 702, 753, 0, 100);
    constrain(IndicadorGraus2,0, 100);
    MedioGraus1     = map(sensorMedio1.getValue(),     742, 899, 0, 80);
    constrain(MedioGraus1,0, 80);
    MedioGraus2     = map(sensorMedio2.getValue(),     747, 900, 0, 100);
    constrain(MedioGraus2,0, 100);
    AnelarGraus1    = map(sensorAnelar1.getValue(),    740, 849, 0, 80);
    constrain(AnelarGraus1,0, 80);
    AnelarGraus2    = map(sensorAnelar2.getValue(),    713, 767, 0, 100);
    constrain(AnelarGraus2,0, 100);
    MindinhoGraus1  = map(sensorMindinho1.getValue(),  721, 871, 0, 80);
    constrain(MindinhoGraus1,0, 80);
    MindinhoGraus2  = map(sensorMindinho2.getValue(),  731, 827, 0, 100);
    constrain(MindinhoGraus2,0, 100);
    grausAbPolInd   = map(sensorAbPolInd.getValue(),   784, 875, 0, -30);
    constrain(grausAbPolInd,0, -30);
    grausAbIndMed   = map(sensorAbIndMed.getValue(),   820, 858, 20, 0);
    constrain(grausAbIndMed,0, 20);
    grausAbMedAne   = map(sensorAbMedAne.getValue(),   875, 910, 20, 0);
    constrain(grausAbMedAne,0, 20);
    grausAbAneMin   = map(sensorAbAneMin.getValue(),   828, 890, 35, 0);
    constrain(grausAbAneMin,0, 35);
  }

  // reseta flag de interrupção e recebe byte de status da interrupção
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  mpuInterrupt2 = false;
  mpuIntStatus2 = mpu2.getIntStatus();

  // recebe contagem da FIFO
  fifoCount = mpu.getFIFOCount();
  fifoCount2 = mpu2.getFIFOCount();
  // verifica se houve overflow
  if (((mpuIntStatus & 0x10) || fifoCount == 1024) || ((mpuIntStatus2 & 0x10) || fifoCount2 == 1024)) {
    // reseta FIFO
    mpu.resetFIFO();
    mpu2.resetFIFO();
    // Serial.println(F("FIFO overflow!"));

    // caso contrário, verifica pela interrupção do DMP
  } else if (mpuIntStatus & 0x02 || mpuIntStatus2 & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // ler pacote da FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu2.getFIFOBytes(fifoBuffer2, packetSize2);

    // observa a contagem da FIFO caso haja > 1 pacote disponível (para leitura imediata, sem esperar uma interrupção)
    fifoCount -= packetSize;
    fifoCount2 -= packetSize2;

    // recebe os valores de orientação na forma de quaterniões
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);

    //verifica se há bytes para ler no buffer de entrada da porta serial
    if (Serial.available() > 0)
    {
      //consome os bytes do buffer de entrada
      Serial.read();
      
      //Construção da string para ser escrita na porta serial
      valores  = String(PolegarGraus1) + ";"; //posição na string: 0
      valores += String(PolegarGraus2) + ";"; //posição na string: 1
      valores += String(IndicadorGraus1) + ";"; //posição na string: 2
      valores += String(IndicadorGraus2) + ";"; //posição na string: 3
      valores += String(MedioGraus1) + ";"; //posição na string: 4
      valores += String(MedioGraus2) + ";"; //posição na string: 5
      valores += String(AnelarGraus1) + ";"; //posição na string: 6
      valores += String(AnelarGraus2) + ";"; //posição na string: 7
      valores += String(MindinhoGraus1) + ";"; //posição na string: 8
      valores += String(MindinhoGraus2) + ";"; //posição na string: 9
      valores += String(grausAbPolInd) + ";"; //posição na string: 10
      valores += String(grausAbIndMed) + ";"; //posição na string: 11
      valores += String(grausAbMedAne) + ";"; //posição na string: 12
      valores += String(grausAbAneMin) + ";"; //posição na string: 13
      valores += String(q.w,4) + ";"; //posição na string: 14
      valores += String(q.x,4) + ";"; //posição na string: 15
      valores += String(q.y,4) + ";"; //posição na string: 16
      valores += String(q.z,4) + ";"; //posição na string: 17
      valores += String(q2.w,4) + ";"; //posição na string: 18
      valores += String(q2.x,4) + ";"; //posição na string: 19
      valores += String(q2.y,4) + ";"; //posição na string: 20
      valores += String(q2.z,4); //posição na string: 21
      
      //envia valores  para porta serial
      Serial.println(valores);
    }
    
    // pisca LED para indicar atividade
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}