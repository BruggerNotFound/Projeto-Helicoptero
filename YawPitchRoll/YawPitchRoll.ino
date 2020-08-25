// ================================================================
// ===                        Circuito                          ===
// ================================================================
/*  Bussola:
 *   VCC = 5V
 *   GND = GND
 *   SCL = 21 (SCL)
 *   SDA = 20 (SDA)
 *   DRDY = Desconectado
 *   
 *  Giroscopio/Acelerometro:
 *   VCC = 5V
 *   GND = GND
 *   SCL = 21 (SCL)
 *   SDA = 20 (SDA)
 *   XCL = Desconectado
 *   XDA = Desconectado
 *   AD0 = 5V (Opcional, troca o endereco I2C de 0x68 para 0x69)
 *   INT = Desconectado
*/

// ================================================================
// ===                 Bibliotecas e variaveis                  ===
// ================================================================
#include "Wire.h" //Comunicação I2C
#include "QMC5883LCompass.h"  //Bussola
#include "MPU6050.h" //Girocel
#include "I2Cdev.h"


// --- Declaracao de Objetos ---
//O endereco padrao da bussola e 0x0D*
/* Na placa e informado que o chip e um HMC5883L, mas apos uma pequena analise descobri que
 *  se tratava de um chip QMC5883L, que possui a mesma funcao, mas algumas diferencas.
 * Uma delas o endereco, que numa HMC seria 0x1E, na QMC passa a ser 0x0D */
QMC5883LCompass bussola;

int azimute, direcao;
char arrayDirecoes[3];

//O endereco padrao do giroscopio/acelerometro e 0x68
/* O endereco pode ser mudado para 0x69 caso o pino AD0 no modulo esteja ligado em high,
 *  nesse caso deve ser especificado o endereco */
const int MPU = 0x68;
MPU6050 girocel;


int rangeGiro, rangeAcel, teste;
int Rx, Ry, Rz;
float acelX, acelY, acelZ;
float giroX, giroY;
float angAcelX, angAcelY, angGiroX, angGiroY;
float yaw, pitch, roll;
float errAcelX = 0, errAcelY = 0, errGiroX = 0, errGiroY = 0;
float tempo = 0, tempoPassado = 0, tempoUltimo = 0;

// --- Configuracao ---
  /* Retire de comentário a linha "#define PRINTAR_BUSSOLA_ANGULO" para printar no monitor
   *  serial o azimute da bussola em graus */
//#define PRINTAR_BUSSOLA_ANGULO
    
  /* Retire de comentário a linha "##define PRINTAR_BUSSOLA_DIRECAO" para printar no
   *  monitor serial o um valor de 0-15 indicando a direcao que a bussola aponta acompanhado
   *  de uma string de 3 caracteres representando o valor com letras */
//#define PRINTAR_BUSSOLA_DIRECAO

  /* Retire de comentário a linha "#define PRINTAR_GIROCEL_RAW" para printar no monitor
   *  serial os valores de acel X/Y/Z e giro X/Y/Z */
//#define PRINTAR_GIROCEL_RAW

  /* Retire de comentário a linha "#define PRINTAR_GIROCEL_RAW" para printar no monitor
   *  serial os valores de Pitch e Roll */
//#define PRINTAR_GIROCEL_PR

  /* Retire de comentário a linha "#define PRINTAR_TUDO" para printar no monitor
   *  serial todos os valores */
#define PRINTAR_TUDO

// ================================================================
// ===                      Setup Inicial                       ===
// ================================================================
void setup()
{
    long vel = 38400; //Velocidae da comunicacao serial
     
    Wire.begin(); //Inicia Rede de comunicacao I2C
    Serial.begin(vel); //Inicia comunicacao Serial, 38400 usado pois funciona bem com 8 e 16MHz
    Serial.println("- Comunicação Serial Iniciada -"); Serial.print(">>Velocidade: "); Serial.println(vel);

    //Inicializacao dos dispositivos I2C:
    Serial.println("\n- Iniciando Dispositivos I2C");
    
    bussola.init(); //QMC5883L
    bussola.setSmoothing(10,0); //Para evitar leituras erradas, a funcao faz uma media movel com N valores antes de retornar, (N=10)
    Serial.println(">>QMC5883L Inicializado...");

    girocel.initialize(); //MPU6050
    Serial.println(">>MPU6050 Inicializado...");
    /* 0x00 = 250 deg/sec
     * 0x01 = 500 deg/sec
     * 0x02 = 1000 deg/sec
     * 0x03 = 2000 deg/sec */
    girocel.setFullScaleGyroRange(0x00); //configura a sensibilidade do giroscopio
    /* 0x00 = 2g
     * 0x01 = 4g
     * 0x02 = 8g
     * 0x03 = 16g */
    girocel.setFullScaleAccelRange(0x00); //configura a sensibilidade do acelerometro
    
    if(girocel.getFullScaleAccelRange() <= 0x01) rangeAcel = (girocel.getFullScaleAccelRange()*2)+2; else rangeAcel = (girocel.getFullScaleAccelRange()*8)-8;
    Serial.print(" >Sensibilidade Acelerometro:\t");Serial.print(rangeAcel); Serial.println("g");
    if(girocel.getFullScaleGyroRange() <= 0x01) rangeGiro = (girocel.getFullScaleGyroRange()*250)+250; else rangeGiro = (girocel.getFullScaleGyroRange()*1000)-1000;
    Serial.print(" >Sensibilidade Giroscopio:\t");Serial.print(rangeGiro); Serial.println(" deg/sec\n");

    //Calibracao do MPU6050
    calcularErroMPU();
    
    //Espera Confirmação
    Serial.print("- Entre qualquer caractere para iniciar o loop: ");
    while(Serial.available() && Serial.read()); //Limpa o buffer
    while(!Serial.available()); //Espera Input
    Serial.println("\n\n\n");
}

// ================================================================
// ===                      Loop Principal                      ===
// ================================================================
void loop()
{
    // --- Bussola ---
    bussola.read(); //Inicia leitura dos valores da bussola
    azimute = bussola.getAzimuth(); //Retorna o azimute (sendo 0 == Norte)
    direcao = bussola.getBearing(azimute); //Retorna um valor de 0-15 (0 == Norte) da direcao no sentido horario
    bussola.getDirection(arrayDirecoes, azimute); //Semelhante ao Bearing, porem no formato de uma string de 3 caracteres com as letras da direcao

    // === PRINTA OS VALORES ===
    #ifdef PRINTAR_BUSSOLA_ANGULO //Printa o azimute (bussola):
      Serial.print("Azimute:\t"); 
      Serial.print(azimute);
      Serial.println();
    #endif
    #ifdef PRINTAR_BUSSOLA_DIRECAO //Printa a direcao (bussola):
      Serial.print("Direcao:\t"); 
      Serial.print(direcao); Serial.print("\t"); //Valor de 0-15
      Serial.print(arrayDirecoes[0]); Serial.print(arrayDirecoes[1]); Serial.print(arrayDirecoes[2]); //3 caracteres
      Serial.println();
    #endif
    
    // --- Giroscopio/Acelerometro ---
    // === ACELEROMETRO ===
    girocel.getAcceleration(&Rx, &Ry, &Rz); //Le valores de aceleracao
    //Ajusta os valores de acordo com a sensibilidade:
    acelX = ((float)Rx*(float)rangeAcel)/32768.0; //Converte os valores de X
    acelY = ((float)Ry*(float)rangeAcel)/32768.0; // - - - de Y
    acelZ = ((float)Rz*(float)rangeAcel)/32768.0; // - - - de Z

    //Calcula Roll e Pitch com os dados do acelerometro
    angAcelX = (atan(acelY/sqrt(pow(acelX, 2)+pow(acelZ, 2))) * 180/PI) + (errAcelX*-1);
    angAcelY = (atan(-1 * acelX/sqrt(pow(acelY, 2) + pow(acelZ, 2))) * 180/PI) + (errAcelY*-1);

    // === GIROSCOPIO ===
    tempoUltimo = tempo;
    tempo = millis(); //Faz a leitura do tempo
    tempoPassado = (tempo - tempoUltimo)/1000; //Divide por 1000 para receber o valor em seg

    girocel.getRotation(&Rx, &Ry, &Rz); //Le valores de rotacao
    //Ajusta os valores de acordo com a sensibilidade:
    giroX = ((float)Rx*(float)rangeGiro)/32768.0; //Converte is valores de X
    giroY = ((float)Ry*(float)rangeGiro)/32768.0; // - - - de Y
    //Corrige de acordo com erro calculado*
    giroX = giroX + (errGiroX*-1);
    giroY = giroY + (errGiroY*-1);
    /* Os valores agora estao em graus por segundo, deg/sec, entao se multiplicarmos por
     *  segundos vamos obter o angulo puro em graus (deg/sec * sec = deg) */
    angGiroX = angGiroX + giroX*tempoPassado;
    angGiroY = angGiroY + giroY*tempoPassado;

    //Obtem os valores da Yaw, Pitch e Roll
    yaw = azimute; //A bussola é a fonte mais confiavel para o valor
    pitch = .95*angGiroY + .05*angAcelY; //Filtro complementar combina os valores do Giro e Acel
    roll = .95*angGiroX + .05*angAcelX;
    
    // === PRINTA OS VALORES ===
    #ifdef PRINTAR_GIROCEL_RAW //Printa os valores de acel e giro em decimal (girocel)
      Serial.print("Acel (p,r):\t");
      Serial.print(angAcelY); Serial.print("\t");
      Serial.print(angAcelX); Serial.print("\t");
      Serial.print("| Giro (p,r):\t");
      Serial.print(angGiroY); Serial.print("\t");
      Serial.print(angGiroX); Serial.println("\t");
    #endif

    #ifdef PRINTAR_GIROCEL_PR //Printa os valores de Pitch e Roll
      Serial.print("Pitch:");
      Serial.print(pitch); Serial.print("\t");
      Serial.print("| Roll:");
      Serial.print(roll); Serial.println("\t");
    #endif

    #ifdef PRINTAR_TUDO
      Serial.print("Yaw/Pitch/Roll:\t");
      Serial.print(yaw); Serial.print("\t");
      Serial.print(pitch); Serial.print("\t");
      Serial.print(roll); Serial.print("\t");
      Serial.print("| Direcao:\t"); 
      Serial.print(direcao); Serial.print("\t");
      Serial.print(arrayDirecoes[0]); Serial.print(arrayDirecoes[1]); Serial.println(arrayDirecoes[2]);
    #endif
    
    //Delay
    //delay(250);
}

// ================================================================
// ===                         Funcoes                          ===
// ================================================================
void calcularErroMPU()
{
    Serial.println("- Realizando calibragem do MPU6050...");
    for(teste = 0; teste < 250; teste++) //Faz X leituras do sensor
    {
      girocel.getAcceleration(&Rx, &Ry, &Rz); //Le valores de aceleracao
      //Ajusta os valores de acordo com a sensibilidade:
      acelX = ((float)Rx*(float)rangeAcel)/32768.0; //Converte os valores de X
      acelY = ((float)Ry*(float)rangeAcel)/32768.0; // - - - de Y
      acelZ = ((float)Rz*(float)rangeAcel)/32768.0; // - - - de Z
      //Soma todas leituras
      errAcelX += (atan(acelY/sqrt(pow(acelX, 2)+pow(acelZ, 2))) * 180/PI);
      errAcelY += (atan(-1 * acelX/sqrt(pow(acelY, 2) + pow(acelZ, 2))) * 180/PI);
    }
    //calculo da media:
    errAcelX /= teste+1;
    errAcelY /= teste+1;
    
    //Repete o processo para o Giroscopio:
    for(teste = 0; teste < 250; teste++)
    {
      girocel.getRotation(&Rx, &Ry, &Rz); //Le valores de rotacao
      //Ajusta os valores de acordo com a sensibilidade:
      giroX = ((float)Rx*(float)rangeGiro)/32768.0; //Converte is valores de X
      giroY = ((float)Ry*(float)rangeGiro)/32768.0; // - - - de Y
      //Soma todas as leituras
      errGiroX += giroX;
      errGiroY += giroY;
    }
    //calculo da media:
    errGiroX /= teste+1;
    errGiroY /= teste+1;

    // --- Printa os valores ---
    Serial.print(">>Erro Acelerometro (X,Y):\t");
    Serial.print(errAcelX); Serial.print("\t"); Serial.println(errAcelY);
    Serial.print(">>Erro Giroscopio (X,Y):\t");
    Serial.print(errGiroX); Serial.print("\t"); Serial.print(errGiroY);
    Serial.println("\n");
}
