#include <SoftwareSerial.h>
#include <Thermistor.h> //INCLUSÃO DA BIBLIOTECA

/*DECLARAÇÃO DE QUAIS PINOS SÃO RESPONSÁVEIS PELA NOVA PORTA SERIAL DEFINITVA VIA SOFTWARE*/
const int rxPin = 11; //porta onze recebe informações do Dimmer AC (nao necessario)
const int txPin = 12; //porta 12 manda informações para o Dimmer AC (é o RX do Dimmer)
const int sensorInterrupt = 0;  // qual interruptor do Arduino será utilizado? Como estamos usando o UNO com o pino 2, é o zero. ler sobre interrupts em arduinos diferentes
const int sensorPin       = 2; // interrupt 0 no Arduino Uno é o pino digital 2, pino do sensor de VAZAO (precisa ser o 2)

const int IN1 = 9; // saida 'positiva' do Driver da bomba (varia com PWM)
const int IN2 = 8; // saida 'negativa' do Driver da bomba (em geral, tem 1V em relação ao GND)
const int pinPump = 10; // pino que controla a velocidade de giro da bomba
const int pinBuzzer = 3; //pino do buzzer

/* SEÇÃO DE DEFINIÇÃO DE PARÂMETROS DE CONTROLE */
int controlTypePump = 4; //0 - on/off 1 - P 2 - PI 3 - PID 4 - MANUAL
float flowSetpoint = 1.0; //setpoint da vazão
float kcPump = 0; //ganho do controlador da bomba, dado nas unidades de %min/L
float tauIPump = 1000000; //tau I em segundos
float tauDPump = 0; //tau D em segundos
float i=0;
float biasPump = 0; //Porcentagem da tensão da bomba inicial. 100 - maxima 0 - desligada
float oldErrorPump = 0; 
float oldTimeControlPump = 0;
float oldIntegralPump = 0;
float hysteresisFlow = 1.0;

//CONTROLE DA TEMPERATURA//
float tempSetpoint = 25; //setpoint da temperatura 
int controlTypeRes = 4; //0 - on/off 1 - P 2 - PI 3 - PID 4 - MANUAL 5 - CASCATAP 6 - CASCATAPI 7 - CASCATAPID 8 - FEEDFORWARD+P 9 - FFPI 10 - FFPID
float kcRes = 0; //ganho do controlador da bomba, dado nas unidades de 1/°C
float tauIRes = 1000000; //tau I em segundos
float tauDRes = 0;  
float j=0;
float biasRes = 0; //Bits da potenia da resistencia inicial. 100 - maxima 0 - desligada
float oldErrorRes = 0;
float oldTimeControlRes = 0;
float oldIntegralRes = 0;
float hysteresisTemp = 3;

/* CONTROLE AVANÇADO - CASCATA E FEEDFORWARD */
float biasFlowSetpoint = 0;
float oldj = 0;
float oldFlowAvg = 0;
float kFF = 0;
float tau1FF = 1000000;
float tau2FF = 1000000;

/* SEÇÃO DE CONFIGURAÇÕES DOS SENSORES */
float interval = 500; //intervalo de tempo entre leituras do sensor de vazao e também do controle
float calibrationFactor = 78.5; //Fator de calibração. Quantos Hertz equivalem a 1 L/min? Use o FreqCount e calibre vc mesmo.
volatile byte pulseCount = 0; // variavel auxiliar na contagem de pulsos
unsigned long oldTime = 0; // tempo inicial do arduino.
float totalMilliLitres = 0; // volume inicial
float flowMax = 1.3; //se a vazao passar disso ele corta

/* SEÇÃO DE CONFIGURAÇÕES DAS BIBLIOTECAS UTILIZADAS */
SoftwareSerial newSerial =  SoftwareSerial(rxPin, txPin);
Thermistor sensorTIN(0); //VARIÁVEL DO TIPO THERMISTOR, INDICANDO O PINO ANALÓGICO (A0) EM QUE O TERMISTOR ESTÁ CONECTADO
Thermistor sensorTOUT(5); //VARIÁVEL DO TIPO THERMISTOR, INDICANDO O PINO ANALÓGICO (A5) EM QUE O TERMISTOR ESTÁ CONECTADO

/* CONFIGURAÇÕES DAS MÉDIAS UTILIZADAS */
/* MÉDIA MÓVEL */
int mediaMovelT = 1; //qual é o tamanho da média movel utilizada. 1 para desligado //qual é o tamanho da média movel utilizada 1 para desligado
int mediaMovelFlow = 5; //qual é o tamanho da média movel utilizada 1 para desligado

float tempInVector[20];
int indexIn = 0;
float tempOutVector[20];
int indexOut = 0;
float flowVector[20];
int k = 0;

/*MÉDIA MOVEL EXPONENCIAL*/
//para desligar a média movel exponencial, fazer alfa = 1.
float alfaTemp = 0.1; //recomenda-se valores entre 0.05 e 0.3. 0.1 indica que a medida atual tem o valor somente de 10%, sendo o 90% sendo o peso das medidas anteriores.
float alfaFlow = 0.4; //recomenda-se valores entre 0.05 e 0.3. 0.1 indica que a medida atual tem o valor somente de 10%, sendo o 90% sendo o peso das medidas anteriores.
float oldTempIn;
float oldTempOut;


/*asd*/
void setup()
{
  Serial.begin(9600); //inicializa porta serial com velocidade de transmissão de 9600
  newSerial.begin(9600); //inicializa a nossa nova serial (porta 11) com velocidade de transmissão de 9600 também
  pinMode(sensorPin, INPUT); //inicialização de pino do sensor de vazão como input
  digitalWrite(sensorPin, HIGH); //inicialização do pino do sensor como LIGADO (os pulsos são contados na queda, por isso tem que começar ligado)
  pinMode(IN1,OUTPUT); //inicializa porta 'positiva' do driver como saida
  pinMode(IN2,OUTPUT);//inicializa porta 'negativa' do driver como saida
  pinMode(pinPump,OUTPUT); //inicializa porta de controle da velocidade da bomba como saida
  digitalWrite(IN1,HIGH); // esses dois comandos definem a direção de giro da bomba. se fosse invertido inverteriamos os estados
  digitalWrite(IN2,LOW);
  pinMode(pinBuzzer,OUTPUT);
  manipulatePump(biasPump); //inicializa a bomba com a velocidade biasPump
  manipulateRes(biasRes);
  /*The Hall-effect sensor is connected to pin 2 which uses interrupt 0. Configured to trigger on a FALLING state change (transition from HIGH
  (state to LOW state)
  Aqui estamos configurando como irá funcionar a interrupção do nosso arduino. Toda vez que houver uma QUEDA (FALLING) de tensão no 
  pino do sensor de vazão, o procedimento "pulseCounter" irá rodar, e enquanto o procedimento estiver rodando, o tempo de processamento
  do millis() não será contado.
  Em suma, configuramos como será feita a contagem de pulsos. Posteriormente será usado para medição de frequencia.*/
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING); //you can use Rising or Falling
  
}

void loop()
{
/* SEÇÃO DE RECEBIMENTO DE INFORMAÇÕES DA PORTA SERIAL */
  while (Serial.available() > 0) 
  {
      String in = Serial.readString(); 
 
      if (in.indexOf("SPF")>=0)
      {
        flowSetpoint = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(in);
      }

      if (in.indexOf("SPT")>=0)
      {
        tempSetpoint = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(tempSetpoint);
      }
  
      if (in.indexOf("CTP")>=0)
      {
        controlTypePump = int(removeInicio(in).toFloat()); 
        buzzerOK();
        //Serial.println(controlTypePump);
      }

      if (in.indexOf("CTR")>=0)
      {
        controlTypeRes = int(removeInicio(in).toFloat()); 
        buzzerOK();
        //Serial.println(controlTypeRes);
      }
  
      if (in.indexOf("BP")>=0)
      {
        biasPump = removeInicio(in).toFloat(); 
        biasPump = float(int(biasPump*255/100 + 0.5))*100/255;
        buzzerOK();
        //Serial.println(biasPump);
      }
  
      if (in.indexOf("BR") >= 0)
      {
        biasRes =  removeInicio(in).toFloat(); 
        biasRes =float(int(biasRes*22/100 + 0.5))*100/22;
        buzzerOK();
      }

      if (in.indexOf("KCP")>=0)
      {
        kcPump = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(kcPump);
      }
      
      if (in.indexOf("TIP")>=0)
      {
        tauIPump = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(tauIPump);
      }

      if (in.indexOf("TDP")>=0)
      {
        tauDPump = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(tauDPump);
      }

      if (in.indexOf("KCR")>=0)
      {
        kcRes = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(kcRes);
      }

      if (in.indexOf("TIR")>=0)
      {
        tauIRes = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(tauIRes);
      }

      if (in.indexOf("TDR")>=0)
      {
        tauDRes = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(tauDRes);
      }

      if (in.indexOf("HF")>=0)
      {
        hysteresisFlow = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("HT")>=0)
      {
        hysteresisTemp = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }
      //reach Steady State
      if (in.indexOf("RSS")>=0)
      {
        biasPump = i;
        biasRes = j; 
        biasFlowSetpoint = flowSetpoint;
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("KFF")>=0)
      {
        kFF = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("T1F")>=0)
      {
        tau1FF = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("T2F")>=0)
      {
        tau2FF = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("MMT")>=0)
      {
        mediaMovelT = removeInicio(in).toInt(); 
        indexIn = 0;
        indexOut = 0;
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("MMF")>=0)
      {
        mediaMovelFlow = removeInicio(in).toInt(); 
        k=0;
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("AT")>=0)
      {
        alfaTemp = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

      if (in.indexOf("AF")>=0)
      {
        alfaFlow = removeInicio(in).toFloat(); 
        buzzerOK();
        //Serial.println(flowSetpoint);
      }

       if (in.indexOf("RWU")>=0)
      {
        oldIntegralPump = 0;
        oldIntegralRes = 0;
        buzzerOK();
        //Serial.println(flowSetpoint);
      }
      
   }
 
   if((millis() - oldTime) > interval)    //Fazer esse cálculo somente quando o intervalo passar.
  { 
    // Disable the interrupt while calculating flow rate and sending the value to the host
    // Ou seja, desabilitar a interrupção enquanto estivermos calculando a vazão da bomba.
    detachInterrupt(sensorInterrupt);

    // Because this loop may not complete in exactly 1 second intervals we calculate the number of milliseconds that have passed since the last execution and use that to scale the output. We also apply the calibrationFactor to scale the output based on the number of pulses per second per units of measure (litres/minute in this case) coming from the sensor.
    // Esse loop pode não completar em um segundo, por isso, calculamos o numero de milissegundos que passaram desde a última execução e fazemos essa correção. (Não completa em um segundo pq o arduino tem que processar o que está dentro do if).
    //Aqui é feita uma extrapolação. Calcula-se a frequência no intervalo mencionado e extrapola-se 1 segundo, para obtermos a vazão total.
    float flowRate = ((interval / (millis() - oldTime)) * pulseCount)*1000 / (interval*calibrationFactor);

    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    float flowMilliLitres = (1000*flowRate / 60); //converte vazão de L/min para mL/min e depois divide por 60 para saber quantos mL estão passando.

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres; //é literalmente uma integração

    // Print the flow rate for this second in litres / minute
    float tempIn = sensorTIN.getTemp();
    float tempOut = sensorTOUT.getTemp();
    
    tempInVector[indexIn] = tempIn;

    if (indexIn == mediaMovelT - 1)
      {
       indexIn = 0;
      }
    else
      {
       indexIn++;
      }

    tempOutVector[indexOut] = tempOut;
    
    if (indexOut == mediaMovelT - 1)
      {
       indexOut = 0;
      }
    else
      {
       indexOut++;
      }

    flowVector[k] = flowRate;
    
    if (k == mediaMovelFlow - 1)
      {
       k = 0;
      }
    else
      {
       k++;
      }

    
    /* SEÇÃO CÁLCULO DAS MÉDIAS MÓVEIS*/
    float flowAvg = average(flowVector,mediaMovelFlow);
    float tempInAvg = average(tempInVector,mediaMovelT);
    float tempOutAvg = average(tempOutVector,mediaMovelT);

    //Filtro contra spike de vazão
    if (flowAvg > flowMax) 
    {
      flowAvg = flowMax;
    }

    
    /* SEÇÃO CÁLCULO DAS MÉDIAS MÓVEIS EXPONENCIAIS */
    //SOMENTE APÓS 5 SEGUNDOS
    if (float(millis())/1000<5)
    {
    oldTempIn = tempInAvg;
    oldTempOut = tempOutAvg;
    oldFlowAvg = flowAvg;
    }
    else
    { 
    tempInAvg = tempInAvg*alfaTemp + (1-alfaTemp)*oldTempIn;
    tempOutAvg = tempOutAvg*alfaTemp + (1-alfaTemp)*oldTempOut;
    flowAvg = flowAvg*alfaFlow + (1-alfaFlow)*oldFlowAvg;

    oldTempIn = tempInAvg;
    oldTempOut = tempOutAvg;
    oldFlowAvg = flowAvg;
    }
    
    switch (controlTypePump)
    {
      case 0: //ON-OFF
        i = onoff(flowAvg,flowSetpoint,hysteresisFlow,i,0,100);
        break;
      case 1: //P
        i = controlP(kcPump,flowAvg,flowSetpoint,biasPump,0,100);
        break;
      case 2: //PI
        i = controlPIPump(kcPump,tauIPump,flowAvg,flowSetpoint,biasPump,0,100);
        break;
      case 3: //PID
        i = controlPIDPump(kcPump,tauIPump,tauDPump,flowAvg,flowSetpoint,biasPump,0,100);
        break;
      case 4:
        i = biasPump;
        break;
    }
    oldTimeControlPump = millis();

    manipulatePump(i); //aciona a bomba com a potência calculada

    switch (controlTypeRes)
    {
      case 0: //ON-OFF
        j = onoff(tempOutAvg,tempSetpoint,hysteresisTemp,j,0,100);
        break;
      case 1: //P
        j = controlP(kcRes,tempOutAvg,tempSetpoint,biasRes,0,100);
        break;
      case 2: //PI
        j = controlPIRes(kcRes,tauIRes,tempOutAvg,tempSetpoint,biasRes,0,100);
        break;
      case 3: //PID
        j = controlPIDRes(kcRes,tauIRes,tauDRes,tempOutAvg,tempSetpoint,biasRes,0,100);
        break;
      case 4:
        j = biasRes;
        break;
      case 5:
        j = biasRes;
        flowSetpoint = controlP(kcRes,tempOutAvg,tempSetpoint,biasFlowSetpoint,0,100);
        break;
      case 6:
        j = biasRes;
        flowSetpoint = controlPIRes(kcRes,tauIRes,tempOutAvg,tempSetpoint,biasFlowSetpoint,0,100);
        break;
      case 7:
        j = biasRes;
        flowSetpoint = controlPIDRes(kcRes,tauIRes,tauDRes,tempOutAvg,tempSetpoint,biasFlowSetpoint,0,100);
        break;
      case 8:
        oldj = j;
        j = feedForwardP(kcRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,biasRes,0,100);
        break;
      case 9:
        oldj = j;
        j = feedForwardPI(kcRes,tauIRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,biasRes,0,100);
        break;
      case 10:
        oldj = j;
        j = feedForwardPI(kcRes,tauIRes,tempOutAvg,tempSetpoint,kFF,tau1FF,tau2FF,flowAvg,biasRes,0,100);
        break;
    }
    oldTimeControlRes = millis();
    manipulateRes(j); //aciona a bomba com a potência calculada

    Serial.print(float(float(millis())/1000),2);

    Serial.print(" ");
    
    Serial.print(flowAvg, 2);  // Print the integer part of the variable      

    Serial.print(" ");  
    
    Serial.print(flowSetpoint);
    
    Serial.print(" ");

    Serial.print(float(int(i*255/100 + 0.5))*100/255);
    
    Serial.print("% ");  

    Serial.print(hysteresisFlow);
    
    Serial.print(" ");

    Serial.print(tempInAvg,1);
    
    Serial.print(" ");
    
    Serial.print(tempOutAvg,1);
    
    Serial.print(" ");
    
    Serial.print(tempSetpoint,1);
    
    Serial.print(" ");

    Serial.print(float(int(j*22/100 + 0.5))*100/22);
    
    Serial.print("% ");

    Serial.println(hysteresisTemp,1);
    // Reset the pulse counter so we can start incrementing again
    //Zera todos os pulsos contados até agora.
    pulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }

}

float onoff(float y, float ysp, float hysteresis, float atual, const int lowerlimit, const int upperlimit)
{
  float u;
  if (y > ysp + hysteresis) {u = lowerlimit;}
  else
    {
    if (y < ysp - hysteresis) 
      {u = upperlimit;}
      else
      {
       u = atual; 
      }
    }
  return u;
}

float controlP(float kc,float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float u = P;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  return u;
}

float controlPIPump(float kc,float tauI, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralPump + kc/tauI * (error + oldErrorPump)/2 * (millis() - oldTimeControlPump)/1000;
  if (I>100){I = 100;}else{if(I<0){I=0;}}
  oldErrorPump = error;
  
  float u = P + I;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit){u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralPump = I;}
  return u;
}

float controlPIDPump(float kc,float tauI, float tauD, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralPump + kc/tauI * (error + oldErrorPump)/2 * (millis() - oldTimeControlPump)/1000;
  float D = kc * tauD * (error - oldErrorPump) / (millis() - oldTimeControlPump)/1000;
  oldErrorPump = error;
  float u = P + I + D;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralPump = I;}
  return u;
}

float controlPIRes(float kc,float tauI, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;
  oldErrorRes = error;
  float u = P + I;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralRes = I;}
  return u;
}

float controlPIDRes(float kc,float tauI, float tauD, float y,float ysp,float Pbias,const int lowerlimit,const int upperlimit)
{
  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;

  float D = kc * tauD * (error - oldErrorPump) / (millis() - oldTimeControlRes)/1000;
  oldErrorRes = error;
  float u = P + I + D;
  //com esses dois checks abaixo, evitamos saturação do atuador.
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  if (u>=lowerlimit && u<=upperlimit){oldIntegralRes = I;}
  return u;
}

float feedForwardP(float kc,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  u = u + P;
  
  if (u > upperlimit) {u = upperlimit;}
  if (u < lowerlimit) {u = lowerlimit;}
  
  return u;
}

float feedForwardPI(float kc,float tauI,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;
  
  oldErrorRes = error;
  u = u + P + I;
  
  if (u > upperlimit) {u = upperlimit;}else{oldIntegralRes = I;}
  if (u < lowerlimit) {u = lowerlimit;}else{oldIntegralRes = I;}
  
  return u;
}

float feedForwardPID(float kc,float tauI,float tauD,float y,float ysp,float kFF,float tau1FF, float tau2FF, float y1,float Pbias,const int lowerlimit,const int upperlimit)
{
  float b1 = kFF*(tau1FF+interval/1000)/(tau2FF+interval/1000);
  float b2 = -kFF*tau1FF/(tau2FF+interval/1000);
  float a1 = -tau2FF/(tau2FF + interval/1000);
  float u = -a1*oldj + b1*y1 + b2*oldFlowAvg;

  float error = ysp - y; //variável de desvio do erro
  float P = Pbias + kc*error; //P é uma variavel em bits que controla quanto de potência irá para o atuador
  float I = oldIntegralRes + kc/tauI * (error + oldErrorRes)/2 * (millis() - oldTimeControlRes)/1000;
  oldIntegralRes = I;
  float D = kc * tauD * (error - oldErrorPump) / (millis() - oldTimeControlRes)/1000;
  oldErrorRes = error;
  u = u + P + I + D;
  
  if (u > upperlimit) {u = upperlimit;}else{oldIntegralRes = I;}
  if (u < lowerlimit) {u = lowerlimit;}else{oldIntegralRes = I;}
  
  return u;
}

void manipulatePump(float i)
{
 analogWrite(pinPump,i*255/100);
}

void manipulateRes(float i)
{
 enviaDimmer(int(i*22/100 + 0.5));
}
//Insterrupt Service Routine - O CÓDIGO QUE SERÁ EXECUTADO TODA VEZ QUE O PULSO CAIR
void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}

void buzzerOK()
{
  //F5
  //E5 A4 B4 C5
  tone(pinBuzzer,698,200);
}

float average(float vetor[], int n)
{
float total = 0;
int i;
    for(i=0; i<n; i++)
    {
      total=total+vetor[i];//loop for calculatin total
    }
  float avg=total/i;//calculate average
  return avg;
}

String removeInicio(String str)
{  
int StringCount = 0;
String strs[20];
  while (str.length() > 0)
  {
    int index = str.indexOf(' ');
    if (index == -1) // No space found
    {
      strs[StringCount++] = str;
      break;
    }
    else
    {
      strs[StringCount++] = str.substring(0, index);
      str = str.substring(index+1);
    }
  }
return strs[1];
}

void enviaDimmer(uint8_t nivel) //0 até 23
{
  newSerial.write(2);
  newSerial.write(1);
  newSerial.write(nivel);
  newSerial.write(3);
}
