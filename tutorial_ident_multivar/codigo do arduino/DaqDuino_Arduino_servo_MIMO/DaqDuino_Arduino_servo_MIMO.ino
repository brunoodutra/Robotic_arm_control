// DAQ_Duino for Matlab (Arduino side code)
// Author: Me.Bruno Gomes Dutra (brunodutra@ufpa.br)

// Descrição: Código para transformar o arduino em um dispositivo Daq, para realizar a aquisição de dados, 
// acionar atuadores por meio das portas PWM e realizar leitura de dados por meio nas portas analógicas

float u1=0,u2=0,u3=0,u4=0,u5=0,u6=0; // sinais de entrada
float y1=0,y2=0,y3=0,y4=0,y5=0,y6=0; // sinais de entrada
#include <Servo.h>

Servo myservo1;  // cria um objeto para controle do servo1
Servo myservo2;  // cria um objeto para controle do servo2
Servo myservo3;  // cria um objeto para controle do servo3
Servo myservo4;  // cria um objeto para controle do servo4
Servo myservo5;  // cria um objeto para controle do servo5
Servo myservo6;  // cria um objeto para controle do servo6


void setup() {
  myservo1.attach(8);  // atribui o pino 8 para controle do servo
  myservo2.attach(9);  // atribui o pino 9 para controle do servo
  myservo3.attach(10);  // atribui o pino 10 para controle do servo
  myservo4.attach(11);  // atribui o pino 11 para controle do servo
  myservo5.attach(12);  // atribui o pino 12 para controle do servo
  myservo6.attach(13);  // atribui o pino 13 para controle do servo

  Serial.begin(9600); // Velocidade da porta Serial
  Serial.setTimeout(3); // tempo de espera máximo para resposta Serial
}

void loop() {
  if (Serial.available() > 0) {
////////////////////Entrada///////////////////
    u1 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 
    u2 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 
    u3 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 
    u4 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 
    u5 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 
    u6 = Serial.parseFloat(); // recebe o valor numérico Float pela porta Serial 

        if (u1<0) {u1=0;} // saturação inferior do sinal de controle
        if (u1>5) {u1=5;} // saturação superior do sinal de controle
        if (u2<0) {u2=0;} // saturação inferior do sinal de controle
        if (u2>5) {u2=5;} // saturação superior do sinal de controle
        if (u3<0) {u3=0;} // saturação inferior do sinal de controle
        if (u3>5) {u3=5;} // saturação superior do sinal de controle
        if (u4<0) {u4=0;} // saturação inferior do sinal de controle
        if (u4>5) {u4=5;} // saturação superior do sinal de controle
        if (u5<0) {u5=0;} // saturação inferior do sinal de controle
        if (u5>5) {u5=5;} // saturação superior do sinal de controle
        if (u6<0) {u6=0;} // saturação inferior do sinal de controle
        if (u6>5) {u6=5;} // saturação superior do sinal de controle

    u1 = u1*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   
    u2 = u2*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   
    u3 = u3*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   
    u4 = u4*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   
    u5 = u5*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   
    u6 = u6*(180/5.0);   // converte o valo dentro da escala de tensão [0,5] para posição angular [0,180]   

    myservo1.write(u1); // manda a refeferência posição angular para o controle onterno do servomotor
    myservo2.write(u2); // manda a refeferência posição angular para o controle onterno do servomotor
    myservo3.write(u3); // manda a refeferência posição angular para o controle onterno do servomotor
    myservo4.write(u4); // manda a refeferência posição angular para o controle onterno do servomotor
    myservo5.write(u5); // manda a refeferência posição angular para o controle onterno do servomotor
    myservo6.write(u6); // manda a refeferência posição angular para o controle onterno do servomotor

 ////////////////////Saída///////////////////
    y1 = analogRead(A0)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts
    y2 = analogRead(A1)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts
    y3 = analogRead(A2)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts
    y4 = analogRead(A3)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts
    y5 = analogRead(A4)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts
    y6 = analogRead(A5)*(5/1023.0); // faz a leitura do pino A0 na escala de 0 a 5 volts

   Serial.print(y1);// Envia o valor de posição angular do servo para a porta Serial
       Serial.print(" ");
    Serial.print(y2);
       Serial.print(" ");
    Serial.print(y3);
       Serial.print(" ");
    Serial.print(y4);
       Serial.print(" ");
    Serial.print(y5);
       Serial.print(" ");
    Serial.println(y6);
  }
}
