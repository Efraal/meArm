#include <Servo.h>
#include <math.h>

Servo base, braco, antebraco, garra;  //Declara todos os servos.

const int cBraco = 84;      //Comprimento do braço.
const int cAntebraco = 82;  //Comprimento do antebraço.
const int acel = 2;         //Aceleração do servo.

const int posInicial[] = {90, 90, 90, 90};     //Posições iniciais.
const int limiteBaixo[] = {0, 50, 30, 60};     //Limites baixos.
const int limiteAlto[] = {180, 140, 100, 90};  //Limites altos.
const int erro[] = {90, 180, -90, -40};        //Erro para cálculos.

int portasJoystick[] = {A0, A1, A2, A3, 10, 11};  //Portas analógicas dos joysticks;
int joystick[] = {0, 0, 0, 0, 0, 0};              //Leituras Joysticks.
int zonaMorta[] = {400, 600};                     //Zona morta dos Joysticks.

int angulos[] = {90, 90, 90, 90};  //Angulos dos servos.
float coordenadas[]={90, 0, 92};   //Coordenadas da garra.

void setup(){
	Serial.begin(9600);  //Inicia a comunicação serial.
  joysticksInicial();  //Declara as portas conectadas aos joysticks.
	conexoes();          //Declara as portas onde cada servo está conectado.
	escritaInicial();    //Coloca os servos nas posições iniciais.
	contagem();          //Faz uma contagem regressiva para que a garra comece a se mover.
}

void loop(){
	atualizacao();  //Atualiza os angulos de acordo com os joysticks.
  incremento();   //Suavizar o movimento dos servos.
	verificacao();  //Verifica se os ângulos estão dentro do limite permitido.
	mover();        //Move os servos.
	calcs();        //Faz os cálculos necessários para descobrir os ângulos dos servos.
	debug();        //Envia informações úteis para diagnóstico.
}

void conexoes(){
	base.attach(3);       //Conecta a base à porta 3.
	braco.attach(5);      //Conecta o braco à porta 5.
	antebraco.attach(6);  //Conecta o antebraco à porta 6.
	garra.attach(9);      //Conecta a garra à porta 9.
}

void joysticksInicial(){
	for(int i = 0; i < 4; i++){  //Declara todas as portas conectadas aos joysticks como entradas.
    pinMode(portasJoystick[i], INPUT);
  }
  for(int i = 4; i < 6; i++){
    pinMode(portasJoystick[i], INPUT_PULLUP);
  }
}

void escritaInicial(){
	base.write(posInicial[0]);       //Posiciona o servo da base na posicao inicial.
	braco.write(posInicial[1]);      //Posiciona o servo do braco na posicao inicial.
  antebraco.write(posInicial[2]);  //Posiciona o servo do antebraco na posicao inicial.
  garra.write(posInicial[3]);      //Posiciona o servo da garra na posicao inicial.
}

void contagem(){
	Serial.println("Pronto.");   //Envia a mensagem pronto para sabermos que os servos se conectaram e chegaram a posição inicial.
  Serial.print("Contagem: ");  //Inicia a Contagem para o início do roteiro.
  for(int i = 3; i > 0; i--){  //Faz a contagem regressiva.
    Serial.print(i);
    if(i!=1){
      Serial.print(", ");
    } else {
      Serial.println("");
    }
  delay(1000);
  }
}

void atualizacao(){
  for(int i = 0; i < 4; i++){
    joystick[i] = analogRead(portasJoystick[i]);
  }
  for(int i = 4; i < 6; i++){
    joystick[i] = portasJoystick[i];
  }
}

void incremento(){
  if(joystick[0]<zonaMorta[0]){
    angulos[0]+=acel;
  } else if(joystick[0]>zonaMorta[1]){
    angulos[0]-=acel;
  }
  if(joystick[1]<zonaMorta[0]){
    angulos[1]+=acel;
  } else if(joystick[1]>zonaMorta[1]){
    angulos[1]-=acel;
  }
  if(joystick[2]<zonaMorta[0]){
    angulos[2]+=acel;
  } else if(joystick[2]>zonaMorta[1]){
    angulos[2]-=acel;
  }
  if(joystick[3]<zonaMorta[0]){
    angulos[3]+=acel;
  } else if(joystick[3]>zonaMorta[1]){
    angulos[3]-=acel;
  }
}

void calcs(){
  float a = cBraco;
  float f = cAntebraco;
  float x = (float)angulos[0];
  float y = (float)angulos[1];
  float z = (float)angulos[2];

  float alpha = y+z-erro[2];  //Calcula o angulo oposto à d no triangulo braco, antebraco e d.

	float d = sqrt((f*f)+(a*a)-(2*a*f*cos(alpha * PI / 180.0)));  //Calcula o vetor entre a posicao final da garra e a base.
  
  float phi = acos(((d*d)+(f*f)-(a*a))/(2*d*f)) * (180.0 / PI);  //Calcula o angulo oposto ao antebraco do trinagulo braco, antebraco e d.
	
  float theta = erro[1] - phi - y;  //Calcula o angulo oposto à aresta da coordenada z.
	
  float s = d * cos(theta * PI / 180) + erro[3];  //Calcula a menor distância entre a base e o vértice z.
	
	coordenadas[0] = s * cos(x * PI / 180.0);      //Converte o valor para inteiro e adiciona o erro.
	coordenadas[1] = s * sin(x * PI / 180.0);      //Converte o valor para inteiro e adiciona o erro.
	coordenadas[2] = d * sin(theta * PI / 180.0);  //Converte o valor para inteiro e adiciona o erro.
}

void verificacao(){
  if (angulos[1]+angulos[2]<100){
    if(angulos[1]+10<limiteAlto[1]){
      angulos[1]+=10;
    } else {
      angulos[2]-=10;
    }
  }
	for(int i = 0; i < 4; i++){
			if (angulos[i] < limiteBaixo[i]){        //Verifica se o angulo desejado é 
				angulos[i] = limiteBaixo[i];           //menor que o limite e o corrige.
			} else if (angulos[i] > limiteAlto[i]){  //Verifica se o angulo calculado é 
				angulos[i] = limiteAlto[i];            //maior que o limite e o corrige.
			}
	}
}

void mover(){
  base.write(angulos[0]);       //Move o servo da base.
  braco.write(angulos[1]);      //Move o servo do braco.
  antebraco.write(angulos[2]);  //Move o servo do antebraco.
  garra.write(angulos[3]);      //Move o servo da garra.
} 

void debug(){
  Serial.println();
  Serial.print(" X: ");          //Escreve no monitor serial uma série
  Serial.print(coordenadas[0]);  //de valores úteis para diagnosticar
  Serial.print(", Y: ");         //o funcionamento da garra.
  Serial.print(coordenadas[1]);
  Serial.print(", Z: ");
  Serial.println(coordenadas[2]);
  Serial.print("Base: ");
  Serial.print(angulos[0]);
  Serial.print(", Braco: ");
  Serial.print(angulos[1]);
  Serial.print(", Antebraco: ");
  Serial.print(angulos[2]);
  Serial.print(", Garra: ");
  Serial.println(angulos[3]);
}