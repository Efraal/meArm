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
    coordenadas[1]+=acel;
  } else if(joystick[0]>zonaMorta[1]){
    coordenadas[1]-=acel;
  }
  if(joystick[1]<zonaMorta[0]){
    coordenadas[0]+=acel;
  } else if(joystick[1]>zonaMorta[1]){
    coordenadas[0]-=acel;
  }
  if(joystick[3]<zonaMorta[0]){
    coordenadas[2]+=acel;
  } else if(joystick[2]>zonaMorta[1]){
    coordenadas[2]-=acel;
  }
  if(joystick[2]<zonaMorta[0]){
    angulos[3]+=acel;
  } else if(joystick[3]>zonaMorta[1]){
    angulos[3]-=acel;
  }
}

void calcs(){
  float a = cBraco;
  float f = cAntebraco;
  float x = coordenadas[0];
  float y = coordenadas[1];
  float z = coordenadas[2];

  float s = sqrt((x*x)+(y*y));  //Calcula a menor distância entre a base e o vértice z. 
  float sg = s+erro[3];         //Tira o comprimento da garra deste valor.
	float d = sqrt((sg*sg)+(z*z));  //Calcula o vetor entre a posicao final da garra e a base.

	float theta = asin(z/d) * (180.0 / PI);                          //Calcula o angulo oposto à aresta da coordenada z.
	float phi = acos(((d*d)+(f*f)-(a*a))/(2*d*f)) * (180.0 / PI);    //Calcula o angulo oposto ao antebraco do trinagulo braco, antebraco e d.
	float alpha = acos(((f*f)+(a*a)-(d*d))/(2*a*f)) * (180.0 / PI);  //Calcula o angulo oposto à d no triangulo braco, antebraco e d.
  if(alpha<10){
    alpha = 10;
  }

	float anguloBraco = - (theta + phi);          //Calcula o angulo do servo do braco.
	float anguloAntebraco = alpha + theta + phi;  //Calcula o angulo do servo do antebraco.
	float anguloBase = atan(y/x) * (180.0 / PI);  //calcula o angulo do servo da base.
	
	angulos[0] = ((int)anguloBase)+erro[0];      //Converte o valor para inteiro e adiciona o erro.
	angulos[1] = ((int)anguloBraco)+erro[1];  //Converte o valor para inteiro e adiciona o erro.
	angulos[2] = ((int)anguloAntebraco)+erro[2];       //Converte o valor para inteiro e adiciona o erro.
}

void verificacao(){
	for(int i = 0; i < 3; i++){
			if (angulos[i] < limiteBaixo[i]){        //Verifica se o angulo calculado é 
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