#include <Servo.h>
#include <math.h>

Servo base, braco, antebraco, garra;           //Declara todos os servos.

const int cBraco = 84;                         //Comprimento do braço.
const int cAntebraco = 82;                     //Comprimento do antebraço.
const int erro[] = {90, 180, -90, -40};        //Erro para cálculos.
const int posInicial[] = {90, 90, 90, 90};     //Posições iniciais.
const int limiteBaixo[] = {0, 50, 30, 60};     //Limites baixos.
const int limiteAlto[] = {180, 140, 100, 90};  //Limites altos.
bool garraAberta = false;                      //Estado da garra.

int frameAtual = 0;          //Frame atual.
const float frame[4][4] = {  //cria um roteiro com as coordenadas da garra em cada frame.
  {122, 0, 84, 1},
  {122, 0, 30, 0},
  {130, 25, 30, 0},
  {130, -25, 30, 1}
  };

int angulos[] = {90, 90, 90, 90};  //Angulos dos servos.
float coordenadas[]={90, 0, 92};  //Coordenadas da garra.

void setup(){
	Serial.begin(9600);  //Inicia a comunicação serial.
	conexoes();          //Declara as portas onde cada servo está conectado.
	escritaInicial();    //Coloca os servos nas posições iniciais.
	contagem();          //Faz uma contagem regressiva para que a garra comece a se mover.
}

void loop(){
	atualizacao();         //Atualiza o frame ao final de cada movimento.
	while(!comparacao()){
    incremento();          //Suavizar o movimento dos servos.
	  calcs();             //Faz os cálculos necessários para descobrir os ângulos dos servos.
	  verificacao();       //Verifica se os ângulos estão dentro do limite permitido.
	  mover();             //Move os servos.
	  debug();             //Envia informações úteis para diagnóstico.
  }
  abrirGarra();            //Controla a abertura da garra.
}

void conexoes(){
	base.attach(3);       //Conecta a base à porta 3.
	braco.attach(5);      //Conecta o braco à porta 5.
	antebraco.attach(6);  //Conecta o antebraco à porta 6.
	garra.attach(9);      //Conecta a garra à porta 9.
}

void escritaInicial(){
	base.write(posInicial[0]);       //Posiciona o servo da base na posicao inicial.
	braco.write(posInicial[1]);      //Posiciona o servo do braco na posicao inicial.
  antebraco.write(posInicial[2]);    //Posiciona o servo do antebraco na posicao inicial.
  garra.write(posInicial[3]);        //Posiciona o servo da garra na posicao inicial.
}

void contagem(){
	Serial.println("Pronto.");   //Envia a mensagem pronto para sabermos que os servos se conectaram e chegaram a posição inicial.
  Serial.print("Contagem: ");    //Inicia a Contagem para o início do roteiro.
  for(int i = 3; i > 0; i--){    //Faz a contagem regressiva.
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
  Serial.println("");           //Escreve uma mensagem para 
  Serial.print("Frame ");       //avisar quando o servo completa
  Serial.print(frameAtual);     //o movimento de um frame.
  Serial.println(" Concluído!");
  Serial.println("");
  frameAtual +=1;
  if (frameAtual>3){
    frameAtual = 0;
  }
}

void incremento(){
  float obj[3];                 //Cria uma variável para guardar o frame atual.
  for(int i = 0; i < 3; i++){   //Salva cada valor do frame atual na variável.
    obj[i]=frame[frameAtual][i];   
  }
	for(int i = 0; i<3; i++){       //Aproxima a posicao atual da posicao definida
		if(coordenadas[i]<obj[i]){  //para o frame atual em pequenos incrementos.
			coordenadas[i]+=1;
		} else if(coordenadas[i]>obj[i]){
			coordenadas[i]-=1;
		} 
	}
}

void calcs(){
  float a = cBraco;
  float f = cAntebraco;
  float x = coordenadas[0];
  float y = coordenadas[1];
  float z = coordenadas[2];

  float s = sqrt((x*x)+(y*y));      //Calcula a menor distância entre a base e o vértice z. 
  float sg = s+erro[3];             //Tira o comprimento da garra deste valor.
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
	
	angulos[0] = ((int)anguloBase)+erro[0];       //Converte o valor para inteiro e adiciona o erro.
	angulos[1] = ((int)anguloBraco)+erro[1];      //Converte o valor para inteiro e adiciona o erro.
	angulos[2] = ((int)anguloAntebraco)+erro[2];  //Converte o valor para inteiro e adiciona o erro.
}

void verificacao(){
	for(int i = 0; i < 3; i++){
			if (angulos[i] < limiteBaixo[i]){        //Verifica se o angulo calculado é 
				angulos[i] = limiteBaixo[i];         //menor que o limite e o corrige.
			} else if (angulos[i] > limiteAlto[i]){  //Verifica se o angulo calculado é 
				angulos[i] = limiteAlto[i];          //maior que o limite e o corrige.
			}
	}
}

void abrirGarra(){
	if(garraAberta != frame[frameAtual][3]){  //Verifica se a garra está na posição ordenada pelo roteiro.
		garraAberta=!garraAberta;             //Altera o estado da garra caso ela não esteja.
	}
  if(!garraAberta){                           //Define o angulo do servo da garra de acordo com o estado dela.
    angulos[3]=limiteAlto[3];
  } else {
    angulos[3]=limiteBaixo[3];
  }
}

void mover(){
  base.write(angulos[0]);       //Move o servo da base.
  braco.write(angulos[1]);      //Move o servo do braco.
  antebraco.write(angulos[2]);  //Move o servo do antebraco.
  garra.write(angulos[3]);      //Move o servo da garra.
} 

void debug(){                      //Escreve no monitor serial uma série
  Serial.println();                //de valores úteis para diagnosticar
  Serial.print("Objetivo: ");      //o funcionamento da garra.
  Serial.print(" X: ");
  Serial.print(frame[frameAtual][0]);
  Serial.print(", Y: ");
  Serial.println(frame[frameAtual][1]);
  Serial.print("Atual: ");
  Serial.print(" X: ");
  Serial.print(coordenadas[0]);
  Serial.print(", Y: ");
  Serial.println(coordenadas[1]);
  Serial.print("Braco: ");
  Serial.print(angulos[0]);
  Serial.print(", Antebraco: ");
  Serial.println(angulos[1]);
}

bool comparacao(){             //Verifica se a garra chegou à posição correta.
  for(int i = 0; i < 3; i++){
    if(coordenadas[i]!=frame[frameAtual][i]){
      return false;
    }
  }
  return true;
}
