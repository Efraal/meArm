#include <Servo.h>
#include <math.h>

Servo base, braco, antebraco, garra; //Declara todos os servos

const int a = 92;                        //Comprimento do braço
const int f = 90;                        //Comprimento do antebraço

int current = 0;                         //Frame atual

int cGarra[]={170, 140};                 //Limites da garra
bool eGarra = false;                     //Estado da garra 

const int initpos[] = {90, 90, 74, 170}; //Posições iniciais
const int pB[] = {50, 30, 0};            //Limites baixos
const int pA[] = {140, 100, 180};        //Limites altos
const int erro[] = {0, -90, 74};         //Erro para cálculos

int angles[] = {65, 80, 74};             //Angulos de cada servo em ordem base, braço e antebraço
float coords[]={90, 0, 92};              //Coordenadas da garra em x(frente), y(esquerda) e z(cima)

const float pos[4][3] = {                //Posições em cada frame
  {45, -25, 20},
  {95, -25, 20},
  {95, 25, 20},
  {45, 25, 20}
  };

void setup(){
	Serial.begin(9600); //Inicia a comunicação serial
	initattach();       //Declara as portas onde cada servo está conectado
	initwrite();        //Coloca os servos nas posições iniciais
	countdown();        //Faz uma contagem regressiva para que a garra comece a se mover
}

void loop(){
	update();  //Atualiza o frame baseado na posição atual da garra e mostra as posições em tempo real pelo monitor serial
	sweep();   //Aproxima a atual posição do servo do objetivo em pequenos incrementos para suavizar seu movimento
	calcs();   //Faz os cálculos necessários para descobrir os ângulos dos servos
	check();   //Verifica se os ângulos estão dentro dos parâmetros
	move();    //Move os servos
	claw();    //Abre e fecha a garra
	delay(5);  //Adiciona um pequeno delay para suavizar todo o processo
}

void initattach(){
	garra.attach(3);     //Conecta a garra à porta 3
	direita.attach(5);   //Conecta o antebraco à porta 5
	esquerda.attach(6);  //Conecta o braco à porta 6
	base.attach(9);      //Conecta a base à porta 9
}

void initwrite(){
	direita.write(initpos[0]);   //Coloca o braco na posicao inicial
  esquerda.write(initpos[1]);  //Coloca o antebraco na posicao inicial
	base.write(initpos[2]);      //Coloca a base na posicao inicial
  garra.write(initpos[3]);     //Coloca a garra na posicao inicial
}

void countdown(){
	Serial.println("Pronto.");       //Escreve pronto para mostrar que os servos foram conectados e posicionados
  Serial.println("Ínicio em: ");   //Começa a contagem regressiva
  for(int i = 3; i > 0; i--){      //Faz a contagem
    Serial.print(i);
    if(i!=1){
        Serial.print(", ");
    }
  delay(1000);
  }
}

void update(){
  if(comparison()){
    Serial.println("");
    Serial.println("Done!");
    Serial.println("");
    current +=1;
    if (current>3){
      current = 0;
    }
  }
  debug();
}

void claw(){
	if((current==1 && !eGarra) || (current == 2 && eGarra)){
		eGarra=!eGarra;
	}
	garra.write(cGarra[eGarra]);
}

void calcs(){
	float L = sqrt(pow(coords[0], 2)+pow(coords[1], 2)+pow(coords[2], 2));

	float theta = asin(coords[2]/L) * (180.0 / PI);
	float phi = acos(((L*L)+(f*f)-(a*a))/(2*L*f)) * (180.0 / PI);
	float alpha = acos(((f*f)+(a*a)-(L*L))/(2*a*f)) * (180.0 / PI);

	float right = 180 - (theta + phi);
	float left = alpha + theta + phi;
	float base = atan(coords[1]/coords[0]) * (180.0 / PI);
	
	angles[0] = ((int)right)+erro[0];
	angles[1] = ((int)left)+erro[1];
	angles[2] = ((int)base)+erro[2];
}

void check(){
	for(int i = 0; i < 3; i++){
			if (angles[i] < pB[i]){
				angles[i] = pB[i];
			} else if (angles[i] > pA[i]){
				angles[i] = pA[i];
			}
	}
}

void move(){
  direita.write(angles[0]);
  esquerda.write(angles[1]);
  base.write(angles[2]);
}

void sweep(){
  float obj[3];
  for(int i = 0; i < 3; i++){
    obj[i]=pos[current][i];
  }
	for(int i = 0; i<3; i++){
		if(coords[i]<obj[i]){
			coords[i]+=1;
		} else if(coords[i]>obj[i]){
			coords[i]-=1;
		} 
	}
}

void debug(){
  Serial.println();
  Serial.print("Objetivo: ");
  Serial.print(" X: ");
  Serial.print(pos[current][0]);
  Serial.print(", Y: ");
  Serial.println(pos[current][1]);
  Serial.print("Atual: ");
  Serial.print(" X: ");
  Serial.print(coords[0]);
  Serial.print(", Y: ");
  Serial.println(coords[1]);
  Serial.print("Direita: ");
  Serial.print(angles[0]);
  Serial.print(", Esquerda: ");
  Serial.println(angles[1]);
}

bool comparison(){
  for(int i = 0; i < 3; i++){
    if(coords[i]!=pos[current][i]){
      return false;
    }
  }
  return true;
}