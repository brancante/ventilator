//# ventilator
//Open Ventilator based on the MIT project (https://e-vent.mit.edu) with some adjustments and improovments. 

#include <AccelStepper.h>
  AccelStepper stepper(1, 8, 9); // direction Digital 9 (CW), pulses Digital 8 (CLK)
#include <MultiStepper.h>

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

#include <Stepper.h>


const int stepsPerRevolution = 200;


int Pot1 = A0;
int Pot2 = A1;
int Pot3 = A2;
float BPMi;
float BPMe;
int BPM;
int ML;
int RATIO;
int REDUCAO = 10; //Fator de redução, padrao eh 1:1
float FATOR = 0.0095   ; //este é o fator de correcao de velocidade por BPM
int FIRST = 0;  //caso primeira vez a rodar e low sw, volta ate sair do sw
int VELr;     //variaveis da red.
int STEPr;    //variaveis da red 
float TEMPm;   //Temperatura motor
int BAGp = 0;
int BAGn = 0;
int INS = 0;
int EXP = 0;
int CICLOS;
int CICLOSB;
int INTR = 0; // int flag
int INTRc = 0;  //interrupt count
int INTRb = 0; //int buffer
int STOP = 3;  //interrupcao para parada
int STOP1;
int BeginPos = 5;
int BeginPos1 = 0;
int SET;
int MODO;           //modo assistido ou mecanico
int MODOt;
int SENSIBILIDADE = 0; // inicia com a sensibilidade do assistido em 1 h20
int SENSIBILIDADEt = 1 ;
int SENSt;
int ALARME = 0 ;  //variavel que marca um alarme
int chk_positive = 0; //variable to check if the pressure was positive
int START = 6;
int START1;
int MODE; 
int WAITr = 0 ; // variavel que espera o tempo da respirada
float PRESSURE;
float PRESSUREb = 5.28;  //to calibrate sensor MPX5100DP
float PRESSUREr;
int Pmax = 0;  //variable for setting the max reached
int Pmaxc = 25; //default value for max pressure
float Vout;
float Pa;
float Pb;
float Pc;
float Ptemp;
float Pcounter;
int WAIT = 0; //variavel de estado para retorno braco no modo assistido
float BUFFER;
float RPM;
float RPMi;
float RPMe;
float SPSi;
float SPSe;
float Tt;
float Ti;
float Te;
float Tm;
int STEPS; //Total de pulsos da respiracao completa
// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.


void setup() {

  Wire.begin();
  Serial.begin(2000000);
  lcd.init();    // Initiate the LCD:
  lcd.backlight();

  pinMode (12,  INPUT_PULLUP); //switch ambu bag(limite movimento) - Fica 1 durante a operaçao
  pinMode (11,  INPUT_PULLUP); //Modo
  pinMode (0,  INPUT_PULLUP); //Sensibilidade
  pinMode(START, INPUT_PULLUP);   //6
  pinMode(STOP, INPUT_PULLUP);   // 3 
  pinMode(BeginPos, INPUT_PULLUP);    //5
  pinMode(10, OUTPUT); // Set buzzer - pin 10 as an output

cli();           // disable all interrupts
TCCR1A = 0; // interrupt for timer and read pressure
TCCR1B = 0;
TCNT1 = 0;
OCR1A = 625;// compare match register 16MHz/256/2Hz -> Para 50 Hz = 1250
TCCR1B |= (1 << WGM12); // CTC MODE
TCCR1B |= (1 << CS12); // 256 PRESCLAER
TIMSK1 |= (1 << OCIE1A); // ENABLE TIMER COMPARE INT
sei();               // enable all interrupts

  
  attachInterrupt(digitalPinToInterrupt(STOP), STOP_PRESSED, FALLING);  //Interrupcao chama outro void
 // attachInterrupt(digitalPinToInterrupt(2), BAG_PRESSED, FALLING);  //Interrupcao chama outro void

}

ISR(TIMER1_COMPA_vect){
  Vout = analogRead(3);   //read the pressure and print serial
  PRESSURE = ((Vout/1024*4.7)-0.2)/0.045;
  PRESSURE = PRESSURE * 10.2+ PRESSUREb; // CONVERSION TO cmH2O and calibrate to zero


           if (PRESSURE >=100) { //strong noize
     PRESSURE = 0; 
  }
   

          if (PRESSURE > 0.51 && PRESSURE < 0.53) { //strong noize
     PRESSURE = 0; 
  }
      if (PRESSURE < -0.51 && PRESSURE > -0.53) {
     PRESSURE = 0; 
  }

   ++Pcounter;    //to remove interference 

  if (Pcounter == 1){
    Pa = PRESSURE;
  if ( Pa == 0){
    if (Pc != 0){
     if (Pb == 0){
    Pc = 0; 
     }
    }
  }
      PRESSURE = Pc;
  }
    
  
    if (Pcounter == 2){
    Pb = PRESSURE;
     if ( Pb == 0){
    if (Pa != 0){
     if (Pc == 0){
    Pa = 0; 
     }
    }
  }
  PRESSURE = Pa;
  }
      if (Pcounter >= 3){
    Pc = PRESSURE;
    Pcounter = 0;
    if ( Pc == 0){
    if (Pb != 0){
     if (Pa == 0){
    Pb = 0; 
     }
    }
  }
  PRESSURE = Pb;
  }
  Serial.print(Pmaxc);
  Serial.print("\t");
  Serial.println(PRESSURE);

  if (PRESSURE >= Pmaxc){    //limit for highpressure
  Pmax = 1;
  }
}

void loop() {




  if (BeginPos1 == 0){ //rotina para voltar o motor para a posiccao zero
  lcd.setCursor(0, 0); 
  lcd.print("Returning to 0                                                                  ");  
  SET = digitalRead (BeginPos); //Le Switch de curso
  if (FIRST == 0 && SET == LOW){ // é a primeira vez e esta low
  VELr = 100 * REDUCAO;
  STEPr = 30 * REDUCAO;
  stepper.setMaxSpeed(VELr);  ;//1100
  stepper.setAcceleration(5000);
  stepper.move(-STEPr);
  //stepper.runToPosition();
  while (stepper.isRunning()){
    stepper.run();
  }
  FIRST++;
  SET = digitalRead (BeginPos); //Le Switch de curso
  }
  if (SET != LOW){ // seta variavel para iniciar rotina principal


  Go_home();

        
    tone(10, 1000); // Send 1KHz sound signal...  //Caso passo aqui, toca um bip
    delay(100);        // 
    noTone(10);     // Stop sound...
    delay(100);        // 
    tone(10, 1000); // Send 1KHz sound signal...
    delay(100); 
    noTone(10);     // Stop sound...// ...
    delay(100);   
    tone(10, 1000); // Send 1KHz sound signal...  //Caso passo aqui, toca um bip
    delay(100);        // 
    noTone(10);     // Stop sound...
    delay(100);        // 
    tone(10, 1000); // Send 1KHz sound signal...
    delay(100);        // ...
    noTone(10);     // Stop sound...// ...
    delay(100);   
  }
  if (SET == LOW){ // seta variavel para iniciar rotina principal
  BeginPos1 = 1;

  }
  }


  if (BeginPos1 == 1){ //==1 posicao inicial de curso


  //sensibility();  //set sensibility
  set_max_pressure(); //set max pressure
 
      
 MODOt = digitalRead(11);   
 if ( MODOt == LOW){ // Modo de operação
  MODO = 1;
 }  else {
  MODO = 0;
 }

  Pressao();
  Le_Pot(); //leitura de potenciometros  
  Tabelas_Pot();
  Calculos(); 
  Mostra_LCD();
  //Serial_Print(); //rotina para mandar serial

  if (BUFFER != 1){
  lcd.setCursor(0, 3); 
  lcd.print("START?");
  }

  START1 = digitalRead (START);

    if (START1 == LOW){ // Verifica se é para começar
    BUFFER = 1;
    }


  if (INTR == 1){ //teve interrupcao? 
    INS = 0;
    EXP = 0;
    INTR =0;

    }

  if (BUFFER == 1 && MODO == 1){ //tem start e é modo 0 (mecanico)?
  if (WAIT == 0){ //volta o braco para o contato com a bolsa caso primeira vez
  volta_bolsa();
  WAIT = 1;  //seta para nao rodar aqui novamente
  }
    Pressao();  //le pressao
    WAITr++; //variavel que conta o tempo sem respirar
     lcd.setCursor(0, 3); 
     lcd.print(" WAIT   "); 
    if ((SENSIBILIDADEt + PRESSURE) <= 0 || WAITr > 13) {
     if (WAITr > 13){
     lcd.setCursor(6, 3); 
     lcd.print(" *"); //flag emergencia (nao teve imposto para respirar 
    tone(10, 1000); // Send 1KHz sound signal...  //Caso passo aqui, toca um bip
    delay(100);        // 
    noTone(10);     // Stop sound...
    delay(100);        // 
    tone(10, 1000); // Send 1KHz sound signal...
    delay(100);        // ...
    noTone(10);     // Stop sound...
      } else {
     lcd.setCursor(6, 3); 
     lcd.print("  ");       
      }
     WAITr = 0 ;
     lcd.setCursor(0, 3); 
     lcd.print(" WAIT   "); 
    lcd.setCursor(0, 3); 
     lcd.print(" A INS  "); 
  SPSi = SPSi * REDUCAO; // multiplica pelo fator
  SPSi = SPSi * (1+(BPM * FATOR)); //correcao por pulso
  STEPS = STEPS * REDUCAO;
  stepper.setMaxSpeed(SPSi);//1100
  stepper.setAcceleration(5000);
  stepper.move(-STEPS);
  //stepper.runToPosition(); 
    while ( stepper.isRunning()  && Pmax != 1 )  //loop until motor stops or key is pressed
  {
    stepper.run();
  }
  stepper.stop();
  Pressao();  //le pressao
  if (PRESSURE <= 0){  //check for positive pressure after ins. 
    chk_positive = 1;
  } else {
    chk_positive = 0;
  }
  CICLOS++;
  if (Pmax == 1){
  lcd.setCursor(0, 3); 
  lcd.print("MAX PRES");  
  } else {
  lcd.setCursor(0, 3); 
  lcd.print(" A EXP  "); 
  }

  SPSe = SPSe * REDUCAO; 
  SPSe = SPSe * (1+(BPM * FATOR)); //correcao por pulso
  stepper.setMaxSpeed(SPSe);//1100
  stepper.setAcceleration(5000);
  stepper.move(STEPS);
  stepper.runToPosition();
  Check_alarm (); //Check pressure on airway
  //lcd.setCursor(0, 3); 
  //lcd.print("            ");
  volta_bolsa();
  Pmax = 0; //reset flag max pressure
//  if (WAITr > 13){ //if it is on auto mode, check the pressure difference
//    Check_alarm (); //Check pressure on airway
//  }

  
    }
    
  }

  if (BUFFER == 1 && MODO == 0){ //tem start e é modo 0 (mecanico)?
  if (INS == 0 && EXP == 0){ //volta o braco para o contato com a bolsa

  volta_bolsa();
  INS = 1;  //Seta para iniciar a inspiração

  }

  if (INS == 1 && EXP == 0){ //inspira
  lcd.setCursor(0, 3); 
  lcd.print("   INS  "); 
  SPSi = SPSi * REDUCAO; // multiplica pelo fator
  SPSi = SPSi * (1+(BPM * FATOR)); //correcao por pulso
  STEPS = STEPS * REDUCAO;
  stepper.setMaxSpeed(SPSi);//1100
  stepper.setAcceleration(5000);
  stepper.move(-STEPS);
  //stepper.runToPosition();
  //stepper.stop();     
  while ( stepper.isRunning()  && Pmax != 1 )  //loop until motor stops or Pmax
  {
    stepper.run();
  }
  stepper.stop();
  Pressao();  //le pressao e mostra
  if (PRESSURE <= 0){  //check for positive pressure after ins. 
    chk_positive = 1;
  } else {
    chk_positive = 0;
  }
  INS = 0;  //Seta para parar a inspiração e comecar a exp
  EXP = 1; 
  CICLOS++;
  }
  if (INS == 0 && EXP == 1){ //expira
  if (Pmax == 1){
  lcd.setCursor(0, 3); 
  lcd.print("MAX PRES");  
  } else {
  lcd.setCursor(0, 3); 
  lcd.print("   EXP  "); 
  } 
  BAGp = 0; //Garante que está zerada a interrupcao
  SPSe = SPSe * REDUCAO; 
  SPSe = SPSe * (1+(BPM * FATOR)); //correcao por pulso
  stepper.setMaxSpeed(SPSe);//1100
  stepper.setAcceleration(5000);
  stepper.move(STEPS);
   
  while (stepper.isRunning() ){  //loop until motor stops or key is pressed X && digitalRead(12) == LOW v
   stepper.run();
  }

  Check_alarm (); //Verifica se nao teve pressao na resp 
  Pressao();
  volta_bolsa(); 
  Pmax = 0;

  INS = 1;  //Seta para iniciar a inspiração e parar a exp
  EXP = 0; // 

  lcd.setCursor(0, 3); 
  lcd.print("        ");
  }
  }  

 }
}

void set_max_pressure(){
    SENSt = digitalRead(0);
  if ( SENSt == LOW){ // Switch NF
  SENSIBILIDADE++;
  if (SENSIBILIDADE >= 1 && SENSIBILIDADE <= 3) {
    Pmaxc = 10;
  }
    if (SENSIBILIDADE >= 4 && SENSIBILIDADE <= 6 ) {
    Pmaxc = 15;
  }
    if (SENSIBILIDADE >=7  && SENSIBILIDADE <= 9) {
    Pmaxc = 20;
  }
      if (SENSIBILIDADE >=10  && SENSIBILIDADE <= 12) {
    Pmaxc = 20;
  }
        if (SENSIBILIDADE >=13  && SENSIBILIDADE <= 15) {
    Pmaxc = 25;
  }
          if (SENSIBILIDADE >=16  && SENSIBILIDADE <= 18) {
    Pmaxc = 30;
  }
      if (SENSIBILIDADE >=19) {
    SENSIBILIDADE = 0;
  }
 } 
}

void sensibility(){  //disable -> set to the optimal
   SENSt = digitalRead(0);
  if ( SENSt == LOW){ // Switch NF
  SENSIBILIDADE++;
  if (SENSIBILIDADE >= 1 && SENSIBILIDADE <= 3) {
    SENSIBILIDADEt = 1;
  }
    if (SENSIBILIDADE >= 4 && SENSIBILIDADE <= 6 ) {
    SENSIBILIDADEt = 2;
  }
    if (SENSIBILIDADE >=7  && SENSIBILIDADE <= 9) {
    SENSIBILIDADEt = 3;
  }
      if (SENSIBILIDADE >=10) {
    SENSIBILIDADE = 0;
  }
 }
}
void BAG_PRESSED() { //INTERRUPCAO

  BAGp = 1;

}

void Max_pressure(){
     stepper.stop(); //stop if max pressure is reached
     stepper.moveTo(0);
     Pmax = 1;
}

void volta_bolsa(){   //volta braço para encostar na bolsa quando perde step no motor de passos
  if (Pmax == 1){
     tone(10, 1000); // Send 1KHz sound signal...  //Caso passo aqui, toca um bip
    delay(50);        // 
    noTone(10);     // Stop sound...
    delay(50);        // 
    tone(10, 1000); // Send 1KHz sound signal...
    delay(50); 
    noTone(10);     // Stop sound...// ...
     tone(10, 1000); // Send 1KHz sound signal...  //Caso passo aqui, toca um bip
    delay(50);        // 
    noTone(10);     // Stop sound...
    delay(50);        // 
    tone(10, 1000); // Send 1KHz sound signal...
    delay(50); 
    noTone(10);     // Stop sound...// ...
  Pmax = 0;
  }
  
  if (digitalRead(12) == HIGH){
  //while(digitalRead(12) == HIGH) {  //enquanto não estiver posicionado fica andando
  VELr = 500 * REDUCAO;
  STEPr = 100 * REDUCAO;
  stepper.setMaxSpeed(VELr);  ;//1100
  stepper.setAcceleration(9000);
  stepper.move(-STEPr);
  //stepper.runToPosition();
  while(stepper.isRunning()  && digitalRead(12) == HIGH) {  //enquanto não estiver posicionado fica andando
  stepper.run();
  }
   
  //}

  VELr = 100 * REDUCAO; // Ajuste fino do Switch para contato com a Bolsa
  STEPr = 6 * REDUCAO;
  stepper.setMaxSpeed(VELr);  ;//1100
  stepper.setAcceleration(9000);
  stepper.move(-STEPr);
  stepper.runToPosition();
 
  
  }
   // stepper.setCurrentPosition(0); //reset everytime the new home. -> This should make the interrupt stop ok
}
void STOP_PRESSED() { //INTERRUPCAO
 
  ++INTRb;

    if (INTRb >= 5){ // Longer press to stop (interrupt) to stop -> Safety
  
      INTRb = 0;
      BeginPos1 = 0;
      BUFFER = 0;
      INTR = 1;
      WAIT = 0; 
      WAITr = 0;
 
    }

}

void Mostra_LCD() {
  //lcd.clear();  //limpa todo o LCD (fica piscando um pouco)
  lcd.setCursor(0, 0); 
  lcd.print("BPM:"); 
  lcd.setCursor(4, 0); 
  lcd.print(BPM); 
  lcd.setCursor(0, 1); 
  lcd.print("ML:"); 
  lcd.setCursor(4, 1); 
  lcd.print(ML); 
  lcd.setCursor(0, 2); 
  lcd.print("I/E 1:"); 
  lcd.setCursor(6, 2); 
  lcd.print(RATIO); 
  
  lcd.setCursor(6, 0); 
  lcd.print("  RPMi:"); 
  lcd.setCursor(13, 0); 
  lcd.print("   ");  //limpar os numeros
  lcd.setCursor(13, 0); 
  lcd.print(RPMi); 
  lcd.setCursor(7, 1); 
  lcd.print("   Ti:"); 
  lcd.setCursor(13, 1); 
  lcd.print("   "); 
  lcd.setCursor(13, 1);
  lcd.print(Ti); 
  lcd.setCursor(7, 2); 
  lcd.print(" PRES:"); 
  lcd.setCursor(13, 2);
  lcd.print("   ");  
  lcd.setCursor(13, 2);
  lcd.print(PRESSURE,0); 
  //lcd.setCursor(16, 2);
 // lcd.print("cH2O"); 


 // lcd.setCursor(8, 3); 
 // lcd.print("S");
 // lcd.setCursor(9, 3); 
 // lcd.print(SENSIBILIDADEt);
    lcd.setCursor(8, 3); 
  lcd.print("MaxP:");
    lcd.setCursor(13, 3); 
  lcd.print(Pmaxc);
    lcd.setCursor(16, 3); 
  lcd.print("C");
  lcd.setCursor(17, 3); 
  lcd.print(CICLOS);
}

void Check_alarm (){    //Check if there has been 3 cycles without pressure 
  if (chk_positive == 1){  //minimum positive pressure after inspiration
  ++ALARME;
  
  if (ALARME >= 3){ //number of times with 0

    lcd.setCursor(0 , 3); 
    lcd.print("NO PRESS");  
    tone(10, 1000); // 
    delay(100);        // ...for 1 sec
    noTone(10);     // 
    delay(100);        // .
    tone(10, 1000); // 
    delay(100);        // 
    noTone(10);
    ALARME = 0; 
      }
   } else {
    ALARME = 0;
         }
}

void Go_home (){
  VELr = 100 * REDUCAO;
  STEPr = 100 * REDUCAO;
  stepper.setMaxSpeed(VELr);  ;//1100
  stepper.setAcceleration(9000);
  stepper.move(STEPr);
 // stepper.runToPosition();  
  INTRb = 0; // zero to the interruption counter
  while(stepper.isRunning()  && digitalRead(BeginPos) == HIGH) {  //enquanto não estiver posicionado fica andando
  stepper.run();

  FIRST++;

  }
}


void Calculos() {
  Tt = 60.00/BPM;
  Ti = Tt/(1+RATIO); 
  Te = Tt-Ti;
  
   if (Ti > 1.2){ //Limites do tempo de inspiracao -> Será necessário alterar o BPM nos extremos
    Tm = Ti - 1.20;
    Ti = 1.2;
    Te = Te + Tm;
   }
     if (Ti < 0.8){ //Limites do tempo de inspiracao
     Tm = Ti - 0.8;
     Ti = 0.8;
     Te = Te - Tm;
   }

  
  BPMi = 30.00/Ti; //
  BPMe = 30.00/Te;
  RPMi = (BPMi*STEPS*2.00)/200.00;   // 
  RPMe = (BPMe*STEPS*2.00)/200.00;
  SPSi = 3.333*RPMi;
  SPSe = 3.333*RPMe;  // Steps por segundo-> Serao necessarios para o Drive com a biblioteca Accel

}
  
void Tabelas_Pot() {
  if (BPM >= 40){ //So para melhorar o final de curso do POT
    BPM = 40.00;
   }
  if (RATIO >= 4){ //So para melhorar o final de curso do POT
    RATIO = 4.00;
  }

  if (RATIO == 1 && BPM > 37.00 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 37.00;
   }

  if (RATIO == 1 && BPM < 25 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 25.00;
   }

  if (RATIO == 2 && BPM > 25.00 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 25.00;
   }

  if (RATIO == 2 && BPM < 17 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 17.00;
   }

   if (RATIO == 3 && BPM > 18.00 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 18.00;
   }

  if (RATIO == 3 && BPM < 13 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 13.00;
   }  
  if (RATIO == 4 && BPM > 15.00 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 15.00;
   }

  if (RATIO == 2 && BPM < 12 ){ // para ajustar os limites embasados nos tempos max e min de inspiraçoes
    BPM = 12.00;
   }
  // STEPS = ML/5;  //apenas uma relacao de teste para ter um numero de ref -> relacao contante, melhor o mapeamento de 50ml de precisao
  if (ML <= 250){ // Seta mapa de ML:
    ML = 200;
    STEPS = 36;
  }
  if (ML >= 251 && ML <= 300){ 
    ML = 250;
    STEPS = 38;
  }
  if (ML >= 301 && ML <= 350){ 
    ML = 300;
    STEPS = 40;
  }
  if (ML >= 351 && ML <= 400){ 
    ML = 350;
    STEPS = 42 ;
  }
  if (ML >= 401 && ML <= 450){ 
    ML = 400;
    STEPS = 44;
  }
  if (ML >= 451 && ML <= 500){ 
    ML = 450;
    STEPS = 46;
  }
  if (ML >= 501 && ML <= 550){ 
    ML = 500;
    STEPS = 50;
  }
  if (ML >= 551 && ML <= 600){ 
    ML = 550;
    STEPS = 55;
  }
  if (ML >= 601 && ML <= 650){ 
    ML = 600;
    STEPS = 60;
  }
  if (ML >= 651 && ML <= 700){ 
    ML = 650;
    STEPS = 65;
  }
  if (ML >= 701 && ML <= 750){ 
    ML = 700;
    STEPS = 75;
  }
  if (ML >= 751 && ML <= 800){ 
    ML = 750;
    STEPS = 80;
  }
  if (ML >= 801 && ML <= 850){ 
    ML = 800;
    STEPS = 85;
  }

}


void Le_Pot() {
  BPM = map(analogRead(Pot1),0,1023,12,45); // RR
  ML = map(analogRead(Pot2),0,1023,200,850); // TV
  RATIO = map(analogRead(Pot3),0,1023,1,5); // I/E Ratio
  
  
}

void Pressao() {

  //  for (int i = 0; i < 2; i++) //para maior precisao, media das leituras
  //{


  lcd.setCursor(13, 2);
  lcd.print("   "); 
  lcd.setCursor(13, 2);
  lcd.print(PRESSURE, 0);
  lcd.setCursor(16, 2);
  lcd.print("cH2O");
}
void Temperatura() {

  TEMPm = 0;
  for (int i = 0; i < 42; i++) //para maior precisao, media das leituras
  {
  float TEMP13 = analogRead(3);
  
  TEMPm = TEMPm + TEMP13;
  } 

  TEMPm = TEMPm/43;
  float mv = ( TEMPm/1024.0)*5000;
  float cel = mv/10;
  TEMPm = cel;
 
}

void Serial_Print() {
  Serial.println(); 
  Serial.print("ML: ");
  Serial.print(ML);
  Serial.println();   
  Serial.print("RATIO: ");
  Serial.print(RATIO);
  Serial.println();  
  Serial.print("BPMi: ");
  Serial.print(BPMi);
  Serial.println();   
  Serial.print("BPMe: ");
  Serial.print(BPMe);
  Serial.println();     
  Serial.print("RPMi: ");
  Serial.print(RPMi);
  Serial.println();   
  Serial.print("RPMe: ");
  Serial.print(RPMe);
  Serial.println(); 
  Serial.print("INTRb: ");
  Serial.print(INTRb);
  Serial.println(); 
  

  }
