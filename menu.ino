#include "ST7565.h"

#define ClockPin PB14  // labeled either as CLK or as A
#define DataPin PB13   // labeled either as DT or as B
#define ButtonPin PB12 // labeled as SW

#include <Task.h>

// include sub files
#include "RotaryEncoderTask.h" // this implements the rotary encoder task

TaskManager taskManager;

// forward declare functions passed to task constructors now required
void HandleButtonChanged(EncoderButtonState state);
void HandleRotationChanged(int8_t rotationDelta);

RotaryEncoderTask RotaryTask(HandleRotationChanged,
    HandleButtonChanged,
    ClockPin,
    DataPin,
    ButtonPin);

#define LED_BUILTIN PC13

//variables for interface and set points
const int flexion_const = 90;
const int extension_const = 0;
const int speed_const = 50;
const int home_const = 45;
const int cycles_const = 10;
int actual_cycles = 0;

//rom
int flexion;
int extension;
char number[4];
//velocidad
int velocidad;
//home
int home_position;
//cycles
int cycles;
//flags
bool flag_rotacion;
int flag_select;
bool select_sentido;
int posicion_actual;
int flag_menu;
bool flag_pause;
bool flag_stop;
bool flag_motion;
bool flag_positionsensor;
int flag_calibracion;

// pin 9 - Serial data out (SID) PA7
// pin 8 - Serial clock out (SCLK) PA5
// pin 7 - Data/Command select (RS or A0) PA9
// pin 6 - LCD reset (RST) PA10
// pin 5 - LCD chip select (CS) PA4
//ST7565 glcd(9, 8, 7, 6, 5);
ST7565 glcd(PA7, PA5, PB10, PB11, PA4);

//variables for control outputs and inputs

//pwm
int pwm_value;
int pwm1 = PB0;
int pwm2 = PB1;

//limit switch
int LS1 = PA15;//amarillo
int LS2 = PB4;
bool flag_LS1;
bool flag_LS2;

//potentiometer
const int sensorPin = PA0;   // Select the input pin for the potentiometer
int sensorValue = 0; // Variable to store the value coming from the sensor
int sensorHighest = 0;
int sensorLowest = 4095;
int sensoraux = 0;
int flexion_calibrated = 2220;
int extension_calibrated = 3200;
int sensoracum = 0;
int contador_global = 0;
int mil = 0, centena = 0, decena = 0, unidad = 0;
char sensorV[5] = "1234";
char fcalibrationstr[5] = "0000";
char ecalibrationstr[5] = "0000";
int actual_pos = 0;

double datavalue = 0;
double timevalue = 0;
int count = 0;

// The setup() method runs once, when the sketch starts
void setup()   { 
  Serial.begin(115200);
  glcd.st7565_init();
  glcd.st7565_command(CMD_DISPLAY_ON);
  glcd.st7565_set_brightness(0x10);

  glcd.display(); // show splashscreen
  delay(1000);
  glcd.clear();
  // draw a string at location (0,0)
  glcd.drawstring(0, 0, "HOLA MUNDO");
  glcd.drawstring(20, 1, "HOLA MUNDO");
  glcd.drawstring(40, 2, "HOLA MUNDO");
  glcd.drawstring(60, 3, "HOLA MUNDO");
  glcd.drawstring(80, 4, "HOLA MUNDO");
  glcd.drawstring(0, 5, "HOLA MUNDO");
  glcd.drawstring(50, 6, "HOLA MUNDO");
  glcd.drawstring(40, 7, "HOLA MUNDO");
  glcd.display();
  delay(500);
  
  //glcd.fillrect(15, 40, 112, 8, 0);//borra renglon
  //glcd.fillrect(0, 40, 15, 8, 0); //borra flecha
  

  taskManager.StartTask(&RotaryTask);

 // pinMode(btn_select, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

 ////////////////AQUI DEBE IGUALARSE A LOS VALORES EEPROM+

  flexion = flexion_const;
  extension = extension_const;
  number[3] = '\0';
  velocidad = speed_const;
  home_position = home_const;
  cycles = cycles_const;
///////////////////////////////////////////////////////// 
  flag_rotacion = false;
  flag_select = 10;
  select_sentido = true;
  flag_menu = 0;
  flag_pause = true;
  flag_stop = true;
  flag_motion = false;
  flag_positionsensor = false;
  flag_calibracion = 0;
  
  menu_inicio();

//pwm
  pinMode(pwm1, PWM);
  pinMode(pwm2, PWM);

  //limit switch
  pinMode(LS1, INPUT_PULLUP);
  pinMode(LS2, INPUT_PULLUP);

  //ADC
  pinMode(sensorPin, INPUT_ANALOG);

  flag_LS1 = false;
  flag_LS2 = false;
}

void loop(){
  
  taskManager.Loop();

  interface();

  pwm_value = (int)(65535*velocidad/100);
/*
  if(millis() - timevalue > 100){
      timevalue = millis();
      sensor_position();
  }
 */ 
//*
  if(digitalRead(LS1) == true){
    flag_LS1 = true;
    flag_LS2 = false;
    delay(50);
  }
  else if(digitalRead(LS2) == false){
    flag_LS1 = false;
    flag_LS2 = true;
    delay(50);
  }

  
  if(flag_stop == false)
  {   
    if(flag_pause == false)
    {
      actuator_control();
    }
    else
    {
      pwmWrite(pwm1, 0);
      pwmWrite(pwm2, 0);
    }    
    
  }
  else
  {
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, 0);
  }
//*/

}

void actuator_control()
{
  /*
  while(sensoracum <= flexion){
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, pwm_value);     
  }

  while(sensoracum >= extension){
    pwmWrite(pwm1, pwm_value);
    pwmWrite(pwm2, 0);     
  }
  */
  sensor_position();
  //Serial.println(sensoracum);
  actual_pos = (90*(extension_calibrated - sensoraux))/(extension_calibrated - flexion_calibrated);
  Serial.println(actual_pos);
  if(actual_pos == flexion){
    digitalWrite(LED_BUILTIN, LOW);
    pwmWrite(pwm1, pwm_value);
    pwmWrite(pwm2, 0);
  }
  else if(actual_pos == extension){
    digitalWrite(LED_BUILTIN, HIGH);
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, pwm_value);
  }
  /*
      if(flag_LS1 == true)
      {
        pwmWrite(pwm1, pwm_value);
        pwmWrite(pwm2, 0);     
      }
      else if(flag_LS2 == true)
      {
        pwmWrite(pwm1, 0);
        pwmWrite(pwm2, pwm_value);
      }
      */
}

void sensor_position()
{
  sensoracum = 0;
  //Read the value from the sensor:
  for(double i = 0; i < 1000; i++){
    //Read the value from the sensor:
    sensoraux = analogRead(sensorPin); 
    sensoracum = sensoracum + sensoraux; 
  }
  sensoracum = sensoracum/1000;
  sensoraux = sensoracum;
  adctoangle();
  /*
  sensoraux = analogRead(sensorPin);
  adctoangle();
  sensorValue = sensoraux;
  adc_value();
  //Serial.print(sensorV);
  
  if(sensoracum > sensorHighest)
    sensorHighest = sensoracum;
  if(sensoracum < sensorLowest)
    sensorLowest = sensoracum;
   */
}

void interface(){
  if(flag_rotacion == true){
    flag_rotacion = false;
    actualizacion_display();    
  }
  if(flag_menu == 0){//inicio
    switch (flag_select){
      case 5: flag_select = 10;
              flag_menu = 3;
              menu_home();
              break;
      case 6: flag_select = 10;
              flag_menu = 1;
              menu_rom();
              break;
      case 7: flag_select = 10;
              flag_menu = 2;
              menu_velocidad();
              break;
      case 0: flag_select = 10;
              flag_menu = 4;
              menu_cycles();
              break;
      case 2: flag_select = 10; 
              flexion = flexion_const;
              extension = extension_const;
              velocidad = speed_const;
              home_position = home_const;
              cycles = cycles_const;
              nvic_sys_reset();
              break;
      case 1: flag_select = 10;
              flag_menu = 5;
              flag_pause = false;
              flag_stop = false;
              menu_start();
              break;
      case 3: flag_select = 10;
              flag_menu = 6;
              menu_settings();
    }  
  }
  else if(flag_menu == 1){//rom
    switch (flag_select){
      case 7: flag_select = 10;
              posicion_actual = 1;              
              break;
      case 1: flag_select = 10;
              posicion_actual = 2;
              break;
      case 2: flag_select = 10;
              posicion_actual = 7;
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              posicion_actual = 3;
              menu_inicio();
              break;
    }   
  }
  else if(flag_menu == 2){//speed
    switch (flag_select){
      case 7: flag_select = 10;
              posicion_actual = 2;              
              break;
      case 2: flag_select = 10;
              posicion_actual = 7;
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              posicion_actual = 3;
              menu_inicio();
              break;
    }      
  }
  else if(flag_menu == 3){//home
    switch (flag_select){
      case 7: flag_select = 10;
              posicion_actual = 2;              
              break;
      case 2: flag_select = 10;
              posicion_actual = 7;
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              posicion_actual = 3;
              menu_inicio();
              break;
    }      
  }
  else if(flag_menu == 4){//cycles
    switch (flag_select){
      case 7: flag_select = 10;
              posicion_actual = 2;              
              break;
      case 2: flag_select = 10;
              posicion_actual = 7;
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              posicion_actual = 3;
              menu_inicio();
              break;
    }      
  }
  else if(flag_menu == 5){//running
    switch (flag_select){
      case 2: flag_select = 10;
              posicion_actual = 2;
              menu_start();
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              posicion_actual = 3;
              menu_inicio();
              break;
    }      
  }else if(flag_menu == 6){//settings
    switch (flag_select){
      case 5: flag_select = 10;
              posicion_actual = 5;
              flexion_calibration();
              break;
      case 3: flag_select = 10;
              flag_menu = 0;
              menu_inicio();
              break;
    }      
  }  
}

void flexion_calibration(){
  digitalWrite(LED_BUILTIN,LOW);
  flexion_calibrated = 0;
  extension_calibrated = 0;
  
  while(digitalRead(LS1) == false){
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, pwm_value);     
  }
  pwmWrite(pwm1, 0);
  pwmWrite(pwm2, 0);
  for(double i = 0; i < 100000; i++){
    //Read the value from the sensor:
    sensoraux = analogRead(sensorPin); 
    flexion_calibrated = flexion_calibrated + sensoraux; 
  }
  flexion_calibrated = flexion_calibrated/100000;
  Serial.println(flexion_calibrated);
  sensorValue = flexion_calibrated;  
  adc_value(); 
  fcalibrationstr[0] = sensorV[0];
  fcalibrationstr[1] = sensorV[1];
  fcalibrationstr[2] = sensorV[2];
  fcalibrationstr[3] = sensorV[3];
  
  while(digitalRead(LS2) == true){
    pwmWrite(pwm1, pwm_value);
    pwmWrite(pwm2, 0);
  }
  pwmWrite(pwm1, 0);
  pwmWrite(pwm2, 0);
  for(double i = 0; i < 100000; i++){
    //Read the value from the sensor:
    sensoraux = analogRead(sensorPin); 
    extension_calibrated = extension_calibrated + sensoraux; 
  }
  extension_calibrated = extension_calibrated/100000;
  Serial.println(extension_calibrated);
  sensorValue = extension_calibrated;
  adc_value();
  ecalibrationstr[0] = sensorV[0];
  ecalibrationstr[1] = sensorV[1];
  ecalibrationstr[2] = sensorV[2];
  ecalibrationstr[3] = sensorV[3];
  digitalWrite(LED_BUILTIN,HIGH);
  menu_settings();
}
void extension_calibration(){
  
}
void HandleButtonChanged(EncoderButtonState state)
{
    if (state == EncoderButtonState_Pressed)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (state == EncoderButtonState_Released)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      if(flag_menu == 0){//menu inicio
        if(posicion_actual == 5)
          flag_select = 5;//home
        else if(posicion_actual == 6)
          flag_select = 6;//rom
        else if(posicion_actual == 7)
          flag_select = 7;//velocidad
        else if(posicion_actual == 0)
          flag_select = 0;//cycles
        else if(posicion_actual == 1)
          flag_select = 1;//iniciar
        else if(posicion_actual == 2)
          flag_select = 2;//reset
        else if(posicion_actual == 3)
          flag_select = 3;//settings
      }
      else if(flag_menu == 1){//menu rom
        if(posicion_actual == 7){
          flag_select = 7;//flexion
          glcd.fillrect(0, 56, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 1, "->");
          glcd.display();
        } 
        else if(posicion_actual == 1){
          flag_select = 1;//extension  
          glcd.fillrect(0, 8, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 2, "->");
          glcd.display();
        }  
        else if(posicion_actual == 2){
          flag_select = 2;//cambiar  
          glcd.fillrect(0, 16, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 7, "->");
          glcd.display();
        }
        else if(posicion_actual == 3){
          flag_select = 3;//ok  
          glcd.display();
        }    
      }
      else if(flag_menu == 2){//speed
        if(posicion_actual == 7){
          flag_select = 7;//velocidad
          glcd.fillrect(0, 56, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 2, "->");
          glcd.display();
        } 
        else if(posicion_actual == 2){
          flag_select = 2;//cambiar  
          glcd.fillrect(0, 16, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 7, "->");
          glcd.display();
        }
        else if(posicion_actual == 3){
          flag_select = 3;//ok  
          glcd.display();
        }         
      }
      else if(flag_menu == 3){//home
        if(posicion_actual == 7){
          flag_select = 7;
          glcd.fillrect(0, 56, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 2, "->");
          glcd.display();
        } 
        else if(posicion_actual == 2){
          flag_select = 2;//cambiar  
          glcd.fillrect(0, 16, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 7, "->");
          glcd.display();
        }
        else if(posicion_actual == 3){
          flag_select = 3;//ok  
          glcd.display();
        }        
      }
      else if(flag_menu == 4){//cycles
        if(posicion_actual == 7){
          flag_select = 7;
          glcd.fillrect(0, 56, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 2, "->");
          glcd.display();
        } 
        else if(posicion_actual == 2){
          flag_select = 2;//cambiar  
          glcd.fillrect(0, 16, 15, 8, 0); //borra flecha
          glcd.drawstring(0, 7, "->");
          glcd.display();
        }
        else if(posicion_actual == 3){
          flag_select = 3;//ok  
          glcd.display();
        }        
      }
      else if(flag_menu == 5){//running
        if(posicion_actual == 2)
        {
          flag_select = 2;
          
          if(flag_pause == true){
            flag_pause = false;   
          }
          else{
            flag_pause = true;
            //digitalWrite(LED_BUILTIN, HIGH);
          }  
        } 
        else if(posicion_actual == 3)
        {
          flag_select = 3;//stop
          flag_pause = true;
          flag_stop = true;
        }     
      }
      else if(flag_menu == 6){//settings
        if(posicion_actual == 5)
        {
          flag_select = 5;//calibrate
        }
        else if(posicion_actual == 3)
        {
          flag_select = 3;//back
        }     
      }
    } 
    else
    {
      //boton mantenido
    }
}

void HandleRotationChanged(int8_t rotationDelta)
{   
    if (rotationDelta > 0)
    {
      flag_rotacion = true;
      select_sentido = false;
      //digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      //digitalWrite(LED_BUILTIN, LOW);
      flag_rotacion = true;
      select_sentido = true;
      //digitalWrite(LED_BUILTIN, HIGH);
    }
}

void actualizacion_display(){
  if(flag_menu == 0){
    switch (posicion_actual){
      case 5: glcd.fillrect(0, 40, 15, 8, 0); //borra flecha
              if(select_sentido == true){
                glcd.drawstring(0, 6, "->");
                posicion_actual = 6;
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              
              break;
      case 6: glcd.fillrect(0, 48, 15, 8, 0); //borra flecha
              if(select_sentido == true){
                glcd.drawstring(0, 7, "->");
                posicion_actual = 7;
              }
              else{
                glcd.drawstring(0, 5, "->");
                posicion_actual = 5;  
              }
              
              break;
      case 7: glcd.fillrect(0, 56, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 0, "->");
                posicion_actual = 0;
              }
              else{
                glcd.drawstring(0, 6, "->");
                posicion_actual = 6;  
              }    
              break;
      case 0: glcd.fillrect(0, 0, 15, 8, 0); //borra flecha
              if(select_sentido == true){
                glcd.drawstring(0, 1, "->");
                posicion_actual = 1;  
              }
              else{
                glcd.drawstring(0, 7, "->");
                posicion_actual = 7;  
              }           
              break;
      case 1: glcd.fillrect(0, 8, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2; 
              }
              else{
                glcd.drawstring(0, 0, "->");
                posicion_actual = 0;  
              }
              break;
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 1, "->");
                posicion_actual = 1;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 5, "->");
                posicion_actual = 5;  
              }
              else{
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              break;
      }    
  }
  else if(flag_menu == 1){
    switch (posicion_actual){
      case 7: glcd.fillrect(15, 56, 15, 8, 0); //borra number
              if(select_sentido == true){
                flexion = flexion + 1;
                if(flexion > 90){
                  flexion = 90;
                }  
              }
              else{
                flexion = flexion - 1;
                if(flexion < extension){
                  flexion = extension + 1;
                }     
              }
              numbers();
              glcd.drawstring(15, 7, number);
              break;
      case 1: glcd.fillrect(15, 8, 15, 8, 0); //borra number
              if(select_sentido == true){
                extension = extension + 1;
                if(extension > flexion){
                  extension = 90;
                }  
              }
              else{
                extension = extension - 1;
                if(extension < 0){
                  extension = 0;
                }     
              }
              numbers_1();
              glcd.drawstring(15, 1, number);
              break;
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              else{
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              break;
    }
  }
  else if(flag_menu == 2){//speed
    switch (posicion_actual){
      case 7: glcd.fillrect(15, 56, 15, 8, 0); //borra number
              if(select_sentido == true){
                velocidad = velocidad + 1;
                if(velocidad > 100){
                  velocidad = 100;
                }  
              }
              else{
                velocidad = velocidad - 1;
                if(velocidad < 0){
                  velocidad = 0;
                }     
              }
              numbers_velocidad();
              glcd.drawstring(15, 7, number);
              break;
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              else{
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              break;
    }    
  }
  else if(flag_menu == 3){ //home
    switch (posicion_actual){
      case 7: glcd.fillrect(15, 56, 15, 8, 0); //borra number
              if(select_sentido == true){
                home_position = home_position + 1;
                if(home_position > 120){
                  home_position = 120;
                }  
              }
              else{
                home_position = home_position - 1;
                if(home_position < 0){
                  home_position = 0;
                }     
              }
              numbers_home();
              glcd.drawstring(15, 7, number);
              break;
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              else{
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              break;
    }    
  }
  else if(flag_menu == 4){ //cycles
    switch (posicion_actual){
      case 7: glcd.fillrect(15, 56, 15, 8, 0); //borra number
              if(select_sentido == true){
                cycles = cycles + 1;
                if(cycles > 120){
                  cycles = 120;
                }  
              }
              else{
                cycles = cycles - 1;
                if(cycles < 1){
                  cycles = 1;
                }     
              }
              numbers_cycles();
              glcd.drawstring(15, 7, number);
              break;
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              else{
                glcd.drawstring(0, 2, "->");
                posicion_actual = 2;  
              }
              break;
    }    
  }
  else if(flag_menu == 5){ //running
    switch (posicion_actual){
      case 2: glcd.fillrect(0, 16, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha
              glcd.drawstring(0, 2, "->");
              posicion_actual = 2;
              break;
    }
  }
  else if(flag_menu == 6){ //settings
    switch (posicion_actual){
      case 5: glcd.fillrect(0, 40, 15, 8, 0); //borra flecha 
              if(select_sentido == true){
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              else{
                glcd.drawstring(0, 3, "->");
                posicion_actual = 3;  
              }
              break;
      case 3: glcd.fillrect(0, 24, 15, 8, 0); //borra flecha
              if(select_sentido == true){
                glcd.drawstring(0, 5, "->");
                posicion_actual = 5;  
              }
              else{
                glcd.drawstring(0, 5, "->");
                posicion_actual = 5;  
              }
              break;
    }
  } 
  glcd.display();
}
void menu_inicio(){
  glcd.clear();
  glcd.drawstring(0, 4, "MENU");          
  glcd.drawstring(0, 5, "->");                         
  glcd.drawstring(15, 5, "Home");        
  glcd.drawstring(15, 6, "ROM");
  glcd.drawstring(15, 7, "Speed");
  glcd.drawstring(15, 0, "Cycles");
  glcd.drawstring(15, 1, "Start");
  glcd.drawstring(15, 2, "Reset");
  glcd.drawstring(15, 3, "Config");
  glcd.drawline(55, 0, 55, 63, 1);
  
  glcd.drawstring(57, 4, "Actual val");
  glcd.drawstring(57, 5, "Home:"); 
  numbers_home();
  glcd.drawstring(102, 5, number);
  glcd.drawchar(120, 5, 0xA7);
  glcd.drawstring(57, 6, "Flex:"); 
  numbers();
  glcd.drawstring(102, 6, number);
  glcd.drawchar(120, 6, 0xA7);
  glcd.drawstring(57, 7, "Ext:"); 
  numbers_1();
  glcd.drawstring(102, 7, number);
  glcd.drawchar(120, 7, 0xA7);
  glcd.drawstring(57, 0, "PWM:"); 
  numbers_velocidad();
  glcd.drawstring(102, 0, number);
  glcd.drawchar(120, 0, '%');
  glcd.drawstring(57, 1, "Cycles:"); 
  numbers_cycles();
  glcd.drawstring(102, 1, number);
  
  
  glcd.display();
  posicion_actual = 5;
}

void menu_rom(){
  glcd.clear();
  glcd.drawstring(0, 4, "Range of motion");
  glcd.drawstring(0, 7, "->");
  glcd.drawstring(15, 6, "Max flexion: ");
  numbers();
  glcd.drawstring(15, 7, number);
  glcd.drawchar(36, 7, 0xA7);
  glcd.drawstring(15, 0, "Max extension: ");
  numbers_1();
  glcd.drawstring(15, 1, number);
  glcd.drawchar(36, 1, 0xA7);
  glcd.drawstring(15, 2, "Change");
  glcd.drawstring(15, 3, "OK");
  glcd.display();
  posicion_actual = 7;
  
}

void menu_velocidad(){
  glcd.clear();
  glcd.drawstring(0, 4, "Desired speed");
  glcd.drawstring(0, 7, "->");
  glcd.drawstring(15, 6, "Duty cycle PWM:");
  numbers_velocidad();              
  glcd.drawstring(15, 7, number);
  glcd.drawstring(36, 7, "%");
  glcd.drawstring(15, 2, "Change");
  glcd.drawstring(15, 3, "OK");
  glcd.display();
  posicion_actual = 7;
}

void menu_home(){
  glcd.clear();
  glcd.drawstring(0, 4, "Set home position");
  glcd.drawstring(0, 7, "->");
  glcd.drawstring(15, 6, "Position:");
  numbers_home();              
  glcd.drawstring(15, 7, number);
  glcd.drawchar(36, 7, 0xA7);
  glcd.drawstring(15, 2, "Change");
  glcd.drawstring(15, 3, "OK");
  glcd.display();
  posicion_actual = 7;
}

void menu_cycles(){
  glcd.clear();
  glcd.drawstring(0, 4, "Select number of cycles");
  glcd.drawstring(0, 7, "->");
  glcd.drawstring(15, 6, "Cycles:");
  numbers_cycles();              
  glcd.drawstring(15, 7, number);
  glcd.drawstring(15, 2, "Change");
  glcd.drawstring(15, 3, "OK");
  glcd.display();
  posicion_actual = 7;
}

void menu_start(){
  glcd.clear();
  glcd.drawstring(0, 4, "Running");
  glcd.drawstring(0, 6, "Done:");
  glcd.drawstring(0, 2, "->");
  if(flag_pause == false){
    glcd.drawstring(15, 2, "Pause");  
  }
  else{
    glcd.drawstring(15, 2, "Continue");
  }
  glcd.drawstring(15, 3, "Stop");

  glcd.drawstring(57, 4, "Actual val");
  glcd.drawstring(57, 5, "Home:"); 
  numbers_home();
  glcd.drawstring(102, 5, number);
  glcd.drawchar(120, 5, 0xA7);
  glcd.drawstring(57, 6, "Flex:"); 
  numbers();
  glcd.drawstring(102, 6, number);
  glcd.drawchar(120, 6, 0xA7);
  glcd.drawstring(57, 7, "Ext:"); 
  numbers_1();
  glcd.drawstring(102, 7, number);
  glcd.drawchar(120, 7, 0xA7);
  glcd.drawstring(57, 0, "PWM:"); 
  numbers_velocidad();
  glcd.drawstring(102, 0, number);
  glcd.drawchar(120, 0, '%');
  glcd.drawstring(57, 1, "Cycles:"); 
  numbers_cycles();
  glcd.drawstring(102, 1, number);
  
  glcd.display();
  posicion_actual = 2;
}

void menu_settings(){
  glcd.clear();
  glcd.drawstring(0, 4, "Configuration");
  glcd.drawstring(0, 5, "->");
  glcd.drawstring(15, 5, "Calibrate");
  glcd.drawstring(15, 6, "Control");
  glcd.drawstring(15, 3, "Back");
  glcd.display();
  posicion_actual = 5;
}

void menu_calibrate(){
  glcd.clear();
  glcd.drawstring(0, 4, "Calibrate");
  glcd.drawstring(15, 5, "Calibrate:");
  glcd.drawstring(21, 6, "Flex_data=");
  glcd.drawstring(84, 6, fcalibrationstr);
  glcd.drawstring(21, 7, "Ext_data =");
  glcd.drawstring(84, 7, ecalibrationstr);
  glcd.drawstring(15, 0, "Samples = 200");
  glcd.drawstring(0, 0, "->");
  glcd.drawstring(15, 1, "Save data");
  glcd.drawstring(15, 3, "Back");
  glcd.display();
  posicion_actual = 0;
}

void numbers(){
  switch (flexion){
    case 0: number[0] = '0'; number[1] = '0'; number[2] = '0'; break;
    case 1: number[0] = '0'; number[1] = '0'; number[2] = '1'; break;
    case 2: number[0] = '0'; number[1] = '0'; number[2] = '2'; break;
    case 3: number[0] = '0'; number[1] = '0'; number[2] = '3'; break;
    case 4: number[0] = '0'; number[1] = '0'; number[2] = '4'; break;
    case 5: number[0] = '0'; number[1] = '0'; number[2] = '5'; break;
    case 6: number[0] = '0'; number[1] = '0'; number[2] = '6'; break;
    case 7: number[0] = '0'; number[1] = '0'; number[2] = '7'; break;
    case 8: number[0] = '0'; number[1] = '0'; number[2] = '8'; break;
    case 9: number[0] = '0'; number[1] = '0'; number[2] = '9'; break;
    case 10: number[0] = '0'; number[1] = '1'; number[2] = '0'; break;
    case 11: number[0] = '0'; number[1] = '1'; number[2] = '1'; break;
    case 12: number[0] = '0'; number[1] = '1'; number[2] = '2'; break;
    case 13: number[0] = '0'; number[1] = '1'; number[2] = '3'; break;
    case 14: number[0] = '0'; number[1] = '1'; number[2] = '4'; break;
    case 15: number[0] = '0'; number[1] = '1'; number[2] = '5'; break;
    case 16: number[0] = '0'; number[1] = '1'; number[2] = '6'; break;
    case 17: number[0] = '0'; number[1] = '1'; number[2] = '7'; break;
    case 18: number[0] = '0'; number[1] = '1'; number[2] = '8'; break;
    case 19: number[0] = '0'; number[1] = '1'; number[2] = '9'; break;
    case 20: number[0] = '0'; number[1] = '2'; number[2] = '0'; break;
    case 21: number[0] = '0'; number[1] = '2'; number[2] = '1'; break;
    case 22: number[0] = '0'; number[1] = '2'; number[2] = '2'; break;
    case 23: number[0] = '0'; number[1] = '2'; number[2] = '3'; break;
    case 24: number[0] = '0'; number[1] = '2'; number[2] = '4'; break;
    case 25: number[0] = '0'; number[1] = '2'; number[2] = '5'; break;
    case 26: number[0] = '0'; number[1] = '2'; number[2] = '6'; break;
    case 27: number[0] = '0'; number[1] = '2'; number[2] = '7'; break;
    case 28: number[0] = '0'; number[1] = '2'; number[2] = '8'; break;
    case 29: number[0] = '0'; number[1] = '2'; number[2] = '9'; break;
    case 30: number[0] = '0'; number[1] = '3'; number[2] = '0'; break;
    case 31: number[0] = '0'; number[1] = '3'; number[2] = '1'; break;
    case 32: number[0] = '0'; number[1] = '3'; number[2] = '2'; break;
    case 33: number[0] = '0'; number[1] = '3'; number[2] = '3'; break;
    case 34: number[0] = '0'; number[1] = '3'; number[2] = '4'; break;
    case 35: number[0] = '0'; number[1] = '3'; number[2] = '5'; break;
    case 36: number[0] = '0'; number[1] = '3'; number[2] = '6'; break;
    case 37: number[0] = '0'; number[1] = '3'; number[2] = '7'; break;
    case 38: number[0] = '0'; number[1] = '3'; number[2] = '8'; break;
    case 39: number[0] = '0'; number[1] = '3'; number[2] = '9'; break;
    case 40: number[0] = '0'; number[1] = '4'; number[2] = '0'; break;
    case 41: number[0] = '0'; number[1] = '4'; number[2] = '1'; break;
    case 42: number[0] = '0'; number[1] = '4'; number[2] = '2'; break;
    case 43: number[0] = '0'; number[1] = '4'; number[2] = '3'; break;
    case 44: number[0] = '0'; number[1] = '4'; number[2] = '4'; break;
    case 45: number[0] = '0'; number[1] = '4'; number[2] = '5'; break;
    case 46: number[0] = '0'; number[1] = '4'; number[2] = '6'; break;
    case 47: number[0] = '0'; number[1] = '4'; number[2] = '7'; break;
    case 48: number[0] = '0'; number[1] = '4'; number[2] = '8'; break;
    case 49: number[0] = '0'; number[1] = '4'; number[2] = '9'; break;
    case 50: number[0] = '0'; number[1] = '5'; number[2] = '0'; break;
    case 51: number[0] = '0'; number[1] = '5'; number[2] = '1'; break;
    case 52: number[0] = '0'; number[1] = '5'; number[2] = '2'; break;
    case 53: number[0] = '0'; number[1] = '5'; number[2] = '3'; break;
    case 54: number[0] = '0'; number[1] = '5'; number[2] = '4'; break;
    case 55: number[0] = '0'; number[1] = '5'; number[2] = '5'; break;
    case 56: number[0] = '0'; number[1] = '5'; number[2] = '6'; break;
    case 57: number[0] = '0'; number[1] = '5'; number[2] = '7'; break;
    case 58: number[0] = '0'; number[1] = '5'; number[2] = '8'; break;
    case 59: number[0] = '0'; number[1] = '5'; number[2] = '9'; break;
    case 60: number[0] = '0'; number[1] = '6'; number[2] = '0'; break;
    case 61: number[0] = '0'; number[1] = '6'; number[2] = '1'; break;
    case 62: number[0] = '0'; number[1] = '6'; number[2] = '2'; break;
    case 63: number[0] = '0'; number[1] = '6'; number[2] = '3'; break;
    case 64: number[0] = '0'; number[1] = '6'; number[2] = '4'; break;
    case 65: number[0] = '0'; number[1] = '6'; number[2] = '5'; break;
    case 66: number[0] = '0'; number[1] = '6'; number[2] = '6'; break;
    case 67: number[0] = '0'; number[1] = '6'; number[2] = '7'; break;
    case 68: number[0] = '0'; number[1] = '6'; number[2] = '8'; break;
    case 69: number[0] = '0'; number[1] = '6'; number[2] = '9'; break;
    case 70: number[0] = '0'; number[1] = '7'; number[2] = '0'; break;
    case 71: number[0] = '0'; number[1] = '7'; number[2] = '1'; break;
    case 72: number[0] = '0'; number[1] = '7'; number[2] = '2'; break;
    case 73: number[0] = '0'; number[1] = '7'; number[2] = '3'; break;
    case 74: number[0] = '0'; number[1] = '7'; number[2] = '4'; break;
    case 75: number[0] = '0'; number[1] = '7'; number[2] = '5'; break;
    case 76: number[0] = '0'; number[1] = '7'; number[2] = '6'; break;
    case 77: number[0] = '0'; number[1] = '7'; number[2] = '7'; break;
    case 78: number[0] = '0'; number[1] = '7'; number[2] = '8'; break;
    case 79: number[0] = '0'; number[1] = '7'; number[2] = '9'; break;
    case 80: number[0] = '0'; number[1] = '8'; number[2] = '0'; break;
    case 81: number[0] = '0'; number[1] = '8'; number[2] = '1'; break;
    case 82: number[0] = '0'; number[1] = '8'; number[2] = '2'; break;
    case 83: number[0] = '0'; number[1] = '8'; number[2] = '3'; break;
    case 84: number[0] = '0'; number[1] = '8'; number[2] = '4'; break;
    case 85: number[0] = '0'; number[1] = '8'; number[2] = '5'; break;
    case 86: number[0] = '0'; number[1] = '8'; number[2] = '6'; break;
    case 87: number[0] = '0'; number[1] = '8'; number[2] = '7'; break;
    case 88: number[0] = '0'; number[1] = '8'; number[2] = '8'; break;
    case 89: number[0] = '0'; number[1] = '8'; number[2] = '9'; break;
    case 90: number[0] = '0'; number[1] = '9'; number[2] = '0'; break;
    case 91: number[0] = '0'; number[1] = '9'; number[2] = '1'; break;
    case 92: number[0] = '0'; number[1] = '9'; number[2] = '2'; break;
    case 93: number[0] = '0'; number[1] = '9'; number[2] = '3'; break;
    case 94: number[0] = '0'; number[1] = '9'; number[2] = '4'; break;
    case 95: number[0] = '0'; number[1] = '9'; number[2] = '5'; break;
    case 96: number[0] = '0'; number[1] = '9'; number[2] = '6'; break;
    case 97: number[0] = '0'; number[1] = '9'; number[2] = '7'; break;
    case 98: number[0] = '0'; number[1] = '9'; number[2] = '8'; break;
    case 99: number[0] = '0'; number[1] = '9'; number[2] = '9'; break;
    case 100: number[0] = '1'; number[1] = '0'; number[2] = '0'; break;
    case 101: number[0] = '1'; number[1] = '0'; number[2] = '1'; break;
    case 102: number[0] = '1'; number[1] = '0'; number[2] = '2'; break;
    case 103: number[0] = '1'; number[1] = '0'; number[2] = '3'; break;
    case 104: number[0] = '1'; number[1] = '0'; number[2] = '4'; break;
    case 105: number[0] = '1'; number[1] = '0'; number[2] = '5'; break;
    case 106: number[0] = '1'; number[1] = '0'; number[2] = '6'; break;
    case 107: number[0] = '1'; number[1] = '0'; number[2] = '7'; break;
    case 108: number[0] = '1'; number[1] = '0'; number[2] = '8'; break;
    case 109: number[0] = '1'; number[1] = '0'; number[2] = '9'; break;
    case 110: number[0] = '1'; number[1] = '1'; number[2] = '0'; break;
    case 111: number[0] = '1'; number[1] = '1'; number[2] = '1'; break;
    case 112: number[0] = '1'; number[1] = '1'; number[2] = '2'; break;
    case 113: number[0] = '1'; number[1] = '1'; number[2] = '3'; break;
    case 114: number[0] = '1'; number[1] = '1'; number[2] = '4'; break;
    case 115: number[0] = '1'; number[1] = '1'; number[2] = '5'; break;
    case 116: number[0] = '1'; number[1] = '1'; number[2] = '6'; break;
    case 117: number[0] = '1'; number[1] = '1'; number[2] = '7'; break;
    case 118: number[0] = '1'; number[1] = '1'; number[2] = '8'; break;
    case 119: number[0] = '1'; number[1] = '1'; number[2] = '9'; break;
    case 120: number[0] = '1'; number[1] = '2'; number[2] = '0'; break;
  } 
}

void numbers_1(){
  switch (extension){
    case 0: number[0] = '0'; number[1] = '0'; number[2] = '0'; break;
    case 1: number[0] = '0'; number[1] = '0'; number[2] = '1'; break;
    case 2: number[0] = '0'; number[1] = '0'; number[2] = '2'; break;
    case 3: number[0] = '0'; number[1] = '0'; number[2] = '3'; break;
    case 4: number[0] = '0'; number[1] = '0'; number[2] = '4'; break;
    case 5: number[0] = '0'; number[1] = '0'; number[2] = '5'; break;
    case 6: number[0] = '0'; number[1] = '0'; number[2] = '6'; break;
    case 7: number[0] = '0'; number[1] = '0'; number[2] = '7'; break;
    case 8: number[0] = '0'; number[1] = '0'; number[2] = '8'; break;
    case 9: number[0] = '0'; number[1] = '0'; number[2] = '9'; break;
    case 10: number[0] = '0'; number[1] = '1'; number[2] = '0'; break;
    case 11: number[0] = '0'; number[1] = '1'; number[2] = '1'; break;
    case 12: number[0] = '0'; number[1] = '1'; number[2] = '2'; break;
    case 13: number[0] = '0'; number[1] = '1'; number[2] = '3'; break;
    case 14: number[0] = '0'; number[1] = '1'; number[2] = '4'; break;
    case 15: number[0] = '0'; number[1] = '1'; number[2] = '5'; break;
    case 16: number[0] = '0'; number[1] = '1'; number[2] = '6'; break;
    case 17: number[0] = '0'; number[1] = '1'; number[2] = '7'; break;
    case 18: number[0] = '0'; number[1] = '1'; number[2] = '8'; break;
    case 19: number[0] = '0'; number[1] = '1'; number[2] = '9'; break;
    case 20: number[0] = '0'; number[1] = '2'; number[2] = '0'; break;
    case 21: number[0] = '0'; number[1] = '2'; number[2] = '1'; break;
    case 22: number[0] = '0'; number[1] = '2'; number[2] = '2'; break;
    case 23: number[0] = '0'; number[1] = '2'; number[2] = '3'; break;
    case 24: number[0] = '0'; number[1] = '2'; number[2] = '4'; break;
    case 25: number[0] = '0'; number[1] = '2'; number[2] = '5'; break;
    case 26: number[0] = '0'; number[1] = '2'; number[2] = '6'; break;
    case 27: number[0] = '0'; number[1] = '2'; number[2] = '7'; break;
    case 28: number[0] = '0'; number[1] = '2'; number[2] = '8'; break;
    case 29: number[0] = '0'; number[1] = '2'; number[2] = '9'; break;
    case 30: number[0] = '0'; number[1] = '3'; number[2] = '0'; break;
    case 31: number[0] = '0'; number[1] = '3'; number[2] = '1'; break;
    case 32: number[0] = '0'; number[1] = '3'; number[2] = '2'; break;
    case 33: number[0] = '0'; number[1] = '3'; number[2] = '3'; break;
    case 34: number[0] = '0'; number[1] = '3'; number[2] = '4'; break;
    case 35: number[0] = '0'; number[1] = '3'; number[2] = '5'; break;
    case 36: number[0] = '0'; number[1] = '3'; number[2] = '6'; break;
    case 37: number[0] = '0'; number[1] = '3'; number[2] = '7'; break;
    case 38: number[0] = '0'; number[1] = '3'; number[2] = '8'; break;
    case 39: number[0] = '0'; number[1] = '3'; number[2] = '9'; break;
    case 40: number[0] = '0'; number[1] = '4'; number[2] = '0'; break;
    case 41: number[0] = '0'; number[1] = '4'; number[2] = '1'; break;
    case 42: number[0] = '0'; number[1] = '4'; number[2] = '2'; break;
    case 43: number[0] = '0'; number[1] = '4'; number[2] = '3'; break;
    case 44: number[0] = '0'; number[1] = '4'; number[2] = '4'; break;
    case 45: number[0] = '0'; number[1] = '4'; number[2] = '5'; break;
    case 46: number[0] = '0'; number[1] = '4'; number[2] = '6'; break;
    case 47: number[0] = '0'; number[1] = '4'; number[2] = '7'; break;
    case 48: number[0] = '0'; number[1] = '4'; number[2] = '8'; break;
    case 49: number[0] = '0'; number[1] = '4'; number[2] = '9'; break;
    case 50: number[0] = '0'; number[1] = '5'; number[2] = '0'; break;
    case 51: number[0] = '0'; number[1] = '5'; number[2] = '1'; break;
    case 52: number[0] = '0'; number[1] = '5'; number[2] = '2'; break;
    case 53: number[0] = '0'; number[1] = '5'; number[2] = '3'; break;
    case 54: number[0] = '0'; number[1] = '5'; number[2] = '4'; break;
    case 55: number[0] = '0'; number[1] = '5'; number[2] = '5'; break;
    case 56: number[0] = '0'; number[1] = '5'; number[2] = '6'; break;
    case 57: number[0] = '0'; number[1] = '5'; number[2] = '7'; break;
    case 58: number[0] = '0'; number[1] = '5'; number[2] = '8'; break;
    case 59: number[0] = '0'; number[1] = '5'; number[2] = '9'; break;
    case 60: number[0] = '0'; number[1] = '6'; number[2] = '0'; break;
    case 61: number[0] = '0'; number[1] = '6'; number[2] = '1'; break;
    case 62: number[0] = '0'; number[1] = '6'; number[2] = '2'; break;
    case 63: number[0] = '0'; number[1] = '6'; number[2] = '3'; break;
    case 64: number[0] = '0'; number[1] = '6'; number[2] = '4'; break;
    case 65: number[0] = '0'; number[1] = '6'; number[2] = '5'; break;
    case 66: number[0] = '0'; number[1] = '6'; number[2] = '6'; break;
    case 67: number[0] = '0'; number[1] = '6'; number[2] = '7'; break;
    case 68: number[0] = '0'; number[1] = '6'; number[2] = '8'; break;
    case 69: number[0] = '0'; number[1] = '6'; number[2] = '9'; break;
    case 70: number[0] = '0'; number[1] = '7'; number[2] = '0'; break;
    case 71: number[0] = '0'; number[1] = '7'; number[2] = '1'; break;
    case 72: number[0] = '0'; number[1] = '7'; number[2] = '2'; break;
    case 73: number[0] = '0'; number[1] = '7'; number[2] = '3'; break;
    case 74: number[0] = '0'; number[1] = '7'; number[2] = '4'; break;
    case 75: number[0] = '0'; number[1] = '7'; number[2] = '5'; break;
    case 76: number[0] = '0'; number[1] = '7'; number[2] = '6'; break;
    case 77: number[0] = '0'; number[1] = '7'; number[2] = '7'; break;
    case 78: number[0] = '0'; number[1] = '7'; number[2] = '8'; break;
    case 79: number[0] = '0'; number[1] = '7'; number[2] = '9'; break;
    case 80: number[0] = '0'; number[1] = '8'; number[2] = '0'; break;
    case 81: number[0] = '0'; number[1] = '8'; number[2] = '1'; break;
    case 82: number[0] = '0'; number[1] = '8'; number[2] = '2'; break;
    case 83: number[0] = '0'; number[1] = '8'; number[2] = '3'; break;
    case 84: number[0] = '0'; number[1] = '8'; number[2] = '4'; break;
    case 85: number[0] = '0'; number[1] = '8'; number[2] = '5'; break;
    case 86: number[0] = '0'; number[1] = '8'; number[2] = '6'; break;
    case 87: number[0] = '0'; number[1] = '8'; number[2] = '7'; break;
    case 88: number[0] = '0'; number[1] = '8'; number[2] = '8'; break;
    case 89: number[0] = '0'; number[1] = '8'; number[2] = '9'; break;
    case 90: number[0] = '0'; number[1] = '9'; number[2] = '0'; break;
  } 
}

void numbers_velocidad(){
  switch (velocidad){
    case 0: number[0] = '0'; number[1] = '0'; number[2] = '0'; break;
    case 1: number[0] = '0'; number[1] = '0'; number[2] = '1'; break;
    case 2: number[0] = '0'; number[1] = '0'; number[2] = '2'; break;
    case 3: number[0] = '0'; number[1] = '0'; number[2] = '3'; break;
    case 4: number[0] = '0'; number[1] = '0'; number[2] = '4'; break;
    case 5: number[0] = '0'; number[1] = '0'; number[2] = '5'; break;
    case 6: number[0] = '0'; number[1] = '0'; number[2] = '6'; break;
    case 7: number[0] = '0'; number[1] = '0'; number[2] = '7'; break;
    case 8: number[0] = '0'; number[1] = '0'; number[2] = '8'; break;
    case 9: number[0] = '0'; number[1] = '0'; number[2] = '9'; break;
    case 10: number[0] = '0'; number[1] = '1'; number[2] = '0'; break;
    case 11: number[0] = '0'; number[1] = '1'; number[2] = '1'; break;
    case 12: number[0] = '0'; number[1] = '1'; number[2] = '2'; break;
    case 13: number[0] = '0'; number[1] = '1'; number[2] = '3'; break;
    case 14: number[0] = '0'; number[1] = '1'; number[2] = '4'; break;
    case 15: number[0] = '0'; number[1] = '1'; number[2] = '5'; break;
    case 16: number[0] = '0'; number[1] = '1'; number[2] = '6'; break;
    case 17: number[0] = '0'; number[1] = '1'; number[2] = '7'; break;
    case 18: number[0] = '0'; number[1] = '1'; number[2] = '8'; break;
    case 19: number[0] = '0'; number[1] = '1'; number[2] = '9'; break;
    case 20: number[0] = '0'; number[1] = '2'; number[2] = '0'; break;
    case 21: number[0] = '0'; number[1] = '2'; number[2] = '1'; break;
    case 22: number[0] = '0'; number[1] = '2'; number[2] = '2'; break;
    case 23: number[0] = '0'; number[1] = '2'; number[2] = '3'; break;
    case 24: number[0] = '0'; number[1] = '2'; number[2] = '4'; break;
    case 25: number[0] = '0'; number[1] = '2'; number[2] = '5'; break;
    case 26: number[0] = '0'; number[1] = '2'; number[2] = '6'; break;
    case 27: number[0] = '0'; number[1] = '2'; number[2] = '7'; break;
    case 28: number[0] = '0'; number[1] = '2'; number[2] = '8'; break;
    case 29: number[0] = '0'; number[1] = '2'; number[2] = '9'; break;
    case 30: number[0] = '0'; number[1] = '3'; number[2] = '0'; break;
    case 31: number[0] = '0'; number[1] = '3'; number[2] = '1'; break;
    case 32: number[0] = '0'; number[1] = '3'; number[2] = '2'; break;
    case 33: number[0] = '0'; number[1] = '3'; number[2] = '3'; break;
    case 34: number[0] = '0'; number[1] = '3'; number[2] = '4'; break;
    case 35: number[0] = '0'; number[1] = '3'; number[2] = '5'; break;
    case 36: number[0] = '0'; number[1] = '3'; number[2] = '6'; break;
    case 37: number[0] = '0'; number[1] = '3'; number[2] = '7'; break;
    case 38: number[0] = '0'; number[1] = '3'; number[2] = '8'; break;
    case 39: number[0] = '0'; number[1] = '3'; number[2] = '9'; break;
    case 40: number[0] = '0'; number[1] = '4'; number[2] = '0'; break;
    case 41: number[0] = '0'; number[1] = '4'; number[2] = '1'; break;
    case 42: number[0] = '0'; number[1] = '4'; number[2] = '2'; break;
    case 43: number[0] = '0'; number[1] = '4'; number[2] = '3'; break;
    case 44: number[0] = '0'; number[1] = '4'; number[2] = '4'; break;
    case 45: number[0] = '0'; number[1] = '4'; number[2] = '5'; break;
    case 46: number[0] = '0'; number[1] = '4'; number[2] = '6'; break;
    case 47: number[0] = '0'; number[1] = '4'; number[2] = '7'; break;
    case 48: number[0] = '0'; number[1] = '4'; number[2] = '8'; break;
    case 49: number[0] = '0'; number[1] = '4'; number[2] = '9'; break;
    case 50: number[0] = '0'; number[1] = '5'; number[2] = '0'; break;
    case 51: number[0] = '0'; number[1] = '5'; number[2] = '1'; break;
    case 52: number[0] = '0'; number[1] = '5'; number[2] = '2'; break;
    case 53: number[0] = '0'; number[1] = '5'; number[2] = '3'; break;
    case 54: number[0] = '0'; number[1] = '5'; number[2] = '4'; break;
    case 55: number[0] = '0'; number[1] = '5'; number[2] = '5'; break;
    case 56: number[0] = '0'; number[1] = '5'; number[2] = '6'; break;
    case 57: number[0] = '0'; number[1] = '5'; number[2] = '7'; break;
    case 58: number[0] = '0'; number[1] = '5'; number[2] = '8'; break;
    case 59: number[0] = '0'; number[1] = '5'; number[2] = '9'; break;
    case 60: number[0] = '0'; number[1] = '6'; number[2] = '0'; break;
    case 61: number[0] = '0'; number[1] = '6'; number[2] = '1'; break;
    case 62: number[0] = '0'; number[1] = '6'; number[2] = '2'; break;
    case 63: number[0] = '0'; number[1] = '6'; number[2] = '3'; break;
    case 64: number[0] = '0'; number[1] = '6'; number[2] = '4'; break;
    case 65: number[0] = '0'; number[1] = '6'; number[2] = '5'; break;
    case 66: number[0] = '0'; number[1] = '6'; number[2] = '6'; break;
    case 67: number[0] = '0'; number[1] = '6'; number[2] = '7'; break;
    case 68: number[0] = '0'; number[1] = '6'; number[2] = '8'; break;
    case 69: number[0] = '0'; number[1] = '6'; number[2] = '9'; break;
    case 70: number[0] = '0'; number[1] = '7'; number[2] = '0'; break;
    case 71: number[0] = '0'; number[1] = '7'; number[2] = '1'; break;
    case 72: number[0] = '0'; number[1] = '7'; number[2] = '2'; break;
    case 73: number[0] = '0'; number[1] = '7'; number[2] = '3'; break;
    case 74: number[0] = '0'; number[1] = '7'; number[2] = '4'; break;
    case 75: number[0] = '0'; number[1] = '7'; number[2] = '5'; break;
    case 76: number[0] = '0'; number[1] = '7'; number[2] = '6'; break;
    case 77: number[0] = '0'; number[1] = '7'; number[2] = '7'; break;
    case 78: number[0] = '0'; number[1] = '7'; number[2] = '8'; break;
    case 79: number[0] = '0'; number[1] = '7'; number[2] = '9'; break;
    case 80: number[0] = '0'; number[1] = '8'; number[2] = '0'; break;
    case 81: number[0] = '0'; number[1] = '8'; number[2] = '1'; break;
    case 82: number[0] = '0'; number[1] = '8'; number[2] = '2'; break;
    case 83: number[0] = '0'; number[1] = '8'; number[2] = '3'; break;
    case 84: number[0] = '0'; number[1] = '8'; number[2] = '4'; break;
    case 85: number[0] = '0'; number[1] = '8'; number[2] = '5'; break;
    case 86: number[0] = '0'; number[1] = '8'; number[2] = '6'; break;
    case 87: number[0] = '0'; number[1] = '8'; number[2] = '7'; break;
    case 88: number[0] = '0'; number[1] = '8'; number[2] = '8'; break;
    case 89: number[0] = '0'; number[1] = '8'; number[2] = '9'; break;
    case 90: number[0] = '0'; number[1] = '9'; number[2] = '0'; break;
    case 91: number[0] = '0'; number[1] = '9'; number[2] = '1'; break;
    case 92: number[0] = '0'; number[1] = '9'; number[2] = '2'; break;
    case 93: number[0] = '0'; number[1] = '9'; number[2] = '3'; break;
    case 94: number[0] = '0'; number[1] = '9'; number[2] = '4'; break;
    case 95: number[0] = '0'; number[1] = '9'; number[2] = '5'; break;
    case 96: number[0] = '0'; number[1] = '9'; number[2] = '6'; break;
    case 97: number[0] = '0'; number[1] = '9'; number[2] = '7'; break;
    case 98: number[0] = '0'; number[1] = '9'; number[2] = '8'; break;
    case 99: number[0] = '0'; number[1] = '9'; number[2] = '9'; break;
    case 100: number[0] = '1'; number[1] = '0'; number[2] = '0'; break;
  } 
}

void numbers_home(){
  switch (home_position){
    case 0: number[0] = '0'; number[1] = '0'; number[2] = '0'; break;
    case 1: number[0] = '0'; number[1] = '0'; number[2] = '1'; break;
    case 2: number[0] = '0'; number[1] = '0'; number[2] = '2'; break;
    case 3: number[0] = '0'; number[1] = '0'; number[2] = '3'; break;
    case 4: number[0] = '0'; number[1] = '0'; number[2] = '4'; break;
    case 5: number[0] = '0'; number[1] = '0'; number[2] = '5'; break;
    case 6: number[0] = '0'; number[1] = '0'; number[2] = '6'; break;
    case 7: number[0] = '0'; number[1] = '0'; number[2] = '7'; break;
    case 8: number[0] = '0'; number[1] = '0'; number[2] = '8'; break;
    case 9: number[0] = '0'; number[1] = '0'; number[2] = '9'; break;
    case 10: number[0] = '0'; number[1] = '1'; number[2] = '0'; break;
    case 11: number[0] = '0'; number[1] = '1'; number[2] = '1'; break;
    case 12: number[0] = '0'; number[1] = '1'; number[2] = '2'; break;
    case 13: number[0] = '0'; number[1] = '1'; number[2] = '3'; break;
    case 14: number[0] = '0'; number[1] = '1'; number[2] = '4'; break;
    case 15: number[0] = '0'; number[1] = '1'; number[2] = '5'; break;
    case 16: number[0] = '0'; number[1] = '1'; number[2] = '6'; break;
    case 17: number[0] = '0'; number[1] = '1'; number[2] = '7'; break;
    case 18: number[0] = '0'; number[1] = '1'; number[2] = '8'; break;
    case 19: number[0] = '0'; number[1] = '1'; number[2] = '9'; break;
    case 20: number[0] = '0'; number[1] = '2'; number[2] = '0'; break;
    case 21: number[0] = '0'; number[1] = '2'; number[2] = '1'; break;
    case 22: number[0] = '0'; number[1] = '2'; number[2] = '2'; break;
    case 23: number[0] = '0'; number[1] = '2'; number[2] = '3'; break;
    case 24: number[0] = '0'; number[1] = '2'; number[2] = '4'; break;
    case 25: number[0] = '0'; number[1] = '2'; number[2] = '5'; break;
    case 26: number[0] = '0'; number[1] = '2'; number[2] = '6'; break;
    case 27: number[0] = '0'; number[1] = '2'; number[2] = '7'; break;
    case 28: number[0] = '0'; number[1] = '2'; number[2] = '8'; break;
    case 29: number[0] = '0'; number[1] = '2'; number[2] = '9'; break;
    case 30: number[0] = '0'; number[1] = '3'; number[2] = '0'; break;
    case 31: number[0] = '0'; number[1] = '3'; number[2] = '1'; break;
    case 32: number[0] = '0'; number[1] = '3'; number[2] = '2'; break;
    case 33: number[0] = '0'; number[1] = '3'; number[2] = '3'; break;
    case 34: number[0] = '0'; number[1] = '3'; number[2] = '4'; break;
    case 35: number[0] = '0'; number[1] = '3'; number[2] = '5'; break;
    case 36: number[0] = '0'; number[1] = '3'; number[2] = '6'; break;
    case 37: number[0] = '0'; number[1] = '3'; number[2] = '7'; break;
    case 38: number[0] = '0'; number[1] = '3'; number[2] = '8'; break;
    case 39: number[0] = '0'; number[1] = '3'; number[2] = '9'; break;
    case 40: number[0] = '0'; number[1] = '4'; number[2] = '0'; break;
    case 41: number[0] = '0'; number[1] = '4'; number[2] = '1'; break;
    case 42: number[0] = '0'; number[1] = '4'; number[2] = '2'; break;
    case 43: number[0] = '0'; number[1] = '4'; number[2] = '3'; break;
    case 44: number[0] = '0'; number[1] = '4'; number[2] = '4'; break;
    case 45: number[0] = '0'; number[1] = '4'; number[2] = '5'; break;
    case 46: number[0] = '0'; number[1] = '4'; number[2] = '6'; break;
    case 47: number[0] = '0'; number[1] = '4'; number[2] = '7'; break;
    case 48: number[0] = '0'; number[1] = '4'; number[2] = '8'; break;
    case 49: number[0] = '0'; number[1] = '4'; number[2] = '9'; break;
    case 50: number[0] = '0'; number[1] = '5'; number[2] = '0'; break;
    case 51: number[0] = '0'; number[1] = '5'; number[2] = '1'; break;
    case 52: number[0] = '0'; number[1] = '5'; number[2] = '2'; break;
    case 53: number[0] = '0'; number[1] = '5'; number[2] = '3'; break;
    case 54: number[0] = '0'; number[1] = '5'; number[2] = '4'; break;
    case 55: number[0] = '0'; number[1] = '5'; number[2] = '5'; break;
    case 56: number[0] = '0'; number[1] = '5'; number[2] = '6'; break;
    case 57: number[0] = '0'; number[1] = '5'; number[2] = '7'; break;
    case 58: number[0] = '0'; number[1] = '5'; number[2] = '8'; break;
    case 59: number[0] = '0'; number[1] = '5'; number[2] = '9'; break;
    case 60: number[0] = '0'; number[1] = '6'; number[2] = '0'; break;
    case 61: number[0] = '0'; number[1] = '6'; number[2] = '1'; break;
    case 62: number[0] = '0'; number[1] = '6'; number[2] = '2'; break;
    case 63: number[0] = '0'; number[1] = '6'; number[2] = '3'; break;
    case 64: number[0] = '0'; number[1] = '6'; number[2] = '4'; break;
    case 65: number[0] = '0'; number[1] = '6'; number[2] = '5'; break;
    case 66: number[0] = '0'; number[1] = '6'; number[2] = '6'; break;
    case 67: number[0] = '0'; number[1] = '6'; number[2] = '7'; break;
    case 68: number[0] = '0'; number[1] = '6'; number[2] = '8'; break;
    case 69: number[0] = '0'; number[1] = '6'; number[2] = '9'; break;
    case 70: number[0] = '0'; number[1] = '7'; number[2] = '0'; break;
    case 71: number[0] = '0'; number[1] = '7'; number[2] = '1'; break;
    case 72: number[0] = '0'; number[1] = '7'; number[2] = '2'; break;
    case 73: number[0] = '0'; number[1] = '7'; number[2] = '3'; break;
    case 74: number[0] = '0'; number[1] = '7'; number[2] = '4'; break;
    case 75: number[0] = '0'; number[1] = '7'; number[2] = '5'; break;
    case 76: number[0] = '0'; number[1] = '7'; number[2] = '6'; break;
    case 77: number[0] = '0'; number[1] = '7'; number[2] = '7'; break;
    case 78: number[0] = '0'; number[1] = '7'; number[2] = '8'; break;
    case 79: number[0] = '0'; number[1] = '7'; number[2] = '9'; break;
    case 80: number[0] = '0'; number[1] = '8'; number[2] = '0'; break;
    case 81: number[0] = '0'; number[1] = '8'; number[2] = '1'; break;
    case 82: number[0] = '0'; number[1] = '8'; number[2] = '2'; break;
    case 83: number[0] = '0'; number[1] = '8'; number[2] = '3'; break;
    case 84: number[0] = '0'; number[1] = '8'; number[2] = '4'; break;
    case 85: number[0] = '0'; number[1] = '8'; number[2] = '5'; break;
    case 86: number[0] = '0'; number[1] = '8'; number[2] = '6'; break;
    case 87: number[0] = '0'; number[1] = '8'; number[2] = '7'; break;
    case 88: number[0] = '0'; number[1] = '8'; number[2] = '8'; break;
    case 89: number[0] = '0'; number[1] = '8'; number[2] = '9'; break;
    case 90: number[0] = '0'; number[1] = '9'; number[2] = '0'; break;
    case 91: number[0] = '0'; number[1] = '9'; number[2] = '1'; break;
    case 92: number[0] = '0'; number[1] = '9'; number[2] = '2'; break;
    case 93: number[0] = '0'; number[1] = '9'; number[2] = '3'; break;
    case 94: number[0] = '0'; number[1] = '9'; number[2] = '4'; break;
    case 95: number[0] = '0'; number[1] = '9'; number[2] = '5'; break;
    case 96: number[0] = '0'; number[1] = '9'; number[2] = '6'; break;
    case 97: number[0] = '0'; number[1] = '9'; number[2] = '7'; break;
    case 98: number[0] = '0'; number[1] = '9'; number[2] = '8'; break;
    case 99: number[0] = '0'; number[1] = '9'; number[2] = '9'; break;
    case 100: number[0] = '1'; number[1] = '0'; number[2] = '0'; break;
    case 101: number[0] = '1'; number[1] = '0'; number[2] = '1'; break;
    case 102: number[0] = '1'; number[1] = '0'; number[2] = '2'; break;
    case 103: number[0] = '1'; number[1] = '0'; number[2] = '3'; break;
    case 104: number[0] = '1'; number[1] = '0'; number[2] = '4'; break;
    case 105: number[0] = '1'; number[1] = '0'; number[2] = '5'; break;
    case 106: number[0] = '1'; number[1] = '0'; number[2] = '6'; break;
    case 107: number[0] = '1'; number[1] = '0'; number[2] = '7'; break;
    case 108: number[0] = '1'; number[1] = '0'; number[2] = '8'; break;
    case 109: number[0] = '1'; number[1] = '0'; number[2] = '9'; break;
    case 110: number[0] = '1'; number[1] = '1'; number[2] = '0'; break;
    case 111: number[0] = '1'; number[1] = '1'; number[2] = '1'; break;
    case 112: number[0] = '1'; number[1] = '1'; number[2] = '2'; break;
    case 113: number[0] = '1'; number[1] = '1'; number[2] = '3'; break;
    case 114: number[0] = '1'; number[1] = '1'; number[2] = '4'; break;
    case 115: number[0] = '1'; number[1] = '1'; number[2] = '5'; break;
    case 116: number[0] = '1'; number[1] = '1'; number[2] = '6'; break;
    case 117: number[0] = '1'; number[1] = '1'; number[2] = '7'; break;
    case 118: number[0] = '1'; number[1] = '1'; number[2] = '8'; break;
    case 119: number[0] = '1'; number[1] = '1'; number[2] = '9'; break;
    case 120: number[0] = '1'; number[1] = '2'; number[2] = '0'; break;
  } 
}

void numbers_cycles(){
  switch (cycles){
    case 0: number[0] = '0'; number[1] = '0'; number[2] = '0'; break;
    case 1: number[0] = '0'; number[1] = '0'; number[2] = '1'; break;
    case 2: number[0] = '0'; number[1] = '0'; number[2] = '2'; break;
    case 3: number[0] = '0'; number[1] = '0'; number[2] = '3'; break;
    case 4: number[0] = '0'; number[1] = '0'; number[2] = '4'; break;
    case 5: number[0] = '0'; number[1] = '0'; number[2] = '5'; break;
    case 6: number[0] = '0'; number[1] = '0'; number[2] = '6'; break;
    case 7: number[0] = '0'; number[1] = '0'; number[2] = '7'; break;
    case 8: number[0] = '0'; number[1] = '0'; number[2] = '8'; break;
    case 9: number[0] = '0'; number[1] = '0'; number[2] = '9'; break;
    case 10: number[0] = '0'; number[1] = '1'; number[2] = '0'; break;
    case 11: number[0] = '0'; number[1] = '1'; number[2] = '1'; break;
    case 12: number[0] = '0'; number[1] = '1'; number[2] = '2'; break;
    case 13: number[0] = '0'; number[1] = '1'; number[2] = '3'; break;
    case 14: number[0] = '0'; number[1] = '1'; number[2] = '4'; break;
    case 15: number[0] = '0'; number[1] = '1'; number[2] = '5'; break;
    case 16: number[0] = '0'; number[1] = '1'; number[2] = '6'; break;
    case 17: number[0] = '0'; number[1] = '1'; number[2] = '7'; break;
    case 18: number[0] = '0'; number[1] = '1'; number[2] = '8'; break;
    case 19: number[0] = '0'; number[1] = '1'; number[2] = '9'; break;
    case 20: number[0] = '0'; number[1] = '2'; number[2] = '0'; break;
    case 21: number[0] = '0'; number[1] = '2'; number[2] = '1'; break;
    case 22: number[0] = '0'; number[1] = '2'; number[2] = '2'; break;
    case 23: number[0] = '0'; number[1] = '2'; number[2] = '3'; break;
    case 24: number[0] = '0'; number[1] = '2'; number[2] = '4'; break;
    case 25: number[0] = '0'; number[1] = '2'; number[2] = '5'; break;
    case 26: number[0] = '0'; number[1] = '2'; number[2] = '6'; break;
    case 27: number[0] = '0'; number[1] = '2'; number[2] = '7'; break;
    case 28: number[0] = '0'; number[1] = '2'; number[2] = '8'; break;
    case 29: number[0] = '0'; number[1] = '2'; number[2] = '9'; break;
    case 30: number[0] = '0'; number[1] = '3'; number[2] = '0'; break;
    case 31: number[0] = '0'; number[1] = '3'; number[2] = '1'; break;
    case 32: number[0] = '0'; number[1] = '3'; number[2] = '2'; break;
    case 33: number[0] = '0'; number[1] = '3'; number[2] = '3'; break;
    case 34: number[0] = '0'; number[1] = '3'; number[2] = '4'; break;
    case 35: number[0] = '0'; number[1] = '3'; number[2] = '5'; break;
    case 36: number[0] = '0'; number[1] = '3'; number[2] = '6'; break;
    case 37: number[0] = '0'; number[1] = '3'; number[2] = '7'; break;
    case 38: number[0] = '0'; number[1] = '3'; number[2] = '8'; break;
    case 39: number[0] = '0'; number[1] = '3'; number[2] = '9'; break;
    case 40: number[0] = '0'; number[1] = '4'; number[2] = '0'; break;
    case 41: number[0] = '0'; number[1] = '4'; number[2] = '1'; break;
    case 42: number[0] = '0'; number[1] = '4'; number[2] = '2'; break;
    case 43: number[0] = '0'; number[1] = '4'; number[2] = '3'; break;
    case 44: number[0] = '0'; number[1] = '4'; number[2] = '4'; break;
    case 45: number[0] = '0'; number[1] = '4'; number[2] = '5'; break;
    case 46: number[0] = '0'; number[1] = '4'; number[2] = '6'; break;
    case 47: number[0] = '0'; number[1] = '4'; number[2] = '7'; break;
    case 48: number[0] = '0'; number[1] = '4'; number[2] = '8'; break;
    case 49: number[0] = '0'; number[1] = '4'; number[2] = '9'; break;
    case 50: number[0] = '0'; number[1] = '5'; number[2] = '0'; break;
    case 51: number[0] = '0'; number[1] = '5'; number[2] = '1'; break;
    case 52: number[0] = '0'; number[1] = '5'; number[2] = '2'; break;
    case 53: number[0] = '0'; number[1] = '5'; number[2] = '3'; break;
    case 54: number[0] = '0'; number[1] = '5'; number[2] = '4'; break;
    case 55: number[0] = '0'; number[1] = '5'; number[2] = '5'; break;
    case 56: number[0] = '0'; number[1] = '5'; number[2] = '6'; break;
    case 57: number[0] = '0'; number[1] = '5'; number[2] = '7'; break;
    case 58: number[0] = '0'; number[1] = '5'; number[2] = '8'; break;
    case 59: number[0] = '0'; number[1] = '5'; number[2] = '9'; break;
    case 60: number[0] = '0'; number[1] = '6'; number[2] = '0'; break;
    case 61: number[0] = '0'; number[1] = '6'; number[2] = '1'; break;
    case 62: number[0] = '0'; number[1] = '6'; number[2] = '2'; break;
    case 63: number[0] = '0'; number[1] = '6'; number[2] = '3'; break;
    case 64: number[0] = '0'; number[1] = '6'; number[2] = '4'; break;
    case 65: number[0] = '0'; number[1] = '6'; number[2] = '5'; break;
    case 66: number[0] = '0'; number[1] = '6'; number[2] = '6'; break;
    case 67: number[0] = '0'; number[1] = '6'; number[2] = '7'; break;
    case 68: number[0] = '0'; number[1] = '6'; number[2] = '8'; break;
    case 69: number[0] = '0'; number[1] = '6'; number[2] = '9'; break;
    case 70: number[0] = '0'; number[1] = '7'; number[2] = '0'; break;
    case 71: number[0] = '0'; number[1] = '7'; number[2] = '1'; break;
    case 72: number[0] = '0'; number[1] = '7'; number[2] = '2'; break;
    case 73: number[0] = '0'; number[1] = '7'; number[2] = '3'; break;
    case 74: number[0] = '0'; number[1] = '7'; number[2] = '4'; break;
    case 75: number[0] = '0'; number[1] = '7'; number[2] = '5'; break;
    case 76: number[0] = '0'; number[1] = '7'; number[2] = '6'; break;
    case 77: number[0] = '0'; number[1] = '7'; number[2] = '7'; break;
    case 78: number[0] = '0'; number[1] = '7'; number[2] = '8'; break;
    case 79: number[0] = '0'; number[1] = '7'; number[2] = '9'; break;
    case 80: number[0] = '0'; number[1] = '8'; number[2] = '0'; break;
    case 81: number[0] = '0'; number[1] = '8'; number[2] = '1'; break;
    case 82: number[0] = '0'; number[1] = '8'; number[2] = '2'; break;
    case 83: number[0] = '0'; number[1] = '8'; number[2] = '3'; break;
    case 84: number[0] = '0'; number[1] = '8'; number[2] = '4'; break;
    case 85: number[0] = '0'; number[1] = '8'; number[2] = '5'; break;
    case 86: number[0] = '0'; number[1] = '8'; number[2] = '6'; break;
    case 87: number[0] = '0'; number[1] = '8'; number[2] = '7'; break;
    case 88: number[0] = '0'; number[1] = '8'; number[2] = '8'; break;
    case 89: number[0] = '0'; number[1] = '8'; number[2] = '9'; break;
    case 90: number[0] = '0'; number[1] = '9'; number[2] = '0'; break;
    case 91: number[0] = '0'; number[1] = '9'; number[2] = '1'; break;
    case 92: number[0] = '0'; number[1] = '9'; number[2] = '2'; break;
    case 93: number[0] = '0'; number[1] = '9'; number[2] = '3'; break;
    case 94: number[0] = '0'; number[1] = '9'; number[2] = '4'; break;
    case 95: number[0] = '0'; number[1] = '9'; number[2] = '5'; break;
    case 96: number[0] = '0'; number[1] = '9'; number[2] = '6'; break;
    case 97: number[0] = '0'; number[1] = '9'; number[2] = '7'; break;
    case 98: number[0] = '0'; number[1] = '9'; number[2] = '8'; break;
    case 99: number[0] = '0'; number[1] = '9'; number[2] = '9'; break;
    case 100: number[0] = '1'; number[1] = '0'; number[2] = '0'; break;
    case 101: number[0] = '1'; number[1] = '0'; number[2] = '1'; break;
    case 102: number[0] = '1'; number[1] = '0'; number[2] = '2'; break;
    case 103: number[0] = '1'; number[1] = '0'; number[2] = '3'; break;
    case 104: number[0] = '1'; number[1] = '0'; number[2] = '4'; break;
    case 105: number[0] = '1'; number[1] = '0'; number[2] = '5'; break;
    case 106: number[0] = '1'; number[1] = '0'; number[2] = '6'; break;
    case 107: number[0] = '1'; number[1] = '0'; number[2] = '7'; break;
    case 108: number[0] = '1'; number[1] = '0'; number[2] = '8'; break;
    case 109: number[0] = '1'; number[1] = '0'; number[2] = '9'; break;
    case 110: number[0] = '1'; number[1] = '1'; number[2] = '0'; break;
    case 111: number[0] = '1'; number[1] = '1'; number[2] = '1'; break;
    case 112: number[0] = '1'; number[1] = '1'; number[2] = '2'; break;
    case 113: number[0] = '1'; number[1] = '1'; number[2] = '3'; break;
    case 114: number[0] = '1'; number[1] = '1'; number[2] = '4'; break;
    case 115: number[0] = '1'; number[1] = '1'; number[2] = '5'; break;
    case 116: number[0] = '1'; number[1] = '1'; number[2] = '6'; break;
    case 117: number[0] = '1'; number[1] = '1'; number[2] = '7'; break;
    case 118: number[0] = '1'; number[1] = '1'; number[2] = '8'; break;
    case 119: number[0] = '1'; number[1] = '1'; number[2] = '9'; break;
    case 120: number[0] = '1'; number[1] = '2'; number[2] = '0'; break;
  } 
}

void adc_value(){
  sensorV[0] = '0';
  sensorV[1] = '0';
  sensorV[2] = '0';
  sensorV[3] = '0';
  sensorV[4] = '\0';
  if(sensorValue/1000 == 0){
    sensorV [0] = '0';  
    mil = 0;
  }
  else if(sensorValue/1000 == 1){
    sensorV [0] = '1';
    mil = 1000; 
  }
  else if(sensorValue/1000 == 2){
    sensorV [0] = '2'; 
    mil = 2000;
  }
  else if(sensorValue/1000 == 3){
    sensorV [0] = '3'; 
    mil = 3000;
  }
  else if(sensorValue/1000 == 4){
    sensorV [0] = '4'; 
    mil = 4000;
  }
  else {
    sensorV [0] = 'a';
  }

  if((sensorValue-mil)/100 == 0){
    sensorV [1] = '0';
    centena = 0;  
  }
  else if((sensorValue-mil)/100 == 1){
    sensorV [1] = '1'; 
    centena = 100;
  }
  else if((sensorValue-mil)/100 == 2){
    sensorV [1] = '2';
    centena = 200; 
  }
  else if((sensorValue-mil)/100 == 3){
    sensorV [1] = '3';
    centena = 300; 
  }
  else if((sensorValue-mil)/100 == 4){
    sensorV [1] = '4';
    centena = 400; 
  }
  else if((sensorValue-mil)/100 == 5){
    sensorV [1] = '5';
    centena = 500;  
  }
  else if((sensorValue-mil)/100 == 6){
    sensorV [1] = '6'; 
    centena = 600;
  }
  else if((sensorValue-mil)/100 == 7){
    sensorV [1] = '7';
    centena = 700; 
  }
  else if((sensorValue-mil)/100 == 8){
    sensorV [1] = '8';
    centena = 800; 
  }
  else if((sensorValue-mil)/100 == 9){
    sensorV [1] = '9';
    centena = 900; 
  }
  else {
    sensorV [1] = 'a';
  }

  if((sensorValue-mil-centena)/10 == 0){
    sensorV [2] = '0';  
    decena = 0;
  }
  else if((sensorValue-mil-centena)/10 == 1){
    sensorV [2] = '1'; 
    decena = 10;
  }
  else if((sensorValue-mil-centena)/10 == 2){
    sensorV [2] = '2';
    decena = 20; 
  }
  else if((sensorValue-mil-centena)/10 == 3){
    sensorV [2] = '3';
    decena = 30; 
  }
  else if((sensorValue-mil-centena)/10 == 4){
    sensorV [2] = '4';
    decena = 40; 
  }
  else if((sensorValue-mil-centena)/10 == 5){
    sensorV [2] = '5';  
    decena = 50;
  }
  else if((sensorValue-mil-centena)/10 == 6){
    sensorV [2] = '6';
    decena = 60; 
  }
  else if((sensorValue-mil-centena)/10 == 7){
    sensorV [2] = '7';
    decena = 70; 
  }
  else if((sensorValue-mil-centena)/10 == 8){
    sensorV [2] = '8';
    decena = 80; 
  }
  else if((sensorValue-mil-centena)/10 == 9){
    sensorV [2] = '9';
    decena = 90; 
  }
  else {
    sensorV [2] = 'a';
  }

  if((sensorValue-mil-centena-decena) == 0){
    sensorV [3] = '0';  
    unidad = 0;
  }
  else if((sensorValue-mil-centena-decena) == 1){
    sensorV [3] = '1'; 
    unidad = 10;
  }
  else if((sensorValue-mil-centena-decena) == 2){
    sensorV [3] = '2';
    unidad = 20; 
  }
  else if((sensorValue-mil-centena-decena) == 3){
    sensorV [3] = '3';
    unidad = 30; 
  }
  else if((sensorValue-mil-centena-decena) == 4){
    sensorV [3] = '4';
    unidad = 40; 
  }
  else if((sensorValue-mil-centena-decena) == 5){
    sensorV [3] = '5';  
    unidad = 50;
  }
  else if((sensorValue-mil-centena-decena) == 6){
    sensorV [3] = '6';
    unidad = 60; 
  }
  else if((sensorValue-mil-centena-decena) == 7){
    sensorV [3] = '7';
    unidad = 70; 
  }
  else if((sensorValue-mil-centena-decena) == 8){
    sensorV [3] = '8';
    unidad = 80; 
  }
  else if((sensorValue-mil-centena-decena) == 9){
    sensorV [3] = '9';
    unidad = 90; 
  }
  else {
    sensorV [3] = 'a';
  }
  sensorV[4] = '\0';
}

void adctoangle()
{
  if(sensoraux < 2161)
  {
    sensoracum = 90;
  }
  else if(sensoraux < 2172)
  {
    sensoracum = 89;
  }
  else if(sensoraux < 2183)
  {
    sensoracum = 88;
  }
  else if(sensoraux < 2194)
  {
    sensoracum = 87;
  }
  else if(sensoraux < 2205)
  {
    sensoracum = 86;
  }
  else if(sensoraux < 2216)
  {
    sensoracum = 85;
  }
  else if(sensoraux < 2227)
  {
    sensoracum = 84;
  }
  else if(sensoraux < 2238)
  {
    sensoracum = 83;
  }
  else if(sensoraux < 2249)
  {
    sensoracum = 82;
  }
  else if(sensoraux < 2260)
  {
    sensoracum = 81;
  }
  else if(sensoraux < 2271)
  {
    sensoracum = 80;
  }
  else if(sensoraux < 2282)
  {
    sensoracum = 79;
  }
  else if(sensoraux < 2293)
  {
    sensoracum = 78;
  }
  else if(sensoraux < 2304)
  {
    sensoracum = 77;
  }
  else if(sensoraux < 2315)
  {
    sensoracum = 76;
  }
  else if(sensoraux < 2326)
  {
    sensoracum = 75;
  }
  else if(sensoraux < 2337)
  {
    sensoracum = 74;
  }
  else if(sensoraux < 2348)
  {
    sensoracum = 73;
  }
  else if(sensoraux < 2359)
  {
    sensoracum = 72;
  }
  else if(sensoraux < 2370)
  {
    sensoracum = 71;
  }
  else if(sensoraux < 2381)
  {
    sensoracum = 70;
  }
  else if(sensoraux < 2392)
  {
    sensoracum = 69;
  }
  else if(sensoraux < 2403)
  {
    sensoracum = 68;
  }
  else if(sensoraux < 2414)
  {
    sensoracum = 67;
  }
  else if(sensoraux < 2425)
  {
    sensoracum = 66;
  }
  else if(sensoraux < 2436)
  {
    sensoracum = 65;
  }
  else if(sensoraux < 2447)
  {
    sensoracum = 64;
  }
  else if(sensoraux < 2458)
  {
    sensoracum = 63;
  }
  else if(sensoraux < 2469)
  {
    sensoracum = 62;
  }
  else if(sensoraux < 2480)
  {
    sensoracum = 61;
  }
  else if(sensoraux < 2491)
  {
    sensoracum = 60;
  }
  else if(sensoraux < 2502)
  {
    sensoracum = 59;
  }
  else if(sensoraux < 2513)
  {
    sensoracum = 58;
  }
  else if(sensoraux < 2524)
  {
    sensoracum = 57;
  }
  else if(sensoraux < 2535)
  {
    sensoracum = 56;
  }
  else if(sensoraux < 2546)
  {
    sensoracum = 55;
  }
  else if(sensoraux < 2557)
  {
    sensoracum = 54;
  }
  else if(sensoraux < 2568)
  {
    sensoracum = 53;
  }
  else if(sensoraux < 2579)
  {
    sensoracum = 52;
  }
  else if(sensoraux < 2590)
  {
    sensoracum = 51;
  }
  else if(sensoraux < 2601)
  {
    sensoracum = 50;
  }
  else if(sensoraux < 2612)
  {
    sensoracum = 49;
  }
  else if(sensoraux < 2623)
  {
    sensoracum = 48;
  }
  else if(sensoraux < 2634)
  {
    sensoracum = 47;
  }
  else if(sensoraux < 2645)
  {
    sensoracum = 46;
  }
  else if(sensoraux < 2656)
  {
    sensoracum = 45;
  }
  else if(sensoraux < 2667)
  {
    sensoracum = 44;
  }
  else if(sensoraux < 2678)
  {
    sensoracum = 43;
  }
  else if(sensoraux < 2689)
  {
    sensoracum = 42;
  }
  else if(sensoraux < 2700)
  {
    sensoracum = 41;
  }
  else if(sensoraux < 2711)
  {
    sensoracum = 40;
  }
  else if(sensoraux < 2722)
  {
    sensoracum = 39;
  }
  else if(sensoraux < 2733)
  {
    sensoracum = 38;
  }
  else if(sensoraux < 2744)
  {
    sensoracum = 37;
  }
  else if(sensoraux < 2755)
  {
    sensoracum = 36;
  }
  else if(sensoraux < 2766)
  {
    sensoracum = 35;
  }
  else if(sensoraux < 2777)
  {
    sensoracum = 34;
  }
  else if(sensoraux < 2788)
  {
    sensoracum = 33;
  }
  else if(sensoraux < 2799)
  {
    sensoracum = 32;
  }
  else if(sensoraux < 2810)
  {
    sensoracum = 31;
  }
  else if(sensoraux < 2821)
  {
    sensoracum = 30;
  }
  else if(sensoraux < 2832)
  {
    sensoracum = 29;
  }
  else if(sensoraux < 2843)
  {
    sensoracum = 28;
  }
  else if(sensoraux < 2854)
  {
    sensoracum = 27;
  }
  else if(sensoraux < 2865)
  {
    sensoracum = 26;
  }
  else if(sensoraux < 2876)
  {
    sensoracum = 25;
  }
  else if(sensoraux < 2887)
  {
    sensoracum = 24;
  }
  else if(sensoraux < 2898)
  {
    sensoracum = 23;
  }
  else if(sensoraux < 2909)
  {
    sensoracum = 22;
  }
  else if(sensoraux < 2920)
  {
    sensoracum = 21;
  }
  else if(sensoraux < 2931)
  {
    sensoracum = 20;
  }
  else if(sensoraux < 2942)
  {
    sensoracum = 19;
  }
  else if(sensoraux < 2953)
  {
    sensoracum = 18;
  }
  else if(sensoraux < 2964)
  {
    sensoracum = 17;
  }
  else if(sensoraux < 2975)
  {
    sensoracum = 16;
  }
  else if(sensoraux < 2986)
  {
    sensoracum = 15;
  }
  else if(sensoraux < 2997)
  {
    sensoracum = 14;
  }
  else if(sensoraux < 3008)
  {
    sensoracum = 13;
  }
  else if(sensoraux < 3019)
  {
    sensoracum = 12;
  }
  else if(sensoraux < 3030)
  {
    sensoracum = 11;
  }
  else if(sensoraux < 3041)
  {
    sensoracum = 10;
  }
  else if(sensoraux < 3052)
  {
    sensoracum = 9;
  }
  else if(sensoraux < 3063)
  {
    sensoracum = 8;
  }
  else if(sensoraux < 3074)
  {
    sensoracum = 7;
  }
  else if(sensoraux < 3085)
  {
    sensoracum = 6;
  }
  else if(sensoraux < 3096)
  {
    sensoracum = 5;
  }
  else if(sensoraux < 3107)
  {
    sensoracum = 4;
  }
  else if(sensoraux < 3118)
  {
    sensoracum = 3;
  }
  else if(sensoraux < 3129)
  {
    sensoracum = 2;
  }
  else if(sensoraux < 3140)
  {
    sensoracum = 1;
  }
  else if(sensoraux < 3151)
  {
    sensoracum = 0;
  }
  
}
