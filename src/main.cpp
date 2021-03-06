#include <Arduino.h>
//-----方格圖-------------
#define R_direction    4  //控制右邊馬達方向的Pin腳
#define RightWheel_Pin 5  //速度
#define LeftWheel_Pin  6  //速度
#define L_direction    7  //控制左邊馬達方向的Pin腳
int trig = 13 , echo = 12;
//-------PID參數--------
float Kp=13,Ki=0.06,Kd=2;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int initial_motor_speed=72; //72
float right_motor_speed;
float left_motor_speed;
//-------PID--------
//-------IRsensor--------
int IRstatus;
int line;
int outline;
int sensor[5]={0, 0, 0, 0, 0};
int SL;
int SML;
int SM;
int SMR;
int SR;
int counterLL=0;
int counterRR=0;  
int counterBB=0; //看到全黑
int count_qiang_zhi_T = 0 ; //數轉彎



int  LED_ON = 9 ; 

bool count_first_quick_left_trun =  false  ; 
bool count_hard_left_turn = false ;
int forward_check = 0  ; 
int after_forward_check = 0 ; 

bool forward_using  = true ; 

void SML_shache() ; 

void All_white() ; 


//-------IRsensor靈敏度調整--------
void IRAD()
{
  if(analogRead(A0) > 40)  {SL = 1;}else{SL = 0;}
  if(analogRead(A1) > 40) {SML = 1;}else{SML = 0;}
  if(analogRead(A2) > 40)  {SM = 1;}else{SM = 0;}
  if(analogRead(A3) > 40) {SMR = 1;}else{SMR = 0;}
  if(analogRead(A4) > 40)  {SR = 1;}else{SR = 0;}
}
void calculate_pid()
{
  line = 1;
  outline = 0;
  sensor[0]= SL;
  sensor[1]= SML;
  sensor[2]= SM;
  sensor[3]= SMR;
  sensor[4]= SR;

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[3]==outline)&&(sensor[4]==line))
    error=4;
  else if((sensor[0]==line)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[4]==outline)&&(sensor[4]==outline))
    error=-4;
  
  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[3]==line)&&(sensor[4]==line))
    error=3;
  else if((sensor[0]==line)&&(sensor[1]==line)&&(sensor[2]==outline)&&(sensor[3]==outline)&&(sensor[4]==outline))
    error=-3;

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==outline)&&(sensor[3]==line)&&(sensor[4]==outline))
      error=2;
  else if((sensor[0]==outline)&&(sensor[1]==line)&&(sensor[2]==outline)&&(sensor[3]==outline)&&(sensor[4]==outline))
    error=-2;

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==line)&&(sensor[3]==line)&&(sensor[4]==outline))
    error=1;
  else if((sensor[0]==outline)&&(sensor[1]==line)&&(sensor[2]==line)&&(sensor[3]==outline)&&(sensor[4]==outline))
    error=-1;

  if((sensor[0]==outline)&&(sensor[1]==outline)&&(sensor[2]==line)&&(sensor[3]==outline)&&(sensor[4]==outline))
    error=0;
    
  P = error;
  I = I + previous_I;
  D = error-previous_error;

  PID_value =(Kp*P) + (Ki*I) + (Kd*D);

  previous_I=I;
  previous_error=error;
}
//-------------------------------------
void motorcontrol() /*PID 控制馬達*/
{
  left_motor_speed = initial_motor_speed+PID_value;
  right_motor_speed = initial_motor_speed-PID_value;  

  if(left_motor_speed < 0)
    left_motor_speed = 0;
  if(right_motor_speed < 0)
    right_motor_speed = 0;
    
  digitalWrite(R_direction,LOW);
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,right_motor_speed);
  analogWrite(LeftWheel_Pin,left_motor_speed);
}


void Forward()
{
  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,100);  //right_motor_speed
  analogWrite(LeftWheel_Pin,100);    //left_motor_speed
}

void Black()
{
  //digitalWrite(LED_ON,HIGH) ;
  counterBB += 1;
  if(counterBB == 3){
  digitalWrite(LED_ON,HIGH) ; 
  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,right_motor_speed);  //right_motor_speed
  analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
  delay(150);
   while(1)
   {
     int mid = analogRead(A2);
     if(mid > 40)
       break;  
   }
   digitalWrite(LED_ON,LOW) ; 
   }

  


   else if(counterBB == 4)
   {
      digitalWrite(R_direction,LOW);  
      digitalWrite(L_direction,LOW);
      analogWrite(RightWheel_Pin,0);  //right_motor_speed
      analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
      delay(30000);
   }
   else
   {
       digitalWrite(R_direction,LOW);  
       digitalWrite(L_direction,LOW);
       analogWrite(RightWheel_Pin,0);                  //right_motor_speed
       analogWrite(LeftWheel_Pin,left_motor_speed);    //left_motor_speed
       delay(150);
       while(1){
         int mid = analogRead(A2);
         if(mid > 40)
           break;  
               }
   }
  // digitalWrite(LED_ON,LOW) ;
}

void TurnLeft()
{
  //digitalWrite(LED_ON,HIGH) ;   //20211211 caesar
  counterLL += 1;
  delay(100) ; 
  int M = analogRead(A2);
  int L = analogRead(A3);
  if(M > 40)       // 前面有路    caesar 20211215 
  {
    if(counterLL<7){
      Forward();
      
    }
    else if(counterLL>=7){

    digitalWrite(R_direction,LOW);  
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,right_motor_speed+9);  //right_motor_speed
    analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
    delay(150);    // boost delay  20211209     xin 
      while(1)
       {
        int mid = analogRead(A2);
        if(mid > 40)
        break;  
        }
      }

     
    
    
  }

  else if(L > 40)   // 前面有路 
  {
    Forward();
  }

  // 修正閃燈但無快速左轉  20211211 caesar  
  // 發現閃燈但不左轉是左轉被關閉掉   20211215 caesar
  //下面這個條件並未被使用到        
  else if(counterRR >=9){

  

   // count_hard_left_turn = true ; 
  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,right_motor_speed+5);  //right_motor_speed
  analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
  delay(150);    // boost delay  20211209     xin 
   while(1)
   {
     int mid = analogRead(A2);
     if(mid > 40)
       break;  
   }

  
  

  }
  else if(count_hard_left_turn ==true){

  digitalWrite(R_direction,LOW) ; 
  digitalWrite(L_direction,LOW) ; 
  analogWrite(RightWheel_Pin, 0 ) ; 
  analogWrite(LeftWheel_Pin,0) ; 
  count_hard_left_turn = false ; 


  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,right_motor_speed);  //right_motor_speed
  analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
  delay(150);    // boost delay  20211209     xin 
   while(1)
   {
     int mid = analogRead(A2);
     if(mid > 40)
       break;  
   }





  }
  else
  {
  
  


  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,right_motor_speed+5);  //right_motor_speed
  analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
  delay(150);    // boost delay  20211209     xin 
   while(1)
   {
     int mid = analogRead(A2);
     if(mid > 40)
       break;  
   }
  }

  //digitalWrite(LED_ON,LOW) ;
}

void TurnRight()
{ 
  
 /*counter+=1;
  if(counter=4)
  { 
    void Forward();
    }
  else
    {*/
   
   //2021/12/7  caesar
   
  /* 
  if(count_first_quick_left_trun== true){ 

    digitalWrite(R_direction,LOW);  
      digitalWrite(L_direction,LOW);
      analogWrite(RightWheel_Pin,0);  //right_motor_speed
      analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
    count_first_quick_left_trun = false  ; 
    delay(100) ; 
  

//    digitalWrite(R_direction,LOW);  
//       digitalWrite(L_direction,LOW);
//       analogWrite(RightWheel_Pin,0);                  //right_motor_speed
//       analogWrite(LeftWheel_Pin,left_motor_speed);    //left_motor_speed
//       delay(50);
       

  }// the frist half circle check 

*/

//Problem 20211211 caesar  counterRR 失效 要忽略的直接進行右轉

  counterRR += 1;
  if(  counterLL >=4  && counterRR>=5 && forward_check==0)   //原版xin 的counterRR 是7  20211211 caesar
    {
    digitalWrite(LED_ON,HIGH) ;   
    Forward();
    delay(100);
    forward_check += 1  ;
    after_forward_check +=1 ; 
    digitalWrite(LED_ON,LOW) ; 
    }
/*
  else if(counterRR == 3)
  {
    delay(50);
       digitalWrite(R_direction,LOW);  
       digitalWrite(L_direction,LOW);
       analogWrite(RightWheel_Pin,0);                  //right_motor_speed
       analogWrite(LeftWheel_Pin,0);    //left_motor_speed
       delay(500);
       
       digitalWrite(R_direction,HIGH);  
       digitalWrite(L_direction,LOW);
       analogWrite(RightWheel_Pin,60);                  //right_motor_speed
       analogWrite(LeftWheel_Pin,70);    //left_motor_speed
       while(1)
       {
        int M = analogRead(A2);
        if(M > 40)
          break;
       }
  }
*/
  else{

      //digitalWrite(LED_ON,HIGH) ; //20211211 caesar

       digitalWrite(R_direction,LOW);  
       digitalWrite(L_direction,LOW);
       analogWrite(RightWheel_Pin,0);                  //right_motor_speed
       
       //出灣卡住
       analogWrite(LeftWheel_Pin,left_motor_speed+5);    //left_motor_speed 
       delay(150);
       while(1){
         int mid = analogRead(A2);
         if(mid > 40)
           break;  
               }

     // digitalWrite(LED_ON,LOW) ;   //20211211 caesar 
  }
     }



void quickLeft()
{
  count_first_quick_left_trun  = true ; 

  delay(100);
  digitalWrite(R_direction,LOW);  
  digitalWrite(L_direction,HIGH);
  analogWrite(RightWheel_Pin,right_motor_speed);  //right_motor_speed
  analogWrite(LeftWheel_Pin,60);                   //left_motor_speed
  delay(500);
  while(1)
  {
    int M = analogRead(A2);
    if(M > 40)
      break;
  }
}

void quickRight()
{
  digitalWrite(R_direction,HIGH);  
  digitalWrite(L_direction,LOW);
  analogWrite(RightWheel_Pin,60);  //right_motor_speed
  analogWrite(LeftWheel_Pin,left_motor_speed);                   //left_motor_speed
  delay(500);
   while(1)
  {
    int M = analogRead(A2);
    if(M > 40)
      break;
  }
}

// add the shache using only SMR  20211209 caesar
//The code below is using for picture C  first ban_yuan 


void count_qiang_zhi_TT(){

if(after_forward_check<2){

  count_qiang_zhi_T += 1 ; 


}

if(count_qiang_zhi_T==2 && after_forward_check==2){

      //強制執行右轉 （在第二個全黑BB）
  
       digitalWrite(R_direction,LOW);  
       digitalWrite(L_direction,LOW);
       analogWrite(RightWheel_Pin,0);                  //right_motor_speed
       
       //出灣卡住
       analogWrite(LeftWheel_Pin,left_motor_speed+5);    //left_motor_speed 
       delay(150);
       while(1){
         int mid = analogRead(A2);
         if(mid > 40)
           break;  
               }

  after_forward_check += 1 ; 
}






}









void SMR_shache(){

  //digitalWrite(LED_ON,HIGH) ; 
  // if 快速左轉有跑 
  if(count_first_quick_left_trun== true){ 

      //digitalWrite(LED_3,HIGH) ;   //2021211 caesar 

      digitalWrite(R_direction,LOW);  
      digitalWrite(L_direction,LOW);
      analogWrite(RightWheel_Pin,0);  //right_motor_speed
      analogWrite(LeftWheel_Pin,0);                   //left_motor_speed
      count_first_quick_left_trun = false  ; 
      delay(100) ; 


      digitalWrite(R_direction,LOW);
      digitalWrite(L_direction,LOW);
      analogWrite(RightWheel_Pin,0);
      analogWrite(LeftWheel_Pin,80);
      delay(700); //500
    
      digitalWrite(R_direction,LOW);
      digitalWrite(L_direction,LOW);
      analogWrite(RightWheel_Pin,85); //85
      analogWrite(LeftWheel_Pin,45);
      while(1)
      {
        int R = analogRead(A2);
        if(R > 40)
        break;
      }

      //20211211 caesar   修正SMR 刹車之後 卡住不動 
      //digitalWrite(R_direction,LOW);  
      //digitalWrite(L_direction,LOW);
      //analogWrite(RightWheel_Pin,0);                  //right_motor_speed
      //analogWrite(LeftWheel_Pin,left_motor_speed+6);    //left_motor_speed
      //delay(150);
       

      // digitalWrite(LED_ON,LOW)  ;//20211211 caesar 
  }

  else{

  motorcontrol() ; 

  }
  
}


// 20211210 caesar 

void SML_shache(){


if(counterRR >=11  ){

  count_hard_left_turn = true ; 

}

if(count_hard_left_turn ==true){

  digitalWrite(R_direction,LOW) ; 
  digitalWrite(L_direction,LOW) ; 
  analogWrite(RightWheel_Pin, 0 ) ; 
  analogWrite(LeftWheel_Pin,0) ; 
  count_hard_left_turn = false ; 


}
else{

  motorcontrol() ; 


}




}









//----------避障------------------------
/*long cm(int trig, int echo) 
{
  float duration, distance;
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration / 58;
  delay(10);
  return distance;
}

/*void US()
{
  int Ultrasound = cm(trig,echo);
  if(Ultrasound < 15 && Ultrasound > 3) 
  {
    digitalWrite(R_direction,LOW);
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,110);
    analogWrite(LeftWheel_Pin,50);
    delay(500);
    
    digitalWrite(R_direction,LOW);
    digitalWrite(L_direction,LOW);
    analogWrite(RightWheel_Pin,80);
    analogWrite(LeftWheel_Pin,130);
    while(1)
    {
      int R = analogRead(A4);
      if(R > 40)
        break;
    }
  }
} 
//--------------------------------------
/*void IRcount() 不知道為什麼寫副程式IRstatus就不會+
{
int IRstatus = 0;
int val = analogRead(A0);  //SL +1
    if(val > 40)
      IRstatus= IRstatus + 1;

    val = analogRead(A1);  //SML +2
    if(val > 40)
      IRstatus= IRstatus + 2;

    val = analogRead(A2);  //SM +4
    if(val > 40)
      IRstatus= IRstatus + 4;

    val = analogRead(A3);  //SMR +8
    if(val > 40)
      IRstatus= IRstatus + 8;

    val = analogRead(A4);  //SR +16
    if(val > 40)
      IRstatus= IRstatus + 16;
}*/

void SC()
{
  switch(IRstatus)
  {
    case 0: //全白
      motorcontrol();
      break;  
    case 4: /*SM*/ 
      motorcontrol();
      break;
    case 2:/*SML*/
       //SML_shache() ;  //20211211 caesar
       motorcontrol() ; 
      break;
    case 8: /*SMR*/
       SMR_shache() ;   // 20211210 caesar
      break;
    case 6: /*SM+SML*/
       motorcontrol();
      break;
    case 12: /*SM+SMR*/
        motorcontrol();
      break;
    case 7: /*SM+SML+SL*/  
      TurnLeft();
      count_qiang_zhi_TT() ; 
      break;
   case 15: /*SMR+SM+SML+SL*/
    TurnLeft();
    count_qiang_zhi_TT() ; 
      break;
    case 3: /*SML+SL*/
    TurnLeft();
    //count_qiang_zhi_TT() ; 
      break;
    case 30: /*SML+SM+SMR+SR*/
     TurnRight();
     count_qiang_zhi_TT() ; 
      break;
    case 28: //SM+SMR+SR
      TurnRight();
      count_qiang_zhi_TT() ; 
      break;
    case 24: //SMR+SR
      TurnRight();
      //count_qiang_zhi_TT() ; 
      break;
    case 5:  //圖C銳角
      quickLeft();
      break;
    case 13: 
      quickLeft();
      break;
    case 31:
      Black();
      break;
    
    case 16:  //修正SR 出彎停住    20211211 caesar 
      //motorcontrol() ; 
      break ; 
  }
}

void Report()
{
  Serial.print(analogRead(A0));Serial.print("\t");
  Serial.print(analogRead(A1));Serial.print("\t");
  Serial.print(analogRead(A2));Serial.print("\t");
  Serial.print(analogRead(A3));Serial.print("\t");
  Serial.print(analogRead(A4));Serial.print("\t");
  Serial.print("左轉次數");Serial.print(counterLL);Serial.print("\t");
  Serial.print("右轉次數");Serial.print(counterRR);Serial.print("\t");
  Serial.println(IRstatus);
}

void setup() 
{
Serial.begin(9600);
pinMode(R_direction,OUTPUT);
pinMode(RightWheel_Pin,OUTPUT);
pinMode(LeftWheel_Pin,OUTPUT);
pinMode(L_direction,OUTPUT);

pinMode(A0,INPUT);
pinMode(A1,INPUT);
pinMode(A2,INPUT);
pinMode(A3,INPUT);
pinMode(A4,INPUT);

pinMode(trig, OUTPUT);
pinMode(echo, INPUT);


pinMode(LED_ON ,OUTPUT);  //20211211 caesar
}

void loop() 
{
  

  IRAD();
  calculate_pid();

  

  
  IRstatus = 0;
    if(SL == 1)
      IRstatus= IRstatus + 1;

    if(SML == 1)
      IRstatus= IRstatus + 2;

    if(SM == 1)
      IRstatus= IRstatus + 4;

    if(SMR == 1)
      IRstatus= IRstatus + 8;

    if(SR == 1)
      IRstatus= IRstatus + 16;



  SC();

  Report();

}

