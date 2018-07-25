//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////// 
/*
feedback_x : toa do x gui ve
feedback_y :toa do y gui ve
v_xd,v_yd : van toc dai banh 1, banh 2
anpha_p : goc dat tinh duoc
feedback_anpha: goc hop huong chinh voi truc co dinh.
w_d : toc do goc cua xe quanh tam.
v_1d,v_2d  : vâ?n tô´c da`i ba´nh 1,2 

*/

//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
#include "Arduino.h"
#include "math.h"
#include "PinChangeInt.h"
//------------------------------------------------------------------------------------------------
//--------------------------------Port connection--------------------------------
//X1 : banh trai ( Note: 9,10 dao? day )
#define X1encodPinA1      2   //  - interrupt                        
#define X1encodPinB1      8                          
#define X1M1              10                           
#define X1M2              9

//X2: banh phai
#define X2encodPinA1      3    // - interrupt                        
#define X2encodPinB1      12                          
#define X2M1              5                           
#define X2M2              6

//------------------------------------------------------------------------------------------------
//--------------------------------Kalman--------------------------------

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include "I2C.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;
#define RESTRICT_PITCH
SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.001);
 
uint8_t i2cData[14]; // Buffer for I2C data
 




//------------------------------------------------------------------------------------------------
//--------------------------------SETUP----------------------------------------------------------------
void setup (){
  Serial.begin(9600);

       //---------------Encoder------------------------------------------ Note:  timer 1 pin 9,10, timer 0 pin 5,6 (default 976Hz) ; timer 2 pin 3,11
  pinMode(X1encodPinA1, INPUT_PULLUP); 
  pinMode(X1encodPinB1, INPUT_PULLUP);
  attachInterrupt(0, encoder1, FALLING);
  TCCR1B = TCCR1B & 0b11111000 | 1; // set tan so cao, set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz

  pinMode(X2encodPinA1, INPUT_PULLUP); 
  pinMode(X2encodPinB1, INPUT_PULLUP);
  attachInterrupt(1, encoder2, FALLING);

  
      //----------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------------------------Kalman------------------------------------------------------------------------------------------
  // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    //Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    // configure Arduino LED pin for output
    
    Wire.begin();
  
}

//--------------------------------------------------------------------------------------------------
//--------------------------------Variable----------------------------------------------------------------

float ouput1,ouput2;                    // PWM

volatile long int xung1 = 0; //so xung tu encoder 1
volatile long int xung2 = 0; //so xung tu encoder 1
 
float w_d;      //  van toc goc quay

double gz_value = 0 ;
double anglez = 0 ;  // MPU
double ax_value=0 ; // MPU

int countt =0;     // Calib ax
float calib_ax = 0;
float sum_calib_ax=0;

int countt_gz =0;     // Calib gz
float calib_gz = 0;
float sum_calib_gz=0;

int spin = 0;

int stepp= 0 ; // if the loop has many steps, we will you this variable to know which step the program r doing. 
//--------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------
// Definition 
//Output_Leftmotor(100); scale from -100% to 100%
//Output_Rightmotor(100); scale from -100% to 100%
//Read_AngularVelocity_Left (); ( e-2 round per sec )
//Read_AngularVelocity_Left ();( e-2 round per sec) 
//PID_MotorSpeed(40,100 );
//PID_angleOfRot(ref1,ref2); (degree)
//Read_AOR_Left();
//PID_angleOfCar(); //  positive - counter-clockwise ( degree)
//updateMPU(); gz & ax will be updated, when you call this function
void PID_MotorSpeed(float ref1, float ref2);
void PID_gocquay(float ref,float v_d);
void PID_position(float ref_x,float ref_y);


//-----------------------------------------------------------------------------------------------------
//-------------------------------------------------LOOP-------------------------------------------------

void loop(){
long int mywatch= millis();
    if ( (200 < mywatch) && (mywatch <3000) )  // Step 0 tư 0.2 - 3s : calib
        { Calib_gz();
        //Serial.println(countt); 
        }
   if ( (mywatch > 3000) && (mywatch <700000) )
         { stepp=1;
            
       //  updateMPU(); 
       PID_angleOfCar(0);
       PID_angleOfRot(2000,2000);
       
        // PID_MotorSpeed(70,70 );
         delay(40);    
         
         //delay(40);   
         //forward();
      //  Serial.print(gz_value);
      //  Serial.print(",");
      //  Serial.println(anglez); 
          }
if (mywatch > 700000)
{ Stop();}
   
}

//--------------------------------------------------------------------------------------------------
//---------------------------------------Calib----------------------------------------------
//--------------------------------------------------------------------------------------------------

void Calib_ax() 
{   updateMPU();
    sum_calib_ax += ax_value;
    countt ++ ;
}

void Calib_gz() 
{   updateMPU();
    sum_calib_gz += gz_value;
    countt_gz ++ ;
}
//--------------------------------------------------------------------------------------------------
//---------------------------------------Update MPU----------------------------------------------
//--------------------------------------------------------------------------------------------------

void updateMPU()
{
static float pre;
float now;
float dt;
// chinh offset cac thong so
//static double anglex = 0 ;
//static double angley = 0 ;

static double posx  = 0 ;
static double vx    = 0 ;
double ax_kalman ;


float vx_es= 0;
float last_vx = 0 ;

float gx_kalman;
float gy_kalman;
float gz_kalman;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;


 accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
 now= millis();
 gz_kalman = -simpleKalmanFilter.updateEstimate((float)gz); // đảo lại để quay trái là dương
 //ax_kalman= simpleKalmanFilter.updateEstimate((float)ax);

// khi goi ham Calib_ax o Step_0 se co tac dung thay doi calib_ax ở các step sau đó -----------------------------
 if ( (stepp > 0) && (countt > 0 ) ) calib_ax = sum_calib_ax / countt ;  
   
 if ( (stepp > 0) && (countt_gz > 0 ) )  calib_gz = sum_calib_gz / countt_gz ;     
    
        ax_value= (ax_kalman/32767)*(2*9.8*100)  -  calib_ax ; // useful value
        gz_value = gz_kalman / 32767*250  - calib_gz ;
 
 dt = (now-pre)/1000; //(second)

 if (stepp > 0) anglez += (now-pre)*gz_value/1000; // Caculate angle

// Caculate position, begin in Step 1
/*if (stepp > 0)
{vx += ax_value*dt; 
vx_es= 0.1*vx + 0.9*last_vx;
Serial.print(ax_value);
Serial.print(",");
Serial.println(vx);  
}*/

 //last_vx =vx_es; 

 
//Serial.print(gz_value);
//Serial.print(",");
//Serial.println(vx); 
 pre = now; 
}
 
//--------------------------------------------------------------------------------------------------

class PID{
  private:
  float sum = 0;
  float pre_e = 0;
  float now=0;
  float past = 0;
  public: float calculate(float ref, float feedback, float Kp , float Ki){
            now= millis();
            float e = ref - feedback;
            float T=(now - past)/1000;
          
            float u = Kp*e + Ki*sum;
            sum = sum + e*T;
            pre_e =e;
            past = now;
            return u;
            }
};
 





//-----------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------
  void Output_Leftmotor(float x)
  {         pwmOut1(x/100*255) ; }
   void Output_Rightmotor(float x)
  {         pwmOut2(x/100*255) ; }          
 void Stop()
 {  Output_Leftmotor(0);
    Output_Rightmotor(0); }
//-----------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------

void PID_position(float ref_x,float ref_y){
  static PID positionx;
  static PID positiony;
  float feedback_x = 20;       //toa do x gui ve
  float feedback_y = 20;       //toa do y gui ve
  float v_xd,v_yd; 
  v_xd = positionx.calculate(ref_x,feedback_x,10,10);
  v_yd = positiony.calculate(ref_y,feedback_y,10,10);
  float tanx = v_yd/v_xd;
  float anpha_p = atan(tanx);
 float v_d = sqrt(v_xd*v_xd+ v_yd*v_yd);
  PID_gocquay(10,v_d);
}

void PID_gocquay(float ref,float v_d){
  static PID goc;
float feedback_anpha = 20;                 //mpu_read(),goc do dc
  int L= 10 ;                          //chieu dai xe
  float w_d = goc.calculate(ref,feedback_anpha,10,10);
  float ref1 = v_d + w_d*L/2;      //ref1,ref2: v_1d,v_2d van toc dai banh 1, banh 2 tinh duoc
  float ref2 = v_d - w_d*L/2; 
  PID_MotorSpeed(ref1,ref2);  
}


//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------- Read Angular velocity ---------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
static float X1_last_pos = 0;
static float X1_pos = 0;
float X1_last_v = 0;
float X1_last_last_v = 0;
float X1_v = 0;
static unsigned long X1_last_t = 0;
static unsigned long X1_t = 0;

  float Read_AngularVelocity_Left ()
  {
     X1_pos = xung1;
  X1_t = micros();
  float dt = (X1_t-X1_last_t)/1000000.0; // don vi giay
  float h = (X1_pos-X1_last_pos)/75/dt; // vong/s
  
   X1_v = h*100;
   
  X1_last_v = 0.8*X1_last_v +0.2*X1_v;
  X1_last_t = X1_t;
  X1_last_pos = X1_pos;
return X1_last_v;
  }

static float X2_last_pos = 0;
static float X2_pos = 0;
float X2_last_v = 0;
float X2_last_last_v = 0;
float X2_v = 0;
static unsigned long X2_last_t = 0;
static unsigned long X2_t = 0;

  float Read_AngularVelocity_Right ()
  {
     X2_pos = xung2;
  X2_t = micros();
  float dt = (X2_t-X2_last_t)/1000000.0; // don vi giay
  float h = (X2_pos-X2_last_pos)/75/dt; // vong/s
  
   X2_v = h*100;
  X2_last_v = 0.8*X2_last_v +0.2*X2_v;
  X2_last_t = X2_t;
  X2_last_pos = X2_pos;
return X2_last_v;
  }

//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------- Read Angle of rotation ---------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------

float Read_AOR_Left()
{
  return xung1/75.00*360 ;
}

float Read_AOR_Right()
{
  return xung2/75.00*360 ;
}
  
//-- ----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------PID_ Velocity of rotation --------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------

void PID_MotorSpeed(float ref1, float ref2)
{
  static PID motor1;
  static PID motor2;
  float feedback1 ;     //encoder_read(), van toc 1 do duoc
  float feedback2;   //encoder_read(), van toc 2 do duoc
  
// tinh van toc doc tu encoder
feedback1= Read_AngularVelocity_Left();
feedback2= Read_AngularVelocity_Right();
  pwmOut1(motor1.calculate(ref1,feedback1,4,1) )  ;  //output1 Pwm
  
  pwmOut2(motor2.calculate(ref2,feedback2,4,1) )  ;   //output2 Pwm
  Serial.print(feedback1);
  Serial.print(",");
  Serial.println(feedback2);  
  
}
//------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------PID - Angle of rotation--------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------

void PID_angleOfRot(float refff1, float refff2)
{
  static PID AOR_Left;
  static PID AOR_Right;
  
  float feedbackAR1 ;     
  float feedbackAR2;   
  
feedbackAR1= Read_AOR_Left();
feedbackAR2= Read_AOR_Right();
float temp3= (feedbackAR1 + feedbackAR2)/2;
int temp1= AOR_Left.calculate(refff1,temp3,0.03,0.001);
int temp2 =AOR_Right.calculate(refff2,temp3,0.03,0.001); // temp1 sẽ bằng temp2

  pwmOut1(temp1 - spin)  ;  //output1 Pwm
  pwmOut2(temp2 + spin)  ;   //output2 Pwm
  Serial.print(feedbackAR1);  
  Serial.print(",");  
 // Serial.println(feedbackAR2);
  Serial.print(temp1  );  
  Serial.print(",");  
  Serial.print(spin );  
  Serial.print(",");
  Serial.print(temp1 - spin );  
  Serial.println(",");
 // Serial.println(temp2 + spin );
}
//------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------PID - Angle of Car (mobile robot)--------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
void PID_angleOfCar(float ref)
{
  static PID AOC;
  float feedback_angle;  
  
// tinh goc doc tu MPU
updateMPU();
feedback_angle = anglez;
int temp= AOC.calculate(ref,feedback_angle,10,2);
spin = temp;
 // pwmOut1(-1*temp)  ;  //output1 Pwm   // tạm thời bỏ qua
//  pwmOut2(temp)  ;   //output2 Pwm
//  Serial.print(feedback_angle); 
//  Serial.print(",");  
 // Serial.println(temp);

}



//---------------------- INTERRUPT TO READ ENCODER----------------------------------------------------------------
void encoder1()  {     
  if (PINB & 0b00000001) // chan 8 dit
    --xung1;
  else
    ++xung1;
}
void encoder2()  {     
  if (PINB & 0b00010000) // chan 12 dit
    --xung2;
  else
    ++xung2;
}
//---------------------- OUTPUT PWM----------------------------------------------------------------
  void pwmOut1(int out) {    
          if (out > 0) {
            out = out + 30;
          if (out > 255) out = 255;
            analogWrite(X1M1, out);       
            analogWrite(X1M2, 0);                                              
          }
          else {
            out = out - 30;
           if (out < -255) out = - 255;
            analogWrite(X1M1, 0);                         
            analogWrite(X1M2, abs(out));         
          }
  }
   void pwmOut2(int out) {    
          if (out > 0) {
             out = out + 30;
            if (out > 255) out = 255;
            analogWrite(X2M1, out);       
            analogWrite(X2M2, 0);                                              
          }
          else {
             out = out - 30;
          if (out < -255) out = - 255;
            analogWrite(X2M1, 0);                         
            analogWrite(X2M2, abs(out));         
          }
  }
