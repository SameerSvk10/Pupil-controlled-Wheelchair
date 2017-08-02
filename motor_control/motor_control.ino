
#define edge_thresh 900
#define left_thresh 900
#define right_thresh 900
// --------------------------------------------------------------------------- Motors
int motor_left[] = {10, 11};
int motor_right[] = {12, 13};
int input1 = A4; // adc channel 0 left 
int input2= A3;
int input3 = A2;

// --------------------------------------------------------------------------- Setup
void setup() {
Serial.begin(9600);
pinMode(input1,INPUT);
pinMode(input2,INPUT);
pinMode(input3,INPUT);
// Setup motors
int i;
for(i = 0; i < 2; i++){
pinMode(motor_left[i], OUTPUT);
pinMode(motor_right[i], OUTPUT);
}

}

// --------------------------------------------------------------------------- Loop
void loop() { 

  while(Serial.available()>0)
  {
    char mov = Serial.read();
     int edge =analogRead(input1);
     delay(50);
     int left=analogRead(input2);
     delay(50);
     int right=analogRead(input3);
     delay(50);
 
  
    switch(mov)
    {
      case 'F':
         if (edge<edge_thresh)
         {
        drive_forward();
         }
        delay(500);
        motor_stop();
        break;
      
      case 'B':
      if (edge<edge_thresh)
         {
        drive_backward();
         }
        delay(500);
        motor_stop();
        break;
      
      case 'L':
      if (edge<edge_thresh)
         {
           if (left>left_thresh)
           {
              turn_left();
           }
         }
        delay(500);
        motor_stop();
        break;

      case 'R':
      if (edge<edge_thresh)
         {
           if (right>right_thresh)
           {
        turn_right();
           }
         }
        delay(500);
        motor_stop();
        break;

     case 'S':
        motor_stop();
        delay(500);
        break;
        
     default:
        motor_stop();
        delay(500);
        break;
    }
 }
motor_stop();
delay(200);

}

// --------------------------------------------------------------------------- Drive

void motor_stop(){
digitalWrite(motor_left[0], LOW); 
digitalWrite(motor_left[1], LOW); 

digitalWrite(motor_right[0], LOW); 
digitalWrite(motor_right[1], LOW);
delay(25);
}

void drive_forward(){
digitalWrite(motor_left[0], HIGH); 
digitalWrite(motor_left[1], LOW); 

digitalWrite(motor_right[0], HIGH); 
digitalWrite(motor_right[1], LOW); 
}

void drive_backward(){
digitalWrite(motor_left[0], LOW); 
digitalWrite(motor_left[1], HIGH); 

digitalWrite(motor_right[0], LOW); 
digitalWrite(motor_right[1], HIGH); 
}

void turn_left(){
digitalWrite(motor_left[0], LOW); 
digitalWrite(motor_left[1], HIGH); 

digitalWrite(motor_right[0], HIGH); 
digitalWrite(motor_right[1], LOW);
}

void turn_right(){
digitalWrite(motor_left[0], HIGH); 
digitalWrite(motor_left[1], LOW); 

digitalWrite(motor_right[0], LOW); 
digitalWrite(motor_right[1], HIGH); 
}
