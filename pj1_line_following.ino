float Kp=40,Ki=0,Kd=160;
float sai_so=0,P=0,I=0,D=0,PID_value=0;
float saiso_truoc=0;

int sensor[6]={0,0,0,0,0,0};
int gia_tri_ban_dau=150;
int PID_phai,PID_trai;
#define In1 4
#define In2 5
#define In3 6
#define In4 7
#define ENA 3
#define ENB 11
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
//void dung();
//void chay_thang();
void setup()
{
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(In3,OUTPUT);
  pinMode(In4,OUTPUT);
  Serial.begin(9600);
}
void read_sensor_values()
{
  sensor[0]=digitalRead(A0);
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);
  sensor[5]=digitalRead(A5);
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==1))
  sai_so=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==1)&&(sensor[4]==1))
  sai_so=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==1)&&(sensor[4]==0))
  sai_so=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[4]==1)&&(sensor[4]==0))
  sai_so=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[4]==0)&&(sensor[4]==0))
  sai_so=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[4]==0)&&(sensor[4]==0))
  sai_so=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  sai_so=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  sai_so=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  sai_so=-4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
    if( sai_so==-4)   sai_so=-5;
    else   sai_so=5;

}
void calculate_pid()
{
    P = sai_so;
    I = I + sai_so;
    D = sai_so - saiso_truoc;
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    saiso_truoc=sai_so;
}
void motor_control()
{
    digitalWrite(In1,HIGH);
    digitalWrite(In2,LOW);
    digitalWrite(In3,LOW);
    digitalWrite(In4,HIGH);
    PID_phai = gia_tri_ban_dau - PID_value;
    PID_trai = gia_tri_ban_dau + PID_value;
    PID_phai = constrain(gia_tri_ban_dau - PID_value,0,150);
    PID_trai = constrain(gia_tri_ban_dau + PID_value,0,150);
    analogWrite(ENA,PID_phai);
    analogWrite(ENB,PID_trai);
}
void loop()
{
    read_sensor_values();
    calculate_pid();
    motor_control();
}
