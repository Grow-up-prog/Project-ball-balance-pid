/*
PROGRAM KESEIMBANGAN BOLA PADA TITIK SETPOINT
Septian Saputra
Pendidikan Teknik Elektro
Universitas Negeri Jakarta
percobaan ke -9
*/

//Pendefinisian servo
#include <Servo.h> //library servo
Servo servo; //variabel nama pada servo
//pendefinisian PIN sensor pada arduino
#define trig 9
#define echo 8
//pendefinisian nilai PID
float Kp =1, Ki = 0.0001, Kd =1000, P, I, D;
double error, sum_error,der_error, error_sebelumnya, delta_time, nilai_pid,nilai_servo, setpoint;


//penggunaan fungi millis
unsigned long interval=20;//sampling time 20ms
unsigned long waktu_awal=0;



void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  servo.attach(4);
  Serial.begin(9600);
  setpoint =10;
  error_sebelumnya =0;
  servo.write(120);

}
void loop() {
  unsigned long waktu_sekarang=millis();//penyimpan waktu 
  if(waktu_sekarang-waktu_awal>=interval){
    //print nilai
    int a = jarak();
    
    delta_time = waktu_sekarang-waktu_awal;//perbedaan waktu
    Serial.print("nilai sekarang");Serial.print(waktu_sekarang);Serial.print("nilai awal");Serial.println(waktu_awal);
    Serial.print("nilai perbedaan waktu");Serial.println(delta_time);
    error = setpoint-a;  //untuk nilai e pada kendali proporsional
    sum_error = sum_error + error; //integral error
    der_error = error-error_sebelumnya; //derivatif error
    Serial.print("jarak bola adalah =");Serial.print(a);Serial.print("cm");Serial.print("  waktu sekarang= ");Serial.println(waktu_sekarang);Serial.println("");   
    Serial.print("nilai error = ");Serial.println(error);
    Serial.print("nilai sum_error = ");Serial.print(sum_error);Serial.print("nilai der_error = ");Serial.println(der_error);
    //control PID
    P = Kp * error; 
    I = ((Ki * sum_error)*delta_time);
    D = ((Kd * der_error)/delta_time);
//pembatasan nilai variabel
    
    Serial.print("nilai proporsional = ");Serial.print(P);Serial.print("nilai Integral = ");Serial.print(I);Serial.print("nilai Derivatif = ");Serial.println(D);
    nilai_pid=P+I+D;
    Serial.print("nilai PID adalah= ");Serial.println(nilai_pid);
    if(nilai_pid >25){
      nilai_pid=20;
    } else if(nilai_pid <=-40){
      nilai_pid=-40;
    }
    nilai_servo=120+nilai_pid;
    servo.write(nilai_servo);
    Serial.print("nilai PID limit adalah= ");Serial.println(nilai_pid);

    waktu_awal=waktu_sekarang;//update nilai waktu
    error_sebelumnya = error;
  }
  
}

long jarak() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long t = pulseIn(echo, HIGH);
  long cm = t*0.034/2;
  if (cm >= 30){ //melakukan pembatasan pada jarak maksimal pengukuran
    cm=30; 
  }
  return cm;
}
