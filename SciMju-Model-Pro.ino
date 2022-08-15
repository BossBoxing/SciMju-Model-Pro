
//////////////////////////
// วันวิทย์ฯ แม่โจ้ 2565     //
// [@ Friend Robot.co ] //
// BossBoxing.Dev       //
//////////////////////////

#include "ModelPro.h"
#include "EEPROM.h"

#define startReffAddress 0 // 0 - 5 
#define startServoSetAddress  10 // 10-11-12-13-14
#define startCanAddress 20 // 20 - 33
#define startColorHandAddress 40 // 40 - 43
#define startColorFloorAddress 60 // 40 - 43

///////////////////////////////////////////////////
int diff_time = 50;
int T1 = 80 + diff_time; //เดินหน้าขึ้นไป เส้นมุมฉาก
int T2 = 390 + diff_time; //เดินหน้าขึ้นไป เส้นมุมแหลม
int T3 = 340 + diff_time; //เดินหน้าขึ้นไป เส้นมุมแหลมแบบคู่
int T4 = 10 + diff_time; //เดินหน้าขึ้นไป เส้นมุมฉาก ธรรมดา

int FT1 = 0 + diff_time; //เดินหน้าผ่าน เส้นมุมฉาก
int FT2 = 430 + diff_time; //เดินหน้าผ่าน เส้นมุมแหลม
int FT3 = 380 + diff_time; //เดินหน้าผ่าน เส้นมุมแหลมแบบคู่
int FT4 = 0 + diff_time; //เดินหน้าผ่าน เส้นมุมสี่แยก

int Ref_LLL = EEPROM.read(startReffAddress + 1); //ค่าแสงตเซ็นเซอร์นับแยกซ้าย
int Ref_LL = EEPROM.read(startReffAddress + 2); //ค่าแสงตเซ็นเซอร์ทางซ้ายสุด
int Ref_L = EEPROM.read(startReffAddress + 3); //ค่าแสงตเซ็นเซอร์ด้านในซ้าย
int Ref_C = EEPROM.read(startReffAddress + 4); //ค่าแสงตเซ็นเซอร์ด้านในซ้าย
int Ref_R = EEPROM.read(startReffAddress + 5); //ค่าแสงตเซ็นเซอร์ด้านในขวา
int Ref_RR = EEPROM.read(startReffAddress + 6); //ค่าแสงตเซ็นเซอร์ทางขวาสุด
int Ref_RRR = EEPROM.read(startReffAddress + 7); //ค่าแสงตเซ็นเซอร์นับแยกขวา

int Ref_CR_R = EEPROM.read(startColorHandAddress + 1);
int Ref_CR_G = EEPROM.read(startColorHandAddress + 2);
int Ref_CR_B = EEPROM.read(startColorHandAddress + 3);
int Ref_CR_BK = EEPROM.read(startColorHandAddress + 4);
int Ref_CG_R = EEPROM.read(startColorHandAddress + 5);
int Ref_CG_G = EEPROM.read(startColorHandAddress + 6);
int Ref_CG_B = EEPROM.read(startColorHandAddress + 7);
int Ref_CG_BK = EEPROM.read(startColorHandAddress + 8);

int Ref_FR_R = EEPROM.read(startColorFloorAddress + 1);
int Ref_FR_G = EEPROM.read(startColorFloorAddress + 2);
int Ref_FR_B = EEPROM.read(startColorFloorAddress + 3);
int Ref_FR_BK = EEPROM.read(startColorFloorAddress + 4);
int Ref_FG_R = EEPROM.read(startColorFloorAddress + 5);
int Ref_FG_G = EEPROM.read(startColorFloorAddress + 6);
int Ref_FG_B = EEPROM.read(startColorFloorAddress + 7);
int Ref_FG_BK = EEPROM.read(startColorFloorAddress + 8);

////////// 1 = ดำ ---- 2 = ขาว ------- 0 = ไม่มีกระป๋อง ////////////
int can1 = EEPROM.read(startCanAddress + 1);
int can2 = EEPROM.read(startCanAddress + 2);
int can3 = EEPROM.read(startCanAddress + 3);
int can4 = EEPROM.read(startCanAddress + 4);
int can5 = EEPROM.read(startCanAddress + 5);
int can6 = EEPROM.read(startCanAddress + 6);
int can7 = EEPROM.read(startCanAddress + 7);
int can8 = EEPROM.read(startCanAddress + 8);
int can9 = EEPROM.read(startCanAddress + 9);
int can10 = EEPROM.read(startCanAddress + 10);
int can11 = EEPROM.read(startCanAddress + 11);
int can12 = EEPROM.read(startCanAddress + 12);
int can13 = EEPROM.read(startCanAddress + 13);

int can_check[16] = {
     0 // 0
    ,0 // 1
    ,0 // 2
    ,0 // 3
    ,0 // 4
    ,0// 5
    ,0 // 6
    ,0 // 7
    ,0 // 8
    ,0 // 9
    ,0 // 10
    ,0 // 11
    ,0 // 12
    ,0 // 13
    ,0 // 14
    ,0 // 15
};
int CanPosition[5]={0,0,0,0,0};
///////////////////////////////////////////////////
int logTime = 100; //เวลาในการหยุดหุ่นยนต์
int timeOutLineT1 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก เส้นมุมฉาก
int timeOutLineT2 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก เส้นมุมแหลม
int timeOutLineT3 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก เส้นมุมแหลมแบบคู่
int timeOutLineT4 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก เส้นมุมฉาก ธรรมดา
int timeOutLineT5 = 120 + diff_time; //เวลาในการวิ่งออกจากแยก เส้นมุมฉาก สามแยก

int timeOutLineFT1 = 70 + diff_time; //เวลาในการวิ่งออกจากแยก FF เส้นมุมฉาก
int timeOutLineFT2 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก FF เส้นมุมแหลม
int timeOutLineFT3 = 80 + diff_time; //เวลาในการวิ่งออกจากแยก FF เส้นมุมแหลมแบบคู่
int timeOutLineFT4 = 40 + diff_time; //เวลาในการวิ่งออกจากแยก FF เส้นมุมสี่แยก

int SS_Can = 5; //ระยะเข้าหนีบกระป๋อง cm
int SS_Can_Slow = 12; //ระยะเข้าหนีบกระป๋๋องก่อนวิ่งช้า

int Clasp_Keep = EEPROM.read(startServoSetAddress + 1) == 255 ? 160 : EEPROM.read(startServoSetAddress + 1) ; //ค่าหนีบกระป๋อง
int Clasp_Place = EEPROM.read(startServoSetAddress + 2) == 255 ? 54 : EEPROM.read(startServoSetAddress + 2) ; //ค่าวางกระป๋อง
int Clasp_Set = EEPROM.read(startServoSetAddress + 3) == 255 ? 24 : EEPROM.read(startServoSetAddress + 3) ; //ค่าเซ็ทมือ หลบกระป๋อง

int Raise_Up = EEPROM.read(startServoSetAddress + 4) == 255 ? 45 : EEPROM.read(startServoSetAddress + 4) ; //ค่ายกมือ
int Raise_Down = EEPROM.read(startServoSetAddress + 5) == 255 ? 19 : EEPROM.read(startServoSetAddress + 5) ; //ค่าเอามือลง
///////////////////////////////////////////////////
#define S_LLL map(analog(0),0,1023,0,100) //พอร์ตเซ็นเซอร์นับแยกซ้าย
#define S_LL map(analog(1),0,1023,0,100)  //พอร์ตเซ็นเซอร์ทางซ้ายสุด
#define S_L map(analog(2),0,1023,0,100) //พอร์ตเซ็นเซอร์ด้านในซ้าย
#define S_C map(analog(3),0,1023,0,100) //พอร์ตเซ็นเซอร์ด้านในซ้าย
#define S_R map(analog(4),0,1023,0,100) //พอร์ตเซ็นเซอร์ด้านในขวา
#define S_RR map(analog(5),0,1023,0,100) //พอร์ตเซ็นเซอร์ทางขวาสุด
#define S_RRR map(analog(6),0,1023,0,100) //พอร์ตเซ็นเซอร์นับแยกขวา

#define S_CR map(analog(9),0,1023,0,169) //พอร์ตเซ็นเซอร์สีแดงบนมือ
#define S_CG map(analog(10),0,1023,0,169) //พอร์ตเซ็นเซอร์สีเขียวบนมือ
#define S_FR map(analog(7),0,1023,0,169) //พอร์ตเซ็นเซอร์สีแดงบนพื้น
#define S_FG map(analog(8),0,1023,0,169) //พอร์ตเซ็นเซอร์สีเขียวบนพื้น
#define S_Can 8 //พอร์ตเซ็นเซอร์เซ็คกระป๋อง

#define Clasp 2 //พอร์ต servo หนีบ
#define Raise 1 //พอร์ต servo ยก

int function = 0;

void setup() {
  ok();
}
void loop() {
  if (function == 0) {
    setServo();
  }
  if (function == 1) {
    setSensor();
  }
  if (function == 2) {
    //start();
    RR_Circle();
    // Pid_Circle(50);
    // code();
  }
  if (function == 3) {
    setSensorHand();
    Stop(100000);
  }
  if (function == 4) {
    setSensorFloor();
    // oledClear();
    // oled(0,10,"LLL: %d ",S_LLL);
    // oled(50,10,"RR: %d ",S_RR);
    // oled(0,20,"LL: %d ",S_LL);
    // oled(50,20,"RRR: %d ",S_RRR);
    // oled(0,30,"L: %d ",S_L);
    // oled(0,40,"C: %d ",S_C);
    // oled(0,50,"R: %d ",S_R);
    // delay(20);
    
    // Stop(100000);
  }
  if (function == 5) {
    CheckCan(0);

    //setCan();

    //Stop(100000);
  }
}
//////////start//////////
void start() {
  Start();
}
//// CAN PART
void Can1(){
  LL(4,1);

  InCan(1);

  LL(4,1);
  FF(4);

  GoPlace();
}
void Can2(){
  FF(4);
  LL(4,1);

  InCan(1);

  RR(4,1);
  FF(4);

  GoPlace();
}
void Can3(){
  RR(4,1);

  InCan(1);

  LL(4,1);

  GoPlace();
}
void Can4(){
  LL(4,1);

  InCan(1);

  RR(4,1);

  GoPlace();
}
///////////finish////////////
void finish() {
  FF(4);
  FF(4);
  LL(4,1);
  Finish_Circle();
  Finish();
}
///////// PART
void GoPlace(){
  if (CanPosition[1] == readCan()){
    goOne();
    PlaceCan("L");
    backOne();
  }
  else if(CanPosition[2] == readCan()){
    goTwo();
    PlaceCan("L");
    backTwo();
  }
  else if(CanPosition[3] == readCan()){
    goThree();
    PlaceCan("R");
    backThree();
  }
  else if(CanPosition[4] == readCan()){
    goFour();
    PlaceCan("R");
    backFour();
  }
  else{
    goCheck();
  }
}
void goCheck(){
  if (CanPosition[1] == 0){
    goOne();
  }
  else if (CanPosition[2] == 0){
    goTwo();
  }
  else if (CanPosition[3] == 0){
    goThree();
  }
  else if (CanPosition[4] == 0){
    goFour();
  }
}
void CheckCan(int Position){

  while (true)
  {

    if (readCan() == readFloor()){

      PlaceCan();

      if (Position == 1){
        backOne();
      }
      else if (Position == 2){
        backTwo();
      }
      else if (Position == 3){
        backThree();
      }
      else if (Position == 4){
        backFour();
      }
      break;
    }
    else{
      CanPosition[Position] = readFloor();
      Position++;
      goNextCan();
    }

  }

}

void goOne(){
  LL(4,1);
  LL_Circle();

  LL(4,1);
  RR(4,1);

  FF_Can();
}
void goTwo(){
  LL(4,1);
  LL_Circle();

  FF(4);
  
  FF_Can();
}
void goThree(){
  RR(4,1);
  RR_Circle();

  FF(4);
  
  FF_Can();
}
void goFour(){
  RR(4,1);
  RR_Circle();

  RR(4,1);
  LL(4,1);

  FF_Can();
}

void goNextCan(){
  Uturn();

  LL(4,1);
  LL(4,1);
  FF_Can();
}

void backOne(){
  Uturn();

  LL(4,1);
  RR(4,1);

  RR(4,1);
  RR_Circle();
}
void backTwo(){
  Uturn();

  FF(4,1);

  RR(4,1);
  RR_Circle();
}
void backThree(){
  Uturn();

  FF(4,1);

  LL(4,1);
  LL_Circle();
}
void backFour(){
  Uturn();

  RR(4,1);
  LL(4,1);

  LL(4,1);
  LL_Circle();
}
