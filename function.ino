float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
long currentTime = 0;
int i = 0;
int Slow_L = 40;
int Slow_R = 40;

void Stop(int t) {
  motor(1, 0); motor(2, 0);
  delay(t);
}
void Fw(int t) {
  motor(1, 30); motor(2, 30);
  delay(t);
}
void FwCan(int t) {
  motor(1, 20); motor(2, 20);
  delay(t);
}
void Bk(int t) {
  motor(1, -30); motor(2, -30);
  delay(t);
}
void Log(int t) {
  motor(1, -10); motor(2, -10);
  delay(t);
}
void Keep() {
  Stop(1);
  servo(Raise, Raise_Down);  delay(50);
  servo(Clasp, Clasp_Keep);  delay(200);
  servo(Raise, Raise_Up);  delay(100);
  Stop(1);
}
void Place() {
  Stop(1);
  servo(Raise, Raise_Down);  delay(200);
  servo(Clasp, Clasp_Place);  delay(100);
  Stop(1);
}
int findError(){
  if(S_LL < Ref_LL && S_L > Ref_L && S_R > Ref_R && S_RR > Ref_RR)
  {
    return ((((S_LL*2) + S_L) ) - (S_R + S_RR));
  }
  else if(S_LL < Ref_LL && S_L < Ref_L && S_R > Ref_R && S_RR > Ref_RR)
  {
    return ((((S_LL*2) + S_L) ) - (S_R + S_RR));
  }

  else if(S_LL > Ref_LL && S_L > Ref_L && S_R > Ref_R && S_RR < Ref_RR)
  {
    return (((S_LL + S_L) ) - (S_R + (S_RR*2)));
  }
  else if(S_LL > Ref_LL && S_L > Ref_L && S_R < Ref_R && S_RR < Ref_RR)
  {
    return (((S_LL + S_L) ) - (S_R + (S_RR*2)));
  }

  else
  {
    return (((S_LL + S_L) ) - (S_R + S_RR));
  }
}
void calculate_pid(int power) {
  error = findError();
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (0.20 * P) + (0.5 * I) + (2 * D);
  previous_I = I;
  previous_error = error;
}
void motor_control(int power) {
  int left_motor_speed = power + PID_value;
  int right_motor_speed = power - PID_value;

  if (left_motor_speed > 100) {
    left_motor_speed = 100;
  }
  if (right_motor_speed > 100) {
    right_motor_speed = 100;
  }

  motor(1, left_motor_speed);
  motor(2, right_motor_speed);
}
void Pid(int power) {
  calculate_pid(power);
  motor_control(power);
}
void calculate_pid_circle(int power) {
  error = findError();
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (0.35 * P) + (0.5 * I) + (1 * D);
  previous_I = I;
  previous_error = error;
}
void Pid_Circle(int power) {
  calculate_pid_circle(power);
  motor_control(power);
}
void PidTime(int power, unsigned int Time) {
  currentTime = millis();
  while (millis() - currentTime < Time) {
    Pid(power);
  }
}
void Track() {
  Pid(60);
}
void TrackSlow() {
  Pid(60);
}
void TrackCan() {
  Pid(20);
}
void TrackSlowSmooth() {
  /*
  if (!(S_R < Ref_R && S_RR < Ref_RR) && !(S_R < Ref_R && S_RR < Ref_RR)){
    Pid(70);
  }
  else {
    motor(1, 50); motor(2, 50);
  }
  */
  TrackFF();
}
void TrackFF() {
  if (S_LL < Ref_LL) {
    motor(1, 30); motor(2, 35);
  }
  else if (S_L < Ref_L) {
    motor(1, 30); motor(2, 40);
  }
  else if (S_R < Ref_R) {
    motor(1, 40); motor(2, 30);
  }
  else if (S_RR < Ref_RR) {
    motor(1, 35); motor(2, 30);
  }
  else {
    motor(1, 30); motor(2, 30);
  }
}
void TrackTime(int Time) {
  int looptime = 0;
  while (looptime < Time) {
    Track();
    delay(1);
    looptime++;
  }
}
void TrackSlowTime(int Time) {
  int looptime = 0;
  while (looptime < Time) {
    TrackSlow();
    delay(1);
    looptime++;
  }
}
void TrackCanTime(int Time) {
  int looptime = 0;
  while (looptime < Time) {
    TrackCan();
    delay(1);
    looptime++;
  }
}
void TrackSlowSmoothTime(int Time) {
  int looptime = 0;
  while (looptime < Time) {
    TrackSlowSmooth();
    delay(1);
    looptime++;
  }
}
void TR90() {
  Stop(50);
  motor(1, Slow_L); motor(2, -(Slow_R + 8)); delay(100);
  // while (S_R <= Ref_R) {
  //   motor(1, Slow_L);
  //   motor(2, -(Slow_R + 10));
  // }
  while (S_R > Ref_R) {
    motor(1, Slow_L);
    motor(2, -(Slow_R + 10));
  }
}
void TL90() {
  Stop(50);
  motor(1, -(Slow_L + 8)); motor(2, Slow_R); delay(120);
  // while (S_L <= Ref_L) {
  //   motor(1, -(Slow_L + 10));
  //   motor(2, Slow_R);
  // }
  while (S_L > Ref_L) {
    motor(1, -(Slow_L + 10));
    motor(2, Slow_R);
  }
}
void U90(){
  motor(1, -40);  motor(2, 40); delay(200);
  TL90();
  TL90();
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
  {
    while ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
    {
      Track();
    }
  }
  TrackTime(50);
  // Stop(30);
}
void FF(int i) {
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
    TrackSlowTime(50);
  }
  while (1) {
    if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
      if (i == 1) {
        Fw(FT1);
        TrackSlowTime(timeOutLineFT1);
      }
      else if (i == 2) {
        Fw(FT2);
        TrackSlowTime(timeOutLineFT2);
      }
      else if (i == 3) {
        Fw(FT3);
        TrackSlowTime(timeOutLineFT3);
      }
      else {
        Fw(FT4);
        TrackSlowTime(timeOutLineFT4);
      }
      break;
    }
    Pid(75);
  }
  // Wait();
}
void LL(int i, int j) { //left
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
    while(S_LLL <= Ref_LLL || S_RRR <= Ref_RRR){TrackSlow();}
    TrackSlowTime(20);
  }
  if (i == 1) { //มุมฉาก
    while (1) {
      if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T1);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT1);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 2) { //มุมแหลม
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T2);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT2);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 3) { //มุมแหลมแบบคู่
    // while (1) {
    //   if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
    //   {
    //     break;
    //   }
    //   Track();
    // }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T3);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT3);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 4) { //มุมฉากธรรมดา
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T4);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT4);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 5) { //มุมฉากธรรมดา
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T4);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT5);
        break;
      }
      TrackSlowSmooth();
    }
  }
  else if (i == 6) { //เลี้ยวก่อนเข้าจุดวาง
    // while (1) {
    //   if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
    //   {
    //     break;
    //   }
    //   Track();
    // }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T3);
        Log(logTime);
        if (j == 1) {
          TL90();
        }
        else if (j == 2) {
          TL90();
          TL90();
        }
        else {
          TL90();
          TL90();
          TL90();
        }
        Stop(20);
        TrackCanTime(timeOutLineT5);
        break;
      }
      TrackSlow();
    }
  }
  // Wait();
}
void RR(int i, int j) { //right
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
    while(S_LLL <= Ref_LLL || S_RRR <= Ref_RRR){TrackSlow();}
    TrackSlowTime(20);
  }

  if (i == 1) { //มุมฉาก
    while (1) {
      if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T1);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT1);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 2) { //มุมแหลม
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T2);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT2);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 3) { //มุมแหลมแบบคู่
    // while (1) {
    //   if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
    //   {
    //     break;
    //   }
    //   Track();
    // }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T3);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT3);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 4) { //มุมฉากธรรมดา
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T4);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT4);
        break;
      }
      TrackSlow();
    }
  }
  else if (i == 5) { //มุมฉากธรรมดา
    while (1) {
      if ((S_LL < Ref_LL) || (S_RR < Ref_RR))
      {
        break;
      }
      Track();
    }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T4);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackSlowTime(timeOutLineT5);
        break;
      }
      TrackSlowSmooth();
    }
  }
  else if (i == 6) { //เลี้ยวก่อนเข้าจุดวาง
    // while (1) {
    //   if ((S_LL < Ref_LL) && (S_RR < Ref_RR))
    //   {
    //     break;
    //   }
    //   Track();
    // }
    while (1) {
      if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
      {
        Fw(T3);
        Log(logTime);
        if (j == 1) {
          TR90();
        }
        else if (j == 2) {
          TR90();
          TR90();
        }
        else {
          TR90();
          TR90();
          TR90();
        }
        Stop(20);
        TrackCanTime(timeOutLineT5);
        break;
      }
      TrackSlow();
    }
  }
  // Wait();
}
void InCan(int i) {
  // boolean flagCan = true;
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
    TrackSlowTime(30);
  }
  if (i != 4) {
    TrackSlowTime(50);
  }
  while (1) {
    if (getdist(S_Can) < SS_Can) //((S_L < Ref_L) && (S_R < Ref_R)) //if (getdist(S_Can) < SS_Can)
    {
      //  CheckCan
      Stop(30);
      if (i == 1) {
        Keep();
        Stop(20);
        motor(1, -40);  motor(2, 40); delay(200);
        TL90();
        Stop(20);
      }
      else if (i == 2) {
        Keep();
        while ((S_L < Ref_L) || (S_R < Ref_R)) {
          Fw(1);
        }
        Stop(30);
        motor(1, -40);  motor(2, 40); delay(200);
        TL90();
        TL90();
        Stop(20);
        if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
          while ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
            Track();
          }
        }
        TrackTime(50);
        // Stop(50);
      }
      else if (i == 3) {
        Keep();
        Stop(20);
        motor(1, 40);  motor(2, -40); delay(200);
        TR90();
        Stop(20);
      }
      else if (i == 4){
        Keep();
        Stop(20);
        motor(1, 40);  motor(2, -40); delay(200);
        TR90();
        Stop(20);
      }
      else{
        Keep();
        Stop(20);
      }
      TrackSlowTime(50);
      break;
    }
    Pid(37);
  }
  // Wait();
}
void PlaceCan(String s) {
    
    Stop(100);
    Place();
      
    motor(1, -30);  motor(2, -30); delay(180);
    Stop(30);
    servo(Clasp, Clasp_Set);  delay(300);
    Stop(70);
    if (s == "R") {
        motor(1, -45);  motor(2, 45); delay(250);
        TL90();
        Stop(20);
      }
    else {
        motor(1, 45);  motor(2, -45); delay(250);
        TR90();
        Stop(20);
      }
    // servo(Clasp, Clasp_Set);  delay(50);
    while(S_LLL <= Ref_LLL || S_RRR <= Ref_RRR){TrackSlowSmooth();}
    TrackSlowSmoothTime(10);
    
}
void Start() {
  motor(1, Slow_R); motor(2, Slow_L);
  delay(580);
}
void Finish() {
  if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
    while ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR)) {
      TrackSlow();
    }
  }
  while (1) {
    if ((S_LLL < Ref_LLL) || (S_RRR < Ref_RRR))
    {
      motor(1, 25);  motor(2, 25); delay(350);
      Stop(20);
      break;
    }
    TrackSlow();
  }
}

void Wait() {
  while (SW_OK() == true) {
    Stop(10);
  }
  while (SW_OK() == false) {

  }
  beep();
}
void ok() {
  XIO();
  servo(Clasp, Clasp_Set); delay(200);
  servo(Raise, Raise_Down); delay(200);

  while (SW_OK() == 1) {
    function = knob(0, 6);
    setTextSize(5);
    oled(50, 15, "%d", function);
    oledClear();
    if (SW_OK() == 0)
    {
      setTextSize(1);
      beep();
      break;
    }
  }
  oledClear();
}
void readServo() {
  int p, d;
  delay(800);
  while (SW_OK() == 1) {
    p = knob(1, 3);
    oled(0, 0, "Port = %d ", p);
    oledClear();
  }
  delay(500);
  while (SW_OK() == 1) {
    d = knob(1, 180);
    servo(p, d);
    oled(0, 0, "Port = %d ", p);
    oled(0, 15, "results = %d ", d);
    oledClear();
    delay(20);
  }
}
void setCan(){
  while (SW_OK() == false) {}
    while (true) {
      int v = knob(1, 16);

      if (v == 1) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 1, knobvalue);
            can1 = EEPROM.read(startCanAddress + 1);

            oled(0, 0,  ">C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  ">C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 2) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 2, knobvalue);
            can2 = EEPROM.read(startCanAddress + 2);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  ">C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  ">C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 3) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 3, knobvalue);
            can3 = EEPROM.read(startCanAddress + 3);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  ">C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  ">C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 4) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 4, knobvalue);
            can4 = EEPROM.read(startCanAddress + 4);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  ">C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  ">C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 5) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 5, knobvalue);
            can5 = EEPROM.read(startCanAddress + 5);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  ">C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  ">C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 6) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 6, knobvalue);
            can6 = EEPROM.read(startCanAddress + 6);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  ">C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  ">C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 7) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 7, knobvalue);
            can7 = EEPROM.read(startCanAddress + 7);

            oled(0, 0,  " C1: %d >C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d >C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 8) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 8, knobvalue);
            can8 = EEPROM.read(startCanAddress + 8);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d >C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d >C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 9) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 9, knobvalue);
            can9 = EEPROM.read(startCanAddress + 9);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d >C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d >C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 10) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 10, knobvalue);
            can10 = EEPROM.read(startCanAddress + 10);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d >C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d >C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 11) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 11, knobvalue);
            can11 = EEPROM.read(startCanAddress + 11);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d >C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d >C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v == 12) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 12, knobvalue);
            can12 = EEPROM.read(startCanAddress + 12);

            oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d >C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d  C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d >C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
      if (v >= 13) {
        if (SW_OK() == false) {
          beep();
          while (SW_OK() == false) {}

          while (SW_OK() == true) {
            int knobvalue = knob(0, 3);
            if (knobvalue > 2) {
              knobvalue = 2;
            }
            EEPROM.update(startCanAddress + 13, knobvalue);
            can13 = EEPROM.read(startCanAddress + 13);

            oled(0, 0,  " C1: %d  C7: %d >C13: %d", can1, can7, can13);
            oled(0, 10,  " C2: %d  C8: %d", can2, can8);
            oled(0, 20,  " C3: %d  C9: %d", can3, can9);
            oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
            oled(0, 40,  " C5: %d  C11: %d", can5, can11);
            oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
            delay(150);
            oledClear();
          }
          beep();
          while (SW_OK() == false) {}
        }
        oled(0, 0,  " C1: %d  C7: %d >C13: %d", can1, can7, can13);
        oled(0, 10,  " C2: %d  C8: %d", can2, can8);
        oled(0, 20,  " C3: %d  C9: %d", can3, can9);
        oled(0, 30,  " C4: %d  C10: %d", can4 , can10);
        oled(0, 40,  " C5: %d  C11: %d", can5, can11);
        oled(0, 50,  " C6: %d  C12: %d ", can6, can12);
        delay(150);
        oledClear();
      }
    }

    can1 = EEPROM.read(startCanAddress + 1);
    can2 = EEPROM.read(startCanAddress + 2);
    can3 = EEPROM.read(startCanAddress + 3);
    can4 = EEPROM.read(startCanAddress + 4);
    can5 = EEPROM.read(startCanAddress + 5);
    can6 = EEPROM.read(startCanAddress + 6);
    can7 = EEPROM.read(startCanAddress + 7);
    can8 = EEPROM.read(startCanAddress + 8);
    can9 = EEPROM.read(startCanAddress + 9);
    can10 = EEPROM.read(startCanAddress + 10);
    can11 = EEPROM.read(startCanAddress + 11);
    can12 = EEPROM.read(startCanAddress + 12);
    can13 = EEPROM.read(startCanAddress + 13);
}

void setServo(){
  
  int count = 1;
  int TimeOK = 0;
  
  while(true){
    TimeOK=0;
    if (count == 1){
      servo(Clasp,knob(180));
      oled(0,0, "1. Clasp_Keep: %d ", knob(180));
    }
    else if (count == 2){
      servo(Clasp,knob(180));
      oled(0,0, "2. Clasp_Place: %d ", knob(180));
    }
    else if (count == 3){
      servo(Clasp,knob(180));
      oled(0,0, "3. Clasp_Set: %d ", knob(180));
    }
    else if (count == 4){
      servo(Raise,knob(180));
      oled(0,0, "4. Raise_Up: %d ", knob(180));
    }
    else if (count == 5){
      servo(Raise,knob(180));
      oled(0,0, "5. Raise_Down: %d ", knob(180));
    }
    if (SW_OK() == false){
      
      while(1){

        if (SW_OK() == false){
          TimeOK++;
        }
        else{
          beep();
         if (count < 5){
          count++;
          break;
          } else {
            count=1;
            break;
          }  
        }

        if (TimeOK > 500){
          beep();
          if (count == 1){
            EEPROM.update(startServoSetAddress + 1 , knob(180) );
          }
          else if(count == 2){
            EEPROM.update(startServoSetAddress + 2 , knob(180) );
          }
          else if(count == 3){
            EEPROM.update(startServoSetAddress + 3 , knob(180) );
          }
          else if(count == 4){
            EEPROM.update(startServoSetAddress + 4 , knob(180) );
          }
          else if(count == 5){
            EEPROM.update(startServoSetAddress + 5 , knob(180) );
          }
          oledClear();
          oled(5,10,"Save! ");
          while(SW_OK() == false){}
          delay(500);
          beep();
          break;
        }
        delay(1);
         // Time++;
      }
      
    }

    oledClear(); delay(50);
  }
  
}

void code() {
  start();
  
  Stop(100000000);
}
void setSensorHand(){
  int WHITE_COLOR=0;
  int BLACK_COLOR=0;
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"WHITE CAN");
    delay(20);
  }
  // while(SW_OK() == 0){}
  Keep();
  oledClear();
  beep();
  delay(500);
  ////
  WHITE_COLOR = S_CR; // EEPROM.update(startColorAddress + 1, S_CR);
  ////
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"SAVE!");
    oled(5,20,"WHITE CAN");
    oled(5,40,"-NEXT-");
    delay(20);
  }
  // while(SW_OK() == 0){}
  Place();
  oledClear();
  beep();
  delay(500);

 setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"BLACK CAN");
    delay(20);
  }
  // while(SW_OK() == 0){}
  Keep();
  oledClear();
  beep();
  delay(500);
  ////
  BLACK_COLOR = S_CR; // EEPROM.update(startColorAddress + 4, S_CR);
                // EEPROM.update(startColorAddress + 1, ((WHITE_COLOR + BLACK_COLOR) / 2 ));
  ////
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"SAVE!");
    oled(5,20,"BLACK CAN");
    oled(5,40,"-NEXT-");
    delay(20);
  }
  // while(SW_OK() == 0){}
  Place();
  oledClear();
  beep();
  delay(500);

  ////
  setTextSize(2);
  oled(10, 0,"ENJOY!");
  delay(20);
  beep();
  delay(500);
  Wait();
  oledClear();


}

void setSensorFloor(){
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"RED FLOOR");
    delay(20);
  }
  // while(SW_OK() == 0){};
  oledClear();
  beep();
  delay(500);
  ////
  EEPROM.update(startColorFloorAddress + 1, S_FR);
  EEPROM.update(startColorFloorAddress + 5, S_FG);
  ////
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"GREEN FLOOR");
    delay(20);
  }
  // while(SW_OK() == 0){};
  oledClear();
  beep();
  delay(500);
  ////
  EEPROM.update(startColorFloorAddress + 2, S_FR);
  EEPROM.update(startColorFloorAddress + 6, S_FG);
  ////
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"BLUE FLOOR");
    delay(20);
  }
  // while(SW_OK() == 0){};
  oledClear();
  beep();
  delay(500);
  ////
  EEPROM.update(startColorFloorAddress + 3, S_FR);
  EEPROM.update(startColorFloorAddress + 7, S_FG);
  ////
  setTextSize(2);
  delay(500);
  while (SW_OK() == 1) {
    oled(10, 0,"INPUT");
    oled(5,20,"BLACK FLOOR");
    delay(20);
  }
  // while(SW_OK() == 0){};
  oledClear();
  beep();
  delay(500);
  ////
  EEPROM.update(startColorFloorAddress + 4, S_FR);
  EEPROM.update(startColorFloorAddress + 8, S_FG);
  ////
  setTextSize(2);
  oled(10, 0,"Saved, ENJOY!");
  delay(20);
  beep();
  delay(500);
  Wait();
  oledClear();


}

void setSensor() {
  delay(500);
  int Max[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int Min[9] = {100, 100, 100, 100, 100, 100, 100, 100, 100};
  int i = 1;

  while (SW_OK() == 1) {
    if (S_LLL > Max[0]) {
      Max[0] = S_LLL;
      i = 1;
    }
    if (S_LLL < Min[0]) {
      Min[0] = S_LLL;
      i = 1;
    }

    if (S_LL > Max[1]) {
      Max[1] = S_LL;
      i = 1;
    }
    if (S_LL < Min[1]) {
      Min[1] = S_LL;
      i = 1;
    }

    if (S_L > Max[2]) {
      Max[2] = S_L;
      i = 1;
    }
    if (S_L < Min[2]) {
      Min[2] = S_L;
      i = 1;
    }

    if (S_C > Max[3]) {
      Max[3] = S_C;
      i = 1;
    }
    if (S_C < Min[3]) {
      Min[3] = S_C;
      i = 1;
    }

    if (S_R > Max[4]) {
      Max[4] = S_R;
      i = 1;
    }
    if (S_R < Min[4]) {
      Min[4] = S_R;
      i = 1;
    }

    if (S_RR > Max[5]) {
      Max[5] = S_RR;
      i = 1;
    }
    if (S_RR < Min[5]) {
      Min[5] = S_RR;
      i = 1;
    }

    if (S_RRR > Max[6]) {
      Max[6] = S_RRR;
      i = 1;
    }
    if (S_RRR < Min[6]) {
      Min[6] = S_RRR;
      i = 1;
    }

    if (i == 1) {
      oledClear();
      oled(0, 0,  "Sen  B  W |Sen  B  W");
      oled(0, 7,  "---------------------");
      oled(0, 13, "LLL %d %d", Min[0], Max[0]);
      oled(60, 13, "|RR    %d %d", Min[5], Max[5]);
      oled(0, 23, "LL  %d %d", Min[1], Max[1]);
      oled(60, 23, "|RRR   %d %d", Min[6], Max[6]);
      oled(0, 33, "L   %d %d", Min[2], Max[2]);
      // oled(60, 33, "|RRR  %d %d", Min[7], Max[7]);
      oled(0, 43, "C    %d %d", Min[3], Max[3]);
      // oled(60, 43, "|RRRR %d %d", Min[8], Max[8]);
      oled(0, 53, "R    %d %d", Min[4], Max[4]);
      delay(10);
      i = 0;
    }
  }
  beep();
  EEPROM.update(startReffAddress + 1, (Max[0] + Min[0]) / 2); // LLL
  EEPROM.update(startReffAddress + 2, (Max[1] + Min[1]) / 2); // LL
  EEPROM.update(startReffAddress + 3, (Max[2] + Min[2]) / 2); // L
  EEPROM.update(startReffAddress + 4, (Max[3] + Min[3]) / 2); // C
  EEPROM.update(startReffAddress + 5, (Max[4] + Min[4]) / 2); // R
  EEPROM.update(startReffAddress + 6, (Max[5] + Min[5]) / 2); // RR
  EEPROM.update(startReffAddress + 7, (Max[6] + Min[6]) / 2); // RRR
  // EEPROM.update(7, (Max[7] + Min[7]) / 2);
  // EEPROM.update(8, (Max[8] + Min[8]) / 2);
  oledClear();
  oled(0, 0,  "Sensor Ref|Sensor Ref");
  oled(0, 7,  "---------------------");
  oled(0, 13, "LLL   %d |RR      %d", EEPROM.read(startReffAddress + 1), EEPROM.read(startReffAddress + 6));
  oled(0, 23, "LL    %d |RRR     %d", EEPROM.read(startReffAddress + 2), EEPROM.read(startReffAddress + 7));
  oled(0, 33, "L     %d", EEPROM.read(startReffAddress + 3));
  oled(0, 43, "C      %d", EEPROM.read(startReffAddress + 4));
  oled(0, 53, "R      %d", EEPROM.read(startReffAddress + 5));
  while (1) {}
}
int readCan(){ // คำสั่งอ่านค่ากระป๋องด้วยเซนเซอร์มือ ออกมาเป็นตัวเลข 1 ขาว 0 ดำ
  
  // if (S_CR >= Ref_CR){
  //   return 1; // WHITE CAN
  // }
  // else{ 
  //   return 0; // YELLOW CAN
  // }
}

void RR_Circle(){
  while(S_R >= Ref_R || S_RR > Ref_RR)
  {
    Pid_Circle(40);
  }
  RR(4,1);
}
void LL_Circle(){
  while(S_L >= Ref_L || S_LL > Ref_LL)
  {
    Pid_Circle(40);
  }
  LL(4,1);
}