//Ver. 1.8

#define Zlmt 2                                             //PORTD 2
#define Ylmt 3                                             //PORTD 3
#define Xlmt 4                                             //PORTD 4
#define PWR 5                                              //PORTD 5
//PORTD 6
#define xdir 11                                            //PORTD 7    11
#define xstep 12                                           //PORTB 0    12
#define ydir 7                                             //PORTB 1    7
#define ystep 8                                            //PORTB 2    8
#define zdir 9                                             //PORTB 3    9
#define zstep 10                                           //PORTB 4    10
#define xendl  0                                           // left side pushing the lmt switch
#define yendl  0
#define zdwend 0
#define xendr 26000                                        //26000
#define yendr 36000
#define zupend 10500
#define xoffset 0
#define yoffset 20
#define zoffset 0
#define CW true
#define CCW false
#define XINC 0
#define XDEC 1
#define YINC 2
#define YDEC 3
#define ZINC 4
#define ZDEC 5
#define VERBOSE  //comment uncomment this line to switch off/on printing more then necessary info

#include <math.h>

bool homingenabled = false;
//bool linIP = true;
//bool circIP = true;
bool inc = false;
bool dbg_enbld = false;
bool moveon = false;
byte movemode = 0;                         //G00/G01/G02/G03 are modal commands
long px = xendr, py = yendr, pz = 0;       //Present position x,y,z
float hfstpdly = 1.75; //stepdelay = 3.5;  //half stepdelay =...stepdelay/2
const float stppmm = 160;

int i = 0, starti = 0, endi = 0;
int Feed = -1, spind_speed, gnum;
float xmm = 0, ymm = 0, zmm = zupend / stppmm;
float I_CXoffset, J_CYoffset;
char datain;
char convbuffer[16];

String NUMdata;
String buffer = "";
String splitbuffer[16];

void setup() {
  Serial.begin(9600);                                        // Open serial connection.
  Serial.println(F("Serial start"));
  DDRD = B10100010;                                        // 0 - input 1 - output
  DDRB = B10111111;
  digitalWrite(PWR, 1);
  Serial.println(F("POWER ON"));
  homeall();
}
//********************************************************************************************************************
void intpolcirc(bool dir, float x, float y, float z, float i, float j) {
  String dbgtxt;
  long ex = px + (x * stppmm + 0.5);
  long ey = py + (y * stppmm + 0.5);
  long ez = pz + (z * stppmm + 0.5);
  long cx = px + (i * stppmm + 0.5);
  long cy = py + (j * stppmm + 0.5);
  float rxr = sq(x * stppmm - i * stppmm) + sq(y * stppmm - j * stppmm);
  float r = sqrt(rxr);
  long lr = r + 0.5;

  if (abs((long)(sqrt(sq(cx - px) + sq(py - cy))) - (long)(sqrt(sq(ex - cx) + sq(ey - cy)))) > 2) { // check cx,cy,ex,ey validity
    dbgtxt = " dbg: WARNING: Wrong p-c-e distances!!!";
    dbgtxt += "\n\r dbg: r1(px,py-cx,cy): " + String((long)(sqrt(sq(cx - px) + sq(py - cy))));
    dbgtxt += ", r2(ex,ey-cx,cy): " + String((long)(sqrt(sq(ex - cx) + sq(ey - cy))));
    Serial.println(dbgtxt);
    // errhndlr(7);
  }

  long sx, sy; // start coordinates of arc may differ from px,py. Routine will correct for difference w/o intervention, but...

  if (abs(px - cx) <= lr) { //  if ((float)(sq(px - cx)) <= rxr) WAS WRONG!!!
    sx = px;
    if (py > cy)
      sy = cy + (long)(sqrt(sq((float)(lr)) - sq(sx - cx))); // sy = cy + (long)(sqrt(rxr - sq(sx - cx)));
    if (py < cy)
      sy = cy - (long)(sqrt(sq((float)(lr)) - sq(sx - cx))); // sy = cy - (long)(sqrt(rxr - sq(sx - cx)));
    if (py == cy)
      sy = cy;
  }
  else {
    if (px > cx) {
      sx = cx + lr; // sx = cx + r;
    }
    else {
      sx = cx - lr; // sx = cx - r;
    }
    sy = cy;
  }

  long fx, fy; // final coordinates of arc

  if (abs(ex - cx) <= lr) { //  if ((float)(sq(ex - cx)) <= rxr) WAS WRONG!!!
    fx = ex;
    if (ey > cy)
      fy = cy + (long)(sqrt(sq((float)(lr)) - sq(fx - cx))); // fy = cy + (long)(sqrt(rxr - sq(fx - cx)));
    if (ey < cy)
      fy = cy - (long)(sqrt(sq((float)(lr)) - sq(fx - cx))); // fy = cy - (long)(sqrt(rxr - sq(fx - cx)));
    if (ey == cy)
      fy = cy;
  }
  else {
    if (ex > cx) {
      fx = cx + lr; // fx = cx + r;
    }
    else {
      fx = cx - lr; //fx = cx - r;
    }
    fy = cy;
  }

  float ln, lt; // ln is length of arc from 0 degrees to starting point sx,sy, lt is temporary length of arc from 0 to fx,fy and total (real) length of arc

  //calculate length of arc from 0 degrees to starting points sx,sy:
  if ((sx - cx) / lr > 1.0) // eliminate rounding anomalities (sxcx distance greater than radius)
    ln = (float)(acos(1.0) * lr);
  else if ((sx - cx) / lr < -1.0)
    ln = (float)(acos(-1.0) * lr);
  else
    ln = (float)(acos((sx - cx) / lr) * lr); // length of null arc from 0 degrees to angle defined by start points px,py --> ln = acos(delta pxcx / r) * r
  if (sy < cy)
    ln = 2 * lr * PI - ln; // modify length if sy < cy (in III and IV quadrants), ln = 2rPI - acos(delta sxcx / r) * r
  // similary calculate length of arc from 0 degrees to final points fx,fy:
  if ((fx - cx) / lr > 1.0)
    lt = (float)(acos(1.0) * lr);
  else if ((fx - cx) / lr < -1.0)
    lt = (float)(acos(-1.0) * lr);
  else
    lt = (float)(acos((fx - cx) / lr) * lr); // length total (from 0 to final point for now ) --> lt = acos(delta fxcx / r) * r
  if (fy < cy)
    lt = 2 * lr * PI - lt;
  // calculate arc length as lt=lt-ln, but account for dir and CW/CCW
  if (dir == CCW) { // if dir ir CCW
    lt -= ln; // length total equals length from 0 to final point minus length from 0 to start point
    if (lt < 0) // but...
      lt += 2 * lr * PI; // modify length total if less than zero (eg: ln is in IV q)
  }
  else { // if dir == CW
    lt = ln - lt; // total length equals length from 0 to start point minus length from 0 to final point
    if (lt < 0) // but...
      lt += 2 * lr * PI; // modify if necessary
  }
  if (fx == px && fy == py) { // spec. case full circle
    // ln = 0;
    lt = 2 * lr * PI;
  }

  float a = 0; // z to l coefficient

  if (lt > 0) // avoid div. by zero
    a = (float)(ez - pz) / lt;

#ifdef VERBOSE
  dbgtxt = " dbg: px: " + String(px);
  dbgtxt += "\n\r dbg: py: " + String(py);
  dbgtxt += "\n\r dbg: pz: " + String(pz);
  dbgtxt += "\n\r dbg: sx: " + String(sx);
  dbgtxt += "\n\r dbg: sy: " + String(sy);
  Serial.println(dbgtxt);
  dbgtxt = " dbg: cx: " + String(cx);
  dbgtxt += "\n\r dbg: cy: " + String(cy);
  dbgtxt += "\n\r dbg: fx: " + String(fx);
  dbgtxt += "\n\r dbg: fy: " + String(fy);
  dbgtxt += "\n\r dbg: ex: " + String(ex);
  dbgtxt += "\n\r dbg: ey: " + String(ey);
  dbgtxt += "\n\r dbg: ez: " + String(ez);
  Serial.println(dbgtxt);
  dbgtxt = " dbg: rxr(ec): " + String(rxr, 6);
  dbgtxt += "\n\r dbg: rxr(pc): " + String((float)(sq(i * stppmm) + sq(j * stppmm)), 6);
  dbgtxt += "\n\r dbg: r: " + String(r, 6);
  dbgtxt += "\n\r dbg: lr: " + String(lr);
  dbgtxt += "\n\r dbg: ln: " + String(ln, 6);
  dbgtxt += "\n\r dbg: lt: " + String(lt, 6);
  dbgtxt += "\n\r dbg: a: " + String(a, 6);
  Serial.println(dbgtxt);
  if (sx != px) {
    dbgtxt = " dbg: px != sx: px, sx: " + String(px) + ", " + String(sx);
    dbgtxt += " CORRECTING X: " + String(px - sx) + " steps!!!";
    Serial.println(dbgtxt);
  }
  if (sy != py) {
    dbgtxt = " dbg: py != sy: py, sy: " + String(py) + ", " + String(sy);
    dbgtxt += " CORRECTING Y: " + String(py - sy) + " steps!!!";
    Serial.println(dbgtxt);
  }
#endif
  if (lt <= 0) {
    Serial.println(" dbg: WARNING: lt <=0!!! Setting it to 2*r*PI");
    lt = 2 * lr * PI; // CAUTION!!!
  }
  char s = 0; // s is sign +1 or -1 WARNING s=0 (undefined) intentionaly!!!
  long ny; // ny is absolute next py
  float lp; // lp is present length of arc (temporarily from 0 degrees, then from ln)

  // After lenghty preparations do the job!
  while (true) {
    // determine quadrant, step on x, set sign +/-1
    if (dir == CW && px >= cx && py > cy) { // 1st quadrant CW
      stepaxisdir(XINC);
      s = 1;
    }
    else if (dir == CW && px < cx && py >= cy) { // 2nd quadrant CW
      stepaxisdir(XINC);
      s = 1;
    }
    else if (dir == CW && px <= cx && py < cy) { // 3rd quadrant CW
      stepaxisdir(XDEC);
      s = -1;
    }
    else if (dir == CW && px > cx && py <= cy) { // 4th quadrant CW
      stepaxisdir(XDEC);
      s = -1;
    }
    else if (dir == CCW && px > cx && py >= cy) { // 1th quadrant CCW
      stepaxisdir(XDEC);
      s = 1;
    }
    else if (dir == CCW && px <= cx && py > cy) { // 2nd quadrant CCW
      stepaxisdir(XDEC);
      s = 1;
    }
    else if (dir == CCW && px < cx && py <= cy) { // 3rd quadrant CCW
      stepaxisdir(XINC);
      s = -1;
    }
    else if (dir == CCW && px >= cx && py < cy) { // 4th quadrant CCW
      stepaxisdir(XINC);
      s = -1;
    }
    if (px == fx && py == fy) // sometimes this happens between steping on x and y!
      break;
    // calculate ny for px:
    if ((float)(sq(px - cx)) <= sq((float)(lr))) // avoid sqrt of negative numbers!!!
      ny = (long)(sqrt(sq((float)(lr)) - sq(px - cx))); // calculate ny relative to cy
    else
      ny = 0;
    ny = s * ny + cy; // calculate absolute ny
    // Now multistep on y and z
    while (py != ny) {
      // calculate length of arc: pl=present length. First fom 0 degrees to present position, then from ln null length
      if ((px - cx) / lr > 1.0) // sometimes this happens, but why!!!
        lp = (float)(acos(1.0) * lr);
      else if ((px - cx) / lr < -1.0) // sometimes may happen!!!
        lp = (float)(acos(-1.0) * lr);
      else // generally
        lp = (float)(acos((px - cx) / lr) * lr); // l-present --> lp = acos(delta pxcx / r) * r
      if (s == -1) // if in quadrants III or IV CAUTION: if(py < cy) IS WRONG HERE!!!
        lp = 2 * lr * PI - lp; //if py < cy lp = 2rPI - acos(delta pxcx / r) * r
      if (dir == CCW) {
        lp -= ln;
        if (lp < 0)
          lp += 2 * lr * PI;
      }
      else { // if dir == CW
        lp = ln - lp;
        if (lp < 0)
          lp += 2 * lr * PI;
      }
      if (lp > lt) {
        dbgtxt = " dbg: WARNING: lp > lt!!!\n\r dbg: px,py,lp,lt: " + String(px) + ", " + String(py) + ", " + String(lp, 6) + ", " + String(lt, 6);
        Serial.println(dbgtxt);
        lt = lp; // CAUTION!!!
        // errhndlr(8);
      }
      while ((ez - pz != (long)(a * (lt - lp))) && (lp != 0.0)) { // lp may be 0.0 at full circles!!!
        if (ez - pz > (long)(a * (lt - lp)))
          stepaxisdir(ZINC);
        if (ez - pz < (long)(a * (lt - lp)))
          stepaxisdir(ZDEC);
      }
      if (py > ny)
        stepaxisdir(YDEC);
      else
        stepaxisdir(YINC);
      if (px == fx && py == fy) // test at the end of while because of full circles!
        break;
    }
    if (px == fx && py == fy) // test once more at the end of while because of full circles!
      break;
  }
  // correct from cex to ex and cey to ey if necessary:
  if (px != ex) {
#ifdef VERBOSE
    dbgtxt = " dbg: px != ex: px, ex: " + String(px) + ", " + String(ex);
    dbgtxt += " CORRECTING X: " + String(ex - px) + " steps!!!";
    Serial.println(dbgtxt);
#endif
    while (px != ex) {
      if (px < ex)
        stepaxisdir(XINC);
      else
        stepaxisdir(XDEC);
    }
  }
  if (py != ey) {
#ifdef VERBOSE
    dbgtxt = " dbg: py != ey: py, ey: " + String(py) + ", " + String(ey);
    dbgtxt += " CORRECTING Y: " + String(ey - py) + " steps!!!";
    Serial.println(dbgtxt);
#endif
    while (py != ey) {
      if (py < ey)
        stepaxisdir(YINC);
      else
        stepaxisdir(YDEC);
    }
  }
  if (pz != ez) {
#ifdef VERBOSE
    dbgtxt = " dbg: pz != ez: pz, ez: " + String(pz) + ", " + String(ez);
    dbgtxt += " CORRECTING Z: " + String(ez - pz) + " steps!!!";
    Serial.println(dbgtxt);
#endif
    while (pz != ez) {
      if (pz < ez)
        stepaxisdir(ZINC);
      else
        stepaxisdir(ZDEC);
    }
  }
#ifdef VERBOSE
  dbgtxt = " dbg: Now at: px,py,pz: " + String(px);
  dbgtxt += ", " + String(py);
  dbgtxt += ", " + String(pz);
  dbgtxt += ", " + (String((float)(px) / (float)(stppmm), 3));
  dbgtxt += ", " + (String((float)(py) / (float)(stppmm), 3));
  dbgtxt += ", " + (String((float)(pz) / (float)(stppmm), 3));
  Serial.println(dbgtxt);
#endif
}
//********************************************************************************************************************
void intpolline(float x, float y, float z) {
  long ex = px + (x * stppmm + 0.5); //end x= present x + float X (G00/G01...X...) in mms converted to steps
  long ey = py + (y * stppmm + 0.5); //end y= present y + Y
  long ez = pz + (z * stppmm + 0.5); //end z= present z + Z
  // General equation of line: dy=a*dx
  // in C: ey-py=a*(ex-px)
  // as we have three axes, we need two coefficients
  float a = 0; // py+=a*(ex-px)...
  float b = 0; // pz+=b*(ex-px)...
  String dbgtxt;
#ifdef VERBOSE
  dbgtxt = " dbg: px: " + String(px);
  dbgtxt += "\n\r dbg: py: " + String(py);
  dbgtxt += "\n\r dbg: pz: " + String(pz);
  dbgtxt += "\n\r dbg: ex: " + String(ex);
  dbgtxt += "\n\r dbg: ey: " + String(ey);
  dbgtxt += "\n\r dbg: ez: " + String(ez);
#endif
  //find the longest distance dx,dy or dz
  if ((abs(ex - px) != 0) && (abs(ex - px) >= abs(ey - py)) && (abs(ex - px) >= abs(ez - pz))) { //dx the greatest distance and not zero (avoid div/zero!!!)
    a = (y / x); //calculate coefficients for the other two lines
    b = (z / x);
#ifdef VERBOSE
    dbgtxt += "\n\r dbg: a=y/x: " + String(a, 6);
    dbgtxt += "\n\r dbg: b=z/x: " + String(b, 6);
    Serial.println(dbgtxt);
#endif
    while (ex != px) { //while not at end x
      if (ex > px) //step on x axis toward end x
        stepaxisdir(XINC);
      else
        stepaxisdir(XDEC);
      while (((ey - py) != (long)(a * (ex - px))) || ((ez - pz) != (long)(b * (ex - px)))) { //while other two axes not at their calculated positions
        if ((ey - py) > (long)(a * (ex - px))) //step on y axis toward calculated position
          stepaxisdir(YINC);
        if (ey - py < (long)(a * (ex - px)))
          stepaxisdir(YDEC);
        if (ez - pz > (long)(b * (ex - px))) //step on z axis toward calculated position
          stepaxisdir(ZINC);
        if (ez - pz < (long)(b * (ex - px)))
          stepaxisdir(ZDEC);
      }
    }
  }
  else if ((abs(ey - py) != 0) && (abs(ey - py) >= abs(ex - px)) && (abs(ey - py) >= abs(ez - pz))) { //dy the greatest distance and not zero (avoid div/zero!!!)
    a = (x / y); //calculate coefficients for the other two lines
    b = (z / y);
#ifdef VERBOSE
    dbgtxt += "\n\r dbg: a=x/y: " + String(a, 6);
    dbgtxt += "\n\r dbg: b=z/x: " + String(b, 6);
    Serial.println(dbgtxt);
#endif
    while (ey != py) { //while not at end y
      if (ey > py) //step on y axis toward end y
        stepaxisdir(YINC);
      else
        stepaxisdir(YDEC);
      while (((ex - px) != (long)(a * (ey - py))) || ((ez - pz) != (long)(b * (ey - py)))) { //while other two axes not at their calculated positions
        if ((ex - px) > (long)(a * (ey - py))) //step on x axis toward calculated position
          stepaxisdir(XINC);
        if (ex - px < (long)(a * (ey - py)))
          stepaxisdir(XDEC);
        if (ez - pz > (long)(b * (ey - py))) //step on z axis toward calculated position
          stepaxisdir(ZINC);
        if (ez - pz < (long)(b * (ey - py)))
          stepaxisdir(ZDEC);
      }
    }
  }
  else if ((abs(ez - pz) != 0) && (abs(ez - pz) >= abs(ex - px)) && (abs(ez - pz) >= abs(ey - py))) { //dz the greatest distance and not zero (avoid div/zero!!!)
    a = (x / z); //calculate coefficients for the other two lines
    b = (y / z);
#ifdef VERBOSE
    dbgtxt += "\n\r dbg: a=x/z: " + String(a, 6);
    dbgtxt += "\n\r dbg: b=y/z: " + String(b, 6);
    Serial.println(dbgtxt);
#endif
    while (ez != pz) { //while not at end z
      if (ez > pz) //step on z axis toward end z
        stepaxisdir(ZINC);
      else
        stepaxisdir(ZDEC);
      while (((ex - px) != (long)(a * (ez - pz))) || ((ey - py) != (long)(b * (ez - pz)))) { //while other two axes not at their calculated positions
        if ((ex - px) > (long)(a * (ez - pz))) //step on x axis toward calculated position
          stepaxisdir(XINC);
        if (ex - px < (long)(a * (ez - pz)))
          stepaxisdir(XDEC);
        if (ey - py > (long)(b * (ez - pz))) //step on y axis toward calculated position
          stepaxisdir(YINC);
        if (ey - py < (long)(b * (ez - pz)))
          stepaxisdir(YDEC);
      }
    }
  }
#ifdef VERBOSE
  dbgtxt = " dbg: Now at: px,py,pz: " + String(px);
  dbgtxt += ", " + String(py);
  dbgtxt += ", " + String(pz);
  dbgtxt += ", " + (String((float)(px) / (float)(stppmm), 3));
  dbgtxt += ", " + (String((float)(py) / (float)(stppmm), 3));
  dbgtxt += ", " + (String((float)(pz) / (float)(stppmm), 3));
  Serial.println(dbgtxt);
#endif
}
//********************************************************************************************************************
void errhndlr(byte reason) {
  switch (reason) {
    case 1: Serial.println(F("ER: XR+!!!"));
      break;
    case 2: Serial.println(F("ER: XL-!!!"));
      break;
    case 3: Serial.println(F("ER: YR+!!!"));
      break;
    case 4: Serial.println(F("ER: YL-!!!"));
      break;
    case 5: Serial.println(F("ER: ZU+!!!"));
      break;
    case 6: Serial.println(F("ER: ZD-!!!"));
      break;
    case 7: Serial.println(F("ER: CRC IJXY NOK!!!"));
      break;
    case 8: Serial.println(F("ER: lp>llt!!!"));
      break;
  }
  while (true) { //HALT!!!
    if (digitalRead(Xlmt)) {
      Serial.println(F("ER: MANUAL OVERRIDE !"));
      return;
    }
  }
}
//********************************************************************************************************************
void stepaxisdir(byte axdir) {
  //For SOFTWARE STEP ONLY uncomment break lines after x++ x-- y++ y-- z++ and z--!!
  switch (axdir) {
    case XINC: if (px == xendr)
        errhndlr(1);
      px++;
      break; // DEBUG ONLY!!!
      PORTB |= 0b00001000; //set dir x bit
      PORTB |= 0b00010000; //set step x bit
      delay(hfstpdly);
      PORTB &= 0b11101111; //clear step x bit
      delay(hfstpdly);
      break;
    case XDEC: if (px == 0)
        errhndlr(2);
      px--;
      break; // DEBUG ONLY!!!
      PORTB &= 0b11110111; //clear dir x bit
      PORTB |= 0b00010000; //set step x bit
      delay(hfstpdly);
      PORTB &= 0b11101111; //clear step x bit
      delay(hfstpdly);
      break;
    case YINC: if (py == yendr)
        errhndlr(3);
      py++;
      break; // DEBUG ONLY!!!
      PORTD |= 0b10000000;  //set dir y bit
      PORTB |= 0b00000001; //set step y bit
      delay(hfstpdly);
      PORTB &= 0b11111110; //clear step y bit
      delay(hfstpdly);
      break;
    case YDEC: if (py == 0)
        errhndlr(4);
      py--;
      break; // DEBUG ONLY!!!
      PORTD &= 0b01111111; //clear dir y bit
      PORTB |= 0b00000001; //set step y bit
      delay(hfstpdly);
      PORTB &= 0b11111110; //clear step y bit
      delay(hfstpdly);
      break;
    case ZINC: if (pz > zupend)
        errhndlr(5);
      pz++;
      break; // DEBUG ONLY!!!
      PORTB &= 0b11111101; //clear dir z bit INVERTED!!!
      PORTB |= 0b00000100; //set step z bit
      delay(hfstpdly);
      PORTB &= 0b11111011; //clear step z bit
      delay(hfstpdly);
      break;
    case ZDEC: if (pz < 0)     // 5
        errhndlr(6);
      pz--;
      break; // DEBUG ONLY!!!
      PORTB |= 0b00000010; //set dir z bit INVERTED!!!
      PORTB |= 0b00000100; //set step z bit
      delay(hfstpdly);
      PORTB &= 0b11111011; //clear step z bit
      delay(hfstpdly);
  }
  if (dbg_enbld) {
    Serial.print(px);
    Serial.print(F(", "));
    Serial.print(py);
    Serial.print(F(", "));
    Serial.print(pz);
    //Serial.print(F("X, Y, Z: "));
    Serial.print(F(", "));
    Serial.print(String((float)(px) / (float)(stppmm), 3));
    Serial.print(F(", "));
    Serial.print(String((float)(py) / (float)(stppmm), 3));
    Serial.print(F(", "));
    Serial.println(String((float)(pz) / (float)(stppmm), 3));
  }
}

void homeall() {                                           // home all axes
  Feed = 140;
  delay(1);
  if (homingenabled) {
    Serial.println(F("Starting homing..."));
    while (!(digitalRead(Zlmt))) {                        // 0 - x, 1 - step, 0 - dir
      stepaxisdir(ZINC);
    }
    while (!(digitalRead(Xlmt) && digitalRead(Ylmt))) {
      if (!(digitalRead(Xlmt))) {
        stepaxisdir(XDEC);
      }
      if (!(digitalRead(Ylmt))) {
        stepaxisdir(YDEC);
      }
    }
    Serial.println(F(" Finished homing..."));
  }
  else
    Serial.println(F(" dbg: homing is disabled in the firmware!"));
  px = 0;
  py = 0;
  pz = zupend;
  delay(100);
  Serial.println(F("RDY"));
}
//********************************************************************************************************************
void loop() {
  int word = 0;
  while (Serial.available() > 0) {                               // if data present
    buffer = "";
    buffer = Serial.readString();
    buffer.trim();                                    // useless line
    buffer.toUpperCase();                                // immune to uppercase letters
    Serial.println(" dbg:" + buffer);
    starti = buffer.lastIndexOf('<');
    endi = buffer.lastIndexOf('>');
    if ((starti >= 0) and (endi > 0) and (endi > starti)) {
      buffer = buffer.substring(starti + 1, endi);
      for (unsigned int i = 0; i < buffer.length(); i++) {
        if (buffer[i] == ' ' and buffer[i + 1] != ' ' ) {
          word++;
          i ++;
        }
        if ((buffer[i] != ' ') and (buffer[i] != '\t'))               // fckng tab
          splitbuffer[word] += buffer[i];
      }
      for (int i = 0; i < 16; i++) {
        if (splitbuffer[i] != "") {
          NUMdata = splitbuffer[i];                ///
          NUMdata = NUMdata.substring(1, NUMdata.length());
          NUMdata.toCharArray(convbuffer, 16);
          switch (splitbuffer[i][0]) {
            case 'M':                    // M^6 toolchange position
              switch ((byte)(atof(convbuffer))) {
                case 6:
                  /* xstack = targX;
                    ystack = targY;
                    zstack = targZ;
                    targX = 0;
                    targY = 0;
                    targZ = 10500;*/
                  break;
                default: Serial.println(F(" ER: Unknown M-code detected!"));
              }
              break;
            case 'G':                    // Set movemode, move_speed
              switch ((byte)(atof(convbuffer))) {
                case 0:
                  movemode = 0;
                  Serial.print(F(" Move:G00 "));
                  break;
                case 1:
                  movemode = 1;
                  Serial.print(F(" Move:G01 "));
                  break;
                case 2:
                  movemode = 2;
                  Serial.print(F(" Move:G02 "));
                  break;
                case 3:
                  movemode = 3;
                  Serial.print(F(" Move:G03 "));
                  break;
                case 90:
                  inc = false;
                  Serial.print(F(" Mode:ABS "));
                  break;
                case 91:
                  inc = true;
                  Serial.print(F(" Mode:INC "));
                  break;
                default: Serial.println(F(" ER: Unknown G-code detected!"));;
              }
              break;
            case 'X':
              xmm = (float)(atof(convbuffer));
              moveon = true;
              break;
            case 'Y':
              ymm = (float)(atof(convbuffer));
              moveon = true;
              break;
            case 'Z':
              zmm = (float)(atof(convbuffer));
              moveon = true;
              break;
            case 'I':
              I_CXoffset = (float)(atof(convbuffer));
              moveon = true;
              break;
            case 'J':
              J_CYoffset = (float)(atof(convbuffer));
              moveon = true;
              break;
            case 'F':
              Feed = atof(convbuffer);
              Serial.print(F(" Feed: "));
              Serial.print(Feed);
              break;
            case 'S':
              spind_speed = atof(convbuffer);
              Serial.print(F(" spind_speed: "));
              Serial.print(spind_speed);
              break;
            case 'D':
              dbg_enbld = !dbg_enbld;
              break;
            default:
              Serial.println(" ER: No match for cmd: " + splitbuffer[i]);
              break;
          }
        }
        splitbuffer[i] = "";
      }
      Serial.println("");
    }
    else {
      Serial.println(F(" ER: missing delimiters < >"));
      Serial.println("RPT");
    }
    if (moveon == true) {
      Serial.println(F(" moving..."));
      // Serial.println(movemode);
      if (movemode == 0)
        hfstpdly = 1.87; //stepdelay = 3.5
      else
        hfstpdly = 1200l * 60 / (2 * Feed * stppmm); //stepdelay = 1200 / ((Feed * 200) / (1.25 * 60));               // 1000 * 1.2
      switch (movemode) {
        case 0:
        case 1: if (inc)
            intpolline(xmm, ymm, zmm);
          else {
            /*Serial.print(" xmm: ");
              Serial.print(xmm);
              Serial.print(" px: ");
              Serial.print(px);
              Serial.print(" X: ");
              Serial.println(xmm - (px / stppmm));*/
            intpolline((float)(xmm - (px / stppmm)), (float)(ymm - (py / stppmm)), (float)( zmm - (pz / stppmm)));
          }
          break;
        case 2: if (inc)
            intpolcirc(CW, xmm, ymm, zmm, I_CXoffset, J_CYoffset);
          else {
            intpolcirc(CW, (float)(xmm - (px / stppmm)), (float)(ymm - (py / stppmm)), (float)(zmm - (pz / stppmm)), I_CXoffset, J_CYoffset);
          }
          break;
        case 3: if (inc)
            intpolcirc(CCW, xmm, ymm, zmm, I_CXoffset, J_CYoffset);
          else {
            intpolcirc(CCW, (float)(xmm - (px / stppmm)), (float)(ymm - (py / stppmm)), (float)(zmm - (pz / stppmm)), I_CXoffset, J_CYoffset);
          }
          break;
      }
      moveon = false;
      if (inc) {
        xmm = 0;
        ymm = 0;
        zmm = 0;
        I_CXoffset = 0;
        J_CYoffset = 0;
      }
      Serial.println(F("RDY"));
    }
    else {
      delay(10);
      Serial.println(F("RDY"));
    }
  }
}
