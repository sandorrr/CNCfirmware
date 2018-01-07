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
#define LED 5
#define transoffset 0.5                                        // transition between step patterns lin IP
#define CLR(x,y) (x&=(~(1<<y)))                                    // speed up with low level code
#define SET(x,y) (x|=(1<<y))
//#define stppmm 160					// 200 /1.25
#define CW true
#define CCW false

#include <math.h>

bool homeingenabled = false;
//bool linIP = true;
//bool circIP = true;
bool inc = false;
bool dbg_enbld = false;
//bool mov_speed = false;                                        // linear interpolation enabled
bool moveon = false;
byte movemode = 0;                         //G00/G01/G02/G03 are modal commands
long px = xendr, py = yendr, pz = 0; //Present position x,y,z =...xpos,ypos,zpos
float hfstpdly = 1.75; //stepdelay = 3.5; //half stepdelay =...stepdelay/2
const float stppmm = 160;

int i = 0, starti = 0, endi = 0;
int Feed = -1, spind_speed, gnum;
int circInt = 0;                                        //Circ interpolation 0- disabled
//             1 - CW
//             2 - CCW
// G1 false, G0 true
float xmm = 0, ymm = 0, zmm = zupend/stppmm;
float I_CXoffset, J_CYoffset;
//float xpos = -1, ypos = -1, zpos = -1;
//float targX = 0, targY = 0 , targZ = 0, xstack, ystack, zstack;
//float feed = 100;                                         // mm/min  x * 160 steps /min = x * 1 mm/min
//float stepdelay = 1200 / ((feed * 200) / (1.25 * 60));                    // 1000 * 1.2
//float newtarg;
//float xdist, ydist;

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
  long ex = px + x * stppmm;
  long ey = py + y * stppmm;
  long ez = pz + z * stppmm;
  long cx = px + i * stppmm;
  long cy = py + j * stppmm;
  float rxr = sq(i * stppmm) + sq(j * stppmm);
  float r = sqrt(rxr);
  if (abs((long)(sqrt(sq(cx - px) + sq(py - cy))) - (long)(sqrt(sq(ex - cx) + sq(ey - cy)))) > 1)
    errhndlr(7);
  rxr = round(rxr);
  long l,dy;
  if (ex == px && ey == py) //in case of full circle
    l = (long)(2 * r * PI); //l=2rPI
  else {
    l = (long)(acos((cx - ex) / r) * r); //acos(dx/r)*r
    if (ey < cy)
      l += (long)(r * PI);
    l -= (long)(acos((cx - px) / r) * r);
    if (py < cy)
      l -= (long)(r * PI);
  }
	Serial.print("dbg:rxr: ");
	Serial.print(rxr);
	Serial.print(" dbg:R: ");
	Serial.println(r);
	Serial.print(" dbg:EX: ");
	Serial.println(ex);
	Serial.print(" dbg:EY: ");
	Serial.println(ey);
	Serial.print(" dbg:CX: ");
	Serial.println(cx);
	Serial.print(" dbg:CY: ");
	Serial.println(cy);
  while (true) {
    if (dir == CW && px >= cx && py > cy) { // 1st quadrant CW
      stepnew(0);
/*
Serial.print("CX , PY, PX, PY, CX-PX, rxr: ");
Serial.print(cx);
Serial.print(", ");
Serial.print(cy);
Serial.print(", ");
Serial.print(px);
Serial.print(", ");
Serial.print(py);
Serial.print(", ");
Serial.print(cx-px);
Serial.print(", ");
Serial.println(rxr);
*/
      if(sq(cx - px) <= rxr)
        dy = (long)(sqrt(rxr - sq(cx - px))) + cy;
      else dy = cy;
      while (py != dy)
        stepnew(3);
    }
    else if (dir == CW && px < cx && py >= cy) { // 2nd quadrant CW
      stepnew(0);
      if(sq(cx - px) <= rxr)
        dy = (long)(sqrt(rxr - sq(cx - px))) + cy;
      else dy= cy;
      while (py != dy)
        stepnew(2);
    }
    else if (dir == CW && px <= cx && py < cy) { // 3rd quadrant CW
      stepnew(1);
      if(sq(cx - px) <= rxr)
        dy = cy - (long)(sqrt(rxr - sq(cx - px)));
      else dy= cy;
      while (py != dy)
        stepnew(2);
    }
    else if (dir == CW && px > cx && py <= cy) { // 4th quadrant CW
      stepnew(1);
      if(sq(cx - px) <= rxr)
        dy = cy - (long)(sqrt(rxr - sq(cx - px)));
      else dy= cy;
      while (py != dy)
        stepnew(3);
    }
    else if (dir == CCW && px > cx && py >= cy) { // 1th quadrant CCW
      stepnew(1);
      if(sq(cx - px) <= rxr) {
        dy = (long)(sqrt(rxr - sq(cx - px))) + cy;
        while (py != dy)
          stepnew(2);
      }
    }
    else if (dir == CCW && px <= cx && py > cy) { // 2nd quadrant CCW
      stepnew(1);
      if(sq(cx - px) <= rxr) {
        dy = (long)(sqrt(rxr - sq(cx - px))) + cy;
        while (py != dy)
          stepnew(3);
      }
    }
    else if (dir == CCW && px < cx && py <= cy) { // 3rd quadrant CCW
      stepnew(0);
      if(sq(cx - px) <= rxr) {
        dy = cy - (long)(sqrt(rxr - sq(cx - px)));
        while (py != dy)
          stepnew(3);
      }
    }
    else if (dir == CCW && px >= cx && py < cy) { // 4th quadrant CCW
      stepnew(0);
      if(sq(cx - px) <= rxr) {
        dy = cy - (long)(sqrt(rxr - sq(cx - px)));
        while (py != dy)
          stepnew(2);
      }
    }
if (px == ex){
  Serial.print("px == ex, py, ey: ");
  Serial.print(py);
  Serial.print(" ");
  Serial.println(ey);
}

    if ((px == ex) && abs(py - ey) < 2)
      break;
  }
}
//********************************************************************************************************************
void intpolline(float x, float y, float z) {
  long ex = px + (long)(x * stppmm); //end x= present x + float X (G00/G01...X...) in mms converted to steps
  long ey = py + (long)(y * stppmm); //end y= present y + Y
  long ez = pz + (long)(z * stppmm); //end z= present z + Z
  // General equation of line: dy=a*dx
  // in C: ey-py=a*(ex-px)
  // as we have three axes, we need two coefficients
  float a = 0; // py+=a*(ex-px)...
  float b = 0; // pz+=b*(ex-px)...
  //find the longest distance dx,dy or dz
  if ((abs(ex - px) != 0) && (abs(ex - px) >= abs(ey - py)) && (abs(ex - px) >= abs(ez - pz))) { //dx the greatest distance and not zero (avoid div/zero!!!)
    a = (y / x); //calculate coefficients for the other two lines
    b = (z / x);
    while (ex != px) { //while not at end x
      if (ex > px) //step on x axis toward end x
        stepnew(0);
      else
        stepnew(1);
      while (((ey - py) != (long)(a * (ex - px))) || ((ez - pz) != (long)(b * (ex - px)))) { //while other two axes not at their calculated positions
        if ((ey - py) > (long)(a * (ex - px))) //step on y axis toward calculated position
          stepnew(2);
        if (ey - py < (long)(a * (ex - px)))
          stepnew(3);
        if (ez - pz > (long)(b * (ex - px))) //step on z axis toward calculated position
          stepnew(4);
        if (ez - pz < (long)(b * (ex - px)))
          stepnew(5);
      }
    }
  }
  else if ((abs(ey - py) != 0) && (abs(ey - py) >= abs(ex - px)) && (abs(ey - py) >= abs(ez - pz))) { //dy the greatest distance and not zero (avoid div/zero!!!)
    a = (x / y); //calculate coefficients for the other two lines
    b = (z / y);
    while (ey != py) { //while not at end y
      if (ey > py) //step on y axis toward end y
        stepnew(2);
      else
        stepnew(3);
      while (((ex - px) != (long)(a * (ey - py))) || ((ez - pz) != (long)(b * (ey - py)))) { //while other two axes not at their calculated positions
        if ((ex - px) > (long)(a * (ey - py))) //step on x axis toward calculated position
          stepnew(0);
        if (ex - px < (long)(a * (ey - py)))
          stepnew(1);
        if (ez - pz > (long)(b * (ey - py))) //step on z axis toward calculated position
          stepnew(4);
        if (ez - pz < (long)(b * (ey - py)))
          stepnew(5);
      }
    }
  }
  else if ((abs(ez - pz) != 0) && (abs(ez - pz) >= abs(ex - px)) && (abs(ez - pz) >= abs(ey - py))) { //dz the greatest distance and not zero (avoid div/zero!!!)
    a = (x / z); //calculate coefficients for the other two lines
    b = (y / z);
    while (ez != pz) { //while not at end z
      if (ez > pz) //step on z axis toward end z
        stepnew(4);
      else
        stepnew(5);
      while (((ex - px) != (long)(a * (ez - pz))) || ((ey - py) != (long)(b * (ez - pz)))) { //while other two axes not at their calculated positions
        if ((ex - px) > (long)(a * (ez - pz))) //step on x axis toward calculated position
          stepnew(0);
        if (ex - px < (long)(a * (ez - pz)))
          stepnew(1);
        if (ey - py > (long)(b * (ez - pz))) //step on y axis toward calculated position
          stepnew(2);
        if (ey - py < (long)(b * (ez - pz)))
          stepnew(3);
      }
    }
  }
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
  }
  while (true){ //HALT!!!
	if(digitalRead(Xlmt)){
		Serial.println(F("ER: MANUAL OVERRIDE !"));
		return;
	}
  }
}
//********************************************************************************************************************
void stepnew(byte axdir) {
//For SOFTWARE STEP ONLY uncomment break lines after x++ x-- y++ y-- z++ and z--!!
  switch (axdir) {
    case 0: if (px == xendr)
        errhndlr(1);
      px++;
      //break;
      PORTB |= 0b00001000; //set dir x bit
      PORTB |= 0b00010000; //set step x bit
      delay(hfstpdly);
      PORTB &= 0b11101111; //clear step x bit
      delay(hfstpdly);
      break;
    case 1: if (px == 0)
        errhndlr(2);
      px--;
      //break;
      PORTB &= 0b11110111; //clear dir x bit
      PORTB |= 0b00010000; //set step x bit
      delay(hfstpdly);
      PORTB &= 0b11101111; //clear step x bit
      delay(hfstpdly);
      break;
    case 2: if (py == yendr)
        errhndlr(3);
      py++;
      //break;
      PORTD |= 0b10000000;  //set dir y bit
      PORTB |= 0b00000001; //set step y bit
      delay(hfstpdly);
      PORTB &= 0b11111110; //clear step y bit
      delay(hfstpdly);
      break;
    case 3: if (py == 0)
        errhndlr(4);
      py--;
      //break;
      PORTD &= 0b01111111; //clear dir y bit
      PORTB |= 0b00000001; //set step y bit
      delay(hfstpdly);
      PORTB &= 0b11111110; //clear step y bit
      delay(hfstpdly);
      break;
    case 4: if (pz > zupend)
        errhndlr(5);
      pz++;
      //break;
      PORTB &= 0b11111101; //clear dir z bit INVERTED!!!
      PORTB |= 0b00000100; //set step z bit
      delay(hfstpdly);
      PORTB &= 0b11111011; //clear step z bit
      delay(hfstpdly);
      break;
    case 5: if (pz < 0)			// 5
        errhndlr(6);
      pz--;
      //break;
      PORTB |= 0b00000010; //set dir z bit INVERTED!!!
      PORTB |= 0b00000100; //set step z bit
      delay(hfstpdly);
      PORTB &= 0b11111011; //clear step z bit
      delay(hfstpdly);
  }
	if(dbg_enbld){
 	 Serial.print(px);
  	Serial.print(F(", "));
  	Serial.print(py);
  	Serial.print(F(", "));
  	Serial.print(pz);
  	//Serial.print(F("X, Y, Z: "));
  	Serial.print(F(", "));
  	Serial.print((float)(px) / (float)(stppmm));
  	Serial.print(F(", "));
  	Serial.print((float)(py) / (float)(stppmm));
  	Serial.print(F(", "));
  	Serial.println((float)(pz) / (float)(stppmm));
	}
}

void homeall() {                                           // home all axes
  Feed = 140;
  delay(1);
  long xi = 0, yi = 0, zi = 0;
  if (homeingenabled) {
    Serial.println(F("Starting homeing..."));
    while (!(digitalRead(Zlmt))){                         // 0 - x, 1 - step, 0 - dir
      stepnew(4);
    }

    while (!(digitalRead(Xlmt) && digitalRead(Ylmt))) {
      if (!(digitalRead(Xlmt))) {
        stepnew(1);
      }
      if (!(digitalRead(Ylmt))) {
        stepnew(3);
      }
    }
    Serial.println(F(" Finished homeing..."));
  }
  else
    Serial.println(F(" dbg: homeing is disabled in the firmware!"));
  px = 0;
  py = 0;
  pz = zupend;
  delay(100);
  Serial.println(F("RDY"));
}
//********************************************************************************************************************
void loop() {
  int word = 0;
  /*if (xpos < 0 || ypos < 0 || zpos < 0 || xpos > xendr || ypos > yendr || zpos > zupend) {
    homeall();
  }*/
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
	//Serial.println(splitbuffer[i]);
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
                  //mov_speed = true;
                  Serial.print(F(" Move:G00 "));
                  break;
                case 1:
                  movemode = 1;
                  //mov_speed = false;
                  Serial.print(F(" Move:G01 "));
                  break;
                case 2:
                  movemode = 2;
                  //mov_speed = false;
                  Serial.print(F(" Move:G02 "));
                  break;
                case 3:
                  movemode = 3;
                 // mov_speed = false;
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
    }
    else {
      Serial.println(F(" ER: missing delimiters < >"));
      Serial.println("RPT");
    }
    if (moveon == true) {
      Serial.print(F(" moveing..."));
     // Serial.println(movemode);
      if (movemode == 0)
        hfstpdly = 1.87; //stepdelay = 3.5
      else
        hfstpdly = 1200 * 60 / (2 * Feed * stppmm); //stepdelay = 1200 / ((Feed * 200) / (1.25 * 60));               // 1000 * 1.2
      switch (movemode) {
        case 0:
        case 1: if(inc)
			intpolline(xmm, ymm, zmm);
		else{
			/*Serial.print(" xmm: ");
			Serial.print(xmm);
			Serial.print(" px: ");
			Serial.print(px);
			Serial.print(" X: ");
			Serial.println(xmm - (px / stppmm));*/
			intpolline((float)(xmm - (px / stppmm)), (float)(ymm - (py / stppmm)),(float)( zmm - (pz / stppmm)));
		}
          break;
        case 2: if(inc)
			intpolcirc(CW, xmm, ymm, zmm, I_CXoffset, J_CYoffset);
		else{
			intpolcirc(CW, (float)(xmm - (px / stppmm)),(float)(ymm - (py / stppmm)), (float)(zmm - (pz / stppmm)), I_CXoffset, J_CYoffset);
		}
          break;
        case 3: if(inc)
			intpolcirc(CCW, xmm, ymm, zmm, I_CXoffset, J_CYoffset);
		else{
			intpolcirc(CCW, (float)(xmm - (px / stppmm)),(float)(ymm - (py / stppmm)), (float)(zmm - (pz / stppmm)), I_CXoffset, J_CYoffset);
		}
	  break;
      }
      moveon = false;
      if(inc){
      	xmm = 0;
      	ymm = 0;
      	zmm = 0;
      	I_CXoffset = 0;
      	J_CYoffset = 0;
      }
      Serial.println(F("RDY"));
    }
    else {
      //CLR(PORTB, LED);
      delay(10);
      Serial.println(F("RDY"));
    }
  }
}
