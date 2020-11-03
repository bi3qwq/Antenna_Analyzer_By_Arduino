/*
  //
  //By BI3QWQ,China
  //QQ ID 657584105,download qq from http://im.qq.com/,and reg user from http://zc.qq.com
  //2018-9-20 UTC +8
  //
  --------------How to use?--------------------

  [Mode]

  1、Config Mode（Cailibration、Freq Range、Reset、Exit）
  turn off system,and always hold down the button.then,turn on system,will have tip "Config Mode".
  at the config mode,u can cailibration,change freq range,reset system rom,and exit mode to Realtime mode.

  2、RealTime Mode
  u can change any freq,rotate button left or right to change freq.
  click button <500ms,u can change freq step,is  1.0、0.1、0.01、0.001.when u roate very fast,the setp will x10 speed.
  press button <2s,system will auto scaning freq,the step is by you set step,u can roate button very fast,scan freq will be x10 speed.roate slow will normal speed.when u click button or  overflow max freq,will stop scaning,and auto enter graphic mode,u can watch the scan result by line chart.
  press button >2s,will enter graphic,the center freq is u selected,the step is u set step .at the graphic mode,program will autoscanning total freq.click button can stop scaning.

  3、Band Mode
  when u rotate button and change freq on <1.000Mhz,will enter Band Mode,roate freq >1.000Mhz will enter  RealTime Mode.
  click button <500ms,u can change freq step,is 0.001 or 0.005
  press button >2s,will auto scanning system default band(3.8-3.86,5.32-5.38,7.0-7.08,14.150-14.35,21.37-21.43,29.57-29.63).
  when u click button or overflow total band,will stop scaning,and auto enter graphic mode,u can watch the scan result by line chart.

  4、Graphic Mode
  this mode have new function,it's autoscaning total freq all the time,when u click button one times,or press button <2s.
  realtime mode or band mode  finished word,will auto enter this mode.
  click button <500ms:stop autoscaning,change roate mode max SWR or move cursor.
  press button <2s:exit mode,and return last mode,for realtime mode or band mode.
  press button >2s:will aoto scan total freq all times.

  -----------------Question?------------------------
  if startup system have tip is "Need Calibration!" and RealTime mode have "SWR!",it's need calibration res.enter Config Mode to cailibration.it's will be ok.

*/
#include "U8glib.h"
#include "EEPROM.h"
#include "math.h"
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C+


char* CallSign = "BI3QWQ";          //Change your callsign

const unsigned int LED_FLAG = 13;		//Change your pin
const unsigned int DDS_SCLK = 12;		//Change your pin
const unsigned int DDS_FQUD = 11;		//Change your pin
const unsigned int DDS_SDAT = 10;		//Change your pin
const unsigned int DDS_RESET = 9;		//Change your pin

const unsigned int Battery_3v3 = A6;	//Change your pin,connect 3v3 pin,or ignore this pin
float Battery_Remain = 0;
float Battery_Vol;

const unsigned int Button = 4;		//Change your pin
const unsigned int ButtonA = 2;		//Change your pin
const unsigned int ButtonB = 3;		//Change your pin

volatile int BtnRing = 0;
volatile int BtnRingSpeed = 1;
volatile int BtnValue = 0;
volatile int BtnLastValue = 1;
volatile long BtnRingLastTime = 0;
volatile byte BtnAFlag = 0;
volatile byte BtnBFlag = 0;
volatile byte BtnReading = 0;

float Freq_Zom = 0;
//float Freq_RL = 0;

float Freq_SWR = 0;
float Freq_MinSWR = 0;
float Freq_MinFreq = 0.0;
float Freq_Start = 0;
float Freq_Min = 1;
float Freq_Max = 30;
float Freq = 1.0;
unsigned int Freq_IsScan = 0;
unsigned int Freq_ThisBand = 0;
unsigned int Freq_Mode = 0; //0=Freq 1=Band

unsigned Band_Cursor = 0;
unsigned Band_Count = 6;
float Band_List[6][5] = {
  { 3.800,  3.860, 3.840, 0, 0}, //12
  { 5.320,  5.380, 5.360, 0, 0}, //12
  { 7.000,  7.080, 7.050, 0, 0}, //16
  {14.150, 14.350, 14.270, 0, 0}, //40
  {21.370, 21.430, 21.400, 0, 0}, //12
  {29.570, 29.630, 29.600, 0, 0}  //12
};

bool  SWR_IsFixd = false;
unsigned int SWR_Avg = 1;
float SWR_Trim = 0.00;
float SWR_Fix[30][2];


const unsigned int SWR_ROM_Index = 10;
const unsigned int SWR_ROM_Width = 7; //0=+-  123=c 456=m*ln()
const unsigned int SWR_ROM_FloatWidth = 3; //1=x/255/255  2=x/255%255 3=x%255

unsigned int Scan_Graphical_Cursor = 0;
unsigned int Scan_Graphical_MaxWidth = 118;

char* ModeCaption = "REALTIME";

bool IsDebug = false;
//EEPROM
//0   FREQ
//3   SWR_Avg
//5   MinFreq
//6   MaxFreq
//7   SWR_TRIM
//10  SWR

void setup() {
  //Serial.begin(9600);

  pinMode(DDS_SCLK, OUTPUT);
  pinMode(DDS_FQUD, OUTPUT);
  pinMode(DDS_SDAT, OUTPUT);
  pinMode(DDS_RESET, OUTPUT);

  pinMode(LED_FLAG, OUTPUT);
  digitalWrite(LED_FLAG, HIGH);

  pinMode(ButtonA, INPUT_PULLUP);
  pinMode(ButtonB, INPUT_PULLUP);
  pinMode(Button, INPUT_PULLUP);

  attachInterrupt(0, ButtonA_interrupt, CHANGE);
  attachInterrupt(1, ButtonB_interrupt, CHANGE);

  digitalWrite(DDS_RESET, HIGH);
  digitalWrite(DDS_RESET, LOW);


  Welcome();
  Config();
  delay(500);

  SWR_IsFixd = LoadSWRFix();
  SWR_Trim = Readfloat(7, 0.00);
  SWR_Avg = ReadInt(3, 15);

  Freq_Min = ReadInt(5, 1);
  Freq_Max = ReadInt(6, 30);

  Freq = Readfloat(0, 7.050);

  Set_Freq(Freq);

}


void Welcome()
{
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_tpssb);
    u8g.drawStr(27, 20, "HF SCANER");
    u8g.drawStr(37, 40, CallSign);

    if (digitalRead(Button) == LOW)
      u8g.drawStr(20, 55, "Config Mode!");
    else if (ReadInt(12, 0) == 0)
      u8g.drawStr(10, 55, "Need Calibration!");
    else
      u8g.drawStr(34, 55, "Loading...");

  } while (u8g.nextPage() );
}

void loop() {
  Scan_RealTime();
}


void Scan_RealTime()
{
  unsigned int Scan_Result[118][2];

  bool Scan_LastCutCount = false;
  bool Band_IsLock = false;
  float Freq_Step = 1.000;
  float Freq_LastStep = 1.000;

  BtnLastValue = BtnValue;

  Set_Freq(Freq);

  do
  {
    unsigned int Pressed = Button_Press();

    if (Pressed == 1)
    {
      if (Freq_IsScan == 0)
      {
        if (Freq_Mode == 0)
        {
          Freq_Step /= 10;
          if (Freq_Step <= 0.0001)
            Freq_Step = 1;
        }
        else if (Freq_Mode == 1)
        {
          Band_IsLock = !Band_IsLock;
          if (Band_IsLock)
          {
            for (int i = 0; i < Band_Count; i++)
            {
              Band_List[i][4] =  Band_List[i][3];
            }
          }

        }
      }
      else if (Freq_IsScan == 1)
      {
        Scan_Finish(Scan_Result, false);
      }
    }
    else if (Pressed == 2)
    {
      for (unsigned int i = 0; i < Scan_Graphical_MaxWidth; i++)
        Scan_Result[i][1] = 0;

      Freq_IsScan = 1;
      Freq_MinSWR = 0;
      Freq_MinFreq = 0;
      Scan_Graphical_Cursor = 0;
      BtnRingSpeed = 1;

      Scan_LastCutCount = 0;
      Band_Cursor = 0;

      if (Freq_Mode == 0)
        Freq_Start = Freq;
      else
        Freq = Band_List[Band_Cursor][0];

      ModeCaption = "SCANING...";
    }
    else if (Pressed == 3)
    {
      if (Freq_Mode == 0)
      {
        Freq_Start = Freq;

        //Scan_Graphical_MaxWidth=112/4=56
        float freq_Width = Scan_Graphical_MaxWidth / 8 * Freq_Step; //0.001=0.056
        float freq_Min = Freq - freq_Width; //7-0.056=6.944
        float freq_Max = Freq + freq_Width; //7+0.056=7.056

        if (freq_Min < Freq_Min)
          freq_Min = Freq_Min;

        if (freq_Max > Freq_Max)
          freq_Max = Freq_Max;

        for (unsigned int i = 0; i < Scan_Graphical_MaxWidth; i++)
          Scan_Result[i][1] = 0;


        unsigned int i = 0;
        for (float freq = freq_Min; freq <= freq_Max; freq += Freq_Step)
        {
          Scan_Result[i][0] = freq * 1000;
          i++;
        }
        Scan_Graphical_Cursor = i - 1;
        Scan_Finish(Scan_Result, true);
      }
      else  if (Freq_Mode == 1)
      {
        //Freq_Step = Freq_Step == 0.001 ? 0.005 : 0.001;
        if (Freq_Step == 0.001)
        {
          Freq_Step = 0.005;
          Config_Tip("STEP 0.005", 30, 1000);

        }
        else
        {
          Freq_Step = 0.001;
          Config_Tip("STEP 0.001", 30, 1000);
        }
      }


    }

    cli();
    if (BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;

      if (BtnRing > 0)
      {
        if (Freq_Mode == 1 && Freq_IsScan == 0)
        {

          ModeCaption = "REALTIME";
          Freq_Mode = 0;
          Freq = 1.000;
          Freq_Step = Freq_LastStep;
          Freq_MinSWR = 0;
          Set_Freq(Freq);
          Savefloat(0, Freq);
        }
        else  if (Freq_Mode == 1 && Freq_IsScan == 1)
        {
          Freq_Step = 0.005;
        }
        else
        {
          Freq += Freq_Step * BtnRingSpeed;
          if (Freq > Freq_Max)
            Freq = Freq_Max;
        }
      }
      else
      {
        if (Freq_Mode == 0)
        {
          Freq -= Freq_Step * BtnRingSpeed;
          if (Freq < Freq_Min)
          {
            Freq = Freq_Min;

            if (Freq_Mode == 0 && Freq_IsScan == 0)
            {
              ModeCaption = "BAND";
              Freq_Mode = 1;
              Freq_LastStep = Freq_Step;
              Freq_Step = 0.005;
              Freq_SWR = 0;
              Freq_Zom = 0;
            }
          }
        }
        else  if (Freq_Mode == 1 && Freq_IsScan == 1)
        {
          Freq_Step = 0.001;
        }
        else
        {


        }
      }

      if (Freq_Mode == 0)
      {
        Freq_MinSWR = 0;
        Set_Freq(Freq);
        Savefloat(0, Freq);
      }

      BtnRing = 0;
    }
    sei();

    if (Freq_IsScan == 0 && Freq_Mode == 0)
    {
      Freq_SWR = Get_Swr(true);
    }
    else if (Freq_IsScan == 1)
    {
      if (Freq_Mode == 0)
      {
        Freq += Freq_Step * BtnRingSpeed ;
        if (Freq > Freq_Max)
          Scan_Finish(Scan_Result, false);

      }
      else if (Freq_Mode == 1)
      {

        Freq += Freq_Step * BtnRingSpeed ;
        if (Freq > Band_List[Band_Cursor][1])
        {
          if (++Band_Cursor >= Band_Count)
          {
            Scan_Finish(Scan_Result, false);
          }
          else
            Freq =  Band_List[Band_Cursor][0];
        }
      }

      if (Freq_IsScan == 1)
      {
        Set_Freq(Freq);
        Freq_SWR = Get_Swr(true);
        Scan_Result[Scan_Graphical_Cursor][0] = Freq * 1000;
        Scan_Result[Scan_Graphical_Cursor][1] = Freq_SWR * 1000;
        Scan_Graphical_Cursor++;


        if (Scan_Graphical_Cursor == Scan_Graphical_MaxWidth)
        {
          unsigned int i_End;
          unsigned int i_Rate = 2;
          unsigned int i_Base = 0;

          if (Scan_LastCutCount == false)
          {
            i_Base = 0;
            i_End = Scan_Graphical_MaxWidth / 2;//Scan_Graphical_MaxWidth/4*2;
            Scan_Graphical_Cursor = Scan_Graphical_MaxWidth / 2;

          }
          else
          {
            i_Base = Scan_Graphical_MaxWidth / 2;
            i_End = Scan_Graphical_MaxWidth / 4; //Scan_Graphical_MaxWidth/4*2;
            Scan_Graphical_Cursor = Scan_Graphical_MaxWidth / 4 * 3;
          }

          Scan_LastCutCount = !Scan_LastCutCount;
          for ( unsigned int i = 0; i < i_End; i++)
          {
            if (Scan_Result[i * i_Rate + i_Base][ 1] <= Scan_Result[i * i_Rate + 1 + i_Base][ 1])
            {
              Scan_Result[i + i_Base][0] = Scan_Result[i * i_Rate + i_Base][0];
              Scan_Result[i + i_Base][1] = Scan_Result[i * i_Rate + i_Base][1];
            }
            else
            {
              Scan_Result[i + i_Base][0] = Scan_Result[i * i_Rate + 1 + i_Base][0];
              Scan_Result[i + i_Base][1] = Scan_Result[i * i_Rate + 1 + i_Base][1];
            }
          }

          for (unsigned int i = i_End + i_Base; i < Scan_Graphical_MaxWidth; i++)
          {
            Scan_Result[i][0] = 0;
            Scan_Result[i][1] = 0;
          }
        }

      }
    }
    else if (Freq_IsScan == 2)
    {
      Freq_SWR = 0;
    }

    if (Freq_MinSWR == 0 || (Freq_SWR > 0 && Freq_SWR < Freq_MinSWR ))
    {
      Freq_MinSWR = Freq_SWR;
      Freq_MinFreq = Freq;
    }

    if (Freq_Mode == 1 && Freq_IsScan == 0)
    {

      int BaseTop = 9;
      int LineHeight = 11;
      for (int i = 0; i < Band_Count; i++)
      {
        Set_Freq(Band_List[i][2]);
        Band_List[i][3] = Get_Swr(true);
      }
      u8g.firstPage();
      do {

        u8g.setFont(u8g_font_04b_03br);
        for (int i = 0; i < Band_Count; i++)
        {
          Printfloat(Band_List[i][2], 0, LineHeight * i + BaseTop, 2, 3);
          Printfloat(Band_List[i][3], 108, LineHeight * i + BaseTop , 2, 2);

          if (Band_List[i][3] < 1.5)
          {
            u8g.drawLine(37,  LineHeight * i + 4 , 37, LineHeight * i + 5);
            u8g.drawLine(37,  LineHeight * i + 7 , 37,  LineHeight * i + 8);

            //u8g.drawLine(52,  LineHeight * i + 4 , 52,  LineHeight * i + BaseTop - 1);
          }
          else if (Band_List[i][3] < 2.0 )
          {
            u8g.drawLine(41,  LineHeight * i + 4 , 41,  LineHeight * i + BaseTop - 1);
            // u8g.drawStr(54, LineHeight * i + BaseTop, "2.0");
          }

          if (!Band_IsLock)
          {
            u8g.drawBox(32, LineHeight * i + 4  , Band_List[i][3] >= 9 ? 70 : 1 + 8 * (Band_List[i][3] - 1), 5);  //2是最低
          }
          else
          {
            u8g.drawBox(32, LineHeight * i + 4  , Band_List[i][3] >= 9 ? 70 : 1 + 8 * (Band_List[i][3] - 1), 3); //31+2是最低
            int ll = Band_List[i][4] >= 9 ? 70 : 8 * (Band_List[i][4] - 1);
            u8g.drawLine(32,  LineHeight * i + 8 , 32 + ll,  LineHeight * i + 8);
          }


        }
      } while (u8g.nextPage() );

    }
    else
    {
      u8g.firstPage();
      do {

        u8g.setFont(u8g_font_helvB18);
        Printfloat(Freq, 0, 30, 2, 3);

        u8g.setFont(u8g_font_04b_03br);
        u8g.drawStr(0, 6, ModeCaption);

        u8g.drawStr(52, 6, "STEP");
        Printfloat(Freq_Step, 75, 6, 1, 3);

        u8g.drawStr(124, 17, "R");

        if (Freq_Zom < 100)
          PrintInt(Freq_Zom, 113, 17);
        else
          PrintInt(Freq_Zom, 108, 17);

        //u8g.drawStr(124, 6, "v");
        //Printfloat(analogRead(Battery_3v3), 106, 6, 2, 2);


        //1023  3.9v
        u8g.drawBox(126, 2, 2, 3);
        u8g.drawFrame(114, 1, 12  , 5);
        u8g.drawBox(115, 1, Get_Voltage()*10, 4);





        if (SWR_IsFixd)
          u8g.drawStr(110, 52, "SWR");
        else
          u8g.drawStr(108, 52, "SWR!");

        for (unsigned int i = 0; i < 8; i++)
          PrintInt(i, i * 13, 52);


        u8g.drawBox(0, 35, Freq_SWR >= 9 ? 128 : 14 * Freq_SWR, 8);

        u8g.drawLine(0, 43, 128, 43);

        u8g.setFont(u8g_font_tpssb);
        if (Freq_SWR < 10)
          Printfloat(Freq_SWR, 102, 30, 2, 2);
        else
          Printfloat(Freq_SWR, 95, 30, 2, 2);

        u8g.setFont(u8g_font_profont11);
        u8g.drawStr(0, 64, "Min      /        Mhz");
        Printfloat(Freq_MinSWR, 23, 64, 2, 2);
        Printfloat(Freq_MinFreq, 66, 64, 2, 3);

      } while (u8g.nextPage() );
    }
  } while (true);
}

void Scan_Finish(unsigned int Scan_Result[118][2], bool IsScan)
{
  Scan_Graphical(Scan_Result, IsScan);
  Freq_IsScan = 0;
  Freq_SWR = 0;
  Freq_Zom = 0;
  BtnRingSpeed = 1;
  Freq = Freq_Start;
  ModeCaption = Freq_Mode == 0 ? "REALTIME" : "BAND";
}

void Scan_Graphical(unsigned int Scan_Result[118][2], bool IsScan) {
  unsigned int Swr_Max = 6;
  unsigned int Pos_XMin = 6;
  unsigned int Pos_XMax = 124;
  unsigned int Pos_YMin = 14;
  unsigned int Pos_YMax = 52;
  unsigned int Draw_Width = Pos_XMax - Pos_XMin;  //118
  unsigned int Draw_Height = Pos_YMax - Pos_YMin; //38
  unsigned int PosCursor = 0;
  unsigned int Pos_YofSWR;

  bool ButtonMode = true;
  //bool IsScan = false;
  unsigned int AotuScan_Index = 0;

  unsigned int AutoSize = Scan_Graphical_MaxWidth / Scan_Graphical_Cursor;

  for (unsigned int i = 0; Scan_Result[i][1] != 0 &&  i < (Scan_Graphical_MaxWidth - 1); i++)
  {
    if ((float)Scan_Result[i][1] / 1000.0f > Swr_Max)
      Swr_Max = (float)Scan_Result[i][1] / 1000.0f;
  }
  Swr_Max++;

  if (Swr_Max > 30)
    Swr_Max = 30;
  Pos_YofSWR =  ((float)Draw_Height / (float)Swr_Max) * ((float)Swr_Max - 1.5) + Pos_YMin;
  if (Pos_YofSWR > Pos_YMax)
    Pos_YofSWR = Pos_YMax;

  do
  {

    unsigned int Pressed = Button_Press();
    if (Pressed == 1 )
    {
      if (IsScan)
        IsScan = false;
      else
        ButtonMode = !ButtonMode;
    }
    else if (Pressed == 2)
    {
      return;
    }
    else if (Pressed == 3)
    {
      IsScan = true;
      AotuScan_Index = 0;
    }

    cli();
    if (BtnLastValue != BtnValue)
    {
      if (BtnLastValue > BtnValue)
        BtnRing--;
      else
        BtnRing++;

      BtnLastValue = BtnValue;


      if (ButtonMode)
      {
        if (BtnRing > 0)
        {
          if ((PosCursor - BtnRingSpeed) > 0 && (PosCursor - BtnRingSpeed) < 65000)
            PosCursor -= BtnRingSpeed;
          else
            PosCursor = 0;
        }
        else
        {
          PosCursor += BtnRingSpeed;
          if (PosCursor > (Scan_Graphical_Cursor - 1))
            PosCursor = Scan_Graphical_Cursor - 1;
        }
        Freq_MinSWR = (float)Scan_Result[PosCursor][1] / 1000.0f;
        Freq_MinFreq = (float)Scan_Result[PosCursor][0] / 1000.0f;
      }
      else
      {
        if (BtnRing > 0)
          Swr_Max ++;
        else if (Swr_Max > 2)
          Swr_Max--;

        Pos_YofSWR = ((float)Draw_Height / (float)Swr_Max) * ((float)Swr_Max - 1.5) + Pos_YMin;
        if (Pos_YofSWR > Pos_YMax)
          Pos_YofSWR = Pos_YMax;
      }

      BtnRing = 0;
    }
    sei();

    if (IsScan)
    {
      Set_Freq(Scan_Result[AotuScan_Index][0] / 1000.0f);
      Freq_MinSWR = Get_Swr(true);
      Scan_Result[AotuScan_Index][1] = Freq_MinSWR * 1000;
      Freq_MinFreq = (float)Scan_Result[AotuScan_Index][0] / 1000.0f;
      AotuScan_Index++;

      if (AotuScan_Index >= Scan_Graphical_Cursor)
        AotuScan_Index = 0;
      PosCursor = AotuScan_Index;

      if (Freq_MinSWR > Swr_Max)
        Swr_Max = Freq_MinSWR;
    }

    u8g.firstPage();

    do {
      u8g.setFont(u8g_font_04b_03br);

      u8g.drawLine(Pos_XMin, Pos_YMin, Pos_XMin, Pos_YMax);
      u8g.drawLine(Pos_XMin, Pos_YMax, Pos_XMax, Pos_YMax);

      u8g.drawLine(PosCursor * AutoSize + Pos_XMin, Pos_YMin, PosCursor * AutoSize + Pos_XMin, Pos_YMax);

      for (unsigned int i = Pos_XMin; i < Pos_XMax; i += 3)
        u8g.drawPixel(i, Pos_YofSWR);

      u8g.drawStr(115, Pos_YofSWR - 2, "1.5");

      PrintInt(Swr_Max, 0, 13);
      u8g.drawStr(0, 52, "0");
      u8g.drawStr(0, 24, "S");
      u8g.drawStr(0, 32, "W");
      u8g.drawStr(0, 40, "R");

      u8g.setFont(u8g_font_profont11);
      u8g.drawStr(53, 64, "/        MHz");
      u8g.drawStr(5, 64, ">");

      Printfloat(Freq_MinSWR, 14, 64, 2, 3);
      Printfloat(Freq_MinFreq, 63, 64, 2, 3);

      for (unsigned int i = 0; Scan_Result[i][1] != 0 &&  i < (Scan_Graphical_MaxWidth - 1); i++)
      {
        float Y = (float)Scan_Result[i][1] / 1000.0f >= Swr_Max ? Swr_Max : (float)Scan_Result[i][1] / 1000.0f;
        float YY = Scan_Result[i + 1][1] == 0 ? Y : (float)Scan_Result[i + 1][1] / 1000.0f >= Swr_Max ? Swr_Max : (float)Scan_Result[i + 1][1] / 1000.0f;

        Y = Draw_Height - Y / Swr_Max * Draw_Height + Pos_YMin;
        YY = Draw_Height - YY / Swr_Max * Draw_Height + Pos_YMin;
        u8g.drawLine(Pos_XMin + i * AutoSize, Y, Pos_XMin + i * AutoSize + AutoSize, YY);
      }

    } while (u8g.nextPage() );
  } while (true);
}

void Config()
{
  if (digitalRead(Button) == HIGH)
    return;
  while (digitalRead(Button) == LOW);

  delay(100);
  unsigned int Mode = 0;
  unsigned int Cursor = 0;
  unsigned int Menu_Pagesize = 4;
  unsigned int Menu_Count = 6;
  unsigned int Pressed = 0;
  char* Menu_List[6] = {"Calibration", "SWR Trim", "SWR Avg", "Freq Range",  "Reset", "Exit"};
  do
  {
    cli();
    if (BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;

      if (BtnRing > 0)
      {
        if (Cursor > 0)
          Cursor--;
      }
      else if (Mode == 0 && (Cursor < (Menu_Count - 1)))
        Cursor++;

      BtnRing = 0;
    }
    sei();

    Pressed = Button_Press();
    if (Pressed == 1)
    {
      if (Mode == 0)
      {
        Mode = Cursor + 1;
        Cursor = 0;
      }

      if (Mode == 1)
      {
        Config_Calibration();
      }
      else if (Mode == 2)
      {
        Config_SWRTrim();
      }
      else if (Mode == 3)
      {
        Config_SWRAvg();
      }
      else if (Mode == 4)
      {
        Config_FreqRange();
      }
      else if (Mode == 5)
      {
        Config_Reset();
      }
      else if (Mode == 6)
      {
        return;
      }
      Mode = 0;
    }

    if (Mode == 0)
    {
      unsigned int Page = Cursor / Menu_Pagesize;
      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_tpssb);
        for (unsigned int i = 0; (Page * Menu_Pagesize + i ) < Menu_Count &&   i < Menu_Pagesize; i++)
          u8g.drawStr(12, 15 + 15 * i, Menu_List[Page * Menu_Pagesize + i]);

        u8g.drawStr(0 , 15 + 15 * (Cursor < Menu_Pagesize ? Cursor : Cursor % Menu_Pagesize) , ">");
      } while (u8g.nextPage() );
    }

  } while (true);

}

void Config_Calibration()
{
  float SWR_Cache[30][4];
  float SWR_Logest[4] = {1, 2, 4, 6};
  unsigned int Times = 4;
  unsigned int Pressed;
  unsigned int Step = 0;
  float Freq_SWR = 0;
  int Freq_Step = 1;
  unsigned int Freq = 1;
  unsigned int Split = 128 / Times;
  bool IsScan = false;

  SWR_IsFixd = false;

  Set_Freq(1);
  do
  {
    Pressed = Button_Press();
    if (Pressed == 1)
    {
      IsScan = true;
      Freq = 1;
      if (Step == Times )
      {
        Config_Tip("Saveing", 40, 1000);
        for (unsigned int i = 0; i < Freq_Max; i++)
          Config_Calibration_Logest(i, Times, SWR_Cache[i], SWR_Logest);
        SaveSWRFix();
        SWR_IsFixd = LoadSWRFix();
        Config_Tip("Saved!", 45, 1000);
        return;
      }
    }
    else if (Pressed >= 2)
      return;

    cli();
    if (IsScan == false && BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;
      if (BtnRing > 0)
      {
        if (SWR_Logest[Step] < 19)
          SWR_Logest[Step]++;
      }
      else if (SWR_Logest[Step] > 1)
        SWR_Logest[Step]--;
      BtnRing = 0;
    }
    sei();

    if (IsScan)
    {
      Set_Freq(Freq);
      Freq_SWR = Get_Swr(false);
      SWR_Cache[Freq - 1][Step] = Freq_SWR;
    }
    else
      Freq_SWR = Get_Swr(false);

    u8g.firstPage();
    do {

      u8g.setFont(u8g_font_04b_03br);
      u8g.drawStr(0, 6, "CALIBRATION");
      u8g.drawStr(102, 16, "SWR");

      u8g.setFont(u8g_font_helvB18);
      Printfloat(Freq, 0, 30, 2, 3);

      u8g.setFont(u8g_font_tpssb);
      Printfloat(Freq_SWR, 86, 30, 2, 3);

      for (unsigned int i = 0; i < Times; i++)
        PrintInt(SWR_Logest[i] * 50, i * Split + 6, 64);

      if (!IsScan)
        u8g.drawStr(Split * Step + 5, 45, "OK?");

      if (Step > 0)
        u8g.drawStr(Split * (Step - 1) , 44, "PASS");

      if (IsScan)
        u8g.drawLine(0, 50, Freq / Freq_Max * 128, 50);
      else
        u8g.drawLine(0, 50, 128, 50);

      u8g.drawBox(0, 45, Split * Step, 4);
    } while (u8g.nextPage() );

    if (IsScan)
    {
      Freq += Freq_Step;
      if (Freq > Freq_Max)
      {
        Step++;
        Freq = 1;

        Set_Freq(Freq);
        Freq_SWR = 0;
        IsScan = false;
      }
    }
  } while (true);
}

void Config_Calibration_Logest(unsigned int idx, int times, float x[], float y[])
{
  float guess[2][3];
  float tmp = 0;
  for (int i = 0; i < times; i++)
    x[i] = log(x[i]);

  for (int i = 0; i <= 1; i++)
  {
    for (int j = 0; j <= 1; j++)
    {
      tmp = 0;
      for (int z = 0; z < times; z++)
        tmp += pow(x[z], i + j);
      guess[i][j] = tmp;
    }

    tmp = 0;
    for (int z = 0; z < times; z++)
      tmp += pow(x[z], i) * y[z];
    guess[i][2] = tmp;

  }

  tmp = 0;
  for (int j = 0; j < 2; j++)
  {
    int k = j;
    double minval = guess[j][j];

    for (int i = j; i < 2; i++)
    {
      if (abs(guess[i][j]) < minval)
      {
        minval = guess[i][j];
        k = i;
      }
    }

    if (k != j)
    {
      for (int x = j; x <= 2; x++)
      {
        tmp = guess[k][x];
        guess[k][x] = guess[j][x];
        guess[j][x] = tmp;
      }
    }

    for (int m = j + 1; m < 2; m++)
    {
      float divval = guess[m][j] / guess[j][j];
      for (int n = j; n <= 2; n++)
        guess[m][n] = guess[m][n] - guess[j][n] * divval;
    }
  }
  float xx[2];
  xx[1] = guess[1][2] / guess[1][ 1];
  for (int i = 0; i >= 0; i--)
  {
    tmp = 0;
    for (int j = i; j < 2; j++)
      tmp += xx[j] * guess[i][j];
    xx[i] = (guess[i][2] - tmp) / guess[i][ i];
  }

  SWR_Fix[idx][0] = xx[0];
  SWR_Fix[idx][1] = xx[1];

}


void Config_SWRTrim()
{
  unsigned int Pressed;
  unsigned int Step;
  float SWR_Fixed;

  SWR_Trim = Readfloat(7, 0.00);

  SWR_IsFixd = LoadSWRFix();
  Set_Freq(7.050);
  Set_Freq(7.050);

  do
  {
    Pressed = Button_Press();
    if (Pressed == 1)
    {
      Step++;
      if (Step > 1)
        Step = 0;
    }
    else if (Pressed >= 1)
    {
      if (Step == 0)
      {
        Config_Tip("Saveing", 40, 1000);
        Savefloat(7, SWR_Trim);
        Config_Tip("Saved!", 45, 1000);
        return;
      }
      else if (Step == 1)
        return;

    }

    cli();
    if (BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;

      if (BtnRing > 0)
      {
        SWR_Trim += 0.01 * BtnRingSpeed;
      }
      else
      {
        SWR_Trim -= 0.01 * BtnRingSpeed;
        if (SWR_Trim < 0)
          SWR_Trim = 0;
      }
      BtnRing = 0;
    }
    sei();


    Freq_SWR = Get_Swr(false);
    SWR_Fixed = Get_Swr(true);

    u8g.firstPage();
    do {

      u8g.setFont(u8g_font_04b_03br);
      u8g.drawStr(0, 6, "SWR TRIM         FREQ 7.050Mhz");

      u8g.setFont(u8g_font_tpssb);

      Printfloat(Freq_SWR, 0, 40, 2, 2);
      Printfloat(SWR_Fixed, 48, 40, 2, 2);
      Printfloat(SWR_Trim, 100, 40, 1, 2);


      u8g.drawStr(0, 20, "Real");
      u8g.drawStr(48, 20, "Fixed");
      u8g.drawStr(100, 20, "Trim");

      u8g.drawLine(0, 23, 128, 23);


      u8g.drawStr(0, 60, "Save");
      u8g.drawStr(100, 60, "Back");

      if (Step == 0)
        u8g.drawLine(0, 62, 27, 62);
      else if (Step == 1)
        u8g.drawLine(100, 62, 128, 62);
    } while (u8g.nextPage() );

  } while (true);
}



void Config_SWRAvg()
{
  unsigned int Pressed;
  unsigned int Step;
  unsigned int Avg;
  float SWR_Fixed;


  Avg = ReadInt(3, 1);

  SWR_IsFixd = LoadSWRFix();
  Set_Freq(7.050);
  Set_Freq(7.050);

  do
  {
    Pressed = Button_Press();
    if (Pressed == 1)
    {
      Step++;
      if (Step > 1)
        Step = 0;
    }
    else if (Pressed >= 1)
    {
      if (Step == 0)
      {
        Config_Tip("Saveing", 40, 1000);
        SaveInt(3, Avg);
        Config_Tip("Saved!", 45, 1000);
        return;
      }
      else if (Step == 1)
        return;

    }

    cli();
    if (BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;

      if (BtnRing > 0)
      {
        Avg += BtnRingSpeed;
      }
      else
      {
        if ((Avg - BtnRingSpeed) < 1)
          Avg = 1;
        else
          Avg -= BtnRingSpeed;
      }
      BtnRing = 0;
    }
    sei();

    SWR_Fixed = 0;
    for (int i = 0; i < Avg; i++)
    {
      SWR_Fixed +=  Get_Swr(false);;
    }
    Freq_SWR = Get_Swr(false);
    SWR_Fixed /=  Avg;



    u8g.firstPage();
    do {

      u8g.setFont(u8g_font_04b_03br);
      u8g.drawStr(0, 6, "SWR AVG         FREQ 7.050Mhz");

      u8g.setFont(u8g_font_tpssb);

      Printfloat(Freq_SWR, 0, 40, 2, 2);
      Printfloat(SWR_Fixed, 48, 40, 2, 2);
      PrintInt(Avg, 100, 40);


      u8g.drawStr(0, 20, "Real");
      u8g.drawStr(48, 20, "Fixed");
      u8g.drawStr(100, 20, "Avg");

      u8g.drawLine(0, 23, 128, 23);


      u8g.drawStr(0, 60, "Save");
      u8g.drawStr(100, 60, "Back");

      if (Step == 0)
        u8g.drawLine(0, 62, 27, 62);
      else if (Step == 1)
        u8g.drawLine(100, 62, 128, 62);
    } while (u8g.nextPage() );

  } while (true);
}

void Config_FreqRange()
{
  unsigned int Pressed;
  unsigned int Step = 0;
  unsigned int Min_Freq = ReadInt(5, 1);
  unsigned int Max_Freq = ReadInt(6, 30);

  do
  {
    Pressed = Button_Press();
    if (Pressed == 1)
    {
      Step++;
      if (Step > 3)
        Step = 0;
    }
    else if (Pressed >= 2)
    {
      if (Step == 2)
      {
        Config_Tip("Saveing", 40, 1000);
        SaveInt(5, Min_Freq);
        SaveInt(6, Max_Freq);
        Config_Tip("Saved!", 45, 1000);
        return;
      }
      else if (Step == 3)
        return;

    }

    cli();
    if (BtnLastValue != BtnValue)
    {

      if (BtnLastValue > BtnValue)
        BtnRing++;
      else
        BtnRing--;

      BtnLastValue = BtnValue;

      if (BtnRing > 0)
      {
        if (Step == 0 && (Min_Freq + 1) < Max_Freq)
          Min_Freq++;
        else if (Step == 1)
          Max_Freq ++;

      }
      else
      {
        if (Step == 0 && Min_Freq > 0)
          Min_Freq--;
        else if (Step == 1 && Max_Freq > (Min_Freq + 1))
          Max_Freq--;

      }
      BtnRing = 0;
    }
    sei();

    u8g.firstPage();
    do {

      u8g.setFont(u8g_font_04b_03br);
      u8g.drawStr(0, 6, "FREQ RANGE");

      u8g.drawStr(42, 22, "Mhz");
      u8g.drawStr(110, 22, "Mhz");


      u8g.setFont(u8g_font_helvB18);

      PrintInt(Min_Freq, Min_Freq >= 10 ? 18 : 25, 46 );
      PrintInt(Max_Freq, Max_Freq >= 10 ? 84 : 89, 46);

      u8g.setFont(u8g_font_tpssb);
      u8g.drawStr(0, 20, "Min");
      u8g.drawStr(64, 20, "Max");
      u8g.drawLine(0, 23, 60, 23);
      u8g.drawLine(64, 23, 128, 23);

      u8g.drawStr(0, 60, "Save");
      u8g.drawStr(100, 60, "Back");

      if (Step == 0)
        u8g.drawLine(16, 48, 46 , 48);
      else if (Step == 1)
        u8g.drawLine(81, 48, 112, 48);
      else if (Step == 2)
        u8g.drawLine(0, 62, 27, 62);
      else if (Step == 3)
        u8g.drawLine(100, 62, 128, 62);
    } while (u8g.nextPage() );

  } while (true);
}

void Config_Reset()
{
  SWR_IsFixd = false;
  Config_Tip("Reseting", 30, 1000);
  for (unsigned int i = 0; i < 255 ; i++)
    EEPROM.write(i, 0);
  Config_Tip("Complete!", 30, 1000);
}

void Config_Tip(char* str, unsigned int idx, unsigned int sleep)
{
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_tpssb);
    u8g.drawStr(idx, 35, str);
  } while (u8g.nextPage() );
  delay(sleep);
}


float Get_Voltage()
{
  // if (analogRead(Battery_3v3) > Battery_Remain)
  // {
  //   Battery_Vol = 34.648 + log(analogRead(Battery_3v3)) * -4.412;
  //   Battery_Remain = analogRead(Battery_3v3);
  //  }
  // return 0.3;
  Battery_Remain = analogRead(Battery_3v3);
  if (Battery_Remain <= 1000)
    return 1.0;
  else if (Battery_Remain <= 1100)
    return 0.9; 
  else if (Battery_Remain <= 1200)
    return 0.8; 
   else if (Battery_Remain >= 1230)
    return 0.5;
  else
    return 0.3;//Battery_Vol;
}

void Set_Freq(float Freq) {
  Freq_ThisBand = (unsigned int)Freq - 1;
  int32_t f = Freq * 4294967295 / 125;

  for (unsigned int b = 0; b < 4; b++, f >>= 8) {
    DDS_Sendbyte(f & 0xFF);
  }
  DDS_Sendbyte(0);

  digitalWrite(DDS_FQUD, HIGH);
  digitalWrite(DDS_FQUD, LOW);

  delay(20);
}

float Get_Swr(bool IsTrim) {
  float FWD = 0;
  float REV = 0;
  float VSWR;
  unsigned int AVG = SWR_IsFixd ? SWR_Avg : SWR_Avg * 2;

  for (unsigned int k = 0; k < AVG; k++) {
    REV = REV + analogRead(A0);
    FWD = FWD + analogRead(A1);
    delay(2);
  }

  FWD /= AVG;
  REV /= AVG;

  if (REV >= FWD)
    VSWR = 0;
  else
    VSWR = (FWD + REV) / (FWD - REV);

  if ((SWR_IsFixd == true && Freq_ThisBand >= 0 && VSWR > 0) || IsTrim==true)
  {
    if (IsTrim)
      VSWR = SWR_Fix[Freq_ThisBand][1] * log(VSWR+SWR_Trim) + (SWR_Fix[Freq_ThisBand][0]);
    else
      VSWR = (SWR_Fix[Freq_ThisBand][1]) * log(VSWR) + (SWR_Fix[Freq_ThisBand][0]);
  }


  Freq_Zom = 50 * VSWR; //(50 * REV) / FWD; //sqrt(VSWR) * 50;

  if (Freq_Zom > 999)
    Freq_Zom = 999;

  if (VSWR < 1)
    VSWR = 1;
  else if (VSWR >= 100)
    VSWR = 99.99;



  //Freq_RL = 20 * log((VSWR + 1) / (VSWR - 1));
  return VSWR;
}


void DDS_Sendbyte(byte data)
{
  for (unsigned int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(DDS_SDAT, data & 0x01);
    digitalWrite(DDS_SCLK, HIGH);
    digitalWrite(DDS_SCLK, LOW);
  }
}

unsigned int Button_Press()
{
  if (digitalRead(Button) == LOW)
  {
    unsigned long PressTime = millis();
    while (digitalRead(Button) == LOW);
    PressTime = millis() - PressTime;

    if (PressTime < 500)
      return 1;
    else if (PressTime < 2000)
      return 2;
    else
      return 3;
  }
  return 0;
}

void ButtonA_interrupt() {
  BtnReading = PIND & 0xC;
  if (BtnReading == B00001100 && BtnAFlag) {
    if (Button_RingSpeed())
      BtnValue++;

    BtnBFlag = 0;
    BtnAFlag = 0;
  }
  else if (BtnReading == B00000100)
    BtnBFlag = 1;
}

void ButtonB_interrupt() {
  delay(2);
  BtnReading = PIND & 0xC;
  if (BtnReading == B00001100 && BtnBFlag) {
    if ( Button_RingSpeed())
      BtnValue--;

    BtnBFlag = 0;
    BtnAFlag = 0;
  }
  else if (BtnReading == B00001000)
    BtnAFlag = 1;
}

bool Button_RingSpeed()
{
  delay(2);
  if (millis() - BtnRingLastTime <= 10)
    return false;
  else if (millis() - BtnRingLastTime <= 50)
    BtnRingSpeed = 10;
  else
    BtnRingSpeed = 1;

  BtnRingLastTime = millis();
  return true;
}

void PrintInt(unsigned int val, unsigned int x, unsigned int y)
{
  char tmp[10];
  itoa(val, tmp, 10);
  u8g.drawStr(x, y, tmp);
}

void Printfloat(float val, unsigned int x, unsigned int y, unsigned int before, unsigned int after)
{
  char tmp[10];
  dtostrf(val, before, after, tmp);
  u8g.drawStr(x, y, tmp);
}

void SaveSWRFix()
{
  for (unsigned int i = 0; i < Freq_Max; i++)
  {
    unsigned int Plus = SWR_Fix[i][0] > 0 ? 0 : 1;

    SaveInt(SWR_ROM_Index + i * SWR_ROM_Width, Plus);
    Savefloat(SWR_ROM_Index + i * SWR_ROM_Width + 1, abs(SWR_Fix[i][0]));
    Savefloat(SWR_ROM_Index + i * SWR_ROM_Width + SWR_ROM_FloatWidth + 1 , SWR_Fix[i][1]);

    //0   100
    //0   102
    //1   104
    //1   106
  }
}

bool LoadSWRFix()
{
  if (ReadInt(SWR_ROM_Index + 3, 0) == 0)
    return false;

  for (unsigned int i = 0; i < Freq_Max; i++)
  {
    SWR_Fix[i][0] = Readfloat(SWR_ROM_Index + i * SWR_ROM_Width + 1, 0);
    SWR_Fix[i][1] = Readfloat(SWR_ROM_Index + i * SWR_ROM_Width + SWR_ROM_FloatWidth + 1, 0);
    if (ReadInt(SWR_ROM_Index + i * SWR_ROM_Width, 0) == 1)
      SWR_Fix[i][0] = -SWR_Fix[i][0];
  }
  return true;
}
void SaveInt(unsigned int idx, unsigned int val)
{
  EEPROM.write(idx, val);
}

unsigned int ReadInt(unsigned int idx, unsigned int def)
{
  if (EEPROM.read(idx) == 0)
  {
    EEPROM.write(idx, def);
    return def;
  }
  else
    return EEPROM.read(idx);
}

void Savefloat(unsigned int idx, float val)
{
  long s = (long)(val * 100000);
  int s1 = s / 255 / 255;
  int s2 = s / 255 % 255;
  int s3 = s % 255;

  EEPROM.write(idx, s1);
  EEPROM.write(idx + 1, s2);
  EEPROM.write(idx + 2, s3);
}

float Readfloat(unsigned int idx, float def)
{
  float s1 = (float)EEPROM.read(idx);
  float s2 = (float)EEPROM.read(idx + 1);
  float s3 = (float)EEPROM.read(idx + 2);


  if (s3 == 0)
    return def;
  return   (s1 * 255 * 255 + s2 * 255 + s3) / 100000.0f;
}
