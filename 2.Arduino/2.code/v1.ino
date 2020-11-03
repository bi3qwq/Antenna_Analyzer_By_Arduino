// Antenna VSWR analyser by dr_phil
// finishing the Arduino job started by K6BEZ (see below)
// Arduinos are now (in 2016) much cheaper than when he published his work
//
// My thanks to all authors credited below for making this work possible
// I've either hacked their code or used it verbatim

// UPDATE 24/8/2017
// Added support for 160 m and 60 m bands

// UPDATE 28/4/2017
// Graphing - display a simple freq versus VSWR plot for single-band sweeps, with autoscaling and minimum VSWR/frequency indication

// UPDATE 21/4/2017
// Discovered that this unit was under-estimating VSWR owing to the low DDS output level
// relative to the detector diode forward voltages - this is widely reported by others who have
// built similar devices
// Fix is to use uncorrected sketch on device to show VSWR at midpoint of each band for resistive loads
// of 50ohm, 100 ohm, 150 ohm and 200 ohm. Then plot recorded values as function of frequency and
// 'actual' VSWR (i.e. 1,2,3 & 4 for the loads listed). Use logarithmic curve fit to determine correction function
// required at each frequency (it's different for each, as DDS output rolls off with increasing frequency). Then
// apply this correction function in the measurement subroutine
// Also, a minimum measurement time has been built in, as the detector circuits need time to charge
// capacitances - from LTSPICE 50 ms per measurement seems to do it.


/***************************************************************************\
*  Name    : DDS_Sweeper.BAS                                                *
*  Author  : Beric Dunn (K6BEZ)                                             *
*  Notice  : Copyright (c) 2013  CC-BY-SA                                   *
*          : Creative Commons Attribution-ShareAlike 3.0 Unported License   *
*  Date    : 9/26/2013                                                      *
*  Version : 1.0                                                            *
*  Notes   : Written using for the Arduino Micro                            *
*          :   Pins:                                                        *
*          :    A0 - Reverse Detector Analog in                             *
*          :    A1 - Forward Detector Analog in                             *
\***************************************************************************/


#include "U8glib.h"
#include "math.h"
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 

//AD9850或AD9851 引脚对应关系
const int SCLK=12;
const int FQ_UD=11;
const int SDAT=10;
const int RESET=9;

//旋钮编码器
static int pinA = 2; //旋钮-左旋E
static int pinB = 3; //旋钮-右旋A
const byte buttonPin = 4; //旋钮-按下B

volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
boolean buttonPressed = 0; // a flag variable
boolean sweepAbort = LOW;

//菜单选择
byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 3; // This is the number of submenus/settings you want
byte setting1 = 0;  // a variable which holds the value we set 
byte setting2 = 0;  // a variable which holds the value we set 
byte setting3 = 0;  // a variable which holds the value we set 
/* Note: you may wish to change settingN etc to int, float or boolean to suit your application. 
 Remember to change "void setAdmin(byte name,*BYTE* setting)" to match and probably add some 
 "modeMax"-type overflow code in the "if(Mode == N && buttonPressed)" section*/

double Fstart_MHz = 1;  // Start Frequency for sweep
double Fstop_MHz = 10;  // Stop Frequency for sweep
double current_freq_MHz; // Temp variable used during sweep
int band_choice = 0; // doesn't need to be zeroed as this happens during initial sweep - used for single band sweeps and graph plotting
double freq_MHz_min_VSWR; // stores the frequency of min swr during a sweep
double min_VSWR;  // stores minimum VSWR found during sweep
long serial_input_number; // Used to build number from serial stream
int num_steps = 100; // Number of steps to use in the sweep
char incoming_char; // Character read from serial stream
char band[6]; // Character holding current band in metres
char minfreq[8];
char maxfreq[8];

boolean use_calibration=HIGH;  // apply logarithmic correction coefficients from resistor measurements

int menuChoice=0;

long hfbands [10][4]={
  {160,1810000,2000000,64},
  {80,3500000,3800000,100}, // wavelength,Fstart,Fstop,nsteps (nsteps chosen for ~3 kHz intervals)
  {60,5258500,5406500,50},
  {40,7000000,7200000,67},
  {30,10100000,10150000,17},
  {20,14000000,14350000,117},
  {17,18068000,18168000,33},
  {15,21000000,21450000,150},
  {12,24890000,24990000,33},
  {10,28000000,29700000,150}  // nsteps for this one should really be 567, but that would take ages!
};

double swr_results [11][4]={
  
  //SWRmin,Freq(SWRmin)in MHz,correction gradient (*ln(X)), correction intercept
  // separate array from hfbands
  // because this stores floats
  
  {0,0,3.500,0.660},  // correction needs to be measured for 160 m
  {0,0,3.548,0.623},
  {0,0,3.575,0.590},  // ditto for 60 m 
  {0,0,3.609,0.570},                        
  {0,0,3.696,0.500},                        
  {0,0,3.758,0.413},
  {0,0,3.767,0.257},
  {0,0,3.804,0.187},
  {0,0,3.818,0.118},
  {0,0,3.754,0.050},
  {0,0,0,0}                         // this last slot is for arbitrary sweeps
};
  
double moving_avg_swr [9];        // nine-point moving average for VSWR

int graph_array [150];          // storage array for single graph - try storing vswr values as ints (after multiplying by 100) to save memory

char minvswr[6];
char minfreqdisp[16];
  
// the setup routine runs once on reset:
void setup() {
  
  //DDS模块腿脚定义
  pinMode(FQ_UD,OUTPUT);
  pinMode(SCLK,OUTPUT);
  pinMode(SDAT,OUTPUT);
  pinMode(RESET,OUTPUT);
  
  //LED定义
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  //编码器定义
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP); 
  attachInterrupt(0,PinA,RISING); 
  attachInterrupt(1,PinB,RISING); 
  
  //编码器按下定义
  pinMode (buttonPin, INPUT_PULLUP); // setup the button pin

  //驻波比信号定义
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  analogReference(INTERNAL);
  
  //串口输出
  Serial.begin(9600);

  //重启DDS模块
  digitalWrite(RESET,HIGH);
  digitalWrite(RESET,LOW);
  
  //初始化端口
  serial_input_number=0;

  //启动画面
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_9x15B);
      u8g.drawStr(5,15,"   HF VSWR");
      u8g.drawStr(0,30," analyser v1");
      u8g.setFont(u8g_font_7x13);
      u8g.drawStr(0,50,"    by BI3QWQ");
      if(use_calibration==LOW) u8g.drawStr(0,63,"Uncalibrated VSWR!");
    } while(u8g.nextPage() );

    Serial.println("HF VSWR analyser by BI3QWQ");

      SetDDSFreq(7000000);  // 默认开启7Mhz

      delay(1000);  // 停留DDS启动画面时间1秒

      //预先读取信号1次
      analogRead(A0);
      delay(100);
      analogRead(A1);
      delay(100);
      
      //上电时候默认扫描80-10米波段
      //按下reset恢复
      
      Sweep_bands();
      Sweep_report();

      //等待按下取消键
      do { 
      } while(buttonCycled() == LOW);

}

void loop() {
  
	cli();  // 禁用中断以防止在操作中改变volatile变量
	if(oldEncPos!=encoderPos){
		if(encoderPos>200){
			encoderPos=0;
		}
		if(encoderPos>2){
			encoderPos=2;
		}
		oldEncPos=encoderPos;
	}
	sei();  //再次打开它们

	//显示菜单 
	u8g.firstPage();
	do {
		u8g.setFont(u8g_font_7x13);
		u8g.drawStr(4,15,"*** Main Menu ***");
		u8g.drawStr(0,30,"  Sweep all bands");
		u8g.drawStr(0,45,"  Sweep one band");
		u8g.drawStr(0,60,"  Single frequency");
		u8g.drawStr(0,30+15*encoderPos,">");
	} while(u8g.nextPage() );

    if(buttonCycled() == HIGH){
		switch (encoderPos){

		//扫描所有HF波段并显示摘要
		case 0:
			Sweep_bands();
			Sweep_report();

			//在清除报告之前等待按钮按下并返回菜单
			do {  
			} while(buttonCycled() == LOW);

			encoderPos=0;
			break;

		//扫一个HF频段
		case 1: 

			band_choice=Choose_band();
			Serial.print(hfbands[band_choice][0]);
			Serial.println('m');

			Fstart_MHz = ((double)hfbands[band_choice][1])/1000000;
			Fstop_MHz = ((double)hfbands[band_choice][2])/1000000;
			num_steps = hfbands[band_choice][3];
			Perform_sweep(band_choice);

			//在清除报告之前等待按钮按下并返回菜单
			do {  
			} while(buttonCycled() == LOW);

			//现在需要生成图表
			plot_graph(band_choice);

			//在清除报告之前等待按钮按下并返回菜单
			do {  
			} while(buttonCycled() == LOW);

			encoderPos=1;
		break;
			//在可调节的现场频率下实时VSWR
			case 2:

			band_choice=Choose_band();
			current_freq_MHz = double(hfbands[band_choice][1] + hfbands[band_choice][2])/2000000;

			Spot_freq();

			encoderPos=2;
			break;
		}
   }

}

//选择扫描频段
int Choose_band(){

	encoderPos=band_choice; //记住最后一个旋钮的选择，而不是每次都要扭曲纠正旋钮
	//循环重复直到按下按钮然后释放
	do {  
		// 禁用中断以防止在操作中改变volatile变量
		cli(); 
		if(oldEncPos!=encoderPos){
			if(encoderPos>200){
				encoderPos=0;
			}
			//在增加两个波段之前是7
			if(encoderPos>9){ 
				encoderPos=9;
			}

			oldEncPos=encoderPos;
		}
		sei();  //再次打开它们

		itoa(hfbands [encoderPos][0],band,10);
		dtostrf(double(hfbands[encoderPos][1])/1000000,6,3,minfreq);
		dtostrf(double(hfbands[encoderPos][2])/1000000,6,3,maxfreq);

		u8g.firstPage();
		do {
			u8g.setFont(u8g_font_7x13);
			u8g.drawStr(4,15,"** Choose Band **");
			u8g.drawStr(0,30,"   >    m band");
			u8g.drawStr(35,30,band);
			u8g.drawStr(0,45,"  Min:       MHz");
			u8g.drawStr(0,60,"  Max:       MHz");
			u8g.drawStr(45,45,minfreq);
			u8g.drawStr(45,60,maxfreq);

		} while(u8g.nextPage() );

	} while(buttonCycled() == LOW);

	return(encoderPos);
}

//扫描全部波段
void Sweep_bands(){
	for(int i=0;i<=9;i++){  //在新波段加入之前是7
		Serial.print(hfbands[i][0]);
		Serial.println('m');
		Fstart_MHz = ((double)hfbands[i][1])/1000000;
		Fstop_MHz = ((double)hfbands[i][2])/1000000;
		num_steps = hfbands[i][3];

		band_choice=i;  //需要确保为波段应用正确的校准因子


		//需要将当前最小VSWR和freq传递给扫描例程
		//最容易使用数组下标
		Perform_sweep(i);

		//测试中止（即按下按钮）
		if(sweepAbort==HIGH){
			sweepAbort=LOW;
			return;
		}
	}
}


void Perform_sweep(int j){
	char freqdisp[16];
	char vswrdisp[16];
	char vswrchar[6];
	char minvswr[6];
	char minfreqdisp[16];
	int avg_ctr=0;  //移动平均（超频）阵列的计数器

	double AVG_VSWR;
	double Fstep_MHz = (Fstop_MHz-Fstart_MHz)/num_steps;

	swr_results[j][0]=0;  //清除此频段最小值
	swr_results[j][1]=0;

	memset(graph_array,0,sizeof(graph_array));  //初始化数组为0

	//开始循环
	for(int i=0;i<=(num_steps+8);i++){  //pad循环，另外8个步骤，9 pt移动平均线
		if(buttonCycled()==HIGH){
			sweepAbort=HIGH;
			return;
		}
			
		//计算当前频率
		current_freq_MHz = Fstart_MHz + (i-4)*Fstep_MHz;  //由于9 pt移动平均线，Fstart之前的四个额外步骤
		//将DDS设置为当前频率
		SetDDSFreq(current_freq_MHz*1000000);
		//等待一段时间 - 需要至少50毫秒（参见开始时的更新信息）
		delay(50+100*(i<1));

		moving_avg_swr[avg_ctr]=Get_VSWR(); //将值加载到平均数组中的下一个可用槽中
		
		//计算阵列的平均值 - 原油但有效
		AVG_VSWR=(moving_avg_swr[0]+moving_avg_swr[1]+moving_avg_swr[2]+moving_avg_swr[3]+moving_avg_swr[4]+moving_avg_swr[5]+moving_avg_swr[6]+moving_avg_swr[7]+moving_avg_swr[8])/9;
	 
		// 通过串行总线将当前线路发送回PC  - 这可能会导致中断问题，但在扫描期间没有理由使用这些线路来读取编码器
		
		if(i>=8){//VSWR值仅在处理了8次测量后才有效
	 
		//如果没有溢出，则将值加载到图形数组中
		if(i<158){
			graph_array[i-8]=int(AVG_VSWR*100);
		}
		
		Serial.print(long((current_freq_MHz-4*Fstep_MHz)*1000000)); //请记住，当前avg_VSWR对应的频率是四次迭代
		Serial.print(",");
		Serial.println(AVG_VSWR);

		//测试最小VSWR  - 存储并通知是否找到新的最小值
		if((AVG_VSWR<swr_results[j][0])||(swr_results[j][0]==0)){
			swr_results[j][0]=AVG_VSWR;
			swr_results[j][1]=current_freq_MHz-4*Fstep_MHz;  //记得减去4个bin偏移量为9 pt移动平均值
		}
		// 测试任何扫描的最小VSWR  - 请注意这个数组的行[10]（在添加波段之前是8）实际上是否已经被使用 - 遗留代码？
		if((AVG_VSWR<swr_results[10][0])||(swr_results[10][0]==0)){
			swr_results[10][0]=AVG_VSWR;
			swr_results[10][1]=current_freq_MHz-4*Fstep_MHz;  //记得减去4个bin偏移量为9 pt移动平均值
		}


		//现在把它放在OLED屏幕上
		//将数字转换为字符串...
		dtostrf(current_freq_MHz-4*Fstep_MHz,6,3,freqdisp);
		dtostrf(AVG_VSWR,5,2,vswrdisp);
		//在整个范围/任何波段上的最小值
		dtostrf(swr_results[j][1],6,3,minfreqdisp);
		dtostrf(swr_results[j][0],5,2,minvswr);

		//这最后两行用于显示[8]而不是[j]  - 对单频段扫描没有用

		//然后用'图片循环'显示它们
		u8g.firstPage();
		do {
			u8g.setFont(u8g_font_7x13);
			u8g.drawStr(0,15,"Sweeping frequency");
			u8g.drawStr(0,30,"Freq:       MHz");
			u8g.drawStr(0,45,"VSWR: "); 
			u8g.drawStr(40,30,freqdisp);
			u8g.drawStr(40,45,vswrdisp);

			u8g.setFont(u8g_font_6x13);
			u8g.drawStr(0,60,"Min       @       MHz");
			u8g.drawStr(24,60,minvswr);
			u8g.drawStr(70,60,minfreqdisp);

		//u8g.drawStr(90,30,"MHz");
		} while(u8g.nextPage() );

	}
    
    avg_ctr++;                    // 增加移动平均数组计数器以填充下一个插槽
    if(avg_ctr>8) avg_ctr=0;      //如果插槽号超过8（=阵列的第九个插槽），则回退为零
      
	}
    
}

//选择频率
void Spot_freq(){
	int level=0;  // 控制编码器'水平'w.r.t.频率：0 =选择数字; 1 =选择数字值
	int digitsPos=0;
	long digitsVal;
	double fHz=(current_freq_MHz*1000000);
	char freqdisp [9];
	char vswrdisp [6];
	encoderPos=0;

	do{  // 重复此代码块，直到用户退出子例程

		cli();  // 读取编码器 - 禁用中断以防止在运行中更改易失变量
		if(oldEncPos!=encoderPos){
			if(encoderPos>200){
				encoderPos=0;
			}
			if(encoderPos>8+1*level){  //根据您所处的调整级别设置允许的最大编码器值
				encoderPos=8+1*level;
			}
			oldEncPos=encoderPos;
		}
		sei(); //再次打开它们


		dtostrf(fHz,8,0,freqdisp);
		dtostrf(Get_VSWR(),5,2,vswrdisp);


		if(level==0)
			digitsPos=encoderPos;

		if(level==1){
			if(digitsVal<encoderPos && encoderPos<=9){
				fHz=fHz+shift(digitsPos);
				if(fHz>40000000) fHz=fHz-shift(digitsPos);  // 测试超出范围并撤消更改，如果它会导致这种情况
					digitsVal=long(fHz/shift(digitsPos)) % 10;  // 从fHz获取digitsVal的新值 - 使用模数学％
			}

			if(digitsVal>encoderPos && encoderPos>=0){
				fHz=fHz-shift(digitsPos);
				if(fHz<1) fHz=fHz+shift(digitsPos);  // 测试超出范围并撤消更改，如果它会导致这种情况
					digitsVal=long(fHz/shift(digitsPos)) % 10;  // 使用模数学％
			}
		}
		//

		//设置DDS频率
		SetDDSFreq(fHz);

		//是否退出按钮
		if(buttonCycled()==HIGH){
			if(digitsPos==8){  //选择EXIT退出
				return;
			}
			if(level==0 && digitsPos<8)
			{
				level=1;
				digitsVal=long(fHz/shift(digitsPos)) % 10;  //使用%号
				encoderPos=digitsVal;
			} 
			else 
			{
				level=0;
				encoderPos=digitsPos;

			};  // 跳转到数字调整或再次输出
		}

		//使用'picture loop'显示当前频率和光标
		u8g.firstPage();
		do {
			u8g.setFont(u8g_font_7x13);
			u8g.drawStr(0,15," Select frequency");
			u8g.drawStr(0,60,"VSWR:         exit");
			u8g.setFont(u8g_font_9x15B);
			u8g.drawStr(90,38,"Hz");
			u8g.drawStr(15,38,freqdisp);
			u8g.drawStr(40,60,vswrdisp);
			if(level==0){
				if(digitsPos<8){
					u8g.drawLine(14+9*digitsPos,40,23+9*digitsPos,40);
				}
				else
				{
					u8g.drawLine(97,62,125,62);
				}
			}
			else 
			{
				u8g.drawBox(14+9*digitsPos,40,9,3);
			}
		} while(u8g.nextPage() );


	}while(HIGH);
}

double shift(int dpos){

	double shift=1;
	for(int i=1;i<=(7-dpos);i++){
		shift=shift*10;
	}
	return shift;
}

//计算驻波
double Get_VSWR(){
	double FWD=0;
	double REV=0;
	double VSWR;
	double FWD5=0;  //对每个频率进行5点平均求和
	double REV5=0;
	double temp_VSWR;

	//读取正向和反向电压 - 复位后在第一次迭代时始终返回无意义（即使我执行多次读取）
	FWD5=0;
	REV5=0;
	for(int k=0;k<5;k++){
		analogRead(A0);
		delay(10);
		REV5 = REV5 + analogRead(A0);
		analogRead(A1);
		delay(10);
		FWD5 = FWD5 + analogRead(A1);
	}
	FWD=FWD5/5;  //进行平均计算
	REV=REV5/5;

	if(REV>=FWD){
		//为避免除以零或负VSWR，则设置为0
		temp_VSWR = 0;
	}
	else
	{
		//计算驻波比
		temp_VSWR = (FWD+REV)/(FWD-REV);
	}

	// 是否校准
	if(temp_VSWR>0 && use_calibration==HIGH){
		temp_VSWR=(swr_results[band_choice][2])*log(temp_VSWR)+(swr_results[band_choice][3]);  //根据定义的校准数组
		if(temp_VSWR<1)temp_VSWR=1;
	}
	return temp_VSWR;
}

//扫描报告
void Sweep_report(){

	char minvswr[6];
	char minfreqdisp[16];

	u8g.firstPage();
	do {
		u8g.setFont(u8g_font_04b_03br);
		u8g.drawStr(1,6,"160m:          @           MHz");
		u8g.drawStr(4,12,"80m:          @           MHz");
		u8g.drawStr(4,18,"60m:          @           MHz");
		u8g.drawStr(4,24,"40m:          @           MHz"); 
		u8g.drawStr(4,30,"30m:          @           MHz");
		u8g.drawStr(4,36,"20m:          @           MHz");
		u8g.drawStr(6,42,"17m:          @           MHz");
		u8g.drawStr(6,48,"15m:          @           MHz");
		u8g.drawStr(6,54,"12m:          @           MHz");
		u8g.drawStr(6,60,"10m:          @           MHz");

		//使用每个波段的最小VSWR值填充表

		for(int i=0;i<10;i++){
			dtostrf(swr_results[i][1],6,3,minfreqdisp);
			dtostrf(swr_results[i][0],5,2,minvswr); 
			u8g.drawStr(25,(6*(i+1)),minvswr);  //删除（i + 1）并替换为（i）
			u8g.drawStr(60,(6*(i+1)),minfreqdisp);
		}

	} while(u8g.nextPage() );
        
}

//DDS设置频率
void SetDDSFreq(double Freq_Hz){
	//设置频率
	int32_t f = Freq_Hz * 4294967295/125000000;

	//发送1个字节
	for (int b=0;b<4;b++,f>>=8){
		send_byte(f & 0xFF);
	}

	//第5个字节需要为零
	send_byte(0);

	//选通更新引脚以告知DDS使用值
	digitalWrite(FQ_UD,HIGH);
	digitalWrite(FQ_UD,LOW);
}

void send_byte(byte data_to_send){
	//位在SPI总线上的字节位
	for (int i=0; i<8; i++,data_to_send>>=1){
		//在输出引脚上设置数据位
		digitalWrite(SDAT,data_to_send & 0x01);
		//选通时钟引脚
		digitalWrite(SCLK,HIGH);
		digitalWrite(SCLK,LOW);
	}
}


//按键-检测
void checkButton() {
  
	//按钮读取非延迟（）去抖
	byte buttonState = digitalRead (buttonPin); 
	if (buttonState != oldButtonState){
		if (millis () - buttonPressTime >= debounceTime){ // 去抖
			buttonPressTime = millis ();  // 当我们关闭开关时
			oldButtonState =  buttonState;  //记住下次
			if (buttonState == LOW){
				buttonPressed = 1;
			}
			else {
				buttonPressed = 0;  
			}  
		} 
	} 

}

//按键-取消
boolean buttonCycled() {    // 用于检查按钮按下和释放的子程序 - 如果按下按钮时调用，则在释放之前不会返回
	checkButton();
	if(buttonPressed == HIGH){
		//在开始之前等待按钮被释放
		do {    
			checkButton();
		} while(buttonPressed == HIGH);
		
		return HIGH;
	} 
	else
	{ 
		return LOW;
	}
}
   


//编码器-中断
void PinA(){

  reading = PIND & 0xC; //读取所有8个引脚值，然后除去pinA和pinB的所有值
  if(reading == B00001100 && aFlag) { //检查我们是否有两个引脚处于制动状态（HIGH），并且我们预计此引脚上升时会产生制动edge
    encoderPos++; //减少编码器的位置计数
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100) bFlag = 1; //表示我们期望pinB发出从自由旋转到制动过渡的信号

}

//编码器-中断
void PinB(){

  reading = PIND & 0xC;
  if (reading == B00001100 && bFlag) {
    encoderPos--; 
    bFlag = 0; 
    aFlag = 0;
  }
  else if (reading == B00001000) aFlag = 1; 
}

//绘制曲线图
void plot_graph(int bc){

	int x;  // points to plot on graph
	int y;  
	int xold;  // last plotted point - used for line drawing
	int yold;
	int maxVSWR=0;
	int ceilmaxVSWR=0;
	int meanVSWR=0;
	char MHZstart[7];
	char MHZstop[7];
	char ylabel[3];

	Fstart_MHz = ((double)hfbands[bc][1])/1000000;
	Fstop_MHz = ((double)hfbands[bc][2])/1000000;
	num_steps = hfbands[bc][3];
	dtostrf(Fstart_MHz,6,3,MHZstart);
	dtostrf(Fstop_MHz,6,3,MHZstop);  

	// find max and mean VSWR in dataset for band
	for(int i=0;i<num_steps;i++){
	if(maxVSWR<graph_array[i]) maxVSWR=graph_array[i];
		meanVSWR=meanVSWR+graph_array[i];
	}
	meanVSWR=meanVSWR/num_steps;

	ceilmaxVSWR=ceil(float(maxVSWR)/100);
	Serial.println(float(maxVSWR));
	Serial.println(float(maxVSWR)/100);
	Serial.println(ceilmaxVSWR);


	// display with a 'picture loop'
	u8g.firstPage();
	do {
		// generate axes and labels
		u8g.setFont(u8g_font_5x7);
		u8g.drawStr(14-5*(Fstart_MHz<10),63,MHZstart);    // this line not working - doesn't like variable. mem problem?
		u8g.drawStr(61,62,"F/MHz");
		u8g.drawStr(96,63,MHZstop);
		u8g.drawStr(6,57,"1");
		u8g.drawStr(0,20,"V");
		u8g.drawStr(0,28,"S");
		u8g.drawStr(0,36,"W");
		u8g.drawStr(0,44,"R");

		itoa(ceilmaxVSWR,ylabel,10);
		u8g.drawStr(6,7,ylabel);

		u8g.drawHLine(12,54,115);
		u8g.drawVLine(13,0,56);


		for(int i=0;i<num_steps;i++){
			// 将数组值映射到OLED上可用的100x54网格
			x=14+int(113*float(i)/float(num_steps));  //自动缩放x
			if(graph_array[i]>=100){
				y=int(54*(1-((float(graph_array[i])/100)-1)/(ceilmaxVSWR-1)));  //自动缩放

			} else {
				y=54;  //有效地不绘制这一点（隐藏在轴下）
			}

			//检查y溢出（对应于尝试绘制OLED的顶部），现在不应该使用自动缩放
			if(y<0) 
				y=0;

			if(i==0){  //在这种情况下，第一个点没有有意义的“旧”值来绘制一条线，因此将旧值设置为当前值
				xold=x;
				yold=y;
			}

			u8g.drawLine(xold,yold,x,y);

			xold=x;  //存储下一次迭代的值
			yold=y;
		}

		// 显示最小VSWR和发生频率 - 尽量远离VSWR曲线

		y=8+40*((meanVSWR-100)>50*ceil(float(maxVSWR/100)));  //通过比较平均VSWR和y轴中点来估计曲线的大小

		dtostrf(swr_results[bc][1],6,3,minfreqdisp);
		dtostrf(swr_results[bc][0],5,2,minvswr); 
		u8g.drawStr(22,y,"Min      @        MHz");
		u8g.drawStr(40,y,minvswr);
		u8g.drawStr(74,y,minfreqdisp);

	} while(u8g.nextPage() );
}
