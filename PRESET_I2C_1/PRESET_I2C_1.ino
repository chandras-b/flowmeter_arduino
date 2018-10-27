// LATEST PRESET I2C 20022018

#include <Average.h>       //Load the Average library
#include <math.h>          //Load the Math library
#include <EEPROMex.h>      //Load the EEPROMex library
#include <EEPROMVar.h>     //Load the EEPROMVar library
#include <TimerOne.h>      //Load the TimerOne library
#include <PinChangeInt.h>  //Load the PinChangeInt library
#include <LiquidCrystal.h> //Load the LCD library
#include <FreqCounter.h>   //Load the FreqCounter library
#include <MCP4725.h>       //Load the MCP4725 library


long int frq;           // set frq as long int
MCP4725 dac(A4, A5);    // pins A4 AND A5 used for 4-20 
float _min,_max,x;      // set _min,_max and x as float for LPM 
float _minV=0.857;      // min voltage calibration for dac to set 4 volts
float _maxV=4.3257;     // min voltage calibration for dac to set 20 volts

                      
#define TIMER_US 1000   // set timer for 64 us per tick                      
#define TICK_COUNTS 2   // set timer for 2 counts                       
volatile long tick_count = TICK_COUNTS;       
volatile bool in_long_isr = false; 

#define LCD_REFRESH_INTERVAL_MS 200   // refresh LCD after every 200 ms
LiquidCrystal lcd(12,3,A0,A1,A2,A3);  //Define the pins for LCD
#define INTERRUPT_INPUT 5   // Pin 5 is allocated for pulse input
double pulse_counter = 0.0; // set pulse counter to 0.0
double pulse_counter1 = 0.0;// set pulse_counter1 to 0.0
double pc = 0.0;            // set pc to 0.0
double pc1 = 0.0;           // set pc1 to 0.0
int bcum = 0.0;             // set bcum to 0.0
int xcum = 0.0;             // set xcum/batch to 0.0
int n=0;                    // set n to 0
int k=0;                    // set k to 0
int batch=0;                // set batch to 0
int sample=0;               // set sample to 0
int bstart=0;               // set bstart to 0
float calib_qty=0;          // set calib_qty to 0
int calib_qty_reg=95;       // set calib_qty_reg to 95
int total_reg_no=99;        // set total_reg_no to 99
float ec_value=0;           //error correction value to 0
int ec_value_reg=103;       // set error correction value register at 103
float microec_value=0;      // set micro error correction value
int microec_value_reg=107;  // set micro error correction value register at 107
float preset_qty=0;         // set preset qty value
int preset_qty_reg=111;     // set preset qty value register at 111
int last_batch_reg=115;     // set last batch qty value register at 115

int DCgone = 11;  //Define the pin for power fail
int unsaved= HIGH;//dont save value when power is there
int DCgoneState= LOW;//save when power fails

unsigned long start, finished, elapsed, start1, fin1, elap1,start2,fin2,elap2,lpm_iter;
float lpm_avg=0;
float lpm_420=4;
float _xcum;          //Delivered qty
double _total;        //Total qty
double _prevTotal;
long _eepromWrites;
int Start_flag=0; 
int InputRoutine=1;
String readString;
int xcount=0;
int BatchPulseCount=0;
long BatchTime=0;
int Reg=0;
int max_reg_points_calibration = 15; // maximum no. of points allowed to be stored
int points_regno = 91; //Register No. where no. of points are stored
int readReg=0;

//int _Points=(sizeof(_Pulses)/sizeof(float));
int _Points=15; // Define max number of points that can be taken for calibration
const unsigned int MAX_INPUT = 10;

byte ButtonA = 6;       // Button A is define on pin 6
byte ButtonB = 7;       // Button B is define on pin 7
byte ButtonC = 8;       // Button C is define on pin 8
byte ButtonD = 9;       // Button D is define on pin 9
byte ButtonE = 10;      // Button E is define on pin 10

String menu;

const int numReadings = 10;
float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

//float _Pulses[]={};
int fq_int=0;

int dec_lpm=0;    //decimal places for lpm display
int dec_batch=0;  //decimal places for batch display
int dec_total=0;  //decimal places for total display
int dec_preset=0;  //decimal places for total display

int lpm_interval=4; //no. of seconds for lpm average display
float preset_qty2=20; //First cutoff Qty

#define RELAY1  13    // Pin 13 is allotted to relay 1                         
#define RELAY2  4     // Pin 4 is alloted to relay 2                       

float dac_v=0;
float val_420=4;
int lpm_mode=2; //1=lps, 2=lpm, 3=lph
int display_mode=1;
//1=Preset without 4-20, 
//2=LPTT with 4-20, 
//3=LPTT without 4-20, 
//4=Preset with 4-20

int power_mode=1;
          //0=without power circuit
          //1=with power circuit

int nnn;

void setup()
{
  pinMode ( DCgone, INPUT ); // set pin 11 as input for power fail
digitalWrite(DCgone, HIGH);  // sets pullup resistor
//EEPROM.setMaxAllowedWrites(29);
  Serial.begin(9600);       // Initialise the serial write 
  
  pinMode(ButtonA, INPUT);     //Button A is set as an input
  digitalWrite(ButtonA, HIGH); // ON when pressed
  pinMode(ButtonB, INPUT);     //Button B is set as an input
  digitalWrite(ButtonB, HIGH);
  pinMode(ButtonC, INPUT);     //Button C is set as an input
  digitalWrite(ButtonC, HIGH);
  pinMode(ButtonD, INPUT);     //Button C is set as an input
  digitalWrite(ButtonD, HIGH);
  pinMode(ButtonE, INPUT);     //Button D is set as an input  
  digitalWrite(ButtonE, HIGH);

  pinMode(RELAY1, OUTPUT);  //Set relay 1 as an output       
  pinMode(RELAY2, OUTPUT);  //Set relay 2 as an output
  
  dac.begin();
  dac.setVoltage(_minV);                   //Set to 4 volts
  pinMode(INTERRUPT_INPUT, INPUT_PULLUP);  //set pulse input as input pullup
  Timer1.initialize(TIMER_US);             // Initalize timer1 and set is to 64 ms          
  Timer1.attachInterrupt( timerIsr );      //attach timerIsr as overflow interupt      
  
  lcd.begin(20,4);    // Tell ardiuno our LCD is 16x4 line display.
  while(!eeprom_is_ready());
  readReg=EEPROM.readInt(points_regno); //EEPROM to read no of calibrations points fed
  while(!eeprom_is_ready());
  calib_qty=EEPROM.readInt(calib_qty_reg);//To read the calibration quantity
  while(!eeprom_is_ready());
  ec_value=EEPROM.readFloat(ec_value_reg);//TO read the registered EC value
  while(!eeprom_is_ready());
  preset_qty=EEPROM.readFloat(preset_qty_reg);// To read the preset qty
  while(!eeprom_is_ready());
  microec_value=EEPROM.readFloat(microec_value_reg);// To read the registered MicroEC value
  if(isnan(microec_value))microec_value=0;
  while(!eeprom_is_ready());
  _total=EEPROM.readDouble(total_reg_no);//Read total
  if(isnan(_total)){_total=0;}
  while(!eeprom_is_ready());
  _xcum=EEPROM.readFloat(last_batch_reg);//Read batch delivered qty
  if(isnan(_xcum)){_xcum=0;}
  
//  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
//    readings[thisReading] = 0;
//  }

_min=0;   //put min lpm value here for 4-20
_max=100;  //put max lpm value here for 4-20
  }

void loop()
{   
  // set for LPM mode
  if(display_mode==2 || display_mode==3){_LPM(0);} 
  
  // set for preset mode display as below
  if(display_mode==1 || display_mode==4){          
  if(xcount==0){
    lcd.clear();
    lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
  
    lcd.print("PRESS MENU1 TO START");

    lcd.setCursor(0,1); //set cursor on 0/first col and 1/second line
    lcd.print("DLQTY=");
    
    lcd.setCursor(0,2); //set cursor on 0/first col and 2/third line
    lcd.print("TOTAL=");
    
    lcd.setCursor(0,3); //set cursor on 0/first col and 3/fourth line
    lcd.print("PRSET=");
        
    lcd.setCursor(6,1); lcd.print(_xcum,dec_batch);//set cursor on 6/seventh col and 1/second line and print del qty + dec qty
    lcd.setCursor(6,2); lcd.print(_total,dec_total);//set cursor on 6/seventh col and 2/third line and print total qty + dec qty
    lcd.setCursor(6,3); lcd.print(preset_qty, dec_preset);//set cursor on 6/seventh col and 3/fourth line and print preset qty only

    xcount++;
  }
  if(digitalRead(ButtonA) == LOW ){  
    _LPM(0);
  }
  if(digitalRead(ButtonB) == LOW){ 
    menu=""; 
    _setPresetQty(0);
  }
  if(digitalRead(ButtonE) == LOW){ 
    menu=""; 
    _xcum=0;
  }
  
  if(digitalRead(ButtonD) == LOW){delay(250);menu+="A";}
  if(digitalRead(ButtonE) == LOW && menu=="A"){delay(250);menu+="B";}
  if(digitalRead(ButtonC) == LOW && menu=="AB"){delay(250);menu+="C";}
  if(menu=="ABC"){delay(250);_calMenu(0);} // Button sequence to initialize cal menu
  }
delay(LCD_REFRESH_INTERVAL_MS);
}


void _LPM(int r){
digitalWrite(RELAY1, HIGH);
digitalWrite(RELAY2, HIGH);
int lpm_init=0;
float round_lpm;
float fq1,fq2,p1,_factor,_lpm,_lpq,m1;
long t1; float lpq1;
int lpm_delay=0;
float _lpm1;
int lpm_iterations=50;
int _Interval=100; 
int kk=0; int nn=0;
lpq1=0;
lpm_iter=(lpm_interval*1000)/_Interval;
lcd.clear();
start=millis();
pc1=0;
BatchPulseCount=0;
//Serial.print("Batch ");
//Serial.print(batch);
//Serial.println(" started..");

lcd.setCursor(0,0);
if(lpm_mode==1){lcd.print("LPS="); }// if lpm mode is set to 1 then display LPS on first col and first line of LCD
if(lpm_mode==2){lcd.print("LPM="); }// if lpm mode is set to 2 the display LPM on first col and first line of LCD
if(lpm_mode==3){lcd.print("LPH="); }// if lpm mode is set to 3 the display LPH on first col and first line of LCD
lcd.setCursor(0,1);             //set cursor on 0/first col and 1/second line
lcd.print("DLQTY=");
lcd.setCursor(0,2);             //set cursor on 0/first col and 2/third line
lcd.print("TOTAL=");
lcd.setCursor(0,3);             //set cursor on 0/first col and 3/fourth line
if(display_mode==1 || display_mode==4)lcd.print("PRSET=");// display PRESET in 4th line
if(display_mode==2)lcd.print("4-20 =");                   // display 4-20 in 4th line
if(display_mode==3)lcd.print("I2C SYSTEMS");              // display I2C SYSTEMS in 4th line

menu=""; batch++; 
//_xcum=0; 
elap2=0; elap1=0; lpm_delay=0; lpm_init=0; _lpm1=0; lpq1=0;
if(nn<lpm_iter){lcd.setCursor(4,0); lcd.print("");}
while(r==0){

if(power_mode==0){DCgoneState == LOW;}
if(power_mode==1){DCgoneState = digitalRead ( DCgone );}  
  if ( DCgoneState == LOW ){
  
  if(pulse_counter>0)lpm_init++;
  if(lpm_delay>lpm_iterations or pulse_counter1==0){lpm_delay=0;}

  // Button sequence to initialize cal menu
  if(digitalRead(ButtonD) == LOW){delay(250);menu+="A";}
  if(digitalRead(ButtonE) == LOW && menu=="A"){delay(250);menu+="B";}
  if(digitalRead(ButtonC) == LOW && menu=="AB"){delay(250);menu+="C";}
  if(menu=="ABC"){delay(250);_calMenu(0);} 


  if(pulse_counter1==0 && _total>_prevTotal){
  //while(!eeprom_is_ready());
  //EEPROM.updateDouble(total_reg_no, _total);
  //while(!eeprom_is_ready());
  //EEPROM.updateFloat(last_batch_reg, _xcum);
  //_eepromWrites++; _prevTotal=_total;
  }

 if(digitalRead(ButtonE) == LOW )
 // EMERGENCY STOP BUTTON TO STOP FLOW AND RESET FROM ZERO
 {  
 menu=""; batch++; _xcum=0; elap2=0; elap1=0; lpm_delay=0; lpm_init=0; _lpm1=0; lpq1=0;val_420=4;
 _BatchReset();
 //         while(!eeprom_is_ready());
 //         EEPROM.writeFloat(last_batch_reg, _xcum);
  digitalWrite(RELAY1,LOW);
  digitalWrite(RELAY2,LOW);
  }


  
  pulse_counter1=pulse_counter;
  pulse_counter=0;
  start1=millis();
  fin2=millis();
  pulse_counter1+=pulse_counter;     //new
  pulse_counter1+=pc1;     //new
  pc1=0;
  BatchPulseCount+=pulse_counter1;
  pc+=pulse_counter;
  elap1=start1-fin1;
  fq1=(float)(pulse_counter1*_Interval*10)/elap1;
  fq_int=round(fq1);
  if(fq_int>84){fq_int=84;}
  //p1=_Pulses[fq_int];
  p1=_maxmin1(fq1,readReg);
  m1=_maxmin2(fq1,readReg);
  if(ec_value>0)calib_qty=ec_value;
  if(p1==0){
    _factor=0;
  }else{
    _factor=(float)calib_qty/p1;
  }
  _lpq=(_factor*fq1)/10;
  if(pulse_counter1>0)_lpq+=microec_value/(100*_Interval);
  if(pulse_counter1>0)_lpq+=m1/10000000;
  _lpm=_lpq*(60000/_Interval);  
  lpq1+=_lpq;  
  _xcum+=_lpq;
  _total+=_lpq;
  if(pulse_counter1>0){
    elap2+=elap1;
  }

  if(lpm_init==3){
  round_lpm=round(_lpm/100)*100;
  lcd.setCursor(4,0); lcd.print("            ");
//  lcd.setCursor(4,0); lcd.print(round_lpm,dec_lpm); 
  lcd.setCursor(4,0); lcd.print(_lpm,dec_lpm); 
  }
  if(nn>lpm_iter){
    lpm_avg=(lpq1*60)/lpm_interval;
    if(lpm_mode==1){lpm_avg=lpm_avg/60;}//if flow is set to LPS
    if(lpm_mode==3){lpm_avg=lpm_avg*60;}//If flow is set to LPH
    lpm_420=lpm_avg-_min;
    if(lpm_avg<0)lpm_420=4;
    x=_minV+(lpm_420*((_maxV-_minV)/(_max-_min)));
    if(x<_minV)x=_minV;    // Set to 4 volts
    if(x>_maxV)x=_maxV;    // Set to 20 volts
//    val_420=((x-_min)*((_maxV-_minV)/(_max-_min)))+4;
    val_420=((x-_minV)*((16)/(_maxV-_minV)))+4;
//    val_420=(_maxV-_minV)/(_max-_min);
//    val_420*=(x-_min);
//    val_420+=4.01;
    dac.setVoltage(x); 
    lpq1=0;
    lcd.setCursor(4,0); lcd.print("            ");
    lcd.setCursor(4,0); lcd.print(lpm_avg,dec_lpm); 
    //dac.setVoltage(dac_v);
    nn=6;
  }

  lcd.setCursor(6,1); lcd.print("        ");
  lcd.setCursor(6,1);lcd.print(_xcum,dec_batch);
//  if(lpm_mode==1){lcd.print(_xcum/1000,dec_batch);}  // Display in LPS
//  if(lpm_mode==2){lcd.print(_xcum*1,dec_batch);}      // Display in LPM
//  if(lpm_mode==3){lcd.print(_xcum*1000,dec_batch);}   // Display in LPH

  lcd.setCursor(6,2); lcd.print(_total,dec_total);
  
  
  if(display_mode==1){lcd.setCursor(6,3);lcd.print(preset_qty, dec_preset);}
  if(display_mode==2){lcd.setCursor(6,3);lcd.print(val_420,1);lcd.print(" ");}
  if(display_mode==4){
    lcd.setCursor(6,3);lcd.print(preset_qty, dec_preset);
    lcd.setCursor(11,3);lcd.print("     ");
    lcd.setCursor(11,3);lcd.print(val_420,1);
  }  
  if(preset_qty>0){
    if( _xcum>=(preset_qty-preset_qty2) ){
      digitalWrite(RELAY1,LOW);
      // Activate 1st Valve
    }
    if( _xcum>=(preset_qty) ){
      digitalWrite(RELAY2,LOW);
    menu=""; batch++; _xcum=0; 
    elap2=0; elap1=0; lpm_delay=0; lpm_init=0; _lpm1=0; lpq1=0;
    _BatchReset();
      // Activate 2nd Valve
    lcd.clear();r=1;xcount=0;delay(200);loop();
    lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
    lcd.print("BATCH COMPLETED     ");
    lcd.setCursor(0,1); //set cursor on 0 /first col and 0 / first line
    lcd.print("PRESS MENU1 TO START");
    }
  }
   
   //WHEN PAUSE BUTTON IS PRESSED
   if(digitalRead(ButtonD) == LOW ){  //PAUSE STOP
      menu=""; batch++; 
      //_xcum=0; 
      elap2=0; elap1=0; lpm_delay=0; lpm_init=0; _lpm1=0; lpq1=0;
      _BatchReset();
      digitalWrite(RELAY1,LOW);
      digitalWrite(RELAY2,LOW);
      r=1;lcd.clear();xcount=0;delay(200);loop();
      lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
      lcd.print("PRESS MENU1 TO CONTU");
    }

   //WHEN EMERGENCY BUTTON IS PRESSED
   if(digitalRead(ButtonE) == LOW ){  //EMERGENCY STOP
      menu=""; batch++; _xcum=0; elap2=0; elap1=0; lpm_delay=0; lpm_init=0; _lpm1=0; lpq1=0;
      _BatchReset();
      digitalWrite(RELAY1,LOW);
      digitalWrite(RELAY2,LOW);
      r=1;lcd.clear();xcount=0;delay(200);loop();
      lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
      lcd.print("PRESS MENU1 TO START");
   }
     
// _SerialPrints();
//  Serial.print("\tlpm_avg=");
//  Serial.print(lpm_avg);
//  Serial.print("\tx=");
//  Serial.println(x,4);

  pc1+=pulse_counter;
  pulse_counter=0;
  fin1=millis();
  lpm_delay++; 
  nn++;
  //delay(_Interval);

  nnn=0;
  while(nnn<20){
  DCgoneState = digitalRead ( DCgone );  
  if ( DCgoneState == LOW ){
    delay(5);
     } else {  
        if ( unsaved == HIGH  )  {   //   we only want to save to eeprom once when power fails
            while(!eeprom_is_ready());
            EEPROM.writeDouble(total_reg_no, _total);
            while(!eeprom_is_ready());
            EEPROM.writeFloat(last_batch_reg, _xcum);
        unsaved = LOW;    
        }
    }
  nnn++;
}

     } else {  
        if ( unsaved == HIGH  )  {   //   we only want to save to eeprom once when power fails
            while(!eeprom_is_ready());
            EEPROM.writeDouble(total_reg_no, _total);
            while(!eeprom_is_ready());
            EEPROM.writeFloat(last_batch_reg, _xcum);
        unsaved = LOW;    
        }
    }
          
}
r=1;
k=0;   
}

    void _BatchReset()
    {
      while(!eeprom_is_ready());
      EEPROM.updateDouble(total_reg_no, _total);
      _eepromWrites++;  
    }

    void _setPresetQty(int r){
    delay(150);
    int nx=0; int px=6; String _val1; int d0,d1,d2,d3,d4,d5,d6;
    _val1="";d0=0;d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
    float new_val;
    lcd.clear();
    lcd.setCursor(0,0);lcd.print("Old Qty=");lcd.print(preset_qty, dec_preset);
    lcd.setCursor(0,1);lcd.print("New =        ");
    lcd.setCursor(0,3);lcd.print("3=Submit 5=Back");

    while(r==0) {
    if (px < 6) {
		px = 6;
	}
    lcd.setCursor(px, 1); 
    if(digitalRead(ButtonA) == LOW) {
		if (digitalRead(ButtonE) == LOW) {
			r = 1;
			lcd.clear();
			_back(3);
			xcount = 0;
			loop();
		}
		nx ++; 
		if (nx > 9) {
			nx = 0;
		}
		lcd.print(nx);
		if (px == 6) {
			d0 = nx;
		}
		if (px == 7) {
			d1 = nx;
		}
		if (px == 8) { 
			d2 = nx;
		}
		if (px == 9) {
			d3 = nx;
		}
		if (px == 10) {
			d4 = nx;
		}
		if (px == 11) {
			d5 = nx;
		}
		if (px==12) {
			d6 = nx;
		}
	}       
	if (digitalRead(ButtonB) == LOW) {
		px ++;
		if (px > 12) {
			px = 6;
		}
    } 
    lcd.setCursor(6, 2);
	lcd.print("         ");
    lcd.setCursor(px, 2);
	lcd.print("^");
    lcd.setCursor(7, 0);
	lcd.print(_val1);
    
    if (digitalRead(ButtonC) == LOW) {
    _val1 = "";
    float xx = (d0 * 1000000) + (d1 * 100000) + (d2 * 10000) + (d3 * 1000) + (d4 * 100) + (d5 * 10) + (d6 * 1);
    int rr = 0;  
    while (rr == 0) {
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(xx, 2);
		lcd.setCursor(0, 1);
		lcd.print("Accept?");
		lcd.setCursor(0, 2);
		lcd.print("1=Yes, 2=No");
		lcd.setCursor(0, 3); //set cursor on 0 /first col and 0 / first line
		lcd.print("PRESS MENU1 TO START");

		if(digitalRead(ButtonA)==LOW){
		while(!eeprom_is_ready());
		EEPROM.writeFloat(preset_qty_reg,xx );
		while(!eeprom_is_ready());
		preset_qty=EEPROM.readFloat(preset_qty_reg);
		_val1="";d0=0;d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
		_back(2);r=1;rr=1;
		lcd.clear();xcount=0;delay(200);loop();
		lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
		lcd.print("PRESS MENU1 TO START");
		}
		if(digitalRead(ButtonB)==LOW){
		_back(2);r=1;rr=1;
		lcd.clear();xcount=0;delay(200);loop();
		lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
		lcd.print("PRESS MENU1 TO START");
				
		}
		delay(150);
	}
     }
      if(digitalRead(ButtonE)==LOW){delay(250);lcd.clear();r=1;xcount=0;loop();}
      delay(150);
        }
     }

  void _calMenu(int r){       //Enter cal mode
    lcd.clear();
    lcd.setCursor(0,0);lcd.print("Calibration Mode"); //Display on line one
    lcd.setCursor(0,1);lcd.print("1=Start");          //Display on line two
    lcd.setCursor(0,2);lcd.print("2=View");           //Display on line three
    lcd.setCursor(0,3);lcd.print("3=Config  4=Edit"); //Display on line four
  while(r==0){
    if(digitalRead(ButtonA)==LOW){delay(200);_startSample(0,0,0,0);}
    if(digitalRead(ButtonB)==LOW){delay(200);_PointsList(0,1);}
    if(digitalRead(ButtonC)==LOW){delay(200);_config(0);}
    if(digitalRead(ButtonD)==LOW){delay(200);_PointsList(0,2);}
    if(digitalRead(ButtonE)==LOW){delay(200);r=1;menu="";xcount=0;loop();}
    delay(150);
    }  
  }

    //Todo: msign: eeprom register value
    // msign display, capture, etc.

    void _editPulse(int r,int _point, int _param){  //Pulse feeding
    delay(150);
    int nx=0; int msign=0; int px=7; String _val1,d0,d1,d2,d3,d4,d5,d6,d7,d8;
    //char _val[9];
    lcd.clear();
    lcd.setCursor(0,0);lcd.print("Point No.");lcd.print(_point);
    lcd.setCursor(0,1);
    if(_param==1)lcd.print("Pulse:");
    if(_param==2)lcd.print("Time :");
    if(_param==3)lcd.print("MicEC:");
    lcd.setCursor(0,3);lcd.print("3=Submit");
  while(r==0){
   if(msign==0){
    lcd.setCursor(6,1);lcd.print("+");
    }else{
    lcd.setCursor(6,1);lcd.print("-");
    }
  if(px<7)px=7;
    lcd.setCursor(px,1); 
  if(digitalRead(ButtonA)==LOW){nx++; if(nx>9){nx=0;} 
    lcd.print(nx);
  if(px==7)d0=nx;
  if(px==8)d1=nx;
  if(px==9)d2=nx;
  if(px==10)d3=nx;
  if(px==11)d4=nx;
  if(px==12)d5=nx;
  if(px==13)d6=nx;
  if(px==14)d7=nx;
  if(px==15)d8=nx;
  }     
  if(digitalRead(ButtonB)==LOW){px++; if(px>15)px=7;} 
    lcd.setCursor(6,2);lcd.print("          ");
    lcd.setCursor(px,2);lcd.print("^");
  if(digitalRead(ButtonD)==LOW){
    if(msign==0){
      msign=1;
      lcd.setCursor(6,1);lcd.print("-");
      }else{
      msign=0;
      lcd.setCursor(6,1);lcd.print("+");
      }
  } 
  if(digitalRead(ButtonE)==LOW){r=1;lcd.noCursor();_back(3);}
    _val1=d0+d1+d2+d3+d4+d5+d6+d7+d8;
  if(msign==1)_val1="-"+_val1;
  if(digitalRead(ButtonC)==LOW){
    int rr=0;  
    while(rr==0){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print(_val1);
      lcd.setCursor(0,1);lcd.print("Accept?");
      lcd.setCursor(0,2);lcd.print("1=Yes, 2=No");
      if(digitalRead(ButtonA)==LOW){
        while(!eeprom_is_ready());
        if(_param==1) EEPROM.writeInt( ((_point-1)*6)+1 , stringToLong(_val1) );
        if(_param==2) EEPROM.writeLong( ((_point-1)*6)+1+2 , stringToLong(_val1) );
        if(_param==3) EEPROM.writeLong( ((_point-1)*6)+201 , stringToLong(_val1) );
        _back(3);r=1;rr=1;
//      lcd.clear();xcount=0;delay(200);loop();
//      lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
//      lcd.print("PRESS MENU1 TO START");
      }
      if(digitalRead(ButtonB)==LOW){
        rr=1;_back(3);nx=0;px=7;
        //_val="";
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("Point No.");lcd.print(_point);
        lcd.setCursor(0,1);
        if(_param==1)lcd.print("Pulse:");
        if(_param==2)lcd.print("Time :");
        if(_param==3)lcd.print("MicEC:");
        lcd.setCursor(0,3);lcd.print("3=Submit");      }
    delay(150);

    }
  }
  delay(150);
  }
}

     //ENTER CONFIG MODE
    void _config(int r){ 
      delay(150);
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Config");
      lcd.setCursor(0,1);lcd.print("1=Qty 2=Points");
      lcd.setCursor(0,2);lcd.print("3=Total 4=EC");
      lcd.setCursor(0,3);lcd.print("5=microEC");
  while(r==0){
      if(digitalRead(ButtonA)==LOW){_setQty(0);}
      if(digitalRead(ButtonB)==LOW){_setPoints(0);}
      if(digitalRead(ButtonC)==LOW){_setTotal(0);}
      if(digitalRead(ButtonD)==LOW){_setEC(0,0,0);}
      if(digitalRead(ButtonE)==LOW){
        //_setMicroEC(0,0,0);
       r=1;
       }
        
    delay(100);
    }
  }

    void _setTotal(int r){  //Reset totaliser
        int rr=0;
    delay(150);
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Reset Total?");
      lcd.setCursor(0,1);lcd.print("1=Yes 5=No");
    while(r==0){
      if(digitalRead(ButtonE)==LOW){delay(250);r=1;_calMenu(0);}
      if(digitalRead(ButtonA)==LOW){
        delay(150);
        lcd.setCursor(0,2);lcd.print("2=Confirm 5=No");
      while(rr==0){
          if(digitalRead(ButtonE)==LOW){delay(250);r=1;_calMenu(0);}
          if(digitalRead(ButtonB)==LOW){
          while(!eeprom_is_ready());
          EEPROM.writeDouble(total_reg_no,0);
          _total=0;
          _back(3);r=1;rr=1;
          }
        }
      delay(150);
      }
    }
}
    //Set EC
    void _setEC(int r,int _point, int _param){  
      delay(150);
      int nx=0; int px=6; String _val1; int d0,d1,d2,d3,d4,d5,d6;
      _val1="";d0=0;d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
      float new_val;
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("EC=");lcd.print(ec_value);
      lcd.setCursor(0,1);
      lcd.print("Value:       ");
      lcd.setCursor(0,3);lcd.print("3=Submit 5=Back");
  while(r==0){
  if(px<6)px=6;
      lcd.setCursor(px,1); 
  if(digitalRead(ButtonA)==LOW){
    if(digitalRead(ButtonE)==LOW){r=1;lcd.noCursor();_back(3);_calMenu(0);}
        nx++; 
    if(nx>9){nx=0;}
        lcd.print(nx);
    if(px==6)d0=nx;
    if(px==7)d1=nx;
    if(px==8)d2=nx;
    if(px==9)d3=nx;
    if(px==10)d4=nx;
    if(px==11)d5=nx;
    if(px==12)d6=nx;
  }       
    if(digitalRead(ButtonB)==LOW){
    px++;
    //if(px==10)px++; 
    if(px>12)px=6;
    } 
      lcd.setCursor(6,2);lcd.print("         ");
      lcd.setCursor(px,2);lcd.print("^");
      lcd.setCursor(2,0);lcd.print(_val1);
    if(digitalRead(ButtonC)==LOW){
      _val1="";
      float xx=(d0*1000000)+(d1*100000)+(d2*10000)+(d3*1000)+(d4*100)+(d5*10)+(d6*1);
       //xx+=d5/10.00;
       //xx+=d6/100.00;
      int rr=0;  
    while(rr==0){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print(xx,2);
      lcd.setCursor(0,1);lcd.print("Accept?");
      lcd.setCursor(0,2);lcd.print("1=Yes, 2=No");
    if(digitalRead(ButtonA)==LOW){
        while(!eeprom_is_ready());
        EEPROM.writeFloat(ec_value_reg,xx );
        while(!eeprom_is_ready());
        ec_value=EEPROM.readFloat(ec_value_reg);
        _val1="";d0=0;d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        _back(3);r=1;rr=1;
      }
      if(digitalRead(ButtonB)==LOW){
        rr=1;_back(3);nx=0;px=6; 
        _val1="";d0=0;d1=0;d2=0;d3=0;d4=0;d5=0;d6=0;
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("EC=");lcd.print(ec_value,4);
        lcd.setCursor(0,1);
        lcd.print("Value:       ");
        lcd.setCursor(0,3);lcd.print("3=Submit 5=Back");     
      }
    delay(150);
    }
  }
  if(digitalRead(ButtonE)==LOW){delay(250);r=1;_calMenu(0);}
    delay(150);
  }
}
    //SET MICRO EC
    void _setMicroEC(int r,int _point, int _param){  //Set micro EC
      delay(150);
        int nx=0; int px=6; String _val1; int d0,d1,d2,d3,d4,d5,d6;
      _val1="";d0=0;d1=0;d2=0;d3=0;d5=0;d6=0;
      float new_val;
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("microEC=");lcd.print(microec_value);
      lcd.setCursor(0,1);
      lcd.print("Value:    .");
      lcd.setCursor(0,8);lcd.print("3=Submit 5=Back");
  while(r==0){
  if(px<6)px=6;
      lcd.setCursor(px,1); 
  if(digitalRead(ButtonA)==LOW){
    if(digitalRead(ButtonE)==LOW){r=1;lcd.noCursor();_back(3);
    //xcount=0;menu="";loop();
    }
    nx++; 
    if(nx>9){nx=0;}
    lcd.print(nx);
    if(px==6)d0=nx;
    if(px==7)d1=nx;
    if(px==8)d2=nx;
    if(px==9)d3=nx;
    if(px==11)d5=nx;
    if(px==12)d6=nx;
  }       
  if(digitalRead(ButtonB)==LOW){
    px++;
    if(px==10)px++; 
    if(px>12)px=6;
    } 
      lcd.setCursor(6,2);lcd.print("         ");
      lcd.setCursor(px,2);lcd.print("^");
      lcd.setCursor(2,0);lcd.print(_val1);
  if(digitalRead(ButtonC)==LOW){
     _val1="";
      float xx=(d0*1000)+(d1*100)+(d2*10)+(d3*1);
      xx+=d5/10.00;
      xx+=d6/100.00;
      int rr=0;  
      while(rr==0){
      lcd.clear();
      lcd.setCursor(0,0);lcd.print(xx,2);
      lcd.setCursor(0,1);lcd.print("Accept?");
      lcd.setCursor(0,2);lcd.print("1=Yes, 2=No");
    if(digitalRead(ButtonA)==LOW){
        while(!eeprom_is_ready());
        EEPROM.writeFloat(microec_value_reg,xx );
        while(!eeprom_is_ready());
        microec_value=EEPROM.readFloat(microec_value_reg);
        _val1="";d0=0;d1=0;d2=0;d3=0;d5=0;d6=0;
        _back(3);r=1;rr=1;
      }
      if(digitalRead(ButtonB)==LOW){ //Set Qty for Micro EC
        rr=1;_back(3);nx=0;px=6; 
        _val1="";d0=0;d1=0;d2=0;d3=0;d5=0;d6=0;
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("microEC=");lcd.print(microec_value,4);
        lcd.setCursor(0,1);
        lcd.print("Value:    .");
        lcd.setCursor(0,8);lcd.print("3=Submit 5=Back");     
      }
    delay(150);
    }
  }
    if(digitalRead(ButtonE)==LOW){delay(250);r=1;
   // xcount=0;
   // menu="";
   // lcd.clear();
   // loop();
   }
    delay(200);
    }
  }
     

      void _setQty(int r){    //Set qty for cal
        delay(150);
          lcd.clear();
          lcd.setCursor(0,0);lcd.print("Qty=");
          lcd.setCursor(0,1);lcd.print("1=Change 2=Set");
          lcd.setCursor(0,2);lcd.print("5=Cancel");
          lcd.setCursor(0,3);lcd.print("Current Qty=");
        if(calib_qty>0){lcd.print(calib_qty);}else{lcd.print("0");}
          int q=8;
          float qty[] = {5,50,100,200,500,1000,2000};
          int qi=0;
      while(r==0){
          lcd.setCursor(4,0);lcd.print(qty[qi]);lcd.print(" Lts.   ");
    if(digitalRead(ButtonA)==LOW){qi++; if(qi>=q){qi=0;}}    
    if(digitalRead(ButtonB)==LOW){
          calib_qty=qty[qi];
      while(!eeprom_is_ready());
          EEPROM.writeInt(calib_qty_reg, calib_qty);
          lcd.setCursor(0,3);lcd.print("                ");
          lcd.setCursor(0,3);lcd.print("Qty Set to ");lcd.print(calib_qty);
          r=1;
      }
    if(digitalRead(ButtonE)==LOW){delay(150);r=1;}
        delay(200);
       }  
    }

      void _setPoints(int r){  //Set no of points for cal
        delay(150);
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("Points=");
        lcd.setCursor(0,1);lcd.print("1=Change 2=Set");
        lcd.setCursor(0,2);lcd.print("5=Cancel");
        lcd.setCursor(0,3);lcd.print("Cur.Points=");
    if(readReg>0){lcd.print(readReg);}else{lcd.print("0");}
        int q=15;
        int qi=0;
  while(r==0){
        lcd.setCursor(7,0);lcd.print(qi);lcd.print("     ");
    if(digitalRead(ButtonA)==LOW){qi++; if(qi>=q){qi=0;}}    
    if(digitalRead(ButtonB)==LOW){
        readReg=qi;
      while(!eeprom_is_ready());
        EEPROM.writeInt(points_regno, readReg);
        lcd.setCursor(0,3);lcd.print("                ");
        lcd.setCursor(0,3);lcd.print("Points Set=");lcd.print(readReg);
        r=1;
      }
    if(digitalRead(ButtonE)==LOW){delay(150);r=1;}
       delay(200);
//             lcd.clear();xcount=0;delay(200);loop();
//      lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
//      lcd.print("PRESS MENU1 TO START");
      }  
    }
    
    void _PointsList(int r, int m){ //To edit pulse and time pointwise
        delay(150);
        float fx1=0.0;
    while(r==0){
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("Edit Points");
        int _point=1,pulse_addr1,time_addr1,micro_addr1,p2;
        long t2,m2;
    while(_point<=max_reg_points_calibration){
        pulse_addr1=((_point-1)*6)+1;
        time_addr1=pulse_addr1+2;
        micro_addr1=((_point-1)*6)+201;
        while(!eeprom_is_ready());p2=EEPROM.readInt(pulse_addr1);
        while(!eeprom_is_ready());t2=EEPROM.readLong(time_addr1);
        while(!eeprom_is_ready());m2=EEPROM.readLong(micro_addr1);
        fx1=(float)p2/(float)t2;
        fx1=(float)fx1*1000;
        lcd.clear();
        
        lcd.setCursor(0,0);lcd.print("Point ");lcd.print(_point);
        lcd.setCursor(0,1);lcd.print("P:");lcd.print (p2);
        lcd.setCursor(0,2);lcd.print("T:");lcd.print (t2);
        lcd.setCursor(0,3);lcd.print("m:");lcd.print (m2);
     if(m==1){lcd.setCursor(0,3);lcd.print("F:");lcd.print (fx1);}
     if(digitalRead(ButtonA)==LOW){if(_point==max_reg_points_calibration){_point=1;}else{_point++;}}     
     if(m==2){  
          lcd.setCursor(10,1);lcd.print("2=Edit");
          if(digitalRead(ButtonB)==LOW){_editPulse(0,_point,1);}
          lcd.setCursor(10,2);lcd.print("3=Edit");
          if(digitalRead(ButtonC)==LOW){_editPulse(0,_point,2);}
          lcd.setCursor(10,3);lcd.print("4=Edit");
          if(digitalRead(ButtonD)==LOW){_editPulse(0,_point,3);}
        }
        if(digitalRead(ButtonE)==LOW){r=1;_point=max_reg_points_calibration+1;_back(3);_calMenu(0);}
        p2=0;t2=0;
        delay(150);
//      lcd.clear();xcount=0;delay(200);loop();
//      lcd.setCursor(0,0); //set cursor on 0 /first col and 0 / first line
//      lcd.print("PRESS MENU1 TO START");
      }
    }
  }

      // CODE FOR PULSE FEED FOR CALIBRATION
      void _startSample(int r,int rr, int rrr, int rrx){ 
          int _Interval=1000;
      pulse_counter=0;
      bstart=0; 
      int _point=0;
      lcd.clear();
      lcd.setCursor(0,0);lcd.print("Waitingfor Pulse");
      Serial.println("Waitingfor Pulse");
      while(r==0){
    if(digitalRead(ButtonE)==LOW){_back(3);rrr=1;rr=1;r=1;}
        pulse_counter1=pulse_counter;
        int p1,pulse_addr,time_addr,reg1; long t1=0;
        int n=0;
      if(pulse_counter>0 && bstart==0){         
        BatchPulseCount=pulse_counter;
        start=millis();
        bstart=1;
        pc1=0;
        pulse_counter=0;
        lcd.setCursor(0,0);lcd.print("Pulse Started...");
        Serial.println("Pulse Started...");
        }
      if(pulse_counter1==0 && bstart==1){           
        finished=millis();
        BatchPulseCount+=pulse_counter;
        pulse_counter=0;
        BatchTime=finished-start;
        float f;
        f=((float)BatchPulseCount/(float)BatchTime)*1000;
        BatchPulseCount+=pulse_counter;
        pulse_counter=0;
        rr=0;  rrx=0; delay(250);          lcd.clear();
       while(rr==0){
         if(rrx==0){
          lcd.setCursor(0,0);lcd.print("T(ms)=");lcd.setCursor(6,0);lcd.print(BatchTime);
          lcd.setCursor(0,1);lcd.print("P");lcd.setCursor(1,1);lcd.print(BatchPulseCount);lcd.setCursor(9,1);lcd.print(" F");lcd.setCursor(10,1);lcd.print(f);
          lcd.setCursor(0,2);lcd.print("Accept Point ");lcd.setCursor(13,2);lcd.print(Reg+1);
          lcd.setCursor(0,3);lcd.print("1=Yes, 2=No");
          Serial.print("T(ms)=");Serial.println(BatchTime);
          Serial.print("P");Serial.print(BatchPulseCount);Serial.print("\tF");Serial.println(f);
          Serial.print("Accept Point ");Serial.println(Reg+1);
          Serial.println("1=Yes, 2=No");
          delay(250);
          rrx=1;
          }
          if(digitalRead(ButtonA)==LOW){
            bstart=0;
            Reg++; 
            pulse_addr=((Reg-1)*6)+1;
            time_addr=pulse_addr+2;
            while(!eeprom_is_ready());
            EEPROM.writeInt(pulse_addr, BatchPulseCount);
            while(!eeprom_is_ready());
            EEPROM.writeInt(points_regno, Reg);
            while(!eeprom_is_ready());
            EEPROM.writeLong(time_addr, BatchTime);
            while(!eeprom_is_ready());
            p1=EEPROM.readInt(pulse_addr);
            while(!eeprom_is_ready());
            t1=EEPROM.readLong(time_addr);
            while(!eeprom_is_ready());
            reg1=EEPROM.readInt(points_regno);
            lcd.clear();
            rrr=0;rrx=0; delay(250);
            while(rrr==0 & rrx==0){
              if((p1==BatchPulseCount)&&(t1=BatchTime)&&(reg1=Reg)){
                lcd.setCursor(0,0);lcd.print("Point-");lcd.print(Reg);lcd.print(" saved");
                lcd.setCursor(0,1);lcd.print("2=Continue");
                Serial.print("Point-");Serial.print(Reg);Serial.println(" saved");
                Serial.println("5=Continue");
                _back(3);_calMenu(0);
                //_calMenu(0);
                //if(digitalRead(ButtonE)==LOW){p1=0;t1=0;_calMenu(0);}
              }else{
                lcd.setCursor(0,0);lcd.print("Error in:");
                if(p1!=BatchPulseCount){lcd.setCursor(0,1);lcd.print("Pulse");}
                if(t1!=BatchTime){lcd.print("*Time");}
                if(reg1!=Reg){lcd.print("*Point");}
                lcd.setCursor(0,2);lcd.print("2=Continue");
                if(digitalRead(ButtonE)==LOW){p1=0;t1=0;_back(3);rrr=1;rr=1;r=1;_calMenu(0);}
              }
              delay(150);rrx=1;
            }
          }
          if(digitalRead(ButtonB)==LOW){
            bstart=0;
            BatchPulseCount=0;
            delay(150);_calMenu(0);
            //_Calibration(0);
          }          
          delay(150);
        }
        }else{
          //lcd.setCursor(0,1);lcd.print(BatchPulseCount);
          BatchPulseCount+=pulse_counter;       
          pulse_counter=0;
          pc1+=pulse_counter1;
          //Serial.println(BatchPulseCount);
        }
    pulse_counter=0;            
    delay(_Interval);
  }
}

      
      float _maxmin1(float x, int cp){
      int p=0; long t=0;
      int y0=0; int y1=0; float t0=0; float t1=0; float fx=0.0; float k=0;
      double k0=1000000;  
      double k1=1000000;  
      int xcalc=0;
      float _ans;    
      for (int j=1; j<=cp; j++) {
      int pulse_addr=0;
      pulse_addr=((j-1)*6)+1;
      long time_addr=0;
      time_addr=pulse_addr+2;
      while(!eeprom_is_ready());
      p=EEPROM.readInt(pulse_addr);
      while(!eeprom_is_ready());
      t=EEPROM.readLong(time_addr);
      if(!isnan(p) && !isnan(t) && p>0 && t>0){
        //Serial.print(j);Serial.print(":\t");
        //Serial.print(p);Serial.print("\t");
        //Serial.println(t);
        float tx;
        tx=(float)t/1000;
        float fx1=0.0;
        fx1=(float)p/tx;
        if(t>0){fx=fx1;}else{fx=0;}
        if(fx<x){k=x-fx;if(k<k0){k0=k;y0=p;t0=tx;}}
        if(fx>x){k=fx-x;if(k<k1){k1=k;y1=p;t1=tx;}}
        if(fx==x){xcalc=1;_ans=p;}
      }
      }
      if(y0==0){xcalc=1;_ans=y1;}
      if(y1==0){xcalc=1;_ans=y0;}
      if(xcalc==0){
      _ans=y0 + ((y1-y0) * ( ((x*t0*t1)-(y0*t1)) / ((y1*t0)-(y0*t1)) ) ) ;
      _ans=((float)_ans);
      }
      return _ans;
      }
      
      float _maxmin2(float x, int cp){
      int p=0; long t=0;  long m=0;
      int y0=0; int y1=0; float t0=0; float t1=0; float m0=0; float m1=0; float fx=0.0; float k=0;
      double k0=1000000;  
      double k1=1000000;  
      int xcalc=0;
      float _ans=0;    
      for (int j=1; j<=cp; j++) {
      int pulse_addr=0;
      pulse_addr=((j-1)*6)+1;
      int micro_addr=0;
      micro_addr=((j-1)*6)+201;
      long time_addr=0;
      time_addr=pulse_addr+2;
      while(!eeprom_is_ready());
      p=EEPROM.readInt(pulse_addr);
      while(!eeprom_is_ready());
      t=EEPROM.readLong(time_addr);
      while(!eeprom_is_ready());
      m=EEPROM.readLong(micro_addr);
      if(!isnan(p) && !isnan(t) && p>0 && t>0){
        float tx;
        tx=(float)t/1000;
        float fx1=0.0;
        fx1=(float)p/tx;
        if(t>0){fx=fx1;}else{fx=0;}
        if(fx<x){k=x-fx;if(k<k0){k0=k;y0=p;t0=tx;m0=m;}}
        if(fx>x){k=fx-x;if(k<k1){k1=k;y1=p;t1=tx;m1=m;}}
        if(fx==x){xcalc=1;_ans=m;}
      }
      }
      if(y0==0){xcalc=1;_ans=m1;}
      if(y1==0){xcalc=1;_ans=m0;}
      if(xcalc==0){
      _ans= m0 - (    (t0*t1*(m0-m1)*(y0-(x*t0)))  /   ((t0*y0*t1)-(y1*t0*t0))   )  ;
      _ans=((float)_ans);
      }
      return _ans;
      }
      
      
      
      void _back(int line_no){
      for (int j=15; j>=0; j--) {
      lcd.setCursor(j,line_no);lcd.print("<");
      Serial.println(j);
      delay(100);
      }
      }
      
      void timerIsr(){
      if (!(--tick_count)){
      tick_count = TICK_COUNTS;
      tick_2s_isr(); 
      }
      }
      void tick_2s_isr(){
        if (in_long_isr){return;}
        in_long_isr = true;
        volatile long i;
        interrupts();
      //                    if(digitalRead(ButtonE)==LOW){loop();}
      attachPinChangeInterrupt(INTERRUPT_INPUT, interrupt_handler, FALLING);
        noInterrupts();
        in_long_isr = false;
      }
      
      void interrupt_handler(){
      
      pulse_counter = pulse_counter + 1;
      }
      
      long stringToLong(String s)
      {
       char arr[12];
       s.toCharArray(arr, sizeof(arr));
       return atol(arr);
      }
      
      void _SerialPrints(){
      
      }

