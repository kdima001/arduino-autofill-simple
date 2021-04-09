#include <Streaming.h>  
#include <avr/wdt.h>

//Простой контроллер осмоса
//Состояния:	наполнение емкости до верхнего датчика	(светодиод медленно мигает)
//				регулярная промывка мембраны			(светодиод быстро мигает)
//				емкость заполнена						(светодиод светится)

//индикаторы (исп. одноцветный диод)
#define	LED_Y 	5
#define	LED_G 	6

//Датчики
#define	RO_UP	2			// верхний уровень в емкости

//исполнительные мех-мы
#define PUMP_RO			13 	//помпа осмоса (повышения давления) - подача RO
//клапана
#define V_RAW			12 	//Главный клапан (на входе из водопровода) и фильтрованная вода
#define V_FLUSH			10 	//Клапан промывки

//константы интервалов в мсек
#define POOL_INT		1000

//время промывки мембраны СЕКУНДЫ
#define T_FLUSH			30
//время работы мембраны до промывки СЕКУНДЫ !!!
#define T_RO			600

//

uint16_t	TimeON_RO;

// Переменные пуллинга					
unsigned long curMillis;
unsigned long prevMillisLED = 0;
unsigned long prevMillis1 = 0;
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = 0;
unsigned long prevMillis4 = 0;

//---------------------------------------------------------------------------
uint8_t SerialDBG, RO;

//---------------------------------------------------------------------------
enum mRO_t {						//режим работы осмосного накопителя
 RO_FILL, 							//наполнение накопителя осмоса (всегда когда RO=0)
 RO_FLUSH,			 				//Промываем мембрану 
 RO_READY			 				//накопитель осмоса заполнен 
};
mRO_t mRO;

void PrintStatusRO(void) {	
	Serial << F(" RO = ");
	switch(mRO) {
		case RO_FLUSH: 		Serial << F("RO_FLUSH") << endl; break;
		case RO_FILL: 		Serial << F("RO_FILL") << endl; break;
		case RO_READY: 		Serial << F("RO_READY") << endl; break;		
	};
} 

//---------------------------------------------------------------------------
void(* Reset) (void) = 0; // Reset CON function

//---------------------------------------------------------------------------

uint16_t old_lv2;
void ProcessSensors(void){
	uint16_t tmpi, tmpj;
	
	/* Датчики:
	осмос верхний уровень 		(RO)		*/
	RO = digitalRead(RO_UP);	
}
//---------------------------------------------------------------------------
struct LED_MODE_t {
	bool blink;
	uint16_t t1, t2; // свечения, пауза
	uint8_t r, g, b;	
};

enum LM_t {	
	LM_FILL,
	LM_FLUSH,
	LM_FULL	
};
LM_t LED_MODE;

LED_MODE_t	 lm[8] = { 	{1, 100, 1000,  0, 255, 0}, //LM_FILL, 	редко мигает 1сек
						{1, 100, 100,	0, 255, 0}, //LM_FLUSH, часто мигает 0,1сек
						{0, 0, 0,   	0, 255, 0}  //LM_FULL, 	горит						
					};
											
//---------------------------------------------------------------------------
void setRGB(uint8_t p_Y, uint8_t  p_G, uint8_t p_B) {
	//инвертнем ибо общий анод и 0 зажигает, а 255 гасит диод
	digitalWrite(LED_Y, 255 - p_Y);
	//digitalWrite(LED_G, 255 - p_G);
	digitalWrite(LED_G, 255 - p_G);
}											

//---------------------------------------------------------------------------
void ProcessLED() { 
	if (lm[LED_MODE].blink) {
		//Если цикл режима светодиода кончился - ставим цикл в начало
		if(curMillis - prevMillisLED > lm[LED_MODE].t1+lm[LED_MODE].t2)
			prevMillisLED = curMillis;
			
		if((prevMillisLED < curMillis) && (curMillis <= (prevMillisLED+lm[LED_MODE].t1))){ //фаза удержания яркости
				setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);
			}	else {
					if(prevMillisLED+lm[LED_MODE].t1 < curMillis && curMillis <= prevMillisLED+lm[LED_MODE].t1+lm[LED_MODE].t2){ //фаза выключения
						setRGB(0, 0, 0);
					}
				}		
	}
	else
		setRGB(lm[LED_MODE].r, lm[LED_MODE].g, lm[LED_MODE].b);		 
}

//---------------------------------------------------------------------------
void BeginFlushRO() { //промывка осуществляется только давлением воды на входе. Насос RO не включается специально, только если был включен
	//Если насос включен 
	if ( digitalRead(PUMP_RO) ) {
		//Выключили насос
		digitalWrite(PUMP_RO, LOW);
		//Подождали 5 сек до сброса давления
		delay(3000);
		wdt_reset();
		//Открываем сброс
		digitalWrite(V_FLUSH, HIGH);
		//Обратно включили насос
		//digitalWrite(PUMP_RO, HIGH);
	}
	else
		digitalWrite(V_FLUSH, HIGH);	
}

//---------------------------------------------------------------------------
void EndFlushRO() {	digitalWrite(V_FLUSH, LOW); }

//---------------------------------------------------------------------------
void PrintSensors() {	
	Serial << endl;
	
	Serial << F("\tM:[") << mRO << F(", ") << TimeON_RO << F("]");	
	Serial << F("\tRO=") << RO;	
	Serial << F("\tLED=") << LED_MODE;
}
//---------------------------------------------------------------------------
void ProcessRO(void) {
	switch (mRO) {
		//********************
		case RO_FLUSH:
			//Льем, считаем 
			TimeON_RO ++;
			//Время промывки прошло (полный цикл из работы насоса и промывки)
			if ( TimeON_RO >= T_RO+T_FLUSH ) {
				mRO = RO_READY;
				PrintStatusRO();
				digitalWrite(PUMP_RO, LOW);
				TimeON_RO = 0;
				EndFlushRO();
			}
		break;
		//********************
		case RO_FILL:
			if (RO) {
				mRO = RO_READY;
				PrintStatusRO();
				digitalWrite(PUMP_RO, LOW);
			}
			//Льем, считаем 
			TimeON_RO ++;	
			//Пора промывать
			if ( TimeON_RO >= T_RO ) {
				mRO = RO_FLUSH;
				PrintStatusRO();
				BeginFlushRO();		
			}		
		break;
		//********************
		case RO_READY:
			if ( !RO ) {
				mRO = RO_FILL;
				PrintStatusRO();
				digitalWrite(PUMP_RO, HIGH);
			}			
		break;		
	}	
}

//---------------------------------------------------------------------------
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
		// Пришлось придумать символ-заменитель переводу строки - в кач-ве параметра в cmd \n передать не удалось
    if (inChar == '\n' || inChar == '@' || inChar == ';') {
      inputString.replace('@', '\n');
			inputString.replace(';', '\n');
			stringComplete = true;
    } 
  }
} 

//--------------------------------------------------------------------------
void SelfTest(void) {
	Serial << endl;
	ProcessSensors();
	
	PrintSensors();
	Serial << F("Test outputs:") << endl;

	wdt_reset(); // не забываем сбросить смотрящую собаку	

	Serial << F("Flush valve ON..."); SetValve(V_FLUSH, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_FLUSH, LOW);
	Serial << F("Pump RO ON..."); digitalWrite(PUMP_RO, HIGH); delay(1000);	Serial << F("OFF") << endl;	digitalWrite(PUMP_RO, LOW);
	Serial << F("Main valve ON..."); SetValve(V_RAW, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_RAW, LOW);
		
	wdt_reset(); // не забываем сбросить смотрящую собаку
	
	Serial << F("Self test complete") << endl;
}

//---------------------------------------------------------------------------
void setup() {
	uint8_t tmpi;
	
	wdt_enable (WDTO_8S); 
  	
	pinMode(PUMP_RO, 		OUTPUT);
	pinMode(V_RAW, 		OUTPUT);
	pinMode(V_FLUSH, 		OUTPUT);
	
	pinMode(LED_Y, 			OUTPUT);
	pinMode(LED_G, 			OUTPUT);
  
  pinMode(RO_UP,  INPUT);
	
	SerialDBG = 0;
	Serial.begin(115200);
	inputString.reserve(200);
	TimeON_RO = 0;	
		
	mRO = RO_FILL;
		
	LED_MODE = LM_FULL;
	
	SelfTest();
	
	PrintStatusRO();	
}

//---------------------------------------------------------------------------
void SetValve(uint8_t pin, uint8_t state) {
	//если клапан на PWM выходе - включаем и переходим в удержание
	/*if (state && (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11 )) {
		//даем на всю железку
		analogWrite(pin, ON_VALVE_VALUE);
		delay(200);
		//переходим на пол-шишечки
		analogWrite(pin, HOLD_VALVE_VALUE);
		return;
	}	*/	
	digitalWrite(pin, state);		
}

//--------------------------------------------------------------------------
void loop() {
		
	wdt_reset(); // не забываем сбросить смотрящую собаку
	
	ProcessLED();

	curMillis = millis();
	
	//проверим на переполнение
	if(prevMillis1 > curMillis || prevMillis2 > curMillis || prevMillis3 > curMillis || prevMillis4 > curMillis || prevMillisLED > curMillis){
		prevMillis1 = 0;
		prevMillis2 = 0;
		prevMillis3 = 0;
		prevMillis4 = 0;
		prevMillisLED = 0;
	}
	
  //Ежесекундные действия
	if(curMillis - prevMillis1 > POOL_INT) {
		prevMillis1 = curMillis;
		
		//хербит
		//if (second()%50 == 0) Serial << F(" ");
  
		ProcessSensors();
		
		ProcessRO();
		
		// print the string when a newline arrives:
		if (stringComplete) {
			inputString.toLowerCase();
			inputString.trim();
			Serial << inputString << endl;
			if (inputString.equals(F("help"))) 
				{ Serial << F("comands:") << endl;
					Serial << F("diag") << endl;
					Serial << F("debug=on") << endl;
					Serial << F("debug=off") << endl; 
				}
			if (inputString.equals(F("status")) || inputString.equals(F("diag"))) { 
				PrintSensors(); 
			}
			if (inputString.equals(F("debug=on")) || inputString.equals(F("debug on")) ) SerialDBG = 1;
			if (inputString.equals(F("debug=off")) || inputString.equals(F("debug off")) ) SerialDBG = 0;
			// clear the string:
			inputString = "";
			stringComplete = false;
		}		
		
		if (SerialDBG) PrintSensors();
	}			

	switch(mRO) {
		case RO_FILL: 	LED_MODE = LM_FILL; break;
		case RO_FLUSH:	LED_MODE = LM_FLUSH; break;
		case RO_READY:	LED_MODE = LM_FULL; break;
	}	
}
