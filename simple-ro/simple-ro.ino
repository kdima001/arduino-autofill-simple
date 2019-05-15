#include <EEPROM.h>
#include <DS1302RTC.h>
#include <Time.h>
#include <Streaming.h>  
#include <avr/wdt.h>

//---------------------------------------------------------------------------
/* карта адресов
0 - режим 1 (с 2го по 4й - заполнение бака)
10-42 кол-во циклов 1го слива
50-82 кол-во циклов 2го слива
*/
const uint16_t ADR_MODE   = 0; 
const uint16_t ADR_COUNT1 = 10;
const uint16_t ADR_COUNT2 = 50;

//индикаторы
#define	LED_Y 	5
#define	LED_G 	6

//исполнительные мех-мы
#define PUMP_RO			13 	//помпа осмоса (повышения давления) - подача RO
//клапана
#define V_RAW			12 	//Главный клапан (на входе из водопровода) и фильтрованная вода
#define V_FLUSH			10 	//Клапан промывки

//датчики
#define	FLOW_METER		2	//Датчик расхода (импульсный)

//константы интервалов в мсек
#define POOL_INT		1000

//время промывки мембраны СЕКУНДЫ
#define T_FLUSH			30
//время работы мембраны до промывки СЕКУНДЫ !!!
#define T_RO			600

uint16_t	TimeON_RO;

//константы для клапанов
#define ON_VALVE_VALUE 		100	//ШИМ на включение клапана
#define HOLD_VALVE_VALUE 	255	//ШИМ на удержание клапана

// Переменные пуллинга					
unsigned long curMillis;
unsigned long prevMillisLED = 0;
unsigned long prevMillis1 = 0;
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = 0;
unsigned long prevMillis4 = 0;

//---------------------------------------------------------------------------
//задатчик кол-ва подмен
int8_t TargetCount, RO_part;
//Переменные состояния
uint8_t CurrentCount;
uint8_t SerialDBG, RO, /*UP1, LW1, UP2, MD2, LW2,*/ ERR_WATER, ERR_SENSOR, ERR_RO, ERR_RTC;
int8_t LV1, LV2; 

//---------------------------------------------------------------------------
enum mRO_t {						//режим работы осмосного накопителя
 RO_UNDEF,
 RO_FLUSH,			 				//Промываем мембрану
 RO_FILL, 							//наполнение накопителя осмоса (всегда когда RO=0)
 RO_READY,			 				//накопитель осмоса заполнен
 RO_DISABLE					
};
mRO_t mRO;

void PrintStatusRO(void) {	
	PrintTime();
	Serial << F(" RO = ");
	switch(mRO) {
		case RO_UNDEF: 		Serial << F("RO_UNDEF") << endl; break;
		case RO_FLUSH: 		Serial << F("RO_FLUSH") << endl; break;
		case RO_FILL: 		Serial << F("RO_FILL") << endl; break;
		case RO_OVER: 		Serial << F("RO_OVER") << endl; break;
		case RO_READY: 		Serial << F("RO_READY") << endl; break;
		case RO_DISABLE:  	Serial << F("RO_DISABLE") << endl; break;
	};
} 

//---------------------------------------------------------------------------
void(* Reset) (void) = 0; // Reset CON function

//---------------------------------------------------------------------------
float map_float(float x, float in_min, float in_max, float out_min, float out_max) { 
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//---------------------------------------------------------------------------

void PrintTime(void){ 
  if (hour() == 0) Serial << F("00"); 
	else { 
		if (hour() < 10)  Serial << F("0"); 
	  Serial << hour();
	}
		
	Serial << F(":"); 
	
	if (minute() == 00) Serial << F("00");
	else {
		if (minute() < 10) Serial << F("0");
		Serial << minute();
	}
	Serial << F(":"); 
	
	if (second()==0) Serial << F("00"); 
	else { 
		if (second() < 10) Serial << F("0"); 
		Serial << second();	
	}
}

//-----------------------------------------------------------------------------
uint16_t old_lv2;
void ProcessSensors(void){
	uint16_t tmpi, tmpj;
	ERR_SENSOR = 0;
	/* Датчики:
	осмос верхний уровень 		(RO)		*/
	tmpi = GetCrossMatrixValue(8, A1);
	RO = tmpi > 500 ? 1:0;	
	if (SerialDBG) Serial << F("RO:\t") << tmpi << F("->") << RO;
	
	/* емкость 1 */
	tmpi = GetCrossMatrixValue(8, A3);
	LV1 = -1;
	if ( tmpi <= 380) 											LV1 = 2; 	// оба датчика в воздухе, на линии 0
	else if ( 380 <= tmpi  && tmpi <=480 )	LV1 = 1;	// нижний датчик сработал 1/3 от 4,6в -> 3в -> 320 на АПЦ
	else if ( 850 <= tmpi  && tmpi <=950 )	LV1 = 3;	// верхний датчик сработал 2/3 от 4,6в -> 3в -> 630 на АПЦ
	if (SerialDBG) Serial << F("\tLV1:\t") << tmpi << F("->") << LV1;

	if ( LV1<0 ) ERR_SENSOR++; 
		
	/* емкость 2 */
	tmpi = GetCrossMatrixValue(8, A2);
	
			
	/*if ( tmpi <= 110 )	tmpj = -1; // все контакты в воздухе, на входе АЦП 0в, уровень не известен
	else 
		//if ( tmpi <= 200 ) tmpj = 0;
		//else 
			tmpj = constrain(round(map_float(tmpi, 110, 950, 0, 10))*10, 0, 100);	// преобразуем сигнал с АЦП в уровень от 0 до 100%
		    //constrain(round(map_float(tmpi, 50,  900, 0, 10))*10, 0, 100);
	//Вывод считанного значения при работе 2го канала
	*/
	if (old_lv2 != tmpi && CH2 >= C2_READY && CH2 <= C2_WAIT ) { old_lv2 = tmpi; PrintTime(); Serial << F("##\t") << old_lv2 << F("\t") << LV2 <<endl; }
	
	if (tmpi >= 947) tmpj = 100; else 
	if (tmpi >= 857) tmpj = 90; else 
	if (tmpi >= 770) tmpj = 80; else 
	if (tmpi >= 683) tmpj = 70; else 
	if (tmpi >= 600) tmpj = 60; else 
	if (tmpi >= 514) tmpj = 50; else 
	if (tmpi >= 431) tmpj = 40; else 
	if (tmpi >= 352) tmpj = 30; else 
	if (tmpi >= 266) tmpj = 20; else 
	if (tmpi >= 182) tmpj = 10; else 
	if (tmpi >= 107) tmpj = 0; else 
	tmpj = -1;

	if (SerialDBG /*|| tmpj!=LV2*/) { Serial << F("\tLV2:\t") << tmpi << F("->") << tmpj << endl; }
	LV2 = tmpj;
}
//---------------------------------------------------------------------------
struct LED_MODE_t {
	bool blink;
	uint16_t t1, t2; // свечения, пауза
	uint8_t r, g, b;	
};
enum LM_t {
	LM_ERR_SENSOR,
	LM_ERR_RTC,
	LM_ERR_WATER,
	LM_ERR_RES,
	LM_NORMAL,
	LM_FILL,
	LM_STOP,
	LM_EMPTY
};
LM_t LED_MODE;

LED_MODE_t	 lm[8] = { 	{1, 100, 100,   0, 255, 0}, //LM_ERR_SENSOR, желтый часто мигает 0,1сек (ошибки датчиков)
						{1, 100, 100,   255, 0, 0}, //LM_ERR_RTC, зеленый часто мигает 0,1сек (ошибка часов)
						{1, 100,1000,   0, 255, 0}, //LM_ERR_WATER, желтый мигает 0,1сек с большой паузой
						{1, 100,1000,   255, 0, 0}, //LM_ERR_RES, зеленый  мигает 0,1сек с большой паузой
						{1, 1000,1000,  255, 0, 0}, //LM_NORMAL, зеленый редко мигает 1/1 сек, идет подмена
						{1, 1000,1000,  0, 255, 0}, //LM_FILL, желтый редко мигает 1/1 сек, идет заполнение емкости 1 
						{0, 0,0,  	    255, 0, 0}, //LM_STOP цикл окончен, зеленый горит
						{0, 0,0,   		0, 255, 0}  //LM_EMPTY желтый горит емкость с концентратом пуста
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
void ClearEEPROM(void) {	
	Serial << F("Clearing EEPROM");
	for (int i = 0; i < 1024; i++)
    { //дабы память не насиловать - если 0 в ячейке нечего ее и тереть
			if (EEPROM.read(i)!=0)
				EEPROM.write(i, 0);
			//типа прогрессбар
			if(i%10==0) {
				setRGB(0, 0, 0);		
				delay(100);	
				setRGB(255, 0, 0);	
				delay(100);
			}
			if(i%100==0)
				Serial << F(".");
			wdt_reset();
		}				
	Serial << F(" done.") << endl;
	setRGB(0, 0, 0);		
}
//---------------------------------------------------------------------------
void ProcessBTN(uint8_t p_pool) {
	static uint8_t btn;
	//Если не пуллинг - то вызов в цикле опроса
	if (!p_pool){	//Проверим кнопку
		btn = digitalRead(BUTTON);
		if (btn && btnTime == 0) {
			delay(5); //защитимся от помех и дребезга пауза  и повтор чтения.
			btn = digitalRead(BUTTON);
			if (btn && btnTime == 0) {
				btnTime = curMillis;
			}
		}
	}
	//Если вызов в пуллинге - это регулярный опрос (1раз в сек)
	else {
		//обработаем кнопку - считаем сколько ее держали
		if (btn) {
			//мигнули 
			setRGB(0, 0, 0);
			delay(100);
			setRGB(255, 0, 0);
			delay(100);
			setRGB(0, 0, 0);
			delay(200);
		}	else {
			if(btnTime>0){
				//Кнопку держали более 10 сек, чистим все, время в 0 сбрасываем цикл
				if( curMillis - btnTime > 10000) {
					//Чистим память
					ClearEEPROM();
					//Время в 0
					//запуск часов
					tm.Year = 1;
          tm.Month = 1;
          tm.Day = 1;
          tm.Hour = 23;
          tm.Minute = 59;
          tm.Second = 50;
					t = makeTime(tm);
					//use the time_t value to ensure correct weekday is set
          if(RTC.set(t) == 0) { // Success
            setTime(t);
            Serial << F("RTC reset") << endl;
						PrintTime();
						Serial << endl;
					}
					else { 
						Serial << F("RTC reset failed!") << endl;
						ERR_RTC = 1;
					}
				} 
				//кнопку держали более 3х секунд, если первый бачок пуст - запускаем его наполнение
				if( curMillis - btnTime > 3000 && CON == C_EMPTY ) {
					PrintTime();
					Serial << F("ReFILL container 1 started...") << endl;
					CON = C_DRAIN;
					PrintStatusCON();
					CH1 = C1_STOP;
					PrintStatusCH1();
					CH2 = C2_STOP;
					PrintStatusCH2();
					CurrentCount1 = CurrentCount2 = 255;
					uint16_t c;
					SetValve(V_DRAIN1, 		LOW);
					SetValve(V_RAW, 			LOW);
					//открыли клапан для слива C2
					SetValve(V_DRAIN2, 		HIGH);
				}
				//отпустили батон
				btnTime = 0; 
			}
		}
	}
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
void PrintSensors(uint8_t detail = 0) {	
	PrintTime();
	Serial << endl;
	//ProcessSensors();
	if (detail) {
		SetCNT();
		Serial << F("CurrentCount1:\t") << CurrentCount1;
		Serial << F("\tCurrentCount2:\t") << CurrentCount2;
		Serial << F("\tTargetCount1:\t") << TargetCount1;
		Serial << F("\tTargetCount2:\t") << TargetCount2;
		Serial << F("\tRO%:\t") << RO_part << endl;
	}
	
	if (detail==2){
		if (RTC.haltRTC()) {
			Serial << F("The DS1302 is stopped.  Please set time") << endl;
			Serial << F("to initialize the time and begin running.") << endl;
			Serial << endl;
		}
		else {
			Serial << F("The DS1302 is started") << endl;
		}
		if (!RTC.writeEN()) Serial << F("The DS1302 is write protected. This normal.") << endl;

		Serial << F("RTC Sync");
			if (timeStatus() == timeSet) {
				Serial << F(" OK!") << endl;
			}
			else {
				Serial << F(" FAIL!") << endl;
				Serial << F("ERR - TIME - RTC is not runing. Check battery, reset system.") << endl;
			}
	}
	
	Serial << F("\tBTN=") << digitalRead(BUTTON);
		
	Serial << F("\tM:[") << mRO << F(", ") << CON << F(", ") << CH1 << F(", ") << CH2 << F("]");
	
	Serial << F("\tLED=") << LED_MODE;
	Serial << F("\tRO=") << RO;
	
	Serial << F("\tLV1=") << LV1;
	Serial << F("\tLV2=") << LV2;	
	
	Serial << F("\tPUMP_TIME=") << TimeON_RO;
	
	Serial << F("\tPR_SENS=") << analogRead(PR_SENS);
	
	Serial << endl;	
}
//---------------------------------------------------------------------------
void ProcessRO(void) {
	switch (mRO) {
		//********************
		case RO_UNDEF:
			// Емкость полная и не нужно наполнять концентрат или второй дозатор
			if (RO) {
				mRO = RO_READY;
				PrintStatusRO();
				digitalWrite(PUMP_RO, LOW);
			}
			else {
				mRO = RO_FILL;
				PrintStatusRO();
				digitalWrite(PUMP_RO, HIGH);
			}
		break;
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
		case RO_OVER:
			if ( CON != C_FILL && CON != C_FILL_WAIT_MIXING && CON != C_FILL_MIXING && CH2 != C2_FILL_RO ) {
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
			if ( CON == C_FILL || CON == C_FILL_WAIT_MIXING || CON == C_FILL_MIXING || CH2 == C2_FILL_RO ) {
				mRO = RO_OVER;
				PrintStatusRO();
				digitalWrite(PUMP_RO, HIGH);
			}
		break;
		//********************
		case RO_DISABLE:
			digitalWrite(PUMP_RO, LOW);
		break;
	}	
	
	//Предохранитель - если все уровни поверху - выключить подачу воды
	if ( mRO!=RO_OVER && RO && LV1>=3 && LV2>=100 && (digitalRead(PUMP_RO) || digitalRead(V_RAW)) ) {
		ERR_WATER = 1;
		digitalWrite(PUMP_RO, LOW);
		SetValve(V_RAW, LOW);
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
	SetCNT();
	PrintSensors(2);
	Serial << F("Test outputs:") << endl;
	Serial << F("TopUp valve ON..."); SetValve(V_TOPUP, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_TOPUP, LOW);
	Serial << F("Drain 1 valve ON..."); SetValve(V_DRAIN1, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_DRAIN1, LOW);
	Serial << F("Fill 1 valve ON..."); SetValve(V_FILL1, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_FILL1, LOW);
	wdt_reset(); // не забываем сбросить смотрящую собаку
	Serial << F("Drain 2 valve ON..."); SetValve(V_DRAIN2, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_DRAIN2, LOW);
	Serial << F("Flush valve ON..."); SetValve(V_FLUSH, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_FLUSH, LOW);
	Serial << F("Pump RO ON..."); digitalWrite(PUMP_RO, HIGH); delay(1000);	Serial << F("OFF") << endl;	digitalWrite(PUMP_RO, LOW);
	Serial << F("Main valve ON..."); SetValve(V_RAW, HIGH); delay(1000);	Serial << F("OFF") << endl;	SetValve(V_RAW, LOW);
		
	wdt_reset(); // не забываем сбросить смотрящую собаку
	
	Serial << F("Mixer ON..."); digitalWrite(MIXER, HIGH); delay(1000);	Serial << F("OFF") << endl;	digitalWrite(MIXER, LOW);
	Serial << F("Self test complete") << endl;
}
//---------------------------------------------------------------------------
void setup() {
	uint8_t tmpi;
	
	wdt_enable (WDTO_8S); 
  	
	pinMode(PUMP_RO, 		OUTPUT);
	pinMode(V_RAW, 		OUTPUT);
	pinMode(V_DRAIN2, 	OUTPUT);
	pinMode(V_FILL1, 		OUTPUT);
	pinMode(V_DRAIN1, 	OUTPUT);
	pinMode(V_FLUSH, 		OUTPUT);
	pinMode(V_TOPUP, 		OUTPUT);

	pinMode(MIXER, 			OUTPUT);
	pinMode(LED_Y, 			OUTPUT);
	pinMode(LED_G, 			OUTPUT);
	pinMode(BUTTON, 		INPUT);
	
	SerialDBG = 0;
	Serial.begin(115200);
	inputString.reserve(200);
	TimeON_RO = 0;
	
		
	mRO = RO_UNDEF;
	CON = C_UNDEF;
	CH1	= C1_UNDEF;
	CH2	= C2_UNDEF;
		
	LED_MODE = LM_NORMAL;
	
	//Прочитаем кол-во подмен с прошлого рестарта 
	for (CurrentCount1 = ADR_COUNT1; CurrentCount1 < ADR_COUNT1+32 && EEPROM.read(CurrentCount1); CurrentCount1 ++);
	CurrentCount1 = CurrentCount1 - ADR_COUNT1;
	
	for (CurrentCount2 = ADR_COUNT2; CurrentCount2 < ADR_COUNT2+32 && EEPROM.read(CurrentCount2); CurrentCount2 ++);
	CurrentCount2 = CurrentCount2 - ADR_COUNT2;
	
	SetCNT();
	//Если текущее кол-во подмен =0, значит продолжать не надо, блокируем канал выставляя кол-во = 255
	//кроме того если текущее кол-во равно или больше целевого - блокируем канал
	if (CurrentCount1 == 0 || CurrentCount1 >= TargetCount1)	CurrentCount1 = 255;
	if (CurrentCount2 == 0 || CurrentCount2 >= TargetCount2)	CurrentCount2 = 255;
	
	//читаем режим работы из флэшки
	tmpi = EEPROM.read(ADR_MODE); 
	if ( C_FILL <= tmpi && tmpi <= C_MIXING ) { // если рабочий
		//Восстанавливаем режим
		CON = (CON_t)tmpi;
		//Блокируем подмены по обоим каналам
		CH1 = C1_DISABLE;
		CH2 = C2_DISABLE;
		Serial << F("CON from eeprom:\t") << CON << F("\tCH1&CH2 are blocked.");
	}						
	
	//setSyncProvider() causes the Time library to synchronize with the
  //external RTC by calling RTC.get() every five minutes by default.
  setSyncProvider(RTC.get);
	setSyncInterval(3600);

	if (timeStatus() == timeSet) ERR_RTC = 0;
	else ERR_RTC = 1;	
	
	SelfTest();
	
	PrintStatusRO();
	PrintStatusCON();
	PrintStatusCH1();
	PrintStatusCH2();
	
	ERR_WATER = 0;
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
void StartCH1(void) {
	SetCNT();
	if ( CurrentCount1==255 ) {
		PrintTime();
		Serial << F(" CH1 command start recieved");
		if ( CH1 == C1_UNDEF || CH1 == C1_STOP ) {
			CurrentCount1 = 0;
			CH1 = C1_READY;
			Serial << F(", start cycle ***") << endl;
			PrintStatusCH1();
		}	else	Serial << F(", but the state does not allow start cycle ***") << endl;
	}
}
//--------------------------------------------------------------------------
void StartCH2(void) {
	SetCNT();
	if (CurrentCount2==255) {
	  PrintTime();
		Serial << F(" CH2 command start recieved");
		if ( CH2 == C2_UNDEF || CH2 == C2_STOP ) {
			CurrentCount2 = 0;
			CH2 = C2_READY;
			Serial << F(", start cycle ***") << endl;
			PrintStatusCH2();
		}	else	Serial << F(", but the state does not allow start cycle ***") << endl;
	}
}
//--------------------------------------------------------------------------
void loop() {
	uint8_t tmpi, btn;
	unsigned long t;
	
	wdt_reset(); // не забываем сбросить смотрящую собаку
	
	ProcessLED();
	ProcessBTN(0);

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
	  
		if ( !ERR_SENSOR && !ERR_WATER ) {
			ProcessSensors();	
			ProcessRO();
			ProcessBTN(1);
		
			if ( hour()==0 && minute()==0 && second()==0 ) {
				Serial << endl << F("*** Timer reached ****") << endl;
			StartCH1();
			StartCH2();
		}
		
		// print the string when a newline arrives:
		if (stringComplete) {
			inputString.toLowerCase();
			inputString.trim();
			Serial << inputString << endl;
			if (inputString.equals(F("help"))) 
				{ Serial << F("comands:") << endl;
					Serial << F("start ch1") << endl;
					Serial << F("start ch2") << endl;
					Serial << F("diag") << endl;
					Serial << F("help") << endl;
					Serial << F("debug=on") << endl;
					Serial << F("debug=off") << endl; 
				}
			if (inputString.equals(F("start ch1"))) StartCH1();
			if (inputString.equals(F("start ch2"))) StartCH2();
			if (inputString.equals(F("status")) || inputString.equals(F("diag"))) 
				{ PrintSensors(2); 
					PrintStatusRO();
					PrintStatusCON();
					PrintStatusCH1();
					PrintStatusCH2();
			  }
			if (inputString.equals(F("help"))) { Serial << "start ch1" << endl << "start ch2" << endl << "status" << endl << F("Command separate are: \\n, @, ;"); };
			if (inputString.equals(F("debug=on")) || inputString.equals(F("debug on")) ) SerialDBG = 1;
			if (inputString.equals(F("debug=off")) || inputString.equals(F("debug off")) ) SerialDBG = 0;
			// clear the string:
			inputString = "";
			stringComplete = false;
		}
		//------------------------------------------------------------------------------------
		// Работа по емкости с концентратом
		// Если емкость 1 пуста и не находится в режимах наполнения (или статус неведомый) - ставим статус пусто
		if ( LV1==1 && CON < C_EMPTY ) {
			CON = C_EMPTY;
			//Блокируем работу канала 1
			CH1 = C1_DISABLE;
			CurrentCount1 = 255;
			PrintStatusCON();
			PrintStatusCH1();
			SetValve(V_FILL1, LOW);
			SetValve(V_DRAIN1, 	LOW);
		}
		// обработка режимов емкости с концентратом (CON)
		switch (CON) {
			//****************
			case C_DRAIN:	//Сливаем только сливаем емкость С2. Концентрат сливать нельзя.
				//Если емкость второго канала пуста - переходим в ожидание дослива по времени
				if ( LV2 == 0 ) {
					//слили воду до датчиков, переходим к ожиданию слива воды под датчиками (по таймауту)
					CON = C_DRAIN_T;
					PrintStatusCON();
					prevMillis3 = curMillis;
				}
				//Если емкость не пуста - ждем опустошения
				else Serial << F(".");
			break;
			//****************
			case C_DRAIN_T:
				if ( curMillis - prevMillis3 > T_DRAIN1	&& curMillis - prevMillis3 > T_DRAIN2 ) {
					//слили все, переходим к наполнению
					SetValve(V_DRAIN1, 		LOW);
					SetValve(V_RAW, 			LOW);
					SetValve(V_FILL1, 		LOW);
					SetValve(V_DRAIN2, 		LOW);
					Serial << endl << F("End of drainage container 2.") << endl << F("Start FILL container 1.") << endl;
					CON = C_FILL;
					EEPROM.write(ADR_MODE, (uint8_t)C_FILL);
					PrintStatusCON();
				}	else Serial << F(".");
			break;
			//****************
			case C_FILL:
				//Если режим наполнения работает и мин. уровень достигнут - запускаем таймер до миксера
				if ( LV1>1 ) {
					prevMillis2 = curMillis;
					CON = C_FILL_WAIT_MIXING;
					PrintStatusCON();
				}
			break;
			//****************
			case C_FILL_WAIT_MIXING:
				//Если минимальный уровень налит и время истекло - включаем миксер
				if ( LV1>1 && curMillis - prevMillis2 > T_MIXING_FILL ) {
					CON = C_FILL_MIXING;
					PrintStatusCON();
					digitalWrite(MIXER, HIGH);
					EEPROM.write(ADR_MODE, (uint8_t)C_FILL_MIXING);
				}
			break;
			//****************
			case C_FILL_MIXING:
				digitalWrite(MIXER, HIGH); 
				//Если емкость 1 наполнена
				if ( LV1==3 ) {
					CON = C_MIXING;
					PrintStatusCON();
					prevMillis2 = curMillis;
					EEPROM.write(ADR_MODE, (uint8_t)C_MIXING);
				}
			break;
			//****************
			case C_MIXING:
				digitalWrite(MIXER, HIGH); 
				//Если емкость 1 наполнена
				if ( curMillis - prevMillis2 > T_MIXING_AF ) {
					CON = C_READY;
					CH1 = C1_STOP;
					CH2 = C2_STOP;
					PrintStatusCON();
					PrintStatusCH1();
					PrintStatusCH2();
					EEPROM.write(ADR_MODE, (uint8_t)C_READY);
					//выключаем миксер и осмос - у нас полные баки 1, 2 и RO
					digitalWrite(MIXER, LOW);
				}
			break;
		}
			
		//--------------------------------------------------------------------------------------
		//обработка первого канала если он не остановлен и статус готов или далее и есть чо в емкости
		switch (CH1) {
			//****************
			case C1_UNDEF:
				if ( CurrentCount1 <= TargetCount1 ) {
					CH1 = C1_DRAIN;
					PrintStatusCH1();
					//Добавим цикл на слив
					TargetCount1++;
					//Наполнение выключаем
					SetValve(V_FILL1, 	LOW);
					//Включаем слив
					SetValve(V_DRAIN1, 		HIGH);
					prevMillis3 = curMillis;
				}
			break;
			//****************
			case C1_READY: //Если готов - наполняем
				//Цикл по первому каналу не закончился
				if ( CurrentCount1 < TargetCount1 ) {
					CH1 = C1_MIXING;
					PrintStatusCH1();
					prevMillis3 = curMillis;
				}
				//Цикл закончился - Выключаем клапана, чистим журнал в EEPROM
				else {
					CH1 = C1_STOP;
					PrintStatusCH1();
					SetValve(V_FILL1, LOW);
					SetValve(V_DRAIN1, 	 LOW);	
					for (tmpi = ADR_COUNT1; tmpi < ADR_COUNT1+32; tmpi++)
						EEPROM.write(tmpi, 0);
					//Блокируем подмены, цикл окончен до следующего времени
					CurrentCount1 = 255;
				}
			break;
			//***************
			case C1_MIXING:
				digitalWrite(MIXER, HIGH);
				if(curMillis - prevMillis3 > T_MIXING) {
					CH1 = C1_FILL;
					PrintStatusCH1();
					SetValve(V_FILL1, HIGH);
					digitalWrite(MIXER, LOW);
					prevMillis3 = curMillis;
				}				
			break;
			//***************
			case C1_FILL: //наполняем и время вышло
				if (curMillis - prevMillis3 > T_FILL1 ) {
					CH1 = C1_DRAIN;
					PrintStatusCH1();
					//Наполнение выключаем
					SetValve(V_FILL1, 	LOW);
					//Включаем слив
					SetValve(V_DRAIN1, 		HIGH);
					prevMillis3 = curMillis;
				}
			break;
			//****************
			case C1_DRAIN:		//Сливаем и время вышло
				if ( curMillis - prevMillis3 > T_DRAIN1 ) {
					CH1 = C1_WAIT;
					PrintStatusCH1();
					//Слив выключаем
					SetValve(V_DRAIN1, 	LOW);
					prevMillis3 = curMillis;
					//Цикл будем дальше гнать
					CurrentCount1 ++;
					EEPROM.write(ADR_COUNT1+CurrentCount1-1, 1);
					Serial << F("CH1 Count:") << CurrentCount1 << F("/") << TargetCount1 << F(" current/target.")  << endl;
				}
			break;
			//****************
			case C1_WAIT: //Прошли итерацию, сделаем паузу до следующей
				if (curMillis - prevMillis3 > T_WAIT1 ) {
					CH1 = C1_READY;
					PrintStatusCH1();
				}
			break;
			//****************
			/*case C1_STOP:
			break;
			//****************
			case C1_DISABLE:
			break;*/
		}			
		
		//--------------------------------------------------------------------------------------
		//обработка второго канала если он не остановлени и статус готов или далее
		//Цикл по второму каналу не закончился
		switch (CH2) {  //Если готов (стартовое состояние) и не ожидание
			//****************
			case C2_UNDEF:
				if ( CurrentCount2 <= TargetCount2 ) { //не прошли нужное кол-во циклов
					CH2 = C2_READY;
					PrintStatusCH2();
				}
			break;
			case C2_EMPTY:
				if ( CurrentCount2 >= TargetCount2 ) { //прошли нужное кол-во циклов
					CH2 = C2_STOP;
					PrintStatusCH2();
					Serial << F("CH2 end cycle") << endl;
					SetValve(V_DRAIN2, LOW);	
					for (tmpi = ADR_COUNT2; tmpi < ADR_COUNT2+32; tmpi++)
						EEPROM.write(tmpi, 0);
					//Блокируем подмены
					CurrentCount2 = 255;
				} else {
					CH2 = C2_READY;
					PrintStatusCH2();
				}
			break;
			//****************
			case C2_READY:
				if ( LV2 == 0 ) { //пуст - наполняем (нормальный старт)
					CH2 = C2_FILL_RAW;
					PrintStatusCH2();
					SetValve(V_RAW, HIGH);
				}
				else {//не нормальный старт, емкость не пуста, не важно что в ней - сливаем
					CH2 = C2_DRAIN;
					PrintStatusCH2();
					Serial << F("CH2 abnormal start, adding 1 cyclce to drain.") << endl;
					//Добавим цикл на слив
					TargetCount2++;
					//Включаем слив
					SetValve(V_DRAIN2, 	HIGH);
					prevMillis4 = curMillis;
				}
			break;
			//****************
			case C2_FILL_RAW:	//Режим наполнения простой водой
				//если не надо осмоса
				if ( LV2 >= 100 && RO_part == 0 ) {
					PrintStatusCH2();
					//Наполнение водой выключаем
					SetValve(V_RAW, 	LOW);
					CH2 = C2_DRAIN;
					//Включаем слив
					SetValve(V_DRAIN2, 	HIGH);
					prevMillis4 = curMillis;
				}
				else {
					//наполняем и налили до метки осмоса водой из фильтра (поэтому 100-RO_part)
					//RO в процентах от - 0 до 100. Уровней дискретно 0-100
					if ( LV2 >= 100-RO_part ) {
						CH2 = C2_FILL_RO;
						PrintStatusCH2();
						//Наполнение водой выключаем
						SetValve(V_RAW, 	LOW);
					}
				}
				if ( (curMillis - prevMillis4) > T_FILL2 ) { //Не дождались наполнения, бросаем ошибку
					CH2 = C2_DISABLE;
					ERR_SENSOR++;
				}				
			break;
			//****************
			case C2_FILL_RO:	//Режим наполнения осмосом
				if ( LV2>=100 ) {
					CH2 = C2_DRAIN;
					PrintStatusCH2();
					//Включаем слив
					SetValve(V_DRAIN2, 	HIGH);
					prevMillis4 = curMillis;
				}
			break;
			//****************
			case C2_DRAIN:				//Слив остатков
				if (LV2==0 || ((curMillis - prevMillis4) > (T_DRAIN_TIMEOUT)) ) {
					if ( (curMillis - prevMillis4) > (T_DRAIN_TIMEOUT)) Serial << F("CH2 Drain time expiried !!!") << endl;
					CH2 = C2_DRAIN_T;
					PrintStatusCH2();
					prevMillis4 = curMillis;
				}
			break;
			//****************
			case C2_DRAIN_T:				//Слив по времени
				if (curMillis - prevMillis4 > T_DRAIN2) {
					CH2 = C2_WAIT;
					PrintStatusCH2();
					//Слив выключаем
					SetValve(V_DRAIN2, 	LOW);
					CurrentCount2 ++;
					EEPROM.write(ADR_COUNT2+CurrentCount2-1, 1);
					Serial << F("CH2 count:") << CurrentCount2 << F("/") << TargetCount2 << F(" current/target.") << endl;
					prevMillis4 = curMillis;
				}
			break;
			//****************
			case C2_WAIT:				//Пауза
				//Прошли итерацию, сделаем паузу до следующей
				if ( curMillis - prevMillis4 > T_WAIT2 ) {
					CH2 = C2_EMPTY;
					PrintStatusCH2();
					prevMillis4 = curMillis;
				}
			break;
			//****************
			/*case C2_STOP:
			break;
			case C2_DISABLE:
			break;*/
		}
		
		if (SerialDBG) PrintSensors(1);
	}	
		//Отображение режима работы
		if (ERR_SENSOR || ERR_RTC || ERR_WATER) { //Ошибка датчиков или часов - все выключить и стоп
			mRO = RO_DISABLE;
			CON  = C_DISABLE;
			CH1 = C1_DISABLE;
			CH2 = C2_DISABLE;
			SetValve(V_DRAIN1, 		LOW);
			SetValve(V_RAW, 			LOW);
			SetValve(V_FILL1, 	LOW);
			SetValve(V_DRAIN2, 		LOW);
			digitalWrite(PUMP_RO, LOW);
			digitalWrite(MIXER, 	LOW);
			Serial.println("");
			if (ERR_SENSOR) {
				LED_MODE = LM_ERR_SENSOR; 
				Serial << F("Sensor error ! System stopped.") << endl;
			}
			if (ERR_RTC) {
				Serial << F("RTC error ! System stopped.") << endl;
				LED_MODE = LM_ERR_RTC;
			}		
			if (ERR_WATER) {
				LED_MODE = LM_ERR_WATER; 
				Serial << F("Water error ! System stopped.") << endl;
			}			
			Serial << endl;
			PrintSensors(1);
			return;    
		}

		if (	 CON  == C_FILL			
				|| CON  == C_FILL_WAIT_MIXING	
				|| CON  == C_FILL_MIXING
				|| mRO  == RO_FILL
			) { 
			LED_MODE = LM_FILL; return;    
		}
		if (	 CH1 == C1_STOP
				&& CH2 == C2_STOP
			) { 
			LED_MODE = LM_STOP; return;    
		}
		if ( CON == C_EMPTY ) {
			LED_MODE = LM_EMPTY; return;    
		}		
		LED_MODE = LM_NORMAL;
	}		
}