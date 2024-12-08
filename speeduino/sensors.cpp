/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Read sensors with appropriate timing / scheduling.
 */
#include "sensors.h"
#include "crankMaths.h"
#include "globals.h"
#include "maths.h"
#include "storage.h"
#include "comms.h"
#include "idle.h"
#include "errors.h"
#include "corrections.h"
#include "pages.h"
#include "decoders.h"
#include "auxiliaries.h"
#include "utilities.h"
#include BOARD_H

uint32_t MAPcurRev; //Tracks which revolution we're sampling on
unsigned int MAPcount; //Number of samples taken in the current MAP cycle
unsigned long MAPrunningValue; //Used for tracking either the total of all MAP readings in this cycle (Event average) or the lowest value detected in this cycle (event minimum)
unsigned long EMAPrunningValue; //As above but for EMAP
bool auxIsEnabled;
uint16_t MAPlast; /**< The previous MAP reading */
unsigned long MAP_time; //The time the MAP sample was taken
unsigned long MAPlast_time; //The time the previous MAP sample was taken
volatile unsigned long vssTimes[VSS_SAMPLES] = {0};
volatile byte vssIndex;

volatile byte flexCounter = 0;
volatile unsigned long flexStartTime;
volatile unsigned long flexPulseWidth;

volatile byte knockCounter = 0;
volatile uint16_t knockAngle;

//These variables are used for tracking the number of running sensors values that appear to be errors. Once a threshold is reached, the sensor reading will go to default value and assume the sensor is faulty
byte mapErrorCount = 0;
//byte iatErrorCount = 0; Not used
//byte cltErrorCount = 0; Not used

static inline void validateMAP(void);

#if defined(ANALOG_ISR)
static volatile uint16_t AnChannel[16];

ISR(ADC_vect)
{
  byte nChannel = (ADMUX & 0x07);

  byte result_low = ADCL;
  byte result_high = ADCH;

  #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
    if (nChannel == 7) { ADMUX = 0x40; }
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if( BIT_CHECK(ADCSRB, MUX5) ) { nChannel += 8; }  //8 to 15
    if(nChannel == 15)
    {
      ADMUX = ADMUX_DEFAULT_CONFIG; //channel 0
      ADCSRB = 0x00; //clear MUX5 bit

      BIT_CLEAR(ADCSRA,ADIE); //Disable interrupt as we're at the end of a full ADC cycle. This will be re-enabled in the main loop
    }
    else if (nChannel == 7) //channel 7
    {
      ADMUX = ADMUX_DEFAULT_CONFIG;
      ADCSRB = 0x08; //Set MUX5 bit
    }
  #endif
    else { ADMUX++; }

  //ADMUX always appears to be one ahead of the actual channel value that is in ADCL/ADCH. Subtract 1 from it to get the correct channel number
  if(nChannel == 0) { nChannel = 16;} 
  AnChannel[nChannel-1] = (result_high << 8) | result_low;
}
#endif

/** Init all ADC conversions by setting resolutions, etc.
 */
void initialiseADC(void)
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this

  #if defined(ANALOG_ISR)
    noInterrupts(); //Interrupts should be turned off when playing with any of these registers

    ADCSRB = 0x00; //ADC Auto Trigger Source is in Free Running mode

    ADMUX = ADMUX_DEFAULT_CONFIG;  //Select AVCC as reference, ADC Right Adjust Result, Starting at channel 0

    //All of the below is the longhand version of: ADCSRA = 0xEE;
    #ifndef ADFR
      #define ADFR 5 //Looks like this is now defined. Retain this for compatibility with earlier versions of Arduino IDE that did not have this.
    #endif
    BIT_SET(ADCSRA,ADFR); //Set free running mode
    BIT_SET(ADCSRA,ADIE); //Set ADC interrupt enabled
    BIT_CLEAR(ADCSRA,ADIF); //Clear interrupt flag

    // Set ADC clock to 125KHz (Prescaler = 128)
    BIT_SET(ADCSRA,ADPS2);
    BIT_SET(ADCSRA,ADPS1);
    BIT_SET(ADCSRA,ADPS0);

    BIT_SET(ADCSRA,ADEN); //Enable ADC

    interrupts();
    BIT_SET(ADCSRA,ADSC); //Start conversion

  #else
    //This sets the ADC (Analog to Digital Converter) to run at 1Mhz, greatly reducing analog read times (MAP/TPS) when using the standard analogRead() function
    //1Mhz is the fastest speed permitted by the CPU without affecting accuracy
    //Please see chapter 11 of 'Practical Arduino' (books.google.com.au/books?id=HsTxON1L6D4C&printsec=frontcover#v=onepage&q&f=false) for more detail
     BIT_SET(ADCSRA,ADPS2);
     BIT_CLEAR(ADCSRA,ADPS1);
     BIT_CLEAR(ADCSRA,ADPS0);
  #endif
#elif defined(ARDUINO_ARCH_STM32) //STM32GENERIC core and ST STM32duino core, change analog read to 12 bit
  analogReadResolution(10); //use 10bits for analog reading on STM32 boards
#endif
  MAPcurRev = 0;
  MAPcount = 0;
  MAPrunningValue = 0;

  //The following checks the aux inputs and initialises pins if required
  auxIsEnabled = false;
  for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
  {
    currentStatus.current_caninchannel = AuxinChan;                   
    if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4)
    && ((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))))
    { //if current input channel is enabled as external input in caninput_selxb(bits 2:3) and secondary serial or internal canbus is enabled(and is mcu supported)                 
      //currentStatus.canin[14] = 22;  Dev test use only!
      auxIsEnabled = true;     
    }
    else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 8)
            || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)  
            || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)))  
    {  //if current input channel is enabled as analog local pin check caninput_selxb(bits 2:3) with &12 and caninput_selxa(bits 0:1) with &3
      byte pinNumber = pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63);
      if( pinIsUsed(pinNumber) )
      {
        //Do nothing here as the pin is already in use.
        BIT_SET(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Tell user that there is problem by lighting up the I/O error indicator
      }
      else
      {
        //Channel is active and analog
        pinMode( pinNumber, INPUT);
        //currentStatus.canin[14] = 33;  Dev test use only!
        auxIsEnabled = true;
      }  
    }
    else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 12)
            || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)
            || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)))
    {  //if current input channel is enabled as digital local pin check caninput_selxb(bits 2:3) with &12 and caninput_selxa(bits 0:1) with &3
       byte pinNumber = (configPage9.Auxinpinb[currentStatus.current_caninchannel]&63) + 1;
       if( pinIsUsed(pinNumber) )
       {
         //Do nothing here as the pin is already in use.
        BIT_SET(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Tell user that there is problem by lighting up the I/O error indicator
       }
       else
       {
         //Channel is active and digital
         pinMode( pinNumber, INPUT);
         //currentStatus.canin[14] = 44;  Dev test use only!
         auxIsEnabled = true;
       }  

    }
  } //For loop iterating through aux in lines
  

  //Sanity checks to ensure none of the filter values are set above 240 (Which would include the 255 value which is the default on a new arduino)
  //If an invalid value is detected, it's reset to the default the value and burned to EEPROM. 
  //Each sensor has it's own default value
  if(configPage4.ADCFILTER_TPS  > 240) { configPage4.ADCFILTER_TPS   = ADCFILTER_TPS_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_CLT  > 240) { configPage4.ADCFILTER_CLT   = ADCFILTER_CLT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_IAT  > 240) { configPage4.ADCFILTER_IAT   = ADCFILTER_IAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_O2   > 240) { configPage4.ADCFILTER_O2    = ADCFILTER_O2_DEFAULT;    writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BAT  > 240) { configPage4.ADCFILTER_BAT   = ADCFILTER_BAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_MAP  > 240) { configPage4.ADCFILTER_MAP   = ADCFILTER_MAP_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BARO > 240) { configPage4.ADCFILTER_BARO  = ADCFILTER_BARO_DEFAULT;  writeConfig(ignSetPage); }
  if(configPage4.FILTER_FLEX    > 240) { configPage4.FILTER_FLEX     = FILTER_FLEX_DEFAULT;     writeConfig(ignSetPage); }

  flexStartTime = micros();

  vssIndex = 0;
}

static inline void validateMAP(void)
{
  //Error checks
  if(currentStatus.MAP < VALID_MAP_MIN)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_LOW;
    mapErrorCount += 1;
    setError(ERR_MAP_LOW);
  }
  else if(currentStatus.MAP > VALID_MAP_MAX)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_HIGH;
    mapErrorCount += 1;
    setError(ERR_MAP_HIGH);
  }
  else
  {
    if(errorCount > 0)
    {
      clearError(ERR_MAP_HIGH);
      clearError(ERR_MAP_LOW);
    }
    mapErrorCount = 0;
  }
}

void instanteneousMAPReading(void)
{
  //Update the calculation times and last value. These are used by the MAP based Accel enrich
  MAPlast = currentStatus.MAP;
  MAPlast_time = MAP_time;
  MAP_time = micros();

  unsigned int tempReading;
  //Instantaneous MAP readings
  #if defined(ANALOG_ISR_MAP)
    tempReading = AnChannel[pinMAP-A0];
  #else
    tempReading = analogRead(pinMAP);
    tempReading = analogRead(pinMAP);
  #endif
  //Error checking
  if( (tempReading >= VALID_MAP_MAX) || (tempReading <= VALID_MAP_MIN) ) { mapErrorCount += 1; }
  else { mapErrorCount = 0; }

  //During startup a call is made here to get the baro reading. In this case, we can't apply the ADC filter
  if(currentStatus.initialisationComplete == true) { currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC); } //Very weak filter
  else { currentStatus.mapADC = tempReading; } //Baro reading (No filter)

  currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
  if(currentStatus.MAP < 0) { currentStatus.MAP = 0; } //Sanity check
  
  //Repeat for EMAP if it's enabled
  if(configPage6.useEMAP == true)
  {
    #if defined(ANALOG_ISR_MAP)
      tempReading = AnChannel[pinEMAP-A0];
    #else
      tempReading = analogRead(pinEMAP);
      tempReading = analogRead(pinEMAP);
    #endif

    //Error check
    if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
      {
        currentStatus.EMAPADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
      }
    else { mapErrorCount += 1; }
    currentStatus.EMAP = fastMap10Bit(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
    if(currentStatus.EMAP < 0) { currentStatus.EMAP = 0; } //Sanity check
  }

}

void readMAP(void)
{
  unsigned int tempReading;
  //MAP Sampling system
  switch(configPage2.mapSample)
  {
    case 0:
      //Instantaneous MAP readings
      instanteneousMAPReading();
      break;

    case 1:
      //Average of a cycle

      if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && ((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) && (currentStatus.startRevolutions > 1) ) //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
      {
        if( (MAPcurRev == currentStatus.startRevolutions) || ( (MAPcurRev+1) == currentStatus.startRevolutions) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif

          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
            MAPrunningValue += currentStatus.mapADC; //Add the current reading onto the total
            MAPcount++;
          }
          else { mapErrorCount += 1; }

          //Repeat for EMAP if it's enabled
          if(configPage6.useEMAP == true)
          {
            #if defined(ANALOG_ISR_MAP)
              tempReading = AnChannel[pinEMAP-A0];
            #else
              tempReading = analogRead(pinEMAP);
              tempReading = analogRead(pinEMAP);
            #endif

            //Error check
            if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
            {
              currentStatus.EMAPADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
              EMAPrunningValue += currentStatus.EMAPADC; //Add the current reading onto the total
            }
            else { mapErrorCount += 1; }
          }
        }
        else
        {
          //Reaching here means that the last cycle has completed and the MAP value should be calculated
          //Sanity check
          if( (MAPrunningValue != 0) && (MAPcount != 0) )
          {
            //Update the calculation times and last value. These are used by the MAP based Accel enrich
            MAPlast = currentStatus.MAP;
            MAPlast_time = MAP_time;
            MAP_time = micros();

            currentStatus.mapADC = udiv_32_16(MAPrunningValue, MAPcount);
            currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
            validateMAP();

            //If EMAP is enabled, the process is identical to the above
            if(configPage6.useEMAP == true)
            {
              currentStatus.EMAPADC = udiv_32_16(EMAPrunningValue, MAPcount); //Note that the MAP count can be reused here as it will always be the same count.
              currentStatus.EMAP = fastMap10Bit(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
              if(currentStatus.EMAP < 0) { currentStatus.EMAP = 0; } //Sanity check
            }
          }
          else { instanteneousMAPReading(); }

          MAPcurRev = currentStatus.startRevolutions; //Reset the current rev count
          MAPrunningValue = 0;
          EMAPrunningValue = 0; //Can reset this even if EMAP not used
          MAPcount = 0;
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to cycle average.
        if(configPage6.useEMAP == true)
        {
          EMAPrunningValue = currentStatus.EMAPADC;
        }
        MAPcount = 1;
      }
      break;

    case 2:
      //Minimum reading in a cycle
      if (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
      {
        if( (MAPcurRev == currentStatus.startRevolutions) || ((MAPcurRev+1) == currentStatus.startRevolutions) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif
          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            if( (unsigned long)tempReading < MAPrunningValue ) { MAPrunningValue = (unsigned long)tempReading; } //Check whether the current reading is lower than the running minimum
          }
          else { mapErrorCount += 1; }
        }
        else
        {
          //Reaching here means that the last cycle has completed and the MAP value should be calculated

          //Update the calculation times and last value. These are used by the MAP based Accel enrich
          MAPlast = currentStatus.MAP;
          MAPlast_time = MAP_time;
          MAP_time = micros();

          currentStatus.mapADC = MAPrunningValue;
          currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
          MAPcurRev = currentStatus.startRevolutions; //Reset the current rev count
          MAPrunningValue = 1023; //Reset the latest value so the next reading will always be lower

          validateMAP();
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC;  //Keep updating the MAPrunningValue to give it head start when switching to cycle minimum.
      }
      break;

    case 3:
      //Average of an ignition event
      if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && ((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) && (currentStatus.startRevolutions > 1) && (! currentStatus.engineProtectStatus) ) //If the engine isn't running, fall back to instantaneous reads
      {
        if( (MAPcurRev == ignitionCount) ) //Watch for a change in the ignition counter to determine whether we're still on the same event
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif

          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
            MAPrunningValue += currentStatus.mapADC; //Add the current reading onto the total
            MAPcount++;
          }
          else { mapErrorCount += 1; }
        }
        else
        {
          //Reaching here means that the  next ignition event has occurred and the MAP value should be calculated
          //Sanity check
          if( (MAPrunningValue != 0) && (MAPcount != 0) && (MAPcurRev < ignitionCount) )
          {
            //Update the calculation times and last value. These are used by the MAP based Accel enrich
            MAPlast = currentStatus.MAP;
            MAPlast_time = MAP_time;
            MAP_time = micros();

            currentStatus.mapADC = udiv_32_16(MAPrunningValue, MAPcount);
            currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
            validateMAP();
          }
          else { instanteneousMAPReading(); }

          MAPcurRev = ignitionCount; //Reset the current event count
          MAPrunningValue = 0;
          MAPcount = 0;
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to ignition event average.
        MAPcount = 1;
      }
      break; 

    default:
    //Instantaneous MAP readings (Just in case)
    instanteneousMAPReading();
    break;
  }
}

/**
在这段代码中，readTPS() 函数用于读取油门踏板传感器（TPS，Throttle Position Sensor）的数据，并将其转换为油门开度百分比（TPS%）。TPS传感器监测油门踏板的位置，并将其转化为相应的电压信号供控制单元使用。

代码解析
功能：
该函数读取油门踏板传感器的模拟输入，并通过ADC转换获取一个数值。
通过滤波器（如果启用）对传感器的原始数据进行平滑。
对油门传感器的原始值进行校准，并将其转换为油门开度的百分比（TPS%）。
如果配置了闭油门位置传感器（CTPS），则判断其是否处于活动状态。
关键部分解析：
读取原始传感器数据：

cpp
复制代码
#if defined(ANALOG_ISR)
  byte tempTPS = fastMap1023toX(AnChannel[pinTPS-A0], 255);
#else
  analogRead(pinTPS);
  byte tempTPS = fastMap1023toX(analogRead(pinTPS), 255);
#endif
analogRead(pinTPS) 读取与油门踏板传感器连接的模拟引脚的值。
fastMap1023toX() 函数将 ADC 数值（0-1023）映射到 0-255 的范围。
滤波器处理：

cpp
复制代码
if(useFilter == true) { currentStatus.tpsADC = ADC_FILTER(tempTPS, configPage4.ADCFILTER_TPS, currentStatus.tpsADC); }
else { currentStatus.tpsADC = tempTPS; }
如果启用滤波器（useFilter == true），则通过 ADC_FILTER() 函数对原始值进行平滑，以减少噪声。
如果不使用滤波器，则直接使用原始的传感器值。
校准并限制 TPS 值范围：

cpp
复制代码
if(configPage2.tpsMax > configPage2.tpsMin)
{
  if (currentStatus.tpsADC < configPage2.tpsMin) { tempADC = configPage2.tpsMin; }
  else if(currentStatus.tpsADC > configPage2.tpsMax) { tempADC = configPage2.tpsMax; }
  currentStatus.TPS = map(tempADC, configPage2.tpsMin, configPage2.tpsMax, 0, 200);
}
else
{
  tempADC = 255 - currentStatus.tpsADC;
  uint16_t tempTPSMax = 255 - configPage2.tpsMax;
  uint16_t tempTPSMin = 255 - configPage2.tpsMin;
  if (tempADC > tempTPSMax) { tempADC = tempTPSMax; }
  else if(tempADC < tempTPSMin) { tempADC = tempTPSMin; }
  currentStatus.TPS = map(tempADC, tempTPSMin, tempTPSMax, 0, 200);
}
首先检查 tpsMax 是否大于 tpsMin。如果是，说明传感器的输出范围是正常的。然后对 TPS ADC 值进行限制，确保它在 tpsMin 和 tpsMax 范围内。
map() 函数将校准后的 TPS 值转换为 0-200 范围的百分比。
如果 tpsMax 和 tpsMin 的配置值被反转（例如接线错误），则通过倒置传感器读取值来进行处理，确保值在正确的范围内。
检查闭油门位置传感器（CTPS）：

cpp
复制代码
if(configPage2.CTPSEnabled == true)
{
  if(configPage2.CTPSPolarity == 0) { currentStatus.CTPSActive = !digitalRead(pinCTPS); }
  else { currentStatus.CTPSActive = digitalRead(pinCTPS); }
}
else { currentStatus.CTPSActive = 0; }
如果启用了闭油门位置传感器（CTPS），则根据配置的极性（CTPSPolarity）检查传感器的状态。
如果极性为 0，表示闭油门传感器是通过接地来激活的（低电平表示闭合）；如果极性为 1，表示通过 5V 来激活传感器。
总结：
readTPS() 函数读取油门踏板传感器的原始数据，并将其转换为相应的百分比值（TPS%）。
数据会通过一个可选的滤波器进行平滑，以减少噪声。
函数还会根据配置的最小和最大值对 TPS 数据进行校准，并根据需要检查闭油门位置传感器的状态。
此函数用于在发动机控制系统中获取油门踏板位置，以便进行燃油喷射和点火时机的调整。

节气门位置传感器（TPS, Throttle Position Sensor）的值是通过检测节气门开度的变化产生的，这个过程涉及机械和电子组件的协同工作。以下是TPS值产生的一般过程：

机械连接：
TPS通常安装在节气门体上，并与节气门轴机械相连。当驾驶员踩下油门踏板时，它通过机械连杆或拉线直接或间接地转动节气门轴，从而改变进入发动机的空气量。
电位计原理：
TPS内部包含一个可变电阻（电位计），其电阻随着节气门轴的旋转而变化。这种变化被转换为电压信号输出。简单来说，当节气门关闭时，电位计提供一个较低的电压；当节气门全开时，它提供一个较高的电压。
电压信号：
传感器输出的电压信号通常是相对于5V参考电压的比例值。例如，在怠速状态下（节气门几乎关闭），传感器可能会输出0.5V左右的电压；而在节气门完全打开时，可能会上升到接近5V。
模数转换（ADC）：
车辆的ECU（电子控制单元）通过内置的模拟-数字转换器（ADC）将来自TPS的模拟电压信号转换为数字值。这一步骤对于现代汽车非常重要，因为ECU需要处理的是数字信息而不是模拟信号。
数据处理：
ECU接收到ADC转换后的数字信号后，会根据预设的校准参数对这些数值进行进一步处理。例如，它会将电压范围映射到代表节气门开度的百分比（如0%到100%），并考虑任何必要的修正或滤波以提高读数的准确性。
反馈与控制：
最终，ECU使用处理后的TPS值来调整燃油喷射、点火时刻等关键参数，确保发动机按照驾驶者的需求平稳且高效地运行。此外，TPS还用于其他系统，比如巡航控制、自动变速器换挡逻辑等。
异常检测：
ECU还会监测TPS信号是否存在异常情况，比如信号丢失或超出预期范围，以便及时采取措施保护发动机或者提示驾驶员检查车辆。

油门踏板 ->
**/
void readTPS(bool useFilter)
{
  currentStatus.TPSlast = currentStatus.TPS;
  #if defined(ANALOG_ISR)
    byte tempTPS = fastMap1023toX(AnChannel[pinTPS-A0], 255); //Get the current raw TPS ADC value and map it into a byte
  #else
    analogRead(pinTPS);
    byte tempTPS = fastMap1023toX(analogRead(pinTPS), 255); //Get the current raw TPS ADC value and map it into a byte
  #endif
  //The use of the filter can be overridden if required. This is used on startup to disable priming pulse if flood clear is wanted
  if(useFilter == true) { currentStatus.tpsADC = ADC_FILTER(tempTPS, configPage4.ADCFILTER_TPS, currentStatus.tpsADC); }
  else { currentStatus.tpsADC = tempTPS; }
  byte tempADC = currentStatus.tpsADC; //The tempADC value is used in order to allow TunerStudio to recover and redo the TPS calibration if this somehow gets corrupted

  if(configPage2.tpsMax > configPage2.tpsMin)
  {
    //Check that the ADC values fall within the min and max ranges (Should always be the case, but noise can cause these to fluctuate outside the defined range).
    if (currentStatus.tpsADC < configPage2.tpsMin) { tempADC = configPage2.tpsMin; }
    else if(currentStatus.tpsADC > configPage2.tpsMax) { tempADC = configPage2.tpsMax; }
    currentStatus.TPS = map(tempADC, configPage2.tpsMin, configPage2.tpsMax, 0, 200); //Take the raw TPS ADC value and convert it into a TPS% based on the calibrated values
  }
  else
  {
    //This case occurs when the TPS +5v and gnd are wired backwards, but the user wishes to retain this configuration.
    //In such a case, tpsMin will be greater then tpsMax and hence checks and mapping needs to be reversed

    tempADC = 255 - currentStatus.tpsADC; //Reverse the ADC values
    uint16_t tempTPSMax = 255 - configPage2.tpsMax;
    uint16_t tempTPSMin = 255 - configPage2.tpsMin;

    //All checks below are reversed from the standard case above
    if (tempADC > tempTPSMax) { tempADC = tempTPSMax; }
    else if(tempADC < tempTPSMin) { tempADC = tempTPSMin; }
    currentStatus.TPS = map(tempADC, tempTPSMin, tempTPSMax, 0, 200);
  }

  //Check whether the closed throttle position sensor is active
  if(configPage2.CTPSEnabled == true)
  {
    if(configPage2.CTPSPolarity == 0) { currentStatus.CTPSActive = !digitalRead(pinCTPS); } //Normal mode (ground switched)
    else { currentStatus.CTPSActive = digitalRead(pinCTPS); } //Inverted mode (5v activates closed throttle position sensor)
  }
  else { currentStatus.CTPSActive = 0; }
}

void readCLT(bool useFilter)
{
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = AnChannel[pinCLT-A0]; //Get the current raw CLT value
  #else
    tempReading = analogRead(pinCLT);
    tempReading = analogRead(pinCLT);
    //tempReading = fastMap1023toX(analogRead(pinCLT), 511); //Get the current raw CLT value
  #endif
  //The use of the filter can be overridden if required. This is used on startup so there can be an immediately accurate coolant value for priming
  if(useFilter == true) { currentStatus.cltADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_CLT, currentStatus.cltADC); }
  else { currentStatus.cltADC = tempReading; }
  
  currentStatus.coolant = table2D_getValue(&cltCalibrationTable, currentStatus.cltADC) - CALIBRATION_TEMPERATURE_OFFSET; //Temperature calibration values are stored as positive bytes. We subtract 40 from them to allow for negative temperatures
}

void readIAT(void)
{
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = AnChannel[pinIAT-A0]; //Get the current raw IAT value
  #else
    tempReading = analogRead(pinIAT);
    tempReading = analogRead(pinIAT);
  #endif
  currentStatus.iatADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_IAT, currentStatus.iatADC);
  currentStatus.IAT = table2D_getValue(&iatCalibrationTable, currentStatus.iatADC) - CALIBRATION_TEMPERATURE_OFFSET;
}

void readBaro(void)
{
  if ( configPage6.useExtBaro != 0 )
  {
    int tempReading;
    // readings
    #if defined(ANALOG_ISR_MAP)
      tempReading = AnChannel[pinBaro-A0];
    #else
      tempReading = analogRead(pinBaro);
      tempReading = analogRead(pinBaro);
    #endif

    if(currentStatus.initialisationComplete == true) { currentStatus.baroADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_BARO, currentStatus.baroADC); }//Very weak filter
    else { currentStatus.baroADC = tempReading; } //Baro reading (No filter)

    currentStatus.baro = fastMap10Bit(currentStatus.baroADC, configPage2.baroMin, configPage2.baroMax); //Get the current MAP value
  }
  else
  {
    /*
    * If no dedicated baro sensor is available, attempt to get a reading from the MAP sensor. This can only be done if the engine is not running. 
    * 1. Verify that the engine is not running
    * 2. Verify that the reading from the MAP sensor is within the possible physical limits
    */

    //Attempt to use the last known good baro reading from EEPROM as a starting point
    byte lastBaro = readLastBaro();
    if ((lastBaro >= BARO_MIN) && (lastBaro <= BARO_MAX)) //Make sure it's not invalid (Possible on first run etc)
    { currentStatus.baro = lastBaro; } //last baro correction
    else { currentStatus.baro = 100; } //Fall back position.

    //Verify the engine isn't running by confirming RPM is 0 and it has been at least 1 second since the last tooth was detected
    unsigned long timeToLastTooth = (micros() - toothLastToothTime);
    if((currentStatus.RPM == 0) && (timeToLastTooth > MICROS_PER_SEC))
    {
      instanteneousMAPReading(); //Get the current MAP value
      /* 
      * The highest sea-level pressure on Earth occurs in Siberia, where the Siberian High often attains a sea-level pressure above 105 kPa;
      * with record highs close to 108.5 kPa.
      * The lowest possible baro reading is based on an altitude of 3500m above sea level.
      */
      if ((currentStatus.MAP >= BARO_MIN) && (currentStatus.MAP <= BARO_MAX)) //Safety check to ensure the baro reading is within the physical limits
      {
        currentStatus.baro = currentStatus.MAP;
        storeLastBaro(currentStatus.baro);
      }
    }
  }
}

void readO2(void)
{
  //An O2 read is only performed if an O2 sensor type is selected. This is to prevent potentially dangerous use of the O2 readings prior to proper setup/calibration
  if(configPage6.egoType > 0)
  {
    unsigned int tempReading;
    #if defined(ANALOG_ISR)
      tempReading = AnChannel[pinO2-A0]; //Get the current O2 value.
    #else
      tempReading = analogRead(pinO2);
      tempReading = analogRead(pinO2);
      //tempReading = fastMap1023toX(analogRead(pinO2), 511); //Get the current O2 value.
    #endif
    currentStatus.O2ADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_O2, currentStatus.O2ADC);
    //currentStatus.O2 = o2CalibrationTable[currentStatus.O2ADC];
    currentStatus.O2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2ADC);
  }
  else
  {
    currentStatus.O2ADC = 0;
    currentStatus.O2 = 0;
  }
  
}

void readO2_2(void)
{
  //Second O2 currently disabled as its not being used
  //Get the current O2 value.
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = AnChannel[pinO2_2-A0]; //Get the current O2 value.
  #else
    tempReading = analogRead(pinO2_2);
    tempReading = analogRead(pinO2_2);
    //tempReading = fastMap1023toX(analogRead(pinO2_2), 511); //Get the current O2 value.
  #endif
  currentStatus.O2_2ADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_O2, currentStatus.O2_2ADC);
  currentStatus.O2_2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2_2ADC);
}

void readBat(void)
{
  int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[pinBat-A0], 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)
  #else
    tempReading = analogRead(pinBat);
    tempReading = fastMap1023toX(analogRead(pinBat), 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)
  #endif

  //Apply the offset calibration value to the reading
  tempReading += configPage4.batVoltCorrect;
  if(tempReading < 0){
    tempReading=0;
  }  //with negative overflow prevention


  //The following is a check for if the voltage has jumped up from under 5.5v to over 7v.
  //If this occurs, it's very likely that the system has gone from being powered by USB to being powered from the 12v power source.
  //Should that happen, we re-trigger the fuel pump priming and idle homing (If using a stepper)
  if( (currentStatus.battery10 < 55) && (tempReading > 70) && (currentStatus.RPM == 0) )
  {
    //Re-prime the fuel pump
    fpPrimeTime = currentStatus.secl;
    currentStatus.fpPrimed = false;
    FUEL_PUMP_ON();

    //Redo the stepper homing
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL) )
    {
      initialiseIdle(true);
    }
  }

  currentStatus.battery10 = ADC_FILTER(tempReading, configPage4.ADCFILTER_BAT, currentStatus.battery10);
}

/**
 * @brief Returns the VSS pulse gap for a given history point
 * 
 * @param historyIndex The gap number that is wanted. EG:
 * historyIndex = 0 = Latest entry
 * historyIndex = 1 = 2nd entry entry
 */
uint32_t vssGetPulseGap(byte historyIndex)
{
  uint32_t tempGap = 0;
  
  noInterrupts();
  int8_t tempIndex = vssIndex - historyIndex;
  if(tempIndex < 0) { tempIndex += VSS_SAMPLES; }

  if(tempIndex > 0) { tempGap = vssTimes[tempIndex] - vssTimes[tempIndex - 1]; }
  else { tempGap = vssTimes[0] - vssTimes[(VSS_SAMPLES-1)]; }
  interrupts();

  return tempGap;
}

uint16_t getSpeed(void)
{
  uint16_t tempSpeed = 0;
  // Get VSS from CAN, Serial or Analog by using Aux input channels.
  if(configPage2.vssMode == 1)
  {
    // Direct reading from Aux channel
    if (configPage2.vssPulsesPerKm == 0)
    {
      tempSpeed = currentStatus.canin[configPage2.vssAuxCh];
    }
    // Adjust the reading by dividing it by set amount.
    else
    {
      tempSpeed = (currentStatus.canin[configPage2.vssAuxCh] / configPage2.vssPulsesPerKm);
    }
    tempSpeed = ADC_FILTER(tempSpeed, configPage2.vssSmoothing, currentStatus.vss); //Apply speed smoothing factor
  }
  // Interrupt driven mode
  else if(configPage2.vssMode > 1)
  {
    uint32_t pulseTime = 0;
    uint32_t vssTotalTime = 0;

    //Add up the time between the teeth. Note that the total number of gaps is equal to the number of samples minus 1
    for(byte x = 0; x<(VSS_SAMPLES-1); x++)
    {
      vssTotalTime += vssGetPulseGap(x);
    }

    pulseTime = vssTotalTime / (VSS_SAMPLES - 1);
    if ( (micros() - vssTimes[vssIndex]) > MICROS_PER_SEC ) { tempSpeed = 0; } // Check that the car hasn't come to a stop. Is true if last pulse was more than 1 second ago
    else 
    {
      tempSpeed = MICROS_PER_HOUR / (pulseTime * configPage2.vssPulsesPerKm); //Convert the pulse gap into km/h
      tempSpeed = ADC_FILTER(tempSpeed, configPage2.vssSmoothing, currentStatus.vss); //Apply speed smoothing factor
    }
    if(tempSpeed > 1000) { tempSpeed = currentStatus.vss; } //Safety check. This usually occurs when there is a hardware issue

  }
  return tempSpeed;
}

byte getGear(void)
{
  byte tempGear = 0; //Unknown gear
  if(currentStatus.vss > 0)
  {
    //If the speed is non-zero, default to the last calculated gear
    tempGear = currentStatus.gear;

    uint16_t pulsesPer1000rpm = udiv_32_16(currentStatus.vss * 10000UL, currentStatus.RPM); //Gives the current pulses per 1000RPM, multiplied by 10 (10x is the multiplication factor for the ratios in TS)
    //Begin gear detection
    if( (pulsesPer1000rpm > (configPage2.vssRatio1 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio1 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 1; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio2 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio2 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 2; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio3 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio3 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 3; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio4 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio4 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 4; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio5 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio5 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 5; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio6 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio6 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 6; }
  }
  
  return tempGear;
}

byte getFuelPressure(void)
{
  int16_t tempFuelPressure = 0;
  uint16_t tempReading;

  if(configPage10.fuelPressureEnable > 0)
  {
    //Perform ADC read
    #if defined(ANALOG_ISR)
      tempReading = AnChannel[pinFuelPressure-A0];
    #else
      tempReading = analogRead(pinFuelPressure);
      tempReading = analogRead(pinFuelPressure);
    #endif

    tempFuelPressure = fastMap10Bit(tempReading, configPage10.fuelPressureMin, configPage10.fuelPressureMax);
    tempFuelPressure = ADC_FILTER(tempFuelPressure, ADCFILTER_PSI_DEFAULT, currentStatus.fuelPressure); //Apply smoothing factor
    //Sanity checks
    if(tempFuelPressure > configPage10.fuelPressureMax) { tempFuelPressure = configPage10.fuelPressureMax; }
    if(tempFuelPressure < 0 ) { tempFuelPressure = 0; } //prevent negative values, which will cause problems later when the values aren't signed.
  }

  return (byte)tempFuelPressure;
}

byte getOilPressure(void)
{
  int16_t tempOilPressure = 0;
  uint16_t tempReading;

  if(configPage10.oilPressureEnable > 0)
  {
    //Perform ADC read
    #if defined(ANALOG_ISR)
      tempReading = AnChannel[pinOilPressure-A0];
    #else
      tempReading = analogRead(pinOilPressure);
      tempReading = analogRead(pinOilPressure);
    #endif


    tempOilPressure = fastMap10Bit(tempReading, configPage10.oilPressureMin, configPage10.oilPressureMax);
    tempOilPressure = ADC_FILTER(tempOilPressure, ADCFILTER_PSI_DEFAULT, currentStatus.oilPressure); //Apply smoothing factor
    //Sanity check
    if(tempOilPressure > configPage10.oilPressureMax) { tempOilPressure = configPage10.oilPressureMax; }
    if(tempOilPressure < 0 ) { tempOilPressure = 0; } //prevent negative values, which will cause problems later when the values aren't signed.
  }


  return (byte)tempOilPressure;
}

/*
 * The interrupt function for reading the flex sensor frequency and pulse width
 * flexCounter value is incremented with every pulse and reset back to 0 once per second
 */
void flexPulse(void)
{
  if(READ_FLEX() == true)
  {
    unsigned long tempPW = (micros() - flexStartTime); //Calculate the pulse width
    flexPulseWidth = ADC_FILTER(tempPW, configPage4.FILTER_FLEX, flexPulseWidth);
    ++flexCounter;
  }
  else
  {
    flexStartTime = micros(); //Start pulse width measurement.
  }
}

/*
 * The interrupt function for pulses from a knock conditioner / controller
 * 
 */
void knockPulse(void)
{
  //Check if this the start of a knock. 
  if(knockCounter == 0)
  {
    //knockAngle = crankAngle + timeToAngleDegPerMicroSec( (micros() - lastCrankAngleCalc) ); 
    knockStartTime = micros();
    knockCounter = 1;
  }
  else { ++knockCounter; } //Knock has already started, so just increment the counter for this

}

/**
 * @brief The ISR function for VSS pulses
 * 
 */
void vssPulse(void)
{
  //TODO: Add basic filtering here
  vssIndex++;
  if(vssIndex == VSS_SAMPLES) { vssIndex = 0; }

  vssTimes[vssIndex] = micros();
}

uint16_t readAuxanalog(uint8_t analogPin)
{
  //read the Aux analog value for pin set by analogPin 
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = AnChannel[analogPin-A0]; //Get the current raw Auxanalog value
  #else
    tempReading = analogRead(analogPin);
    tempReading = analogRead(analogPin);
  #endif
  return tempReading;
} 

uint16_t readAuxdigital(uint8_t digitalPin)
{
  //read the Aux digital value for pin set by digitalPin 
  unsigned int tempReading;
  tempReading = digitalRead(digitalPin); 
  return tempReading;
} 
