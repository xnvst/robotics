//MoPa IMU is a free inertia mesurement unit which is based on ROBOTIS OPEN CM9.04 and GY-80 10DOF sensor
//main idea in this project is prediction and non-linear estimation for kalman filter extention
//every one can use this code for free... :P and send an mail to me for queastion
//my name is Mojtaba and i fall in love with her (Parinaz) during this project, and so this project name is MoPa :)
//how ever, you can use this code and make your robot to know where is it's orientation

#include <Wire.h>            // I2C communication lib
#include <MapleFreeRTOS.h>   // MapleFree Real-Time Operating System lib 
#include "EEPROM.h"          // eeprom lib for save some settings

#define Debug_Mode 1

//Global Deffenition of low and high byte of integer registers
#define _LOBYTE(w)   ((unsigned char)(((unsigned long)(w)) & 0xff))        //get low byte (8 first bits) from one 16 bit value
#define _HIBYTE(w)   ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff)) //get high byte (8'th to 16'th bits) from 16 bit value

//define register area include eeprom(64 byte) and ram(192 bytes) >> total is 254 bytes
#define _Eram_Size (30)  //eeprom memory area registers defenition (64 bytes)
#define _Ram_Size  (128) //ram area memory registers defenition (192 bytes)

//registers definition
//eeprom area (64 bytes)
//----------------------------------
//general defenition of eeprom registers (Reg 0 to 20)
#define Reg_Model_Number_L       (0)
#define Reg_Model_Number_H       (1)
#define Reg_Frimvare_Ver         (2)
#define Reg_MoPa_Id              (3)  //internal CM904 Id for communication (0 to 253)
#define Reg_Baud_Rate            (4)
#define Reg_Return_Delay_Time    (5)
#define Reg_Status_Return_Level  (16)

//filtering calibration registers (Reg xx to xx)
//complementry filter calibration value 
#define Reg_IMU_Kp_percent_X_L      (17) //?? make this for all of X,Y and Z
#define Reg_IMU_Kp_percent_X_H      (18)

//kalman filter calibration value
#define Reg_IMU_Kalman_Filter_Q_Angle_L            (19)
#define Reg_IMU_Kalman_Filter_Q_Angle_H            (20)

#define Reg_IMU_Kalman_Filter_Q_Bias_L             (21)
#define Reg_IMU_Kalman_Filter_Q_Bias_H             (22)

#define Reg_IMU_Kalman_Filter_R_Measure_L          (23)
#define Reg_IMU_Kalman_Filter_R_Measure_H          (24)        

//internal Real_Time tasks update frequency (Reg xx to xx)
#define Reg_SIMU_Task_Update_Frequency             (25)   //SIMU  task update frequency (1~500 Hz)

//calibration registers for sensor
//Ram area...
//--------------------------------------------
//********************************************
//ram area (192 bytes) >start from register 64 to
//gyroscope raw data value registers
#define Reg_IMU_Gyro_X_L         (30)
#define Reg_IMU_Gyro_X_H         (31)

#define Reg_IMU_Gyro_Y_L         (32)
#define Reg_IMU_Gyro_Y_H         (33)

#define Reg_IMU_Gyro_Z_L         (34)
#define Reg_IMU_Gyro_Z_H         (35)

//accelorometer raw data value registers
#define Reg_IMU_Accel_X_L        (36)
#define Reg_IMU_Accel_X_H        (37)

#define Reg_IMU_Accel_Y_L        (38)
#define Reg_IMU_Accel_Y_H        (39)

#define Reg_IMU_Accel_Z_L        (40)
#define Reg_IMU_Accel_Z_H        (41)

//compass raw data value registers
#define Reg_IMU_Comp_X_L         (42)
#define Reg_IMU_Comp_X_H         (43)

#define Reg_IMU_Comp_Y_L         (44)
#define Reg_IMU_Comp_Y_H         (45)

#define Reg_IMU_Comp_Z_L         (46)
#define Reg_IMU_Comp_Z_H         (47)
//-----------------------------------
//filtered value of orientation (read only)
//kalman filter estimation of X,Y,Z (read only)
#define Reg_IMU_KX_L             (48)
#define Reg_IMU_KX_H             (49)

#define Reg_IMU_KY_L             (50)
#define Reg_IMU_KY_H             (51)

#define Reg_IMU_KZ_L             (52)
#define Reg_IMU_KZ_H             (53)

// general update system value
//internal SIMU Real-Time task real value update frequency
#define Reg_SIMU_Task_Hz_L       (54)
#define Reg_SIMU_Task_Hz_H       (55)

//real-time clock value for system
//real-time second value (0 to 65535)
#define Reg_RTC_Sec_L            (56)  //RTC
#define Reg_RTC_Sec_H            (57)

//-------------------------------------
EEPROM _EEPROM;              //make eeprom object
Dynamixel Dxl(1);            // Dynamixel on Serial1(USART1)
HardwareTimer Timer(1);      // Hardware timer for RTC

unsigned short int  RTC_Sec=0;
signed short SIMUTaskCNT=0 , SIMUTaskHz=0;               //Sensor IMU task internal frequency val

byte _Ram[_Ram_Size];
byte T_Parameters[128];
byte PC_TX_Buffer[128];
byte T_State=0;
byte T_Len=0;
byte T_Checksum=0;
byte T_Id=0;
byte T_Packet_Finish=0;
byte T_Data_Len=0;
byte T_Parameter_Index=0;
byte T_Ins=0;
byte T_Plen=0; 


float GX_Integral=0.0f;
float GY_Integral=0.0f;
float GZ_Integral=0.0f;
  
float X=0.0f;
float Y=0.0f;
float Z=0.0f;
    

//initialize system
void setup()
{
  SerialUSB.begin();                // Config serialUSB port
  #ifdef Debug_Mode
        delay(2000);
	SerialUSB.println("\nAUT-Man IMU and DCM system start to initialize...");
  #endif
  
  SerialUSB.attachInterrupt(usbInterrupt);
  #ifdef Debug_Mode
        delay(500);
	SerialUSB.println("USB Serial initialize interupt...OK.");
  #endif
  
  RTC_Setup_Timer(1000000);         //initialize RTC for 1 mili secound
  #ifdef Debug_Mode
        delay(2000);
	SerialUSB.println("Real-Time Clock initialize Ok.");
  #endif  
  
  _EEPROM.begin();                  //initialize EEPROM
  #ifdef Debug_Mode
	SerialUSB.println("EEPROM Memory initialize Ok.");
  #endif
  
  Init_Factory_EEPROM();            //init factory defult value
  #ifdef Debug_Mode
        delay(1000);
	SerialUSB.println("System Memory for internal value initialize Ok.");
  #endif
  
  Wire.begin(1,0);                  // Initialize i2c comunication port (sda and scl)
  #ifdef Debug_Mode
        delay(1000);
	SerialUSB.println("I2C communication port initialize Ok.");
  #endif
  
  Serial1.begin(1000000);
  #ifdef Debug_Mode
	SerialUSB.println("Serial Port 1 initialize Ok.");
  #endif
  
  Dxl.begin((byte)_EEPROM.read(Reg_Baud_Rate));                     //initialize Dynamixel defult bus (1000000bps)
  #ifdef Debug_Mode
	SerialUSB.println("Dynamixel Communication on Port 1 initialize Ok.");
  #endif
  
  Serial1.attachInterrupt(Half_Duplex_serial_Interrupt);
  #ifdef Debug_Mode
	SerialUSB.println("Serial 1 interrupt initialize Ok.");
  #endif
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  
  #ifdef Debug_Mode
	SerialUSB.println("RTOS (Real Time Operating System) initializing tasks...");
  #endif
  //RTOS system config
  xTaskCreate( vSIMUTask       ,( signed char * ) "SIMU" ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init IMU task for read and run fusion alghorithm
  xTaskCreate( vINFOTask       ,( signed char * ) "INFO" ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init IMU task for read and run fusion alghorithm

  vTaskStartScheduler();      //Start schaduler of task, this function from RTOS lib and defult system if pipe line system
  
}

//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount){
  SerialUSB.print("nCount =");
  SerialUSB.println(nCount);
  for(unsigned int i=0; i < nCount;i++)  //printf_SerialUSB_Buffer[N]_receive_Data
    SerialUSB.print((char)buffer[i]);
  SerialUSB.println("");
  
  if (buffer[0]=='0')
  {
    X=0;
    Y=0;
    Z=0;
    
    //MARGFilter.Update_quaternion_init(X,Y,Z);
    SerialUSB.println("Set to zero All data Ok.");
  }
}


/*
* read data from ram area and eeprom area
*/
byte Read_Ram(byte index) //read data from ram or eeprom memory
{
    if (index<(_Eram_Size)) return (byte)_EEPROM.read(index);
    return _Ram[(byte)(index-(byte)_Eram_Size)];
}

/*
* write ram to ram and eeprom area
*/
void Write_Ram(byte index,byte data) //write data to ram or eeprom memory
{
    if (index<=_Eram_Size) _EEPROM.write(index,data);
    if ( (index>(_Eram_Size)) && (index<=(byte)_Ram_Size)) _Ram[(byte)(index-(byte)_Eram_Size)]=data;  
}

/*
* real time clock initialize
*/
void RTC_Setup_Timer(long Period)   //period in microsecound
{
  Timer.pause();                                  // Pause the timer while we're configuring it
  Timer.setPeriod(Period);                // Set up period in microseconds
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
  Timer.setCompare(TIMER_CH1, 1);                 // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, RTC_INT);      // atach interrupt function to timer
  Timer.refresh();                                // Refresh the timer's count, prescale, and overflow
  Timer.resume(); 
}

/*
* init factory setting of eeprom registers
*/
void Init_Factory_EEPROM()
{
  if((byte)_EEPROM.read(Reg_Model_Number_L)!=93) //for firs time after flash the micro
  {
    Write_Ram(Reg_Model_Number_L,94); //model number Open CM9.04
    Write_Ram(Reg_Model_Number_H,0);
    
    Write_Ram(Reg_Frimvare_Ver,11); // frimvare version 1.1
    Write_Ram(Reg_MoPa_Id,100);       // defult Id
    Write_Ram(Reg_Baud_Rate,1);
    Write_Ram(Reg_Return_Delay_Time,0);
  
    //Write_Ram(Reg_IMU_LowPass_Filter,10);   //low pass filter present
    //Write_Ram(Reg_IMU_Update_Frequency,4);  //IMU frequency value in 'ms' , 10 means 100Hz
    Write_Ram(Reg_Status_Return_Level,1);   //status return level  can be 0, 1 and 2 >> 0=no return , 1=read only, 2=return all
  }
}

/*
* serial port interrupt for ttl communication
*/
void Half_Duplex_serial_Interrupt(byte buffer)
{
        byte Data=buffer;
        T_Packet_Finish=0; 
        switch(T_State)
        {
            case 0: //0xff (start  of packet)  
                    if (Data==255) T_State=1;
                    break;
            case 1: //0xff (start packet ok!)         
                    T_State=0;
                    if (Data==255) T_State=2; 
                    break;
            case 2: //Id                
                    T_State=0;
                    if (Data==255) break;
                    T_Checksum=Data;
                    T_Id=0;
                    if((Data==(byte)_EEPROM.read(Reg_MoPa_Id)) || (Data==(byte)BROADCAST_ID)) {T_State=3; T_Id=Data;} //id or broadcast id 254
                    break;      
            case 3: //Lenght
                    T_Len=Data;
                    T_Data_Len=Data;    
                    T_State=4;
                    T_Checksum +=Data;
                    break;
            case 4://Inestruction
                    T_Ins = Data;
                    --T_Len;
                    T_Checksum += Data; 
                    T_Parameter_Index=0;            
                    if (T_Len <= 1) T_State = 6; else  T_State = 5;                      
                    break;
            case 5: //Parameters 
                    T_Parameters[T_Parameter_Index++]=Data;
                    T_Checksum += Data;
                    if (--T_Len <= 1) T_State = 6; // if all needed data has been read:                       
                    break;
            case 6: //Checksum
                    T_State=0;
                    T_Packet_Finish=1; //packet resive finish
                    //if (Data != (~T_Checksum)) {T_Ins=255;} //T_Packet_Finish=0;}// data is corrupted                   
                    break;     
        }
        
        if(T_Packet_Finish==1) //if packet resive ok and completly
        {
            switch(T_Ins)
            {                
                case 1: //Ping request
                    if (T_Id==(byte)BROADCAST_ID) break;
                    //digitalWrite(BOARD_LED_PIN, LOW);
                    PC_TX_Buffer[0]=255;
                    PC_TX_Buffer[1]=255;
                    PC_TX_Buffer[2]=(byte)_EEPROM.read(Reg_MoPa_Id); //Id  
                    PC_TX_Buffer[3]=2;      //len
                    PC_TX_Buffer[4]=0;      //Err
                    PC_TX_Buffer[5]=~(PC_TX_Buffer[2]+PC_TX_Buffer[3]);
                    for (T_Plen=0;T_Plen<=5;T_Plen++) Dxl.writeRaw(PC_TX_Buffer[T_Plen]);   //send data to the serial bus               
                    break;
                
                case 2: //Read command
                    if (T_Id==254) break; 
                    PC_TX_Buffer[0]=255;
                    PC_TX_Buffer[1]=255;
                    PC_TX_Buffer[2]=(byte)_EEPROM.read(Reg_MoPa_Id);   //Id  
                    PC_TX_Buffer[3]=T_Parameters[1]+2;      //len
                    PC_TX_Buffer[4]=0;                      //Err     
                    T_Parameter_Index =5;  
                    T_Checksum=PC_TX_Buffer[2]+PC_TX_Buffer[3]+PC_TX_Buffer[4];
                    for (T_Plen=T_Parameters[0];T_Plen<=((T_Parameters[0]+T_Parameters[1])-1);T_Plen++)
                    {
                        PC_TX_Buffer[T_Parameter_Index++]=Read_Ram(T_Plen); 
                        T_Checksum += Read_Ram(T_Plen);
                    }                    
                    PC_TX_Buffer[T_Parameter_Index]=~T_Checksum;   
                    for (T_Plen=0;T_Plen<=(PC_TX_Buffer[3]+3);T_Plen++) Dxl.writeRaw(PC_TX_Buffer[T_Plen]);  //send data to the serial bus                                    
                    break;
                    
                case 3: //Write command                  
                    T_Data_Len -=4;
                    T_Data_Len +=T_Parameters[0];
                    T_Parameter_Index = 1;
                    for(T_Plen=T_Parameters[0];T_Plen<=T_Data_Len;T_Plen++) Write_Ram(T_Plen,T_Parameters[T_Parameter_Index++]);   
                    if ((byte)_EEPROM.read(Reg_Status_Return_Level)==2)  //Status Return level is return to all
                    {
                        PC_TX_Buffer[0]=255;
                        PC_TX_Buffer[1]=255;
                        PC_TX_Buffer[2]=(byte)_EEPROM.read(Reg_MoPa_Id); //Id  
                        PC_TX_Buffer[3]=2;                     //len
                        PC_TX_Buffer[4]=0;                     //Err
                        PC_TX_Buffer[5]=~(PC_TX_Buffer[2]+PC_TX_Buffer[3]);
                        for (T_Plen=0;T_Plen<=5;T_Plen++) Dxl.writeRaw(PC_TX_Buffer[T_Plen]);  //send data packet to the serial bus
                    } 
                    break;  
            }//ins switch  
        }//packet resive complete and ok
}

/*
* Hardware interrupt for real time clock
*/
void RTC_INT(void) 
{  
    //real time clock in mili sec
    //RTC_Sec++;
    //_Ram[Reg_RTC_Sec_L-(byte)_Eram_Size]=_LOBYTE((int)RTC_Sec);
    //_Ram[Reg_RTC_Sec_H-(byte)_Eram_Size]=_HIBYTE((int)RTC_Sec);

    //calculate the imu internal task frequency
    SIMUTaskHz = SIMUTaskCNT;
    SIMUTaskCNT=0;

    //show data info (frequency hz of each action) to serial usb port
    #ifdef Debug_Mode
	SerialUSB.print("\n SIMU task initilize frequency of work @ "); SerialUSB.print(SIMUTaskHz);SerialUSB.println(" Hz");
    #endif
}

/*
* Write register to i2c device with specific i2c
*/
void i2c_writeRegister(byte deviceAddress, byte address, byte val) 
{
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);           // send value to write
    Wire.endTransmission();    // end transmission
}

/*
* Read a byte from i2c device with specific address
*/
byte i2c_readRegister(byte deviceAddress, byte address)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);                 // register to read
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 2);  // read a byte
    while(!Wire.available()) { } //wait until ready
    return (byte)Wire.read();
}

//main loop for micro, but it's not used. the RTOS task is running
void loop()
{
   //action in task
}




