/**************** MKS SERVOxxD Close Loop Step Motor ***************
******************** arduino Example 4 ********************
**	Example Name：Read Input Pulses
**  Example Purpose：Read input pulses through the serial port
**  Example Phenomenon：After the program runs, it can be observed that
** 1. The motor runs at a constant speed, changing the speed every 10 seconds (realized by timer 1 interrupt)
** 2. The LED light flashes once per second, that is, the serial port sends a command to read the number of pulses
** 3. The serial monitor can observe that a position value is output per second pulses = xxxx
**  Precautions：
** 1. The serial port and the USB download port share the serial port (0,1). When uploading the program via USB, unplug the serial cable first to avoid program upload failure
** 2. Motor working mode is set to CR_vFOC
** 3. After the program is downloaded, open the serial monitor to observe the output results (Tools->Serial monitor, select the baud rate as 38400)
** 4. If the program upload fails, you can try: press and hold the RESET button of UNO, and then click upload, and when the arduino displays "uploading", quickly release the button
**********************************************************/

int EN_PIN = 10;     //Define the enable signal port
int STP_PIN = 9;    //Define the pulse signal port
int DIR_PIN = 8;    //Define the direction signal port

 uint8_t txBuffer[20];      //send data array
 uint8_t rxBuffer[20];      //Receive data array
uint8_t rxCnt=0;          //Receive data count

uint8_t getCheckSum(uint8_t *buffer,uint8_t len);
void readInputPulses(uint8_t slaveAddr);
bool waitingForACK(uint8_t len);
void printToMonitor(uint8_t *value);

#define  TIM_COUNTER1   62536
#define  TIM_COUNTER2   64036
int timer1_counter;
uint32_t loop_cnt = 0;
 

// Timer 1 overflow interrupt
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;
  digitalWrite(STP_PIN,digitalRead(STP_PIN)^1); //pulse signal flip
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);//Set the LED light port as output
  pinMode(EN_PIN, OUTPUT);    //Set the enable signal port to output mode
  pinMode(STP_PIN, OUTPUT);   //Set the pulse signal port to output mode
  pinMode(DIR_PIN, OUTPUT);   //Set the direction signal port to output mode

  // Start the serial port, set the rate to 38400
  Serial.begin(38400);
  //Wait for the serial port initialization to complete
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

// Power-on delay of 5000 milliseconds, waiting for the motor to be initialized
  delay(5000);
  digitalWrite(EN_PIN, LOW);  //enable motor
  digitalWrite(DIR_PIN, HIGH); //output direction signal

  //Initialize Timer 1 Interrupt
  noInterrupts();   //disable interrupt
  TCCR1A = 0;
  TCCR1B = 0;

/*If the interrupt frequency of xHz is obtained, timer1_counter = 65536-(16000000/xHz)
** timer1_counter   interrupt frequency    Rotation Speed
** 62536                        48RPM
** 64036                        96RPM
*/
  timer1_counter = TIM_COUNTER1;  //timer1 interrupt frequency is 5333Hz  
  TCNT1 = timer1_counter;   //Preloading timer1
  TCCR1B |= (1<<CS10);    //prescaler is 1
  TIMSK1 |= (1<<TOIE1);   //Enable timer overflow interrupt
  interrupts();           //Allow interrupts

 }

void loop() {

  bool ackStatus;
  
  digitalWrite(LED_BUILTIN, HIGH); //light up
  readInputPulses(1); //Slave address=1, issue query pulse number command

  ackStatus = waitingForACK(8);      //Wait for the motor to answer

  if(ackStatus == true)        //Received pulse count
  {
    printToMonitor(&rxBuffer[3]); // Pulse count output to serial monitor
    digitalWrite(LED_BUILTIN, LOW); //Lights off
    
  }
  else                      //Failed to receive pulse number information (1. Check the connection of the serial port cable; 2. Check whether the motor is powered on; 3. Check the slave address and baud rate)
  {
    while(1)                //Flashing light quickly, indicating that the operation failed
    {
      digitalWrite(EN_PIN, HIGH);  //release motor
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  loop_cnt++;
  if(loop_cnt == 10) //Change speed every 10 seconds
  {
    loop_cnt = 0;
    timer1_counter += 100;
    if(timer1_counter > TIM_COUNTER2)
    {
      timer1_counter = TIM_COUNTER1;
    }
  }  
  
  delay(1000);    //Delay 1000 milliseconds
}

/*
Function: read the number of input pulses
Input: slaveAddr slave address
output: none
 */
void readInputPulses(uint8_t slaveAddr)
{
 
  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0x33;       //function code
  txBuffer[3] = getCheckSum(txBuffer,3);  //Calculate checksum
  Serial.write(txBuffer,4);   //The serial port issues a command to read the input pulse number
}

/*
Function: Calculate the checksum of a set of data
Input: buffer data to be verified
        size The number of data to be verified
output: checksum
*/
uint8_t getCheckSum(uint8_t *buffer,uint8_t size)
{
  uint8_t i;
  uint16_t sum=0;
  for(i=0;i<size;i++)
    {
      sum += buffer[i];  //Calculate accumulated value
    }
  return(sum&0xFF);     //return checksum
}

/*
Function: wait for the slave to answer, set the timeout to 3000ms
Input: len Length of the response frame
output:
   run successfully true
   failed to run false
   timeout no response false
*/
bool waitingForACK(uint8_t len)
{
  bool retVal;       //return value
  unsigned long sTime;  //timing start time
  unsigned long time;  //current moment
  uint8_t rxByte;      

  sTime = millis();    //get the current moment
  rxCnt = 0;           //Receive count value set to 0
  while(1)
  {
    if (Serial.available() > 0)     //The serial port receives data
    {
      rxByte = Serial.read();       //read 1 byte data
      if(rxCnt != 0)
      {
        rxBuffer[rxCnt++] = rxByte; //Storing data
      }
      else if(rxByte == 0xFB)       //Determine whether the frame header
      {
        rxCnt = 0;
        rxBuffer[rxCnt++] = rxByte;   //store frame header
      }
    }

    if(rxCnt == len)    //Receive complete
    {
      if(rxBuffer[len-1] == getCheckSum(rxBuffer,len-1))
      {
        retVal = true;   //checksum correct
        break;                  //Exit while(1)
      }
      else
      {
        rxCnt = 0;  //Verification error, re-receive the response
      }
    }

    time = millis();
    if((time - sTime) > 3000)   //Judging whether to time out
    {
      retVal = false;
      break;                    //timeout, exit while(1)
    }
  }
  return(retVal);
}

/*
Function: output pulse number to serial monitor
       1. Tools -> Serial Monitor
       2. Baud rate selection 38400
       3. It can be observed that one pulse per second is output pulses = xxxx
Input: *value start address of pulse number
output: none
*/
void printToMonitor(uint8_t *value)
{
  int32_t iValue;
  String  tStr;
  iValue = (int32_t)(
                      ((uint32_t)value[0] << 24)    |
                      ((uint32_t)value[1] << 16)    |
                      ((uint32_t)value[2] << 8)     |
                      ((uint32_t)value[3] << 0)
                    );

  
  tStr = String(iValue);
  Serial.print("  pulses = ");
  Serial.println(tStr);
 }
