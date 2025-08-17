/*
 * 
 * This sample code can be used with the Arduino Uno to control the AMT22 encoder.
 * It uses SPI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor.
 * For more information or assistance contact CUI Inc for support.
 * 
 * After uploading code to Arduino Uno open the open the Serial Monitor under the Tools 
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT22.
 * 
 * Arduino Pin Connections
 * SPI Chip Select Enc 0(motor):   Pin  2
 * SPI Chip Select Enc 1(pendulum):   Pin  4//3
 * SPI MOSI                 Pin 51
 * SPI MISO                 Pin 50
 * SPI SCLK:                Pin 52
 * 
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 */
//************************************************************************************************************************
/* Include the SPI library for the arduino boards */
#include <SPI.h>
/* Serial rates for UART */
#define BAUDRATE        115200
/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70
/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09
/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14
/* SPI pin connections to Arduino Board*/
#define ENC_0            2
#define ENC_1            3
#define SPI_MOSI        51      
#define SPI_MISO        50
#define SPI_SCLK        52
//*******************************************************************************************************************************
//motor output pins
#define motor_plus_pin 9
#define motor_minus_pin 8
//variables
double motor_angle = 0;
double pendulum_angle = 0;
double motor_angle_prev = 0;
double pendulum_angle_prev = 0;
double delta_motor_angle = 0;
double delta_pendulum_angle = 0;
double V_m = 0;
double k_mot = 0;
double k_pend = 0;
double k_dmot = 0;
double k_dpend = 0;
int motor_plus_val = 0;
int motor_minus_val = 0;
double time = 0;
double time_prev = 0;
double del_time = 0;
double pi_val = 3.141592654;
//***********************************************************************************************************************
void setup() 
{
  //Set the modes for the SPI IO8
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);//The only INPUT to the Arduino
  pinMode(ENC_0, OUTPUT);//wtf?
  pinMode(ENC_1, OUTPUT);//wtf?
  //Initialising Serial Monitor
  Serial.begin(BAUDRATE);
  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);//omg!
  digitalWrite(ENC_1, HIGH);

  //set the clockrate. Uno(Mega too) clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz
  
  //start SPI bus
  SPI.begin();
}

void loop() 
{
  //create a 16 bit variable for the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;
  //if you want to set the zero position before beggining uncomment the following function call
  setZeroSPI(ENC_0);
  setZeroSPI(ENC_1);
  Serial.println(" ");
  Serial.println("Started...");
  //*******************************************************************************************************************************************
  delay(5000);//wait 10 seconds in Arduino to let the USER erect the pendulum and rotate the motor to avoid motor encoder full rotation
//**********************************************************************************************************************************************
  //once we enter this loop we will run forever
  //initialising the state variables
  motor_angle = 180.0;
  pendulum_angle = 180.0;
  motor_angle_prev = 180.0;
  pendulum_angle_prev = 180.0;
  //********************************************************************************************************************************************
  while(1)
  {
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;
    //************************************************************************************************************************************************
    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(ENC_0, RES14); 
    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(ENC_0, RES14); //try again
    }

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      Serial.print("Motor Angle error. Attempts: ");
      Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      //Serial.write(TAB);
    }
    else //position was good, print to serial stream
    {
      
      //Serial.print("Motor Angle: ");
      //Serial.print(encoderPosition, DEC); //print the position in decimal format
      //Serial.write(TAB);
      motor_angle = encoderPosition*360.0/16384.0;
    }
    //////////again for second encoder//////////////////////////////
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;
    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(ENC_1, RES14); 
    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(ENC_1, RES14); //try again
    }

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      Serial.print("Pendulum Angle error. Attempts: ");
      Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      //Serial.write(NEWLINE);
    }
    else //position was good, print to serial stream
    {
      //Serial.print("Pendulum Angle: ");
      //Serial.print(encoderPosition, DEC); //print the position in decimal format
      //Serial.write(NEWLINE);
      pendulum_angle = encoderPosition*360.0/16384.0;
    }
    //angle sensing part ends here
    
    time = millis();
	  time = time/1000.000;//returns Current Run Time in Seconds
    del_time = time - time_prev;//time difference b/w steps in Seconds
    time_prev = time;//storing current time(in Seconds) for next step
    //************************************************************************************************************************************************
    //control part
    //already obtained
    //motor_angle
    //pendulum_angle
    //the derivatives of angles...
    delta_motor_angle = motor_angle - motor_angle_prev;
    delta_pendulum_angle = pendulum_angle - pendulum_angle_prev;
    k_mot = -1.9099;//-1.9099;//-1.9099;
    k_pend = 96.7428;//96.3806;//96.7428;
    k_dmot = -2.2963;//-2.2918;//-2.2963;
    k_dpend = 11.7282;//11.7488;//11.7282;
    //control variable
    //+ve V_m moves motor in increasing direction of Theta(increasing motor angle)
    //also convert degrees in SI units, Radians
    //if ((pendulum_angle<=183) && (pendulum_angle>=177))
    
      V_m = -(255/12)*(pi_val/180.0)*( k_mot*(motor_angle-180) + k_pend*(pendulum_angle-180) + k_dmot*(delta_motor_angle/del_time) + k_dpend*(delta_pendulum_angle/del_time) );
    
    //if(motor_angle>180){V_m = -255;}
    //if(motor_angle<180){V_m = 255;}
    //********************************************************************************************************************************************************
    //update the previous angle values...do this at the end of all computations
    motor_angle_prev = motor_angle;
    pendulum_angle_prev = pendulum_angle;
    //********************************************************************************************************************************************************
    //actuation part
    //clip V_m
    if(V_m>255){V_m = 255;}
    if(V_m<-255){V_m = -255;}
    motor_plus_val = floor(128 + (V_m/2.000));
    motor_minus_val = floor(128 - (V_m/2.000));
    analogWrite(motor_plus_pin, motor_plus_val);//floor(V_m/2.0));
    analogWrite(motor_minus_pin, motor_minus_val);//floor(V_m/2.0));
    //analogWrite(motor_plus_pin, 100);
    //analogWrite(motor_minus_pin, 0);
    //delay(100);
    //analogWrite(motor_plus_pin, 0);
    //analogWrite(motor_minus_pin, 100);
    //delay(100);
    //************************************************************************************************************************************************
    //Printing the State and Input
    Serial.print("Motor Angle: ");
    Serial.print(motor_angle);
    Serial.print("\t Pendulum Angle: ");
    Serial.print(pendulum_angle);
    Serial.print("\t Time: ");
    Serial.print(time);
    Serial.print("\t Previous Time Step: ");
    Serial.print(del_time);
    Serial.print("\t Mot_Vel: ");
    Serial.print(delta_motor_angle/del_time);
    Serial.print("\t Pend_Vel: ");
    Serial.print(delta_pendulum_angle/del_time);
    Serial.print("\t Vm: ");
    Serial.println(V_m);
    
  }
}
//*******************************************************************************************************************************************************
//function definitions
/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 */
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit. 
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required betw mveen bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}
