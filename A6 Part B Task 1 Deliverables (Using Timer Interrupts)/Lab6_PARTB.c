#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define StartOfExceptionVectorTable 0x0B000000
int channel_num;
unsigned char channel_data;
/**********************************************************************************************
**	Parallel port addresses
**********************************************************************************************/

#define PortA *(volatile unsigned char *)(0x00400000) // red leds 0-7
#define PortB *(volatile unsigned char *)(0x00400002) // red leds 8 and 9
#define PortC *(volatile unsigned char *)(0x00400004) // not connected
#define PortD *(volatile unsigned char *)(0x00400006) // not connected
#define PortE *(volatile unsigned char *)(0x00400008) // not connected

/*********************************************************************************************
**	Hex 7 seg displays port addresses
*********************************************************************************************/

#define HEX_A *(volatile unsigned char *)(0x00400010)
#define HEX_B *(volatile unsigned char *)(0x00400012)
#define HEX_C *(volatile unsigned char *)(0x00400014) // de2 only
#define HEX_D *(volatile unsigned char *)(0x00400016) // de2 only

/********************************************************************************************
**	Timer Port addresses
*********************************************************************************************/

#define Timer1Data *(volatile unsigned char *)(0x00400030)
#define Timer1Control *(volatile unsigned char *)(0x00400032)
#define Timer1Status *(volatile unsigned char *)(0x00400032)

#define Timer2Data *(volatile unsigned char *)(0x00400034)
#define Timer2Control *(volatile unsigned char *)(0x00400036)
#define Timer2Status *(volatile unsigned char *)(0x00400036)

#define Timer3Data *(volatile unsigned char *)(0x00400038)
#define Timer3Control *(volatile unsigned char *)(0x0040003A)
#define Timer3Status *(volatile unsigned char *)(0x0040003A)

#define Timer4Data *(volatile unsigned char *)(0x0040003C)
#define Timer4Control *(volatile unsigned char *)(0x0040003E)
#define Timer4Status *(volatile unsigned char *)(0x0040003E)

#define Timer5Data *(volatile unsigned char *)(0x00400130)
#define Timer5Control *(volatile unsigned char *)(0x00400132)
#define Timer5Status *(volatile unsigned char *)(0x00400132)

#define Timer6Data *(volatile unsigned char *)(0x00400134)
#define Timer6Control *(volatile unsigned char *)(0x00400136)
#define Timer6Status *(volatile unsigned char *)(0x00400136)

#define Timer7Data *(volatile unsigned char *)(0x00400138)
#define Timer7Control *(volatile unsigned char *)(0x0040013A)
#define Timer7Status *(volatile unsigned char *)(0x0040013A)

#define Timer8Data *(volatile unsigned char *)(0x0040013C)
#define Timer8Control *(volatile unsigned char *)(0x0040013E)
#define Timer8Status *(volatile unsigned char *)(0x0040013E)

/*********************************************************************************************
**	RS232 port addresses
*********************************************************************************************/

#define RS232_Control *(volatile unsigned char *)(0x00400040)
#define RS232_Status *(volatile unsigned char *)(0x00400040)
#define RS232_TxData *(volatile unsigned char *)(0x00400042)
#define RS232_RxData *(volatile unsigned char *)(0x00400042)
#define RS232_Baud *(volatile unsigned char *)(0x00400044)

/*********************************************************************************************
**	PIA 1 and 2 port addresses
*********************************************************************************************/

#define PIA1_PortA_Data *(volatile unsigned char *)(0x00400050) // combined data and data direction register share same address
#define PIA1_PortA_Control *(volatile unsigned char *)(0x00400052)
#define PIA1_PortB_Data *(volatile unsigned char *)(0x00400054) // combined data and data direction register share same address
#define PIA1_PortB_Control *(volatile unsigned char *)(0x00400056)

#define PIA2_PortA_Data *(volatile unsigned char *)(0x00400060) // combined data and data direction register share same address
#define PIA2_PortA_Control *(volatile unsigned char *)(0x00400062)
#define PIA2_PortB_data *(volatile unsigned char *)(0x00400064) // combined data and data direction register share same address
#define PIA2_PortB_Control *(volatile unsigned char *)(0x00400066)

/*********************************************************************************************************************************
(( DO NOT initialise global variables here, do it main even if you want 0
(( it's a limitation of the compiler
(( YOU HAVE BEEN WARNED
*********************************************************************************************************************************/
unsigned char data_potentiometer;
unsigned char data_thermistor;
unsigned char data_light;
unsigned int counter;
/*******************************************************************************************
** Function Prototypes
*******************************************************************************************/
int sprintf(char *out, const char *format, ...);

/*********************************************************************************************
**  Subroutine to initialise the RS232 Port by writing some commands to the internal registers
*********************************************************************************************/
void Init_RS232(void)
{
  RS232_Control = 0x15; //  %00010101 set up 6850 uses divide by 16 clock, set RTS low, 8 bits no parity, 1 stop bit, transmitter interrupt disabled
  RS232_Baud = 0x1;     // program baud rate generator 001 = 115k, 010 = 57.6k, 011 = 38.4k, 100 = 19.2, all others = 9600
}

/*********************************************************************************************************
**  Subroutine to provide a low level output function to 6850 ACIA
**  This routine provides the basic functionality to output a single character to the serial Port
**  to allow the board to communicate with HyperTerminal Program
**
**  NOTE you do not call this function directly, instead you call the normal putchar() function
**  which in turn calls _putch() below). Other functions like puts(), printf() call putchar() so will
**  call _putch() also
*********************************************************************************************************/

int _putch(int c)
{
  while ((RS232_Status & (char)(0x02)) != (char)(0x02)) // wait for Tx bit in status register or 6850 serial comms chip to be '1'
    ;

  RS232_TxData = (c & (char)(0x7f)); // write to the data register to output the character (mask off bit 8 to keep it 7 bit ASCII)
  return c;                          // putchar() expects the character to be returned
}

/*********************************************************************************************************
**  Subroutine to provide a low level input function to 6850 ACIA
**  This routine provides the basic functionality to input a single character from the serial Port
**  to allow the board to communicate with HyperTerminal Program Keyboard (your PC)
**
**  NOTE you do not call this function directly, instead you call the normal getchar() function
**  which in turn calls _getch() below). Other functions like gets(), scanf() call getchar() so will
**  call _getch() also
*********************************************************************************************************/
int _getch(void)
{
  char c;
  while ((RS232_Status & (char)(0x01)) != (char)(0x01)) // wait for Rx bit in 6850 serial comms chip status register to be '1'
    ;

  return (RS232_RxData & (char)(0x7f)); // read received character, mask off top bit and return as 7 bit ASCII character
}

/*********************************************************************************************************************************
**  IMPORTANT FUNCTION
**  This function install an exception handler so you can capture and deal with any 68000 exception in your program
**  You pass it the name of a function in your code that will get called in response to the exception (as the 1st parameter)
**  and in the 2nd parameter, you pass it the exception number that you want to take over (see 68000 exceptions for details)
**  Calling this function allows you to deal with Interrupts for example
***********************************************************************************************************************************/

void InstallExceptionHandler(void (*function_ptr)(), int level)
{
  volatile long int *RamVectorAddress = (volatile long int *)(StartOfExceptionVectorTable); // pointer to the Ram based interrupt vector table created in Cstart in debug monitor

  RamVectorAddress[level] = (long int *)(function_ptr); // install the address of our function into the exception table
}

/*************************************************************
** PCF 8591 chip (ADC/DAC converter)
**************************************************************/
/* The PCF 8591 chip actually has pins labelled A0, A1 and A2 that would in theory allows up to 8 of these devices
to exist in a system, but only A0 is brought out for configuration.
The A1 and A2 pins of the PCF 8591 chip have been wired to logic '0' on the PCB, so only
two of these modules could hang off the same IIC controller, one with A0=0, the other
with A0 = 1.
On my breadboard, the A0 pin of the PCF 8591 chip has been connected to logic 0 (GND). */
#define CONVERTER_ADDR (unsigned char)0x90 // table 5. The control byte for AD channel 1 is 8'b1001_0000 = 0x90
// commands
#define CONVERTER_WRITE (unsigned char)0x00 // write is active low
#define CONVERTER_READ (unsigned char)0x01  // read is active high

/*If the auto-increment mode is desired in applications where the internal oscillator is used,
the analog output enable flag must be set in the control byte (bit 6). This allows the
internal oscillator to run continuously, by this means preventing conversion errors
resulting from oscillator start-up delay. (data sheet)*/
// set bit 6 of the control byte to enable analog output
#define DAC (unsigned char)0x40 // 8'b0100_0000

// The following info is from handout Page 6

#define ADC_CHAN_1 (unsigned char)0x01
#define ADC_CHAN_2 (unsigned char)0x02
#define ADC_CHAN_3 (unsigned char)0x03
/*************************************************************
** IIC Controller registers
**************************************************************/
/*
Name	   Address  Width  Access			Description								Register Addr in 68k System
PRERlo		0x00	  8      RW       Clock Prescale register lo - byte					0x408000
PRERhi		0x01      8      RW       Clock Prescale register hi - byte					0x408002
CTR			0x02      8      RW       Control register									0x408004
TXR			0x03      8       W       Transmit register									0x408006
RXR			0x03      8       R       Receive register									0x408006
CR			0x04      8       W       Command register									0x408008
SR			0x04      8       R       Status register									0x408008
*/
#define IIC_PRERlo (*(volatile unsigned char *)(0x408000))
#define IIC_PRERhi (*(volatile unsigned char *)(0x408002))
#define IIC_CTR (*(volatile unsigned char *)(0x408004))
#define IIC_TXR (*(volatile unsigned char *)(0x408006))
#define IIC_RXR (*(volatile unsigned char *)(0x408006))
#define IIC_CR (*(volatile unsigned char *)(0x408008))
#define IIC_SR (*(volatile unsigned char *)(0x408008))

/*		Control Byte for the slave
The A1 and A2 pins of the EEPROM are connected to logic 0 (ground) on the breadboard.
When we want to read/write the first block of EEProm, the control byte is 8'b1010_0000 = 0xA0 with B0 = 0
When we want to read/write the second block of EEProm, the control byte is 8'b1010_1000 = 0xA8 with B0 = 1
*/
#define EEPROM_LO (unsigned char)0xA0 // I2C-bus slave address of the lower block of the EEPROM (addr space of the block: 00000-0FFFF)
#define EEPROM_HI (unsigned char)0xA8 // I2C-bus slave address of the upper block of the EEPROM (addr space of the block: 10000-1FFFF)

#define READ_EEPROM (unsigned char)0x01  // in the control byte, the R/W_L bit is the LSB!, 1 means read
#define WRITE_EEPROM (unsigned char)0x00 // in the control byte, the R/W_L bit is the LSB!, 0 means write

// useful IIC bit positions that can be used to synthesize commands
//
#define WR (unsigned char)0x10  // 8'b0001_0000, WR = CR[4] = 1
#define RD (unsigned char)0x20  // 8'b0010_0000, RD = CR[5] = 1
#define STO (unsigned char)0x40 // 8'b0100_0000, STO = CR[6] = 1
#define STA (unsigned char)0x80 // 8'b1000_0000, STA = CR[7] = 1

#define NACK (unsigned char)0x08 // 8'b0000_1000
#define IACK (unsigned char)0x01 // 8'b0000_0001, IACK = CR[0] = 1

/* Commands synthesized by using the bit position info. Please note that all reserved bits are read as zeros.
 * To ensure forward compatibility, they should be written as zeros.
 */
#define WRITE_START (WR | STA)
#define WRITE_STOP (WR | STO)
#define READ_END (STO | RD | NACK)

// useful IIC status register values
#define IF (unsigned char)0x01          // interrupt flag, 8'b0000_0001, IACK = SR[0] = 1
#define TIP (unsigned char)0x02         // TIP, Transfer in progress. 1 when transferring data, 0 when transfer complete
#define NO_ACK_BACK (unsigned char)0x80 // SR[7] = 1, No acknowledge received from the addressed slave.

// useful IIC control register values
#define IIC_ENABLE (unsigned char)0x80  // CTR[7:6] = 2'b10, this will enable the core as well as disable interrupt
#define IIC_DISABLE (unsigned char)0x00 // CTR[7:6] = 0, the core is disabled, the interrupt is disabled

#define NUM_CHAN_SAMPLES 2
unsigned int receive_flag;

/*************************************************************
** IIC Interface and EEPROM function definitions
**************************************************************/
// don't forget to wait for the ACK back from the slave AFTER each write.
void WaitForACK(void)
{
  while ((IIC_SR & NO_ACK_BACK) == NO_ACK_BACK)
    ;
}

// Check the status register TIP bit (bit 1) to see when transmit has finished
void WaitForCompletion(void)
{
  while ((IIC_SR & TIP) == TIP)
    ;
}

// Important: If you are using the version that doesn't have the cache (i.e., CPU clock freq = 25 MHz), then IIC_PRERlo and IIC_PRERhi need to be changed!
void IIC_Init(void)
{
  // Change the value of the prescale register only when the EN bit is cleared.
  IIC_CTR = IIC_DISABLE; // disable IIC controller

  /* set the clock frequency for 100Khz
	 * clock freq of the SPI controller = CPU_clock_freq = 25 MHz (with cache), desired_SCL = 100 KHz (from lab handout),
	 * prescale = CPU_clock_freq / (5 * desired_SCL) - 1 = 25 * 1000 / (5 * 100) - 1 = 49 = 0x31
	 */
  IIC_PRERlo = (unsigned char)0x31;
  IIC_PRERhi = (unsigned char)0x00;

  IIC_CTR = IIC_ENABLE;
}

void writeByte_IIC(unsigned char data, unsigned char command)
{
  /* send data */
  IIC_TXR = data;   // put the data to be transmitted into TX register
  IIC_CR = command; // write something to the command register that indicates that you want to write something

  // Check the status register TIP bit (bit 1) to see when transmit has finished
  WaitForCompletion();
  // don't forget to wait for the ACK back from the slave AFTER each write.
  WaitForACK();
}

/* This function checks for EEPROM internal write completion.
 * If the internal write is not complete, the master will wait until it is complete.
 */
void waitWriteComplete_EEPROM(unsigned char control_byte, unsigned char command)
{
  while (1)
  {
    /* send control_byte */
    IIC_TXR = control_byte;
    IIC_CR = command;

    WaitForCompletion();

    if ((IIC_SR & NO_ACK_BACK) != NO_ACK_BACK)
      break;
  }
}

/* The EEPROM chip is physically organised as 2 x 64Kbyte chips packaged inside the same physical device;
so a different IIC address is required to access the lower 64k vs. the upper 64k halves.
The block select bit B0 in the control byte should be set differently based on the value of the address.
If the address is in the range [0x0, 0x0FFFF], B0 is set to 0.
If the address is in the range [0x10000, 1FFFF], B0 is set to 1.
This function takes the address you want to access in the EEPROM chip, and returns
the corresponding control byte.
*/
unsigned char selectEEPRomBlock(unsigned int addr)
{
  unsigned char block_select_bit = (unsigned char)((addr >> 16) & 0xFF);
  if (block_select_bit == 0)
  {
    //printf("\r\nThe lower 64Kbyte block is selected.");
    return EEPROM_LO;
  }
  else
  {
    //printf("\r\nThe upper 64Kbyte block is selected.");
    return EEPROM_HI;
  }
}

void writeByte_EEPROM(unsigned char control_byte, unsigned short addr, unsigned data)
{
  unsigned char addr_hi = (unsigned char)((addr >> 8) & 0xFF);
  unsigned char addr_lo = (unsigned char)(addr & 0xFF);

  /* Send the control byte to the EEPROM*/
  writeByte_IIC(control_byte, WRITE_START);

  /* Send the address to write the data */
  writeByte_IIC(addr_hi, WR); // send upper byte addr
  writeByte_IIC(addr_lo, WR); // send lower byte addr

  /* Send data */
  writeByte_IIC(data, WRITE_STOP);

  //while(checkWriteComplete_EEPROM((control_byte), WRITE_START) == 1) {}
  waitWriteComplete_EEPROM(control_byte, WRITE_START);
}

/*
pass arguments: *data    - pointer to data that is received
				*command - action to the data
*/
void readByte_IIC(unsigned char *data, unsigned char command)
{
  // send command
  IIC_CR = command;

  WaitForCompletion();

  // retrieve data from the IIC core
  *data = IIC_RXR;
}

unsigned char read_ADC_Channel(int channel_num)
{
  unsigned char data = (unsigned char)0;
  unsigned int i = 0;
  unsigned int j = 0;

  writeByte_IIC((CONVERTER_ADDR | CONVERTER_WRITE), WRITE_START);

  if (channel_num == 1)
    writeByte_IIC(ADC_CHAN_1, WRITE_STOP);
  else if (channel_num == 2)
    writeByte_IIC(ADC_CHAN_2, WRITE_STOP);
  else if (channel_num == 3)
    writeByte_IIC(ADC_CHAN_3, WRITE_STOP);
  else
    return data;

  writeByte_IIC((CONVERTER_ADDR | CONVERTER_READ), WRITE_START);

  for (j = 0; j < NUM_CHAN_SAMPLES; j++)
  {
    if (j != NUM_CHAN_SAMPLES - 1)
      readByte_IIC(&data, (RD & ~NACK));
    else
    {
      readByte_IIC(&data, (STO | RD | (~(0xF7))));
      break;
    }
  }
  return data;
}
/*********************************************************************************************
** These addresses and definitions were taken from Appendix 7 of the Can Controller
** application note and adapted for the 68k assignment
*********************************************************************************************/

/*
** definition for the SJA1000 registers and bits based on 68k address map areas
** assume the addresses for the 2 can controllers given in the assignment
**
** Registers are defined in terms of the following Macro for each Can controller,
** where (i) represents an registers number
*/

#define CAN0_CONTROLLER(i) (*(volatile unsigned char *)(0x00500000 + (i << 1)))
#define CAN1_CONTROLLER(i) (*(volatile unsigned char *)(0x00500200 + (i << 1)))

/* Can 0 register definitions */
#define Can0_ModeControlReg CAN0_CONTROLLER(0)
#define Can0_CommandReg CAN0_CONTROLLER(1)
#define Can0_StatusReg CAN0_CONTROLLER(2)
#define Can0_InterruptReg CAN0_CONTROLLER(3)
#define Can0_InterruptEnReg CAN0_CONTROLLER(4) /* PeliCAN mode */
#define Can0_BusTiming0Reg CAN0_CONTROLLER(6)
#define Can0_BusTiming1Reg CAN0_CONTROLLER(7)
#define Can0_OutControlReg CAN0_CONTROLLER(8)

/* address definitions of Other Registers */
#define Can0_ArbLostCapReg CAN0_CONTROLLER(11)
#define Can0_ErrCodeCapReg CAN0_CONTROLLER(12)
#define Can0_ErrWarnLimitReg CAN0_CONTROLLER(13)
#define Can0_RxErrCountReg CAN0_CONTROLLER(14)
#define Can0_TxErrCountReg CAN0_CONTROLLER(15)
#define Can0_RxMsgCountReg CAN0_CONTROLLER(29)
#define Can0_RxBufStartAdr CAN0_CONTROLLER(30)
#define Can0_ClockDivideReg CAN0_CONTROLLER(31)

/* address definitions of Acceptance Code & Mask Registers - RESET MODE */
#define Can0_AcceptCode0Reg CAN0_CONTROLLER(16)
#define Can0_AcceptCode1Reg CAN0_CONTROLLER(17)
#define Can0_AcceptCode2Reg CAN0_CONTROLLER(18)
#define Can0_AcceptCode3Reg CAN0_CONTROLLER(19)
#define Can0_AcceptMask0Reg CAN0_CONTROLLER(20)
#define Can0_AcceptMask1Reg CAN0_CONTROLLER(21)
#define Can0_AcceptMask2Reg CAN0_CONTROLLER(22)
#define Can0_AcceptMask3Reg CAN0_CONTROLLER(23)

/* address definitions Rx Buffer - OPERATING MODE - Read only register*/
#define Can0_RxFrameInfo CAN0_CONTROLLER(16)
#define Can0_RxBuffer1 CAN0_CONTROLLER(17)
#define Can0_RxBuffer2 CAN0_CONTROLLER(18)
#define Can0_RxBuffer3 CAN0_CONTROLLER(19)
#define Can0_RxBuffer4 CAN0_CONTROLLER(20)
#define Can0_RxBuffer5 CAN0_CONTROLLER(21)
#define Can0_RxBuffer6 CAN0_CONTROLLER(22)
#define Can0_RxBuffer7 CAN0_CONTROLLER(23)
#define Can0_RxBuffer8 CAN0_CONTROLLER(24)
#define Can0_RxBuffer9 CAN0_CONTROLLER(25)
#define Can0_RxBuffer10 CAN0_CONTROLLER(26)
#define Can0_RxBuffer11 CAN0_CONTROLLER(27)
#define Can0_RxBuffer12 CAN0_CONTROLLER(28)

/* address definitions of the Tx-Buffer - OPERATING MODE - Write only register */
#define Can0_TxFrameInfo CAN0_CONTROLLER(16)
#define Can0_TxBuffer1 CAN0_CONTROLLER(17)
#define Can0_TxBuffer2 CAN0_CONTROLLER(18)
#define Can0_TxBuffer3 CAN0_CONTROLLER(19)
#define Can0_TxBuffer4 CAN0_CONTROLLER(20)
#define Can0_TxBuffer5 CAN0_CONTROLLER(21)
#define Can0_TxBuffer6 CAN0_CONTROLLER(22)
#define Can0_TxBuffer7 CAN0_CONTROLLER(23)
#define Can0_TxBuffer8 CAN0_CONTROLLER(24)
#define Can0_TxBuffer9 CAN0_CONTROLLER(25)
#define Can0_TxBuffer10 CAN0_CONTROLLER(26)
#define Can0_TxBuffer11 CAN0_CONTROLLER(27)
#define Can0_TxBuffer12 CAN0_CONTROLLER(28)

/* read only addresses */
#define Can0_TxFrameInfoRd CAN0_CONTROLLER(96)
#define Can0_TxBufferRd1 CAN0_CONTROLLER(97)
#define Can0_TxBufferRd2 CAN0_CONTROLLER(98)
#define Can0_TxBufferRd3 CAN0_CONTROLLER(99)
#define Can0_TxBufferRd4 CAN0_CONTROLLER(100)
#define Can0_TxBufferRd5 CAN0_CONTROLLER(101)
#define Can0_TxBufferRd6 CAN0_CONTROLLER(102)
#define Can0_TxBufferRd7 CAN0_CONTROLLER(103)
#define Can0_TxBufferRd8 CAN0_CONTROLLER(104)
#define Can0_TxBufferRd9 CAN0_CONTROLLER(105)
#define Can0_TxBufferRd10 CAN0_CONTROLLER(106)
#define Can0_TxBufferRd11 CAN0_CONTROLLER(107)
#define Can0_TxBufferRd12 CAN0_CONTROLLER(108)

/* CAN1 Controller register definitions */
#define Can1_ModeControlReg CAN1_CONTROLLER(0)
#define Can1_CommandReg CAN1_CONTROLLER(1)
#define Can1_StatusReg CAN1_CONTROLLER(2)
#define Can1_InterruptReg CAN1_CONTROLLER(3)
#define Can1_InterruptEnReg CAN1_CONTROLLER(4) /* PeliCAN mode */
#define Can1_BusTiming0Reg CAN1_CONTROLLER(6)
#define Can1_BusTiming1Reg CAN1_CONTROLLER(7)
#define Can1_OutControlReg CAN1_CONTROLLER(8)

/* address definitions of Other Registers */
#define Can1_ArbLostCapReg CAN1_CONTROLLER(11)
#define Can1_ErrCodeCapReg CAN1_CONTROLLER(12)
#define Can1_ErrWarnLimitReg CAN1_CONTROLLER(13)
#define Can1_RxErrCountReg CAN1_CONTROLLER(14)
#define Can1_TxErrCountReg CAN1_CONTROLLER(15)
#define Can1_RxMsgCountReg CAN1_CONTROLLER(29)
#define Can1_RxBufStartAdr CAN1_CONTROLLER(30)
#define Can1_ClockDivideReg CAN1_CONTROLLER(31)

/* address definitions of Acceptance Code & Mask Registers - RESET MODE */
#define Can1_AcceptCode0Reg CAN1_CONTROLLER(16)
#define Can1_AcceptCode1Reg CAN1_CONTROLLER(17)
#define Can1_AcceptCode2Reg CAN1_CONTROLLER(18)
#define Can1_AcceptCode3Reg CAN1_CONTROLLER(19)
#define Can1_AcceptMask0Reg CAN1_CONTROLLER(20)
#define Can1_AcceptMask1Reg CAN1_CONTROLLER(21)
#define Can1_AcceptMask2Reg CAN1_CONTROLLER(22)
#define Can1_AcceptMask3Reg CAN1_CONTROLLER(23)

/* address definitions Rx Buffer - OPERATING MODE - Read only register*/
#define Can1_RxFrameInfo CAN1_CONTROLLER(16)
#define Can1_RxBuffer1 CAN1_CONTROLLER(17)
#define Can1_RxBuffer2 CAN1_CONTROLLER(18)
#define Can1_RxBuffer3 CAN1_CONTROLLER(19)
#define Can1_RxBuffer4 CAN1_CONTROLLER(20)
#define Can1_RxBuffer5 CAN1_CONTROLLER(21)
#define Can1_RxBuffer6 CAN1_CONTROLLER(22)
#define Can1_RxBuffer7 CAN1_CONTROLLER(23)
#define Can1_RxBuffer8 CAN1_CONTROLLER(24)
#define Can1_RxBuffer9 CAN1_CONTROLLER(25)
#define Can1_RxBuffer10 CAN1_CONTROLLER(26)
#define Can1_RxBuffer11 CAN1_CONTROLLER(27)
#define Can1_RxBuffer12 CAN1_CONTROLLER(28)

/* address definitions of the Tx-Buffer - OPERATING MODE - Write only register */
#define Can1_TxFrameInfo CAN1_CONTROLLER(16)
#define Can1_TxBuffer1 CAN1_CONTROLLER(17)
#define Can1_TxBuffer2 CAN1_CONTROLLER(18)
#define Can1_TxBuffer3 CAN1_CONTROLLER(19)
#define Can1_TxBuffer4 CAN1_CONTROLLER(20)
#define Can1_TxBuffer5 CAN1_CONTROLLER(21)
#define Can1_TxBuffer6 CAN1_CONTROLLER(22)
#define Can1_TxBuffer7 CAN1_CONTROLLER(23)
#define Can1_TxBuffer8 CAN1_CONTROLLER(24)
#define Can1_TxBuffer9 CAN1_CONTROLLER(25)
#define Can1_TxBuffer10 CAN1_CONTROLLER(26)
#define Can1_TxBuffer11 CAN1_CONTROLLER(27)
#define Can1_TxBuffer12 CAN1_CONTROLLER(28)

/* read only addresses */
#define Can1_TxFrameInfoRd CAN1_CONTROLLER(96)
#define Can1_TxBufferRd1 CAN1_CONTROLLER(97)
#define Can1_TxBufferRd2 CAN1_CONTROLLER(98)
#define Can1_TxBufferRd3 CAN1_CONTROLLER(99)
#define Can1_TxBufferRd4 CAN1_CONTROLLER(100)
#define Can1_TxBufferRd5 CAN1_CONTROLLER(101)
#define Can1_TxBufferRd6 CAN1_CONTROLLER(102)
#define Can1_TxBufferRd7 CAN1_CONTROLLER(103)
#define Can1_TxBufferRd8 CAN1_CONTROLLER(104)
#define Can1_TxBufferRd9 CAN1_CONTROLLER(105)
#define Can1_TxBufferRd10 CAN1_CONTROLLER(106)
#define Can1_TxBufferRd11 CAN1_CONTROLLER(107)
#define Can1_TxBufferRd12 CAN1_CONTROLLER(108)

/* bit definitions for the Mode & Control Register */
#define RM_RR_Bit 0x01 /* reset mode (request) bit */
#define LOM_Bit 0x02   /* listen only mode bit */
#define STM_Bit 0x04   /* self test mode bit */
#define AFM_Bit 0x08   /* acceptance filter mode bit */
#define SM_Bit 0x10    /* enter sleep mode bit */

/* bit definitions for the Interrupt Enable & Control Register */
#define RIE_Bit 0x01  /* receive interrupt enable bit */
#define TIE_Bit 0x02  /* transmit interrupt enable bit */
#define EIE_Bit 0x04  /* error warning interrupt enable bit */
#define DOIE_Bit 0x08 /* data overrun interrupt enable bit */
#define WUIE_Bit 0x10 /* wake-up interrupt enable bit */
#define EPIE_Bit 0x20 /* error passive interrupt enable bit */
#define ALIE_Bit 0x40 /* arbitration lost interr. enable bit*/
#define BEIE_Bit 0x80 /* bus error interrupt enable bit */

/* bit definitions for the Command Register */
#define TR_Bit 0x01  /* transmission request bit */
#define AT_Bit 0x02  /* abort transmission bit */
#define RRB_Bit 0x04 /* release receive buffer bit */
#define CDO_Bit 0x08 /* clear data overrun bit */
#define SRR_Bit 0x10 /* self reception request bit */

/* bit definitions for the Status Register */
#define RBS_Bit 0x01 /* receive buffer status bit */
#define DOS_Bit 0x02 /* data overrun status bit */
#define TBS_Bit 0x04 /* transmit buffer status bit */
#define TCS_Bit 0x08 /* transmission complete status bit */
#define RS_Bit 0x10  /* receive status bit */
#define TS_Bit 0x20  /* transmit status bit */
#define ES_Bit 0x40  /* error status bit */
#define BS_Bit 0x80  /* bus status bit */

/* bit definitions for the Interrupt Register */
#define RI_Bit 0x01  /* receive interrupt bit */
#define TI_Bit 0x02  /* transmit interrupt bit */
#define EI_Bit 0x04  /* error warning interrupt bit */
#define DOI_Bit 0x08 /* data overrun interrupt bit */
#define WUI_Bit 0x10 /* wake-up interrupt bit */
#define EPI_Bit 0x20 /* error passive interrupt bit */
#define ALI_Bit 0x40 /* arbitration lost interrupt bit */
#define BEI_Bit 0x80 /* bus error interrupt bit */

/* bit definitions for the Bus Timing Registers */
#define SAM_Bit 0x80 /* sample mode bit 1 == the bus is sampled 3 times, 0 == the bus is sampled once */

/* bit definitions for the Output Control Register OCMODE1, OCMODE0 */
#define BiPhaseMode 0x00 /* bi-phase output mode */
#define NormalMode 0x02  /* normal output mode */
#define ClkOutMode 0x03  /* clock output mode */

/* output pin configuration for TX1 */
#define OCPOL1_Bit 0x20 /* output polarity control bit */
#define Tx1Float 0x00   /* configured as float */
#define Tx1PullDn 0x40  /* configured as pull-down */
#define Tx1PullUp 0x80  /* configured as pull-up */
#define Tx1PshPull 0xC0 /* configured as push/pull */

/* output pin configuration for TX0 */
#define OCPOL0_Bit 0x04 /* output polarity control bit */
#define Tx0Float 0x00   /* configured as float */
#define Tx0PullDn 0x08  /* configured as pull-down */
#define Tx0PullUp 0x10  /* configured as pull-up */
#define Tx0PshPull 0x18 /* configured as push/pull */

/* bit definitions for the Clock Divider Register */
#define DivBy1 0x07      /* CLKOUT = oscillator frequency */
#define DivBy2 0x00      /* CLKOUT = 1/2 oscillator frequency */
#define ClkOff_Bit 0x08  /* clock off bit, control of the CLK OUT pin */
#define RXINTEN_Bit 0x20 /* pin TX1 used for receive interrupt */
#define CBP_Bit 0x40     /* CAN comparator bypass control bit */
#define CANMode_Bit 0x80 /* CAN mode definition bit */

/*- definition of used constants ---------------------------------------*/
#define YES 1
#define NO 0
#define ENABLE 1
#define DISABLE 0
#define ENABLE_N 0
#define DISABLE_N 1
#define INTLEVELACT 0
#define INTEDGEACT 1
#define PRIORITY_LOW 0
#define PRIORITY_HIGH 1

/* default (reset) value for register content, clear register */
#define ClrByte 0x00

/* constant: clear Interrupt Enable Register */
#define ClrIntEnSJA ClrByte

/* definitions for the acceptance code and mask register */
#define DontCare 0xFF

/*  bus timing values for
**  bit-rate : 100 kBit/s
**  oscillator frequency : 25 MHz, 1 sample per bit, 0 tolerance %
**  maximum tolerated propagation delay : 4450 ns
**  minimum requested propagation delay : 500 ns
**
**  https://www.kvaser.com/support/calculators/bit-timing-calculator/
**  T1 	T2 	BTQ 	SP% 	SJW 	BIT RATE 	ERR% 	BTR0 	BTR1
**  17	8	25	    68	     1	      100	    0	      04	7f
*/

// initialisation for Can controller 0
void Init_CanBus_Controller0(void)
{
  while ((Can0_ModeControlReg & RM_RR_Bit) == ClrByte)
  {
    /* other bits than the reset mode/request bit are unchanged */
    Can0_ModeControlReg = Can0_ModeControlReg | RM_RR_Bit;
  }
  Can0_ClockDivideReg = CANMode_Bit | CBP_Bit | DivBy2;

  Can0_InterruptEnReg = ClrIntEnSJA;

  Can0_AcceptCode0Reg = ClrByte;
  Can0_AcceptCode1Reg = ClrByte;
  Can0_AcceptCode2Reg = ClrByte;
  Can0_AcceptCode3Reg = ClrByte;
  Can0_AcceptMask0Reg = DontCare;
  Can0_AcceptMask1Reg = DontCare;
  Can0_AcceptMask2Reg = DontCare;
  Can0_AcceptMask3Reg = DontCare;

  Can0_BusTiming0Reg = 0x04;
  Can0_BusTiming1Reg = 0x7f;

  Can0_OutControlReg = Tx1Float | Tx0PshPull | NormalMode;

  do
  {
    Can0_ModeControlReg = ClrByte;
  } while ((Can0_ModeControlReg & RM_RR_Bit) != ClrByte);
}

// initialisation for Can controller 1
void Init_CanBus_Controller1(void)
{
  // TODO - put your Canbus initialisation code for CanController 1 here
  // See section 4.2.1 in the application note for details (PELICAN MODE)
  while ((Can1_ModeControlReg & RM_RR_Bit) == ClrByte)
  {
    /* other bits than the reset mode/request bit are unchanged */
    Can1_ModeControlReg = Can1_ModeControlReg | RM_RR_Bit;
  }
  Can1_ClockDivideReg = CANMode_Bit | CBP_Bit | DivBy2;

  Can1_InterruptEnReg = ClrIntEnSJA;

  Can1_AcceptCode0Reg = ClrByte;
  Can1_AcceptCode1Reg = ClrByte;
  Can1_AcceptCode2Reg = ClrByte;
  Can1_AcceptCode3Reg = ClrByte;
  Can1_AcceptMask0Reg = DontCare;
  Can1_AcceptMask1Reg = DontCare;
  Can1_AcceptMask2Reg = DontCare;
  Can1_AcceptMask3Reg = DontCare;

  Can1_BusTiming0Reg = 0x04;
  Can1_BusTiming1Reg = 0x7f;

  Can1_OutControlReg = Tx1Float | Tx0PshPull | NormalMode;

  do
  {
    Can1_ModeControlReg = ClrByte;
  } while ((Can1_ModeControlReg & RM_RR_Bit) != ClrByte);
}

// Transmit for sending a message via Can controller 0
void CanBus0_Transmit(unsigned char data, unsigned char channel_num)
{
  // TODO - put your Canbus transmit code for CanController 0 here
  // See section 4.2.2 in the application note for details (PELICAN MODE)
  do
  {
  } while ((Can0_StatusReg & TBS_Bit) != TBS_Bit);

  Can0_TxFrameInfo = 0x08;
  Can0_TxBuffer1 = 0xA5;
  Can0_TxBuffer2 = 0x20;
  Can0_TxBuffer3 = data;
  Can0_TxBuffer4 = channel_num;
  Can0_TxBuffer5 = data;
  Can0_TxBuffer6 = data;

  Can0_CommandReg = TR_Bit;

  do
  {
  } while ((Can0_StatusReg & TCS_Bit) != TCS_Bit);
}

// Transmit for sending a message via Can controller 1
void CanBus1_Transmit(unsigned char data, unsigned char channel_num)
{
  // TODO - put your Canbus transmit code for CanController 1 here
  // See section 4.2.2 in the application note for details (PELICAN MODE)
  do
  {
  } while ((Can1_StatusReg & TBS_Bit) != TBS_Bit);

  Can1_TxFrameInfo = 0x08;
  Can1_TxBuffer1 = 0xA5; // ID 1
  Can1_TxBuffer2 = 0x20; // ID 2
  Can1_TxBuffer3 = data;
  Can1_TxBuffer4 = channel_num;
  Can1_TxBuffer5 = data;
  Can1_TxBuffer6 = data;

  Can1_CommandReg = TR_Bit;
  do
  {
  } while ((Can1_StatusReg & TCS_Bit) != TCS_Bit);
}

// Receive for reading a received message via Can controller 0
void CanBus0_Receive(int *channel_num, unsigned char *channel_data)
{
  // TODO - put your Canbus receive code for CanController 0 here
  // See section 4.2.4 in the application note for details (PELICAN MODE)
  unsigned char c[7];

  do
  {
  } while ((Can0_StatusReg & RBS_Bit) != RBS_Bit);

  c[2] = Can0_RxBuffer3 & 0xFF;
  c[3] = Can0_RxBuffer4 & 0xFF;
  c[4] = Can0_RxBuffer5 & 0xFF;
  c[5] = Can0_RxBuffer6 & 0xFF;
  c[6] = Can0_RxBuffer7 & 0xFF;

  Can0_CommandReg = Can0_CommandReg & RRB_Bit;

  *channel_num = c[3];
  *channel_data = c[2];
}

// Receive for reading a received message via Can controller 1
void CanBus1_Receive(int *channel_num, unsigned char *channel_data)
{
  // TODO - put your Canbus receive code for CanController 1 here
  // See section 4.2.4 in the application note for details (PELICAN MODE)
  unsigned char c[7];

  do
  {
  } while ((Can1_StatusReg & RBS_Bit) != RBS_Bit);

  c[2] = Can1_RxBuffer3 & 0xFF;
  c[3] = Can1_RxBuffer4 & 0xFF;
  c[4] = Can1_RxBuffer5 & 0xFF;
  c[5] = Can1_RxBuffer6 & 0xFF;
  c[6] = Can1_RxBuffer7 & 0xFF;

  Can1_CommandReg = Can1_CommandReg & RRB_Bit;

  *channel_num = c[3];
  *channel_data = c[2];
}

void Switches_ISR(void)
{
  if (Timer1Status == 1)
  {                    // Did Timer 1 produce the Interrupt?
    Timer1Control = 3; // reset the timer to clear the interrupt, enable interrupts and allow counter to run
    CanBus1_Transmit(PortA, 0x04);
    receive_flag = 1;
  }
}

void Potentiometer_ISR(void)
{
  if (Timer2Status == 1)
  {                    // Did Timer 2 produce the Interrupt?
    Timer2Control = 3; // reset the timer to clear the interrupt, enable interrupts and allow counter to run
    CanBus1_Transmit(read_ADC_Channel(1), 0x01);
    receive_flag = 1;
  }
}
void LightSensor_ISR(void)
{
  if (Timer3Status == 1)
  {                    // Did Timer 3 produce the Interrupt?
    Timer3Control = 3; // reset the timer to clear the interrupt, enable interrupts and allow counter to run
    CanBus1_Transmit(read_ADC_Channel(3), 0x03);
    receive_flag = 1;
  }
}
void Thermistor_ISR(void)
{
  if (Timer6Status == 1)
  {                    // Did Timer 6 produce the Interrupt?
    Timer6Control = 3; // reset the timer to clear the interrupt, enable interrupts and allow counter to run
    if (counter == 3)
    {
      counter = 0; // reset counter
      CanBus1_Transmit(read_ADC_Channel(2), 0x02);
      receive_flag = 1;
    }
    else
    {
      counter = counter + 1;
    }
  }
}
void Timer_ISR(void)
{
  {
    Init_CanBus_Controller0();
    Init_CanBus_Controller1();
    Thermistor_ISR();
    LightSensor_ISR();
    Potentiometer_ISR();
    Switches_ISR();
  }
}

/******************************************************************************************************************************
* Start of user program
******************************************************************************************************************************/

void main()
{
  receive_flag = 0;
  channel_num = 0;
  channel_data = 0;

  InstallExceptionHandler(Timer_ISR, 30);
  InstallExceptionHandler(Timer_ISR, 29);
  InstallExceptionHandler(Timer_ISR, 28);
  InstallExceptionHandler(Timer_ISR, 27);

  Timer1Data = 0x25; //  100 ms
  Timer2Data = 0x4B; // 200 ms
  Timer3Data = 0xBD; // 500 ms
  Timer6Data = 0xBD; // 500ms * 4  = 2 secs

  /* Now write binary 00000011 to timer control register:
  Bit0 = 1 (enable interrupt from that timer)
  Bit 1 = 1 enable counting*/
  Timer1Control = 3;
  Timer2Control = 3;
  Timer3Control = 3;
  Timer6Control = 3;

  counter = 0;
  Init_RS232(); // initialise the RS232 port for use with hyper terminal
  IIC_Init();
  Init_CanBus_Controller0();
  Init_CanBus_Controller1();

  while (1)
  {
    if (receive_flag == 1)
    {
      CanBus0_Receive(&channel_num, &channel_data);
      if (channel_num == 0x01)
        printf("\r\n  Potentiometer = 0x%x ", channel_data);
      if (channel_num == 0x02)
        printf("\r\n  Thermistor = 0x%x ", channel_data);
      if (channel_num == 0x03)
        printf("\r\n  Light sensor = 0x%x ", channel_data);
      if (channel_num == 0x04)
        printf("\r\n  Switches = 0x%x ", channel_data);
      receive_flag = 0;
    } // receive a message via Controller 0 (and display it)
  };

  // programs should NOT exit as there is nothing to Exit TO !!!!!!
  // There is no OS - just press the reset button to end program and call debug
}