#include <c8051f340.h>                 // SFR declarations
#include <stdio.h>
#include <communication.h>
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK      12000000           // SYSCLK frequency in Hz
#define BAUDRATE      230400           // Baud rate of UART in bps


//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init (void);
void UART0_Init (void);
void PORT_Init (void);
void UART1_Init (void);
//====test
void Putchar(unsigned char cData);
void UART1Putchar(unsigned char cData);
void AccessStatin(EStationCommand Command);
void Delay(float time);
void RxPackInfo();
void Command_ACK();
void Command_NAK();
void StationANA();
void RunStationCommand();
void Timer2_Init();
void StationCmd();
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

#define UART_BUFFERSIZE 10
unsigned char UART_Buffer[UART_BUFFERSIZE];
unsigned char UART_Buffer_Size = 0;
unsigned char UART_Input_First = 0;
unsigned char UART_Output_First = 0;
unsigned char TX_Ready =1;

static char Byte;
unsigned char STX = 2;
unsigned char ETX = 3;
unsigned char BCC = 0;
unsigned char _ACK = 13;
unsigned char _NAK = 15;
bit Reved;

xdata TPacker Packer ;
unsigned char RxTimeOutCount=0; 
byte *Pointer_Packer = &Packer.UART_Mark ; 
byte *Station_Pointer = &Packer.UART_Mark ; 
bit	IsRxTimeOut =0;	
bit IsRxCommand=0;
byte Command;
bit IsRxReady = 0;
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void Putchar(unsigned char cData)
{
	unsigned int i=10000;
	while( TX_Ready == 0) ;
	TX_Ready = 0;
	TI0 = 0;
	SBUF0 = cData;
	while(i--);	
}
//-----------------------------------------------------------------------------
void Timer2_ISR (void) interrupt INTERRUPT_TIMER2 
{
	TF2H=0;
	if (RxTimeOutCount ++ >=3)
	{
		TR2 =0;
		TMR2H = 0x00;
		TMR2L = 0x00;
		RxTimeOutCount=0;
		Station_Pointer = &Packer.Station.State;
		IsRxTimeOut =1;
	}
}
//-----------------------------------------------------------------------------
void Timer2_Init()
{
	TMR2CN = 0x00;
	TMR2L  = 0x00;
	TMR2H  = 0x00;
    ET2 = 1;
}
//-----------------------------------------------------------------------------

void UART1Putchar(unsigned char xData)
{
	unsigned int i=10000;
	while( (SCON1 & 0x20) ==0) ;
	SBUF1 = xData;
/*
	if(cData == Packer.ErrorCode){
		EIE2 |= 0x02;
		SCON1&=0x3F;
	}*/
	while(i--);	
}

//----------------------------------------------------------------------------------
void StationANA()
{
	char	i;
	int		CASNUMTemp;	
	for(i=0;i<5;i++)
	{
		Packer.StationInfo[i] = StationState_Finish;
	}
	for(i = 0;i<Packer.Station.CasseteQTY;i++)
	{
		CASNUMTemp = (Packer.CasseteInfo[i].HNumber & 0x0F) * 100 +
					 (Packer.CasseteInfo[i].TNumber & 0x0F) *  10 +
					 (Packer.CasseteInfo[i].SNumber & 0x0F);
		Packer.StationInfo[CASNUMTemp/20] = StationState_Busy;		
	}

}
unsigned char StaBCC = 0x00;
void AccessStatin(char i)
{							 
	UART1Putchar(UART_Mark) ;
	
	if(Command == BoardCommand_Start)
	{
		UART1Putchar(EStationCommand_PackStart);
		StaBCC = EStationCommand_PackStart;
	}
	else if(Command == BoardCommand_ReStart)
	{
		UART1Putchar(EStationCommand_RePack);
		StaBCC = EStationCommand_RePack;
	}
	else if(Command == BoardCommand_LEDON)
	{
		UART1Putchar(EStationCommand_LEDON);
		StaBCC = EStationCommand_LEDON;
	}
	else if(Command == BoardCommand_LEDOFF)
	{
		UART1Putchar(EStationCommand_LEDOFF);
		StaBCC = EStationCommand_LEDOFF;
	}
//	UART1Putchar(Packer.Station.CasseteQTY);

	for(i = 0;i<Packer.Station.CasseteQTY;i++)
	{
		UART1Putchar(Packer.CasseteInfo[i].HNumber);
		StaBCC ^= Packer.CasseteInfo[i].HNumber;
		UART1Putchar(Packer.CasseteInfo[i].TNumber);
		StaBCC ^= Packer.CasseteInfo[i].TNumber;
		UART1Putchar(Packer.CasseteInfo[i].SNumber);
		StaBCC ^= Packer.CasseteInfo[i].SNumber;
		UART1Putchar(Packer.CasseteInfo[i].QTYTNum);
		StaBCC ^= Packer.CasseteInfo[i].QTYTNum;
		UART1Putchar(Packer.CasseteInfo[i].QTYSNum);
		StaBCC ^= Packer.CasseteInfo[i].QTYSNum;
	}	
	UART1Putchar(ETX);
	StaBCC^= ETX;
	UART1Putchar(StaBCC);
	IsRxCommand =0;
	/*
	while(!IsRxCommand) 
	{
		if (i++ >= 12000)	
		{
			i=0;
			Pointer_Packer = &Packer.UART_Mark;
			Packer.UART_Mark = UART_NoMark ;
			//Cpu1CommandIdel();
			return;
		}
	}*/
	Pointer_Packer = &Packer.UART_Mark ;
	
}

void Command_ACK()
{
	Putchar(STX);
	Putchar(_ACK);
	Putchar('0');
	Putchar('0');
	Putchar('1');
	Putchar(ETX);	
}
void Command_NAK()
{
	Putchar(STX);
	Putchar(_NAK);
	Putchar(ETX);
}

void Command_ASK()
{
	Putchar(STX);
	Putchar('0');
	Putchar('1');
	Putchar('Q');
	Putchar(ETX);
}
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its maximum frequency

   CLKSEL = 0x00;                      // Select the internal osc. as
                                       // the SYSCLK source
   OSCICL = 0x10;
   CLKSEL |= 0x03;
   RSTSRC   |= 0x02;
}
void main (void)
{
   unsigned char i,Finish = 0,j=0;
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer
                                       // enable)
   PORT_Init();                        // Initialize Port I/O
   SYSCLK_Init ();                     // Initialize Oscillator
   UART0_Init();
   UART1_Init();
   Timer2_Init();
   EA = 1;
   for(i =0; i<80;i++)
   		Packer.RecevieData[i] = 0x30;
   Command = BoardCommand_Busy;
   while(1)
   {	
 //  		RunStationCommand();
		Delay(1000);
 		if(Command == BoardCommand_Start || Command == BoardCommand_ReStart)
		{
			StationANA();
			AccessStatin(0);
			Command = BoardCommand_Busy;
			Finish = 0;
			Packer.Board.State = State_Packing;
			Delay(5000);
			while(Packer.Board.State != State_Finish)
			{			
				for( i =0;i<Packer.Station.CasseteQTY;i++)
				{
					UART1Putchar(UART_Mark) ;
					UART1Putchar(EStationCommand_Return);
					StaBCC = EStationCommand_Return;
					UART1Putchar(Packer.CasseteInfo[i].HNumber);
					StaBCC ^= Packer.CasseteInfo[i].HNumber;
					UART1Putchar(Packer.CasseteInfo[i].TNumber);
					StaBCC ^= Packer.CasseteInfo[i].TNumber;
					UART1Putchar(Packer.CasseteInfo[i].SNumber);
					StaBCC ^= Packer.CasseteInfo[i].SNumber;
					UART1Putchar(ETX);
					StaBCC ^= ETX;
					UART1Putchar(StaBCC);
					Delay(3000);
					Packer.CasseteInfo[i].State = Packer.Station.State;
				}
				StationCmd();
				if(Packer.Board.State == State_Error)
					break;
//				Delay(100);
			}
			if(Command != BoardCommand_ACK && Packer.Board.State == State_Finish)
			{
				RunStationCommand();
				Delay(200);
			}
			if(Packer.Board.State == State_Error)
			{
				while(Command != BoardCommand_ACK)
				{
					Putchar(STX);
					Putchar('E');
					for(j=0;j<Packer.Station.CasseteQTY;j++)
					{
						if(Packer.CasseteInfo[j].State == TabletState_Fail)
						{
							Putchar(Packer.CasseteInfo[j].HNumber);
							Putchar(Packer.CasseteInfo[j].TNumber);
							Putchar(Packer.CasseteInfo[j].SNumber);
						}
					}
					Putchar(ETX);
				}
			}
		}	
		else if(Command == BoardCommand_LEDON)
		{
			StationANA();
			AccessStatin(0);
			Delay(5000);
			Packer.Board.State = State_Finish;
			Command = BoardCommand_Idel;
		} 
		else if(Command == BoardCommand_LEDOFF)
		{
			StationANA();
			AccessStatin(0);
			Delay(5000);
			Packer.Board.State = State_Finish;
			Command = BoardCommand_Idel;
		}
  }
}

//-----------------------------------------------------------------------------

void PORT_Init (void)
{
	P0MDOUT  |= 0x10;
	P1MDOUT   = 0x00;
	P1MDIN    = 0xFF;
//	P0SKIP    = 0xCF;
//  P1SKIP    = 0x03;
    XBR0      = 0x01;
    XBR1      = 0x40;
    XBR2      = 0x01;
}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the system clock to use the internal oscillator
// at its maximum frequency.
// Also enables the Missing Clock Detector.
//-----------------------------------------------------------------------------

void SYSCLK_Init (void)
{
   OSCICN |= 0x03;                     // Configure internal oscillator for
                                       // its maximum frequency
   RSTSRC  = 0x04;                     // Enable missing clock detector
}
void UART1_Init (void){
//	SBRLL1    = 0x3C;
//    SBRLH1    = 0xF6;
	SBRLL1    = 0x8F;
    SBRLH1    = 0xFD;
    SCON1     = 0x10;		
    SMOD1     = 0x0c;
    SBCON1    = 0x43;  
	EIE2 |= 0x02;//開啟UART1中斷
//	EIP2 = 0x02;//提高UART1中斷優先權
}

void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08;
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   TX_Ready = 1;                       // Flag showing that UART can transmit
   IP |= 0x10;                         // Make UART high priority
   ES0 = 1;                            // Enable UART0 interrupts

}


//-----------------------------------------------------------------------------
void RunStationCommand()
{
	Putchar(STX);
	Putchar('Z');
	Putchar('0');
	Putchar('0');
	Putchar('1');
	Putchar(ETX);
}

bit DataRv = 0;
unsigned char temp = 0;
void UART1_ISR(void)   interrupt 16 
{
	EIE2 &= ~0x02;
	if (SCON1 & 0x02)		// TI1
	{
		SCON1 &= (~0x02);
	}

	if (SCON1 & 0x01 )		// RI1
	{
		SCON1 &= (~0x01);
		
		*(Station_Pointer) = SBUF1;
		
		if(Station_Pointer == &Packer.UART_Mark)
		{
			if (Packer.UART_Mark == UART_Mark)
			{
				Packer.UART_Mark = UART_NoMark ;
				Station_Pointer = &Packer.Station.State;
			}
			else
			{	
				Station_Pointer = &Packer.UART_Mark;
			}
		}
		else if(Station_Pointer == &Packer.Station.State)
		{	
//			StationCmd();
			temp = Packer.Station.State;
			Station_Pointer = &Packer.UART_Mark;
		}	
		else 
		{
			Station_Pointer = &Packer.UART_Mark;
		}
	}
	EIE2 |= 0x02;
}
void StationCmd()
{
	unsigned char seq = 0;
	unsigned char Sucess=0x00,Fail = 0x00;;
	for(seq = 0;seq<Packer.Station.CasseteQTY;seq++)
	{
		if(Packer.CasseteInfo[seq].State == TabletState_Fail)
			Fail ++;
		else if(Packer.CasseteInfo[seq].State == TabletState_Sucess)
			Sucess ++;
	}
	if((Fail+Sucess) == Packer.Station.CasseteQTY)
	{
		if(Fail >0)
			Packer.Board.State = State_Error;
		else
			Packer.Board.State = State_Finish;
	}
	else
		Packer.Board.State = State_Packing;
}
//-----------------------------------------------------------------------------



void RunCommand()
{
	unsigned char i=0;
	Packer.Station.CasseteQTY = (Packer.RecevieData[1] & 0x0F) * 10 +
					 				(Packer.RecevieData[2] & 0x0F);	
	for(i=0;i<Packer.Station.CasseteQTY;i++)
	{
		Packer.CasseteInfo[i].HNumber = Packer.RecevieData[5*i+4];
		Packer.CasseteInfo[i].TNumber = Packer.RecevieData[5*i+5];
		Packer.CasseteInfo[i].SNumber = Packer.RecevieData[5*i+6];
		Packer.CasseteInfo[i].QTYTNum = Packer.RecevieData[5*i+7];
		Packer.CasseteInfo[i].QTYSNum = Packer.RecevieData[5*i+8];
		Packer.CasseteInfo[i].State = TabletState_Busy;	
	}
	if(Packer.RecevieData[0] == BoardCommand_RxCasseteInfo)
	{		
		Command = BoardCommand_Start;	
	}
	else if(Packer.RecevieData[0] == BoardCommand_ACK)
	{
		Command = BoardCommand_ACK;
	}
	else if(Packer.RecevieData[0] == BoardCommand_ReStart)
	{
		Command = BoardCommand_ReStart;
	}
	else if(Packer.RecevieData[0] == BoardCommand_LEDON)
	{
		Command = BoardCommand_LEDON;
	}
	else if(Packer.RecevieData[0] == BoardCommand_LEDOFF)
	{
		Command = BoardCommand_LEDOFF;
	}
}

//-----------------------------------------------------------------------------

byte BCCTemp,BCC;
//bit TAG = 0;
char CIndex = 0;
void UART0_Interrupt (void) interrupt 4  //接收PC資料
{
   if (TI0 == 1)                   // Check if transmit flag is set
   {
      TI0 = 0;                           // Clear interrupt flag
 	  TX_Ready = 1;

   }
   if (RI0 == 1)
   {
  	    RI0 = 0;                           // Clear interrupt flag
  	    *(Pointer_Packer) = SBUF0;                      // Read a character from UART
		BCCTemp ^= SBUF0;
		if(Pointer_Packer == &Packer.UART_Mark)
		{
			if (Packer.UART_Mark == UART_Mark)
			{
				Packer.UART_Mark = UART_NoMark ;
				Pointer_Packer = &Packer.RecevieData[CIndex];
			}
			else
			{	
				Pointer_Packer = &Packer.UART_Mark;
			}
		}
		else if(Pointer_Packer == &Packer.RecevieData[CIndex])
		{
			if(CIndex == 0)
				BCCTemp = SBUF0;
			if(Packer.RecevieData[CIndex] == ETX)
			{
				Pointer_Packer = &Packer.Station.BCC;
				BCC = BCCTemp;
				
			}
			else
			{
				CIndex++;
				Pointer_Packer = &Packer.RecevieData[CIndex];
				
			}
		}
		else if(Pointer_Packer == &Packer.Station.BCC)	
		{
			Pointer_Packer = &Packer.UART_Mark;
			
   			if(BCC ==Packer.Station.BCC )
				RunCommand();
			else
				Command_NAK();
			CIndex = 0;
		}
		else 
		{
			Pointer_Packer = &Packer.UART_Mark;
		}
   }


}

void Delay(float time)
{

	while(time --);

}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------