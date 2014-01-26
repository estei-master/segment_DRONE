/** @file imu.c */

#include "imu.h"
#include "task.h"

/******************************************************************************
 **	 	 DEBUGGING MACROS
 *****************************************************************************/

/** Used to fill the pcModule argument of the ulDebugMsg() call. */
#define MODULE		"imu         "

/******************************************************************************
 **	 	 STATIC FUNCTIONS DECLARATION
 *****************************************************************************/

static inline enum IMUErrorMask prvAltitudeValid( const struct IMUData * const pxIMUData );
static inline enum IMUErrorMask prvAngleValid( const struct IMUData * const pxIMUData );
//static inline enum IMUErrorMask prvSpeedValid( const struct IMUData * const pxIMUData );
//static inline enum IMUErrorMask prvAccelValid( const struct IMUData * const pxIMUData );
static inline void usart_init( void );
static inline void imu_main( struct IMUData * const pxIMUData );

/******************************************************************************
 **	 	 PUBLIC FUNCTIONS DEFINITION
 *****************************************************************************/

/** Initializes the IMU */
inline void vIMUInit( void )
{
	ulDebugMsg( xTaskGetTickCount(), "INFO ", ( signed char * ) "---", 0, MODULE, "vIMUInit()",
			"Initializing IMU UART communication" );

	usart_init();
}

/** Tests correct Initialization of the IMU
@return 0 if test successful, 1 otherwise. */
inline uint8_t ucIMUInitTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "---", 0, MODULE, "ucIMUInitTest()",
			"Does nothing ATM" );

	/** @todo Implement ucIMUInitTest() during integration */
	return 0;
}

/** Tests correct behavior of the IMU */
inline uint8_t ucIMUTest( void )
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "ucIMUTest()", "Does nothing ATM" );

	return 0;
}

/** Gets IMU attitude measurements */
inline void vIMUGetData( struct IMUData * const pxIMUData )
{
	ulDebugMsg( xTaskGetTickCount(), "INFO ", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), MODULE, "vIMUGetData()", "Receiving IMU measurements" );

	/** @todo replace critical section by attempt counter in imu_main : if no
	critical section, the task gets preempted (sleeping/waking up constantly),
	otherwise it blocks the whole system if communication is lost.
	Sample of no critical section output :
	...
	1022  INFO  Flt     4  FreeRTOS  traceTASK_SWITCHED_IN()  Task running
    1023  INFO  Flt     4  FreeRTOS  traceTASK_SWITCHED_OUT()  Task switched out of running state
    1023  INFO  Flt     4  FreeRTOS  traceTASK_SWITCHED_IN()  Task running
    1024  INFO  Flt     4  FreeRTOS  traceTASK_SWITCHED_OUT()  Task switched out of running state
    ...
	*/
	taskENTER_CRITICAL();
//	taskDISABLE_INTERRUPTS();
	imu_main( pxIMUData );
//	taskENABLE_INTERRUPTS();
	taskEXIT_CRITICAL();

//	printf( "\r\n\t\tIMU measurements :"
//			"\r\n\t\t\tPitch = %d\r\n\t\t\tRoll = %d\r\n\t\t\tYaw = %d",
//			pxIMUData->plAngle[ IMU_AXIS_Y ],
//			pxIMUData->plAngle[ IMU_AXIS_X ],
//			pxIMUData->plAngle[ IMU_AXIS_Z ] );
}

/** Tests validity of pointed data */
inline enum IMUErrorMask eIMUDataValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	eErrorMask |= prvAltitudeValid( pxIMUData )
			| prvAngleValid( pxIMUData );
			/*
			| prvSpeedValid( pxIMUData )
			| prvAccelValid( pxIMUData );
			*/

	if( eErrorMask == IMU_ERR_NONE )
	{
		ulDebugMsg( xTaskGetTickCount(), "INFO ", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), MODULE, "eIMUDataValid()", "IMU measure is in valid range" );
	}
	else
	{
		ulDebugMsg( xTaskGetTickCount(), "ERROR", pcTaskGetTaskName( NULL ),
				uxTaskPriorityGet( NULL ), MODULE, "eIMUDataValid()", "IMU measure is not in valid range !" );
	}

	return eErrorMask;
}

/******************************************************************************
 **	 	 STATIC FUNCTIONS DEFINITION
 *****************************************************************************/

static inline enum IMUErrorMask prvAltitudeValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->lAltitude > imuMAX_VALID_ALTITUDE
			|| pxIMUData->lAltitude < imuMIN_VALID_ALTITUDE )
	{
		eErrorMask = IMU_ERR_ALTITUDE;
	}

	return eErrorMask;
}

static inline enum IMUErrorMask prvAngleValid( const struct IMUData * const pxIMUData )
{
enum IMUErrorMask eErrorMask = IMU_ERR_NONE;

	if( pxIMUData->plAngle[ IMU_AXIS_X ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_X ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_XANGLE;
	}

	if( pxIMUData->plAngle[ IMU_AXIS_Y ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_Y ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_YANGLE;
	}

	if( pxIMUData->plAngle[ IMU_AXIS_Z ] > imuMAX_VALID_ANGLE
			|| pxIMUData->plAngle[ IMU_AXIS_Z ] < imuMIN_VALID_ANGLE )
	{
		eErrorMask |= IMU_ERR_ZANGLE;
	}

	return eErrorMask;
}

//static inline enum IMUErrorMask prvSpeedValid( const struct IMUData * const pxIMUData )
//{
//enum IMUErrorMask eErrorMask = IMU_ERR_NONE;
//
//	if( pxIMUData->plSpeed[ IMU_AXIS_X ] > imuMAX_VALID_SPEED
//			|| pxIMUData->plSpeed[ IMU_AXIS_X ] < imuMIN_VALID_SPEED )
//	{
//		eErrorMask |= IMU_ERR_XSPEED;
//	}
//
//	if( pxIMUData->plSpeed[ IMU_AXIS_Y ] > imuMAX_VALID_SPEED
//			|| pxIMUData->plSpeed[ IMU_AXIS_Y ] < imuMIN_VALID_SPEED )
//	{
//		eErrorMask |= IMU_ERR_YSPEED;
//	}
//
//	if( pxIMUData->plSpeed[ IMU_AXIS_Z ] > imuMAX_VALID_SPEED
//			|| pxIMUData->plSpeed[ IMU_AXIS_Z ] < imuMIN_VALID_SPEED )
//	{
//		eErrorMask |= IMU_ERR_ZSPEED;
//	}
//
//	return eErrorMask;
//}

//static inline enum IMUErrorMask prvAccelValid( const struct IMUData * const pxIMUData )
//{
//enum IMUErrorMask eErrorMask = IMU_ERR_NONE;
//
//	if( pxIMUData->plAccel[ IMU_AXIS_X ] > imuMAX_VALID_ACCEL
//			|| pxIMUData->plAccel[ IMU_AXIS_X ] < imuMIN_VALID_ACCEL )
//	{
//		eErrorMask |= IMU_ERR_XACCEL;
//	}
//
//	if( pxIMUData->plAccel[ IMU_AXIS_Y ] > imuMAX_VALID_ACCEL
//			|| pxIMUData->plAccel[ IMU_AXIS_Y ] < imuMIN_VALID_ACCEL )
//	{
//		eErrorMask |= IMU_ERR_YACCEL;
//	}
//
//	if( pxIMUData->plAccel[ IMU_AXIS_Z ] > imuMAX_VALID_ACCEL
//			|| pxIMUData->plAccel[ IMU_AXIS_Z ] < imuMIN_VALID_ACCEL )
//	{
//		eErrorMask |= IMU_ERR_ZACCEL;
//	}
//
//	return eErrorMask;
//}

/******************************************************************************
 **		Code integrated from Sylvain JUILLAT
 *****************************************************************************/

/* Set to static to avoid collisions */
/** @todo reuse the same GPIO_InitTypeDef and USART_InitTypeDef to initialize
every USART (memory saving) */
/** Sets up the USART3 for IMU communication */
static inline void usart_init( void )
{
	/* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive enabled
	*/
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Connect PXx to USARTx_Rx*/

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART3, ENABLE);

}

/** Receives the IMU frame via USART3 and converts it into int16_t numbers */
static inline void imu_main( struct IMUData * const pxIMUData )
{
uint16_t pusBuff[ 20 ];
uint8_t ucIndex = 0;

	/** @todo Check we don't go past index 19 of pusBuff */
	/** @todo Add altitude */

	while( pusBuff[ ucIndex ] != 'S' )
	{
		while( USART_GetFlagStatus( USART3, USART_FLAG_RXNE ) == RESET )
		{
			/* Do nothing */
		}

		if( ucIndex > 0 && USART_ReceiveData( USART3 ) == 'R' )
		{
			ucIndex = 0;
			pusBuff[ ucIndex ] = USART_ReceiveData( USART3 );
		}

		pusBuff[ ucIndex ] = USART_ReceiveData( USART3 );
		ucIndex++;
	}

	/* Roll */
	pxIMUData->plAngle[ IMU_AXIS_X ] = 0;
	/* Pitch */
	pxIMUData->plAngle[ IMU_AXIS_Y ] = 0;
	/* Yaw */
	pxIMUData->plAngle[ IMU_AXIS_Z ] = 0;
	ucIndex = 0;

	if( pusBuff[ ucIndex ] == 'R' )
	{
		ucIndex++;
		if( pusBuff[ ucIndex ] == '-' )
		{
			ucIndex++;
			while( pusBuff[ ucIndex ] != 'P' )
			{
				pxIMUData->plAngle[ IMU_AXIS_X ] = pxIMUData->plAngle[ IMU_AXIS_X ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
			pxIMUData->plAngle[ IMU_AXIS_X ] = -pxIMUData->plAngle[ IMU_AXIS_X ];
		}
		else
		{
			while( pusBuff[ ucIndex ] != 'P' )
			{
				pxIMUData->plAngle[ IMU_AXIS_X ] =  pxIMUData->plAngle[ IMU_AXIS_X ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
		}
		ucIndex++;

		if( pusBuff[ ucIndex ] == '-' )
		{
			ucIndex++;
			while( pusBuff[ ucIndex ] != 'Y' )
			{
				pxIMUData->plAngle[ IMU_AXIS_Y ] = pxIMUData->plAngle[ IMU_AXIS_Y ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
			pxIMUData->plAngle[ IMU_AXIS_Y ] = -pxIMUData->plAngle[ IMU_AXIS_Y ];
		}
		else
		{
			while( pusBuff[ ucIndex ] != 'Y' )
			{
				pxIMUData->plAngle[ IMU_AXIS_Y ] = pxIMUData->plAngle[ IMU_AXIS_Y ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
		}
		ucIndex++;

		if( pusBuff[ ucIndex ] == '-' )
		{
			ucIndex++;
			while( pusBuff[ ucIndex ] != 'S' )
			{
				pxIMUData->plAngle[ IMU_AXIS_Z ] = pxIMUData->plAngle[ IMU_AXIS_Z ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
			pxIMUData->plAngle[ IMU_AXIS_Z ] = -pxIMUData->plAngle[ IMU_AXIS_Z ];
		}
		else
		{
			while( pusBuff[ ucIndex ] != 'S' )
			{
				pxIMUData->plAngle[ IMU_AXIS_Z ] = pxIMUData->plAngle[ IMU_AXIS_Z ] * 10 + ( ( int16_t ) pusBuff[ ucIndex ] ) - '0';
				ucIndex++;
			}
		}
	}
}
