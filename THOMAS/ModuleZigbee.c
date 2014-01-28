/**
Déclarer un pointeur Buffer en global
*/
#include "zigbee.h"

/* Enum servant à la communication entre les deux modules Zigbee */
enum tramDataId {      
    PARAM_DRONE_CONFIG_ulMaxAngle,
    PARAM_DRONE_CONFIG_ulTakeoffAltitude,
    PARAM_DRONE_CONFIG_ulMinAltitude,
    PARAM_DRONE_CONFIG_ulCritBatteryLvl,
    PARAM_DRONE_CONFIG_ulCritObstacleDist,
    PARAM_DRONE_CONFIG_ulCritZigbeeSignalLvl,
    PARAM_DRONE_CONFIG_xBatteryMonitoringPeriod,
    PARAM_DRONE_CONFIG_xDetectObstaclePeriod,
    PARAM_DRONE_CONFIG_xZigbeeReceivePeriod,
    PARAM_DRONE_CONFIG_xFlightCtrlPeriod,
    PARAM_DRONE_CONFIG_xGPSReceivePeriod,
    PARAM_DRONE_CONFIG_xVideoTogglePeriod,
    PARAM_DRONE_CONFIG_xIMUDataTimeout,
    PARAM_DRONE_CONFIG_xZigbeeCmdTimeout,
    PARAM_DRONE_CONFIG_xZigbeeReceiveTimeout,
    PARAM_DRONE_CONFIG_xTelemeterTimeout,
    PARAM_DRONE_CONFIG_xBatteryTimeout,
    PARAM_DRONE_CONFIG_xGPSTimeout,
	PARAM_CMD_ID,
    PARAM_CMD_DRONE,
    PARAM_FLY_STATE,
    PARAM_DRONE_ERROR,
	PARAM_IMU_ROLL,
	PARAM_IMU_PITCH,
	PARAM_IMU_YAW,
	PARAM_IMU_HEIGHT,
    PARAM_IMU_xUpdateTime,
	PARAM_GPS_DATA,
    PARAM_GPS_xUpdateTime,
    PARAM_TELM_xUpdateTime,
    PARAM_BAT_ulPowerLvl,
    PARAM_BAT_xUpdateTime
};

inline void Zigbee_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	/* Link GPIOA pin3 pour USART2_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);	/* Link GPIOA pin2 pour USART2_Tx */

	/* Configure USART1_Tx */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1_Rx */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

inline uint8_t ucZigbeeInitTest( void ) //!
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", ( signed char * ) "-", 0,
			"zigbee", "ucZigbeeInitTest()", "Does nothing ATM" );

	return 0;
}

inline uint8_t ucZigbeeTest( void )//!
{
	ulDebugMsg( xTaskGetTickCount(), "TODO", pcTaskGetTaskName( NULL ),
			uxTaskPriorityGet( NULL ), "zigbee", "ucZigbeeTest()",
			"Does nothing ATM" );

	return 0;
}

inline void vZigbeeSendData( const uint8_t * const pucData ) //!
{
	
}

inline void Zigbee_Receiv_Usart2(char* pBuffer)
{
	int i=0;

	do {
		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
		pBuffer[i++] = USART_ReceiveData(USART2);
		USART_ClearFlag(USART2,USART_FLAG_RXNE);

	} while ((pBuffer[i-1] != '\0') || (pBuffer[i-1] != '\r'));
}

inline void Zigbee_Receive_Data( struct zigbeeData * const pxZigbeeData )
{
	int i=0;
	int indexParam;
	
	Receiv_Usart2(Buffer);
	indexParam = Buffer[i++];
	
	while(indexParam >= 0)
	{
		switch(Buffer[i++])
		{
			case PARAM_DRONE_CONFIG_ulMaxAngle:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulMaxAngle = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++;
				break;
			case PARAM_DRONE_CONFIG_ulTakeoffAltitude:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulTakeoffAltitude = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++;
				break;
			case PARAM_DRONE_CONFIG_ulMinAltitude:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulMinAltitude = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++;
				break;
			case PARAM_DRONE_CONFIG_ulCritBatteryLvl:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulCritBatteryLvl = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_ulCritObstacleDist:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulCritObstacleDist = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_ulCritZigbeeSignalLvl:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->ulCritZigbeeSignalLvl = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xBatteryMonitoringPeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xBatteryMonitoringPeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xDetectObstaclePeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xDetectObstaclePeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xZigbeeReceivePeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xZigbeeReceivePeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xFlightCtrlPeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xFlightCtrlPeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xGPSReceivePeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xGPSReceivePeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xVideoTogglePeriod:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xVideoTogglePeriod = (Buffer[i]+(256*(Buffer[i+1]+(256*(Buffer[i+2]+(Buffer[i+3]*256)))))); 
				i+=4;
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xIMUDataTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xIMUDataTimeout = Buffer[i++];
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xZigbeeCmdTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xZigbeeCmdTimeout = Buffer[i++];
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xZigbeeReceiveTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xZigbeeReceiveTimeout = Buffer[i++];
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xTelemeterTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xTelemeterTimeout = Buffer[i++]; 
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xBatteryTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xBatteryTimeout = Buffer[i++]; 
				indexParam++
				break;
			case PARAM_DRONE_CONFIG_xGPSTimeout:
				pxZigbeeData.eCmdId = CMD_SET_CONFIG;
				xCmdParam.xDroneConfig->xGPSTimeout = Buffer[i++]; 
				indexParam++
				break;
			case PARAM_CMD_ID:
				i++;
				pxZigbeeData.eCmdId = Buffer[i++];
				indexParam++;
				break;
			case PARAM_CMD_DRONE:
				pxZigbeeData.eCmdId = CMD_FLT_MANEUVER;
				xCmdParam.xFlightCommand->cTransX = Buffer[i++];
				xCmdParam.xFlightCommand->cTransY = Buffer[i++];
				xCmdParam.xFlightCommand->cTransZ = Buffer[i++];
				xCmdParam.xFlightCommand->cRotZ = Buffer[i++];
				indexParam++
				break;
		}
	}
}

inline void Zigbee_Send_Usart2(char Trame[])
{
	int i=0;

	while(Trame[i] != '\0')
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART2,Trame[i++]);
		USART_ClearFlag(USART2,USART_FLAG_TXE);
	}
}


inline void Zigbee_Send_Status(const struct droneState *const pxCurrState, const struct droneIMUWrapper *const pxCurrIMUWrapper, 
								const struct droneZigbeeWrapper *const pxCurrZigbeeWrapper, const struct droneGPSWrapper *const pxCurrGPSWrapper, 
								const struct droneTelemeterWrapper *const pxCurrTelemeterWrapper, const struct droneBatteryWrapper *const pxCurrBatteryWrapper)
{
	int i = 0 ;
	Buffer[i++] = Data_length; //Data_length a dimensionner
	
	//Drone Status
	Buffer[i++] = PARAM_FLY_STATE;
	Buffer[i++] = pxCurrState->eFltSate;
	Buffer[i++] = PARAM_DRONE_ERROR;
	Buffer[i++] = (int8_t) pxCurrState->eErrorMask;
	Buffer[i++] = (int8_t) (pxCurrState->eErrorMask >>8);
	//IMU Status
	Buffer[i++] = PARAM_IMU_ROLL;
	Buffer[i++] = (int8_t) pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_X];
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_X]>>8);
	Buffer[i++] = PARAM_IMU_PITCH;
	Buffer[i++] = (int8_t) pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_Y];
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_Y]>>8);
	Buffer[i++] = PARAM_IMU_YAW;
	Buffer[i++] = (int8_t) pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_Z];
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.plAngle[IMU_AXIS_Z]>>8);
	Buffer[i++] = PARAM_IMU_HEIGHT;
	Buffer[i++] = (int8_t) pxCurrIMUWrapper->xIMUData.lAltitude;
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.lAltitude>>8);
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.lAltitude>>16);
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xIMUData.lAltitude>>24);
	Buffer[i++] = PARAM_IMU_XUpdateTime;
	Buffer[i++] = (int8_t) pxCurrIMUWrapper->xUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xUpdateTime>>16);
	Buffer[i++] = (int8_t) (pxCurrIMUWrapper->xUpdateTime>>24);
	//Zigbee Status
	Buffer[i++] = PARAM_ZIGBEE_LEVEL
	Buffer[i++] = pxCurrZigbeeWrapper->ulSignalLvl;
	Buffer[i++] = PARAM_ZIGBEE_xCmdUpdateTime
	Buffer[i++] = (int8_t) pxCurrZigbeeWrapper->xCmdUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xCmdUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xCmdUpdateTime>>16);
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xCmdUpdateTime>>24);
	Buffer[i++] = PARAM_ZIGBEE_xZigBeeUpdateTime
	Buffer[i++] = (int8_t) pxCurrZigbeeWrapper->xZigbeeUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xZigbeeUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xZigbeeUpdateTime>>16);
	Buffer[i++] = (int8_t) (pxCurrZigbeeWrapper->xZigbeeUpdateTime>>24);
	//GPS Status	
	Buffer[i++] = PARAM_GPS_DATA;
	Buffer[i++] = 
	
	Buffer[i++] = PARAM_GPS_xUpdateTime;
	Buffer[i++] = (int8_t) pxCurrGPSWrapper->xUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrGPSWrapper->xUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrGPSWrapper->xUpdateTime>>16);
	Buffer[i++] = (int8_t) (pxCurrGPSWrapper->xUpdateTime>>24);
	//Telemeter Status
	Buffer[i++] = PARAM_TELM_BACK;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usBackDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usBackDist>>8);
	Buffer[i++] = PARAM_TELM_BOTTOM;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usBottomDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usBottomDist>>8);
	Buffer[i++] = PARAM_TELM_FRONT;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usFrontDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usFrontDist>>8);
	Buffer[i++] = PARAM_TELM_BACK;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usBackDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usBackDist>>8);
	Buffer[i++] = PARAM_TELM_LEFTBACK;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usLeftBackDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usLeftBackDist>>8);
	Buffer[i++] = PARAM_TELM_LEFTFRONT;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usLeftFrontDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usLeftFrontDist>>8);
	Buffer[i++] = PARAM_TELM_RIGHTBACK;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usRightBackDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usRightBackDist>>8);
	Buffer[i++] = PARAM_TELM_RIGHTFRONT;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->telemeterData.usRightFrontDist;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->telemeterData.usRightFrontDist>>8);
		Buffer[i++] = PARAM_TELM_xUpdateTime;
	Buffer[i++] = (int8_t) pxCurrTelemeterWrapper->xUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->xUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->xUpdateTime>>16);	
	Buffer[i++] = (int8_t) (pxCurrTelemeterWrapper->xUpdateTime>>24);
	//Battery Status
	Buffer[i++] = PARAM_BAT_ulPowerLvl;
	Buffer[i++] = (int8_t) pxCurrBatteryWrapper->ulPowerLvl;
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->ulPowerLvl>>8);
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->ulPowerLvl>>16);
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->ulPowerLvl>>24);
	Buffer[i++] = (int8_t) pxCurrBatteryWrapper->xUpdateTime;
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->xUpdateTime>>8);
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->xUpdateTime>>16);
	Buffer[i++] = (int8_t) (pxCurrBatteryWrapper->xUpdateTime>>24);

	Buffer[i] = NULL;
	Send_Usart2(Buffer);
}

static inline int Zigbee_Ok(char* pBuffer)
{
	if(pBuffer[0] == 'o' && pBuffer[1] == 'k')
		return 1;
	else
		return 0;
}

inline int Zigbee_Config(char* pBuffer)
{
	Send_Usart2("+++");			//Entre dans le mode de configuration
	Receiv_Usart2(pBuffer);
	if(!Zigbee_Ok(pBuffer))
		return 0;

	Send_Usart2("ATDH");			//Demande adresse haute de destinataire (voir avec guillaume)
	Receiv_Usart2(pBuffer);
	if(strcmp(pBuffer,"ABCD") != 0)	// Comparaison avec l'adresse haute destinataire en mémoire
		return 0;

	Send_Usart2("ATDL");			//Demande adresse basse de destinataire (voir avec guillaume)
	Receiv_Usart2(pBuffer);
	if(strcmp(pBuffer,"EFGH") != 0 )	// Comparaison avec l'adresse basse destinataire en mémoire
		return 0;

	Send_Usart2("ATCN");			//ATCN quitte le mode de configuration
	Receiv_Usart2(pBuffer);
	if(!Zigbee_Ok(pBuffer))
		return 0;
	else
		return 1;
}

inline int Zigbee_Reset(char* pBuffer)
{
	Send_Usart2("+++");			//Entre dans le mode de configuration
	Receiv_Usart2(pBuffer);
	if(!Zigbee_Ok(pBuffer))
		return 0;

	Send_Usart2("ATFR");			//Permet un Reset Soft du module Zigbee
	Receiv_Usart2(pBuffer);
	if(!Zigbee_Ok(pBuffer))
		return 0;
	else
		return 1;
}

inline int Zigbee_Level(char* pBuffer)
{
	Send_Usart2("+++");			//Entre dans le mode de configuration
	Receiv_Usart2(pBuffer);
	if(!Zigbee_Ok(pBuffer))
		return 0;
		
	Send_Usart2("AT");			//Entre dans le mode de configuration
	Receiv_Usart2(pBuffer);
	sSend_Usart2("ATCN");
	
	return &pBuffer;
}

