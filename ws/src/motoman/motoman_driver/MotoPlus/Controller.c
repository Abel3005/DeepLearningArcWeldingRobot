﻿// Controller.c
//
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Yaskawa America, Inc., nor the names 
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/ 

#include "MotoROS.h"

extern STATUS setsockopt
    (
    int    s,                 /* target socket */
    int    level,             /* protocol level of option */
    int    optname,           /* option name */
    char * optval,            /* pointer to option value */
    int    optlen             /* option length */
    );

//-----------------------
// Function Declarations
//-----------------------
BOOL Ros_Controller_Init(Controller* controller);
BOOL Ros_Controller_WaitInitReady(Controller* controller);
BOOL Ros_Controller_IsValidGroupNo(Controller* controller, int groupNo);
int Ros_Controller_OpenSocket(int tcpPort);
void Ros_Controller_ConnectionServer_Start(Controller* controller);
// Status related
void Ros_Controller_StatusInit(Controller* controller);
BOOL Ros_Controller_IsAlarm(Controller* controller);
BOOL Ros_Controller_IsError(Controller* controller);
BOOL Ros_Controller_IsPlay(Controller* controller);
BOOL Ros_Controller_IsTeach(Controller* controller);
BOOL Ros_Controller_IsRemote(Controller* controller);
BOOL Ros_Controller_IsOperating(Controller* controller);
BOOL Ros_Controller_IsHold(Controller* controller);
BOOL Ros_Controller_IsServoOn(Controller* controller);
BOOL Ros_Controller_IsEStop(Controller* controller);
BOOL Ros_Controller_IsWaitingRos(Controller* controller);
int Ros_Controller_GetNotReadySubcode(Controller* controller);
int Ros_Controller_StatusToMsg(Controller* controller, SimpleMsg* sendMsg);
BOOL Ros_Controller_StatusRead(Controller* controller, USHORT ioStatus[IO_ROBOTSTATUS_MAX]);
BOOL Ros_Controller_StatusUpdate(Controller* controller);
// Wrapper around MPFunctions
BOOL Ros_Controller_GetIOState(ULONG signal);
void Ros_Controller_SetIOState(ULONG signal, BOOL status);
int Ros_Controller_GetAlarmCode();
void Ros_Controller_ErrNo_ToString(int errNo, char errMsg[ERROR_MSG_MAX_SIZE], int errMsgSize);

//-----------------------
// Function implementation
//-----------------------

//Report version info to display on pendant (DX200 only)
void reportVersionInfoToController()
{
#if DX100 || FS100
	return;
#else
	MP_APPINFO_SEND_DATA appInfoSendData;
	MP_STD_RSP_DATA stdResponseData;

	sprintf(appInfoSendData.AppName, "MotoROS");

	sprintf(appInfoSendData.Version, "v%s", APPLICATION_VERSION);
	sprintf(appInfoSendData.Comment, "Motoman ROS-I driver");

	mpApplicationInfoNotify(&appInfoSendData, &stdResponseData); //don't care about return value
#endif
}

//-------------------------------------------------------------------
// Initialize the controller structure
// This should be done before the controller is used for anything
//------------------------------------------------------------------- 
BOOL Ros_Controller_Init(Controller* controller)
{
	int grpNo;
	int i;
	BOOL bInitOk;
	STATUS status;
	
	printf("Initializing controller\r\n");

	reportVersionInfoToController();

	// Turn off all I/O signal
	Ros_Controller_SetIOState(IO_FEEDBACK_WAITING_MP_INCMOVE, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_CONNECTSERVERRUNNING, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_STATESERVERCONNECTED, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_IOSERVERCONNECTED, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, FALSE);
	
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_1, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_2, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_3, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_4, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_5, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_6, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_7, FALSE);
	Ros_Controller_SetIOState(IO_FEEDBACK_RESERVED_8, FALSE);
	
	// Init variables and controller status
	bInitOk = TRUE;
	controller->bRobotJobReady = FALSE;
	controller->bStopMotion = FALSE;
	status = GP_isPflEnabled(&controller->bPFLEnabled);
	if (status != OK)
		bInitOk = FALSE;
	if (controller->bPFLEnabled)
		printf("System has PFL Enabled\r\n");
	controller->bPFLduringRosMove = FALSE;
	controller->bMpIncMoveError = FALSE;
	Ros_Controller_StatusInit(controller);
	Ros_Controller_StatusRead(controller, controller->ioStatus);
	
	// wait for controller to be ready for reading parameter
	Ros_Controller_WaitInitReady(controller);

#if (DX100)
	// Determine if the robot is a DX100 SDA which requires a special case for the motion API
	status = GP_isSdaRobot(&controller->bIsDx100Sda);
	if (status != OK)
		bInitOk = FALSE;
#endif

	// Get the interpolation clock
	status = GP_getInterpolationPeriod(&controller->interpolPeriod);
	if(status!=OK)
		bInitOk = FALSE;
	
	// Get the number of groups
	controller->numGroup = GP_getNumberOfGroups();
#ifdef DEBUG
	printf("controller->numGroup = %d\n", controller->numGroup);
#endif
	if(controller->numGroup < 1)
		bInitOk = FALSE;

	if (controller->numGroup > MAX_CONTROLLABLE_GROUPS)
	{
		printf("!!!---Detected %d control groups.  MotoROS will only control %d.---!!!\n", controller->numGroup, MAX_CONTROLLABLE_GROUPS);
		controller->numGroup = MAX_CONTROLLABLE_GROUPS;
	}
	
	controller->numRobot = 0;
	
	// Check for each group
	for(grpNo=0; grpNo < MP_GRP_NUM; grpNo++)
	{
		if(grpNo < controller->numGroup)
		{
			// Determine if specific group exists and allocate memory for it
			controller->ctrlGroups[grpNo] = Ros_CtrlGroup_Create(grpNo,								//Zero based index of the group number(0 - 3)
																(grpNo==(controller->numGroup-1)),	//TRUE if this is the final group that is being initialized. FALSE if you plan to call this function again.
																controller->interpolPeriod);		//Value of the interpolation period (ms) for the robot controller.
			if(controller->ctrlGroups[grpNo] != NULL)
			{
				Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos); // set the current commanded pulse
				controller->numRobot++;  //This counter is required for DX100 controllers with two control-groups (robot OR ext axis)
			}
			else
				bInitOk = FALSE;
		}
		else
			controller->ctrlGroups[grpNo] = NULL;
	}

#ifdef DEBUG
	printf("controller->numRobot = %d\n", controller->numRobot);
#endif
	
	// Initialize Thread ID and Socket to invalid value
	controller->tidConnectionSrv = INVALID_TASK;

	for (i = 0; i < MAX_IO_CONNECTIONS; i++)
	{
		controller->sdIoConnections[i] = INVALID_SOCKET;
		controller->tidIoConnections[i] = INVALID_TASK;
	}

	for (i = 0; i < MAX_STATE_CONNECTIONS; i++)
	{
		controller->sdStateConnections[i] = INVALID_SOCKET;
		controller->tidStateSendState[i] = INVALID_TASK;
	}

	for (i = 0; i < MAX_MOTION_CONNECTIONS; i++)
	{
		controller->sdMotionConnections[i] = INVALID_SOCKET;
		controller->tidMotionConnections[i] = INVALID_TASK;
	}
	controller->tidIncMoveThread = INVALID_TASK;

#ifdef DX100
	controller->bSkillMotionReady[0] = FALSE;
	controller->bSkillMotionReady[1] = FALSE;
#endif

	if(bInitOk)
	{
		// Turn on initialization done I/O signal
		Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, TRUE);
	}
	else
	{
		Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
		printf("Failure to initialize controller\r\n");
	}
	
	return bInitOk;
}


//-------------------------------------------------------------------
// Wait for the controller to be ready to start initialization
//------------------------------------------------------------------- 
BOOL Ros_Controller_WaitInitReady(Controller* controller)
{
	do  //minor alarms can be delayed briefly after bootup
	{
		puts("Waiting for robot alarms to clear...");
		Ros_Sleep(2500);
		Ros_Controller_StatusRead(controller, controller->ioStatus);

	}while(Ros_Controller_IsAlarm(controller));

	return TRUE;
}


//-------------------------------------------------------------------
// Check the number of inc_move currently in the specified queue
//-------------------------------------------------------------------
BOOL Ros_Controller_IsValidGroupNo(Controller* controller, int groupNo)
{
	if((groupNo >= 0) && (groupNo < controller->numGroup))
		return TRUE;
	else
	{
		printf("ERROR: Attempt to access invalid Group No. (%d) \r\n", groupNo);
		return FALSE;
	}
}


//-------------------------------------------------------------------
// Open a socket to listen for incomming connection on specified port
// return: <0  : Error
// 		   >=0 : socket descriptor
//-------------------------------------------------------------------
int Ros_Controller_OpenSocket(int tcpPort)
{
	int sd;  // socket descriptor
	struct sockaddr_in	serverSockAddr;
	int ret;

	// Open the socket
	sd = mpSocket(AF_INET, SOCK_STREAM, 0);
	if (sd < 0)
		return -1;

	// Set structure
	memset(&serverSockAddr, 0, sizeof(struct sockaddr_in));
	serverSockAddr.sin_family = AF_INET;
	serverSockAddr.sin_addr.s_addr = INADDR_ANY;
	serverSockAddr.sin_port = mpHtons(tcpPort);

	//bind to network interface
	ret = mpBind(sd, (struct sockaddr *)&serverSockAddr, sizeof(struct sockaddr_in)); 
	if (ret < 0)
		goto closeSockHandle;

	//prepare to accept connections
	ret = mpListen(sd, SOMAXCONN);
	if (ret < 0)
		goto closeSockHandle;

	return sd;

closeSockHandle:
	printf("Error in Ros_Controller_OpenSocket\r\n");

	if(sd >= 0)
		mpClose(sd);

	return -2;
}


//-----------------------------------------------------------------------
// Main Connection Server Task that listens for new connections
//-----------------------------------------------------------------------
void Ros_Controller_ConnectionServer_Start(Controller* controller)
{
	int     sdMotionServer = INVALID_SOCKET;
	int     sdStateServer = INVALID_SOCKET;
	int     sdIoServer = INVALID_SOCKET;
	int		sdMax;
	struct  fd_set  fds;
	int     sdAccepted = INVALID_SOCKET;
	struct  sockaddr_in     clientSockAddr;
	int     sizeofSockAddr;
	int     useNoDelay = 1;
	STATUS  s;

	//Set feedback signal (Application is installed and running)
	Ros_Controller_SetIOState(IO_FEEDBACK_CONNECTSERVERRUNNING, TRUE);

	printf("Controller connection server running\r\n");

	//New sockets for server listening to multiple port
	sdMotionServer = Ros_Controller_OpenSocket(TCP_PORT_MOTION);
	if(sdMotionServer < 0)
		goto closeSockHandle;
	
	sdStateServer = Ros_Controller_OpenSocket(TCP_PORT_STATE);
	if(sdStateServer < 0)
		goto closeSockHandle;
	
	sdIoServer = Ros_Controller_OpenSocket(TCP_PORT_IO);
	if(sdIoServer < 0)
		goto closeSockHandle;

	sdMax = max(sdMotionServer, sdStateServer);
	sdMax = max(sdMax, sdIoServer);

	FOREVER //Continue to accept multiple connections forever
	{
		FD_ZERO(&fds);
		FD_SET(sdMotionServer, &fds); 
		FD_SET(sdStateServer, &fds); 
		FD_SET(sdIoServer, &fds); 
		
		if(mpSelect(sdMax+1, &fds, NULL, NULL, NULL) > 0)
		{
			memset(&clientSockAddr, 0, sizeof(clientSockAddr));
			sizeofSockAddr = sizeof(clientSockAddr);
			
			//Check motion server
			if(FD_ISSET(sdMotionServer, &fds))
			{
				sdAccepted = mpAccept(sdMotionServer, (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
				if (sdAccepted < 0)
					break;
					
				s = setsockopt(sdAccepted, IPPROTO_TCP, TCP_NODELAY, (char*)&useNoDelay, sizeof (int));
				if( OK != s )
				{
					printf("Failed to set TCP_NODELAY.\r\n");
				}
				
				Ros_MotionServer_StartNewConnection(controller, sdAccepted);
			}
			
			//Check state server
			if(FD_ISSET(sdStateServer, &fds))
			{
				sdAccepted = mpAccept(sdStateServer, (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
				if (sdAccepted < 0)
					break;
					
				s = setsockopt(sdAccepted, IPPROTO_TCP, TCP_NODELAY, (char*)&useNoDelay, sizeof (int));
				if( OK != s )
				{
					printf("Failed to set TCP_NODELAY.\r\n");
				}
				
				Ros_StateServer_StartNewConnection(controller, sdAccepted);
			}
			
			//Check IO server
			if(FD_ISSET(sdIoServer, &fds))
			{
				sdAccepted = mpAccept(sdIoServer, (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
				if (sdAccepted < 0)
					break;
					
				s = setsockopt(sdAccepted, IPPROTO_TCP, TCP_NODELAY, (char*)&useNoDelay, sizeof (int));
				if( OK != s )
				{
					printf("Failed to set TCP_NODELAY.\r\n");
				}
				
				Ros_IoServer_StartNewConnection(controller, sdAccepted);
			}
		}
	}
	
closeSockHandle:
	printf("Error!?... Connection Server is aborting.  Reboot the controller.\r\n");

	if(sdIoServer >= 0)
		mpClose(sdIoServer);
	if(sdMotionServer >= 0)
		mpClose(sdMotionServer);
	if(sdStateServer >= 0)
		mpClose(sdStateServer);

	//disable feedback signal
	Ros_Controller_SetIOState(IO_FEEDBACK_CONNECTSERVERRUNNING, FALSE);
}



/**** Controller Status functions ****/

//-------------------------------------------------------------------
// Initialize list of Specific Input to keep track of
//-------------------------------------------------------------------
void Ros_Controller_StatusInit(Controller* controller)
{
	controller->ioStatusAddr[IO_ROBOTSTATUS_ALARM_MAJOR].ulAddr = 50010;		// Alarm
	controller->ioStatusAddr[IO_ROBOTSTATUS_ALARM_MINOR].ulAddr = 50011;		// Alarm
	controller->ioStatusAddr[IO_ROBOTSTATUS_ALARM_SYSTEM].ulAddr = 50012;		// Alarm
	controller->ioStatusAddr[IO_ROBOTSTATUS_ALARM_USER].ulAddr = 50013;			// Alarm
	controller->ioStatusAddr[IO_ROBOTSTATUS_ERROR].ulAddr = 50014;				// Error
	controller->ioStatusAddr[IO_ROBOTSTATUS_PLAY].ulAddr = 50054;				// Play
	controller->ioStatusAddr[IO_ROBOTSTATUS_TEACH].ulAddr = 50053;				// Teach
	controller->ioStatusAddr[IO_ROBOTSTATUS_REMOTE].ulAddr = 80011; //50056;	// Remote  // Modified E.M. 7/9/2013
	controller->ioStatusAddr[IO_ROBOTSTATUS_OPERATING].ulAddr = 50070;			// Operating
	controller->ioStatusAddr[IO_ROBOTSTATUS_HOLD].ulAddr = 50071;				// Hold
	controller->ioStatusAddr[IO_ROBOTSTATUS_SERVO].ulAddr = 50073;   			// Servo ON
	controller->ioStatusAddr[IO_ROBOTSTATUS_ESTOP_EX].ulAddr = 80025;   		// External E-Stop
	controller->ioStatusAddr[IO_ROBOTSTATUS_ESTOP_PP].ulAddr = 80026;   		// Pendant E-Stop
	controller->ioStatusAddr[IO_ROBOTSTATUS_ESTOP_CTRL].ulAddr = 80027;   		// Controller E-Stop
	controller->ioStatusAddr[IO_ROBOTSTATUS_WAITING_ROS].ulAddr = IO_FEEDBACK_WAITING_MP_INCMOVE; // Job input signaling ready for external motion
	controller->ioStatusAddr[IO_ROBOTSTATUS_INECOMODE].ulAddr = 50727;			// Energy Saving Mode
#if (YRC1000||YRC1000u)
	controller->ioStatusAddr[IO_ROBOTSTATUS_PFL_STOP].ulAddr = 81702;			// PFL function stopped the motion
	controller->ioStatusAddr[IO_ROBOTSTATUS_PFL_ESCAPE].ulAddr = 81703;			// PFL function escape from clamping motion
	controller->ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOIDING].ulAddr = 15120;		// PFL function avoidance operating
	controller->ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_JOINT].ulAddr = 15124;	// PFL function avoidance joint enabled
	controller->ioStatusAddr[IO_ROBOTSTATUS_PFL_AVOID_TRANS].ulAddr = 15125;	// PFL function avoidance translation enabled
#endif
	controller->alarmCode = 0;
}


BOOL Ros_Controller_IsAlarm(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_ALARM_MAJOR]!=0) 
		|| (controller->ioStatus[IO_ROBOTSTATUS_ALARM_MINOR]!=0)
		|| (controller->ioStatus[IO_ROBOTSTATUS_ALARM_SYSTEM]!=0)
		|| (controller->ioStatus[IO_ROBOTSTATUS_ALARM_USER]!=0) );
}

BOOL Ros_Controller_IsError(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_ERROR]!=0));
}

BOOL Ros_Controller_IsPlay(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_PLAY]!=0));
}

BOOL Ros_Controller_IsTeach(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_TEACH]!=0));
}

BOOL Ros_Controller_IsRemote(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_REMOTE]!=0));
}

BOOL Ros_Controller_IsOperating(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_OPERATING]!=0));
}

BOOL Ros_Controller_IsHold(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_HOLD]!=0));
}

BOOL Ros_Controller_IsServoOn(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_SERVO] != 0) && (controller->ioStatus[IO_ROBOTSTATUS_INECOMODE] == 0));
}

BOOL Ros_Controller_IsEcoMode(Controller* controller)
{
	return (controller->ioStatus[IO_ROBOTSTATUS_INECOMODE] != 0);
}

BOOL Ros_Controller_IsEStop(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_ESTOP_EX]==0) 
		|| (controller->ioStatus[IO_ROBOTSTATUS_ESTOP_PP]==0)
		|| (controller->ioStatus[IO_ROBOTSTATUS_ESTOP_CTRL]==0) );
}

BOOL Ros_Controller_IsWaitingRos(Controller* controller)
{
	return ((controller->ioStatus[IO_ROBOTSTATUS_WAITING_ROS]!=0));
}

BOOL Ros_Controller_IsMotionReady(Controller* controller)
{
	BOOL bMotionReady;
	
#ifndef DUMMY_SERVO_MODE	
	bMotionReady = controller->bRobotJobReady && Ros_Controller_IsOperating(controller) && Ros_Controller_IsRemote(controller)
		&& !Ros_Controller_IsPflActive(controller) && !controller->bMpIncMoveError;
#else
	bMotionReady = controller->bRobotJobReady && Ros_Controller_IsOperating(controller);
#endif


#ifdef DX100
	if(controller->numRobot < 2)
		bMotionReady = bMotionReady && controller->bSkillMotionReady[0];
	else
		bMotionReady = bMotionReady && controller->bSkillMotionReady[0] && controller->bSkillMotionReady[1];
#endif

	return bMotionReady;
}


BOOL Ros_Controller_IsPflActive(Controller* controller)
{
#if (YRC1000||YRC1000u)
	if (controller->bPFLEnabled) {
		if (controller->bPFLduringRosMove || controller->ioStatus[IO_ROBOTSTATUS_PFL_STOP] || controller->ioStatus[IO_ROBOTSTATUS_PFL_ESCAPE] ||
			(controller->ioStatus[IO_ROBOTSTATUS_PFL_AVOIDING]
				&& (controller->ioStatus[IO_ROBOTSTATUS_PFL_AVOID_JOINT] || controller->ioStatus[IO_ROBOTSTATUS_PFL_AVOID_TRANS])) != 0)
		{
			return TRUE;
		}
	}
#endif
	return FALSE;
}


int Ros_Controller_GetNotReadySubcode(Controller* controller)
{
	// Check alarm
	if(Ros_Controller_IsAlarm(controller))
		return ROS_RESULT_NOT_READY_ALARM;
	
	// Check error
	if(Ros_Controller_IsError(controller))
		return ROS_RESULT_NOT_READY_ERROR;
	
	// Check e-stop
	if(Ros_Controller_IsEStop(controller))
		return ROS_RESULT_NOT_READY_ESTOP;

	// Check play mode
	if(!Ros_Controller_IsPlay(controller))
		return ROS_RESULT_NOT_READY_NOT_PLAY;
	
#ifndef DUMMY_SERVO_MODE
	// Check remote
	if(!Ros_Controller_IsRemote(controller))
		return ROS_RESULT_NOT_READY_NOT_REMOTE;
	
	// Check servo power
	if(!Ros_Controller_IsServoOn(controller))
		return ROS_RESULT_NOT_READY_SERVO_OFF;
#endif

	// Check hold
	if(Ros_Controller_IsHold(controller))
		return ROS_RESULT_NOT_READY_HOLD;

	// Check PFL active
	if (Ros_Controller_IsPflActive(controller))
		return ROS_RESULT_NOT_READY_PFL_ACTIVE;

	// Check if Incremental Motion was rejected
	if (controller->bMpIncMoveError)
		return ROS_RESULT_NOT_READY_INC_MOVE_ERROR;

	// Check operating
	if(!Ros_Controller_IsOperating(controller))
		return ROS_RESULT_NOT_READY_NOT_STARTED;
	
	// Check ready I/O signal (should confirm wait)
	if(!Ros_Controller_IsWaitingRos(controller))
		return ROS_RESULT_NOT_READY_WAITING_ROS;

	// Check skill send command
	if(!Ros_Controller_IsMotionReady(controller))
		return ROS_RESULT_NOT_READY_SKILLSEND;
		
	return ROS_RESULT_NOT_READY_UNSPECIFIED;
}

BOOL Ros_Controller_IsInMotion(Controller* controller)
{
	int i;
	int groupNo;
	long fbPulsePos[MAX_PULSE_AXES];
	long cmdPulsePos[MAX_PULSE_AXES];
	BOOL bDataInQ;
	CtrlGroup* ctrlGroup;

	bDataInQ = Ros_MotionServer_HasDataInQueue(controller);

	if (bDataInQ == TRUE)
		return TRUE;
	else if (bDataInQ == ERROR)
		return ERROR;
	else
	{
		//for each control group
		for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
		{
			//Check group number valid
			if (!Ros_Controller_IsValidGroupNo(controller, groupNo))
				continue;

			//Check if the feeback position has caught up to the command position
			ctrlGroup = controller->ctrlGroups[groupNo];

			Ros_CtrlGroup_GetFBPulsePos(ctrlGroup, fbPulsePos);
			Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, cmdPulsePos);

			for (i = 0; i < MP_GRP_AXES_NUM; i += 1)
			{
				if (ctrlGroup->axisType.type[i] != AXIS_INVALID)
				{
					// Check if position matches current command position
					if (abs(fbPulsePos[i] - cmdPulsePos[i]) > START_MAX_PULSE_DEVIATION)
						return TRUE;
				}
			}
		}
	}

	return FALSE;
}

// Creates a simple message of type: ROS_MSG_ROBOT_STATUS = 13
// Simple message containing the current state of the controller
int Ros_Controller_StatusToMsg(Controller* controller, SimpleMsg* sendMsg)
{
	//initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyRobotStatus);
	
	// set header information
	sendMsg->header.msgType = ROS_MSG_ROBOT_STATUS;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	sendMsg->body.robotStatus.drives_powered = (int)(Ros_Controller_IsServoOn(controller));
	sendMsg->body.robotStatus.e_stopped = (int)(Ros_Controller_IsEStop(controller));
	sendMsg->body.robotStatus.error_code = controller->alarmCode;
	sendMsg->body.robotStatus.in_error = (int)Ros_Controller_IsAlarm(controller);
	sendMsg->body.robotStatus.in_motion = (int)(Ros_Controller_IsInMotion(controller));
	if(Ros_Controller_IsPlay(controller))
		sendMsg->body.robotStatus.mode = 2;
	else
		sendMsg->body.robotStatus.mode = 1;
	sendMsg->body.robotStatus.motion_possible = (int)(Ros_Controller_IsMotionReady(controller));
	
	return(sendMsg->prefix.length + sizeof(SmPrefix));
}

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_StatusRead(Controller* controller, USHORT ioStatus[IO_ROBOTSTATUS_MAX])
{
	return (mpReadIO(controller->ioStatusAddr, ioStatus, IO_ROBOTSTATUS_MAX) == 0);
}

//-------------------------------------------------------------------
// Update I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_StatusUpdate(Controller* controller)
{
	USHORT ioStatus[IO_ROBOTSTATUS_MAX];
	int i;
	BOOL prevReadyStatus;

	prevReadyStatus = Ros_Controller_IsMotionReady(controller);
	
	if(Ros_Controller_StatusRead(controller, ioStatus))
	{
		// Check for change of state and potentially react to the change
		for(i=0; i<IO_ROBOTSTATUS_MAX; i++)
		{
			if(controller->ioStatus[i] != ioStatus[i])
			{
				//printf("Change of ioStatus[%d]\r\n", i);
				
				controller->ioStatus[i] = ioStatus[i];
				switch(i)
				{
					case IO_ROBOTSTATUS_ALARM_MAJOR: // alarm
					case IO_ROBOTSTATUS_ALARM_MINOR: // alarm
					case IO_ROBOTSTATUS_ALARM_SYSTEM: // alarm
					case IO_ROBOTSTATUS_ALARM_USER: // alarm
					{
						if(ioStatus[IO_ROBOTSTATUS_ALARM_USER] == 0)
							controller->alarmCode = 0;
						else
							controller->alarmCode = Ros_Controller_GetAlarmCode();

						break;
					}

					case IO_ROBOTSTATUS_WAITING_ROS: // Job input signaling ready for external motion
					{
						if(ioStatus[IO_ROBOTSTATUS_WAITING_ROS] == 0)  // signal turned OFF
						{
							// Job execution stopped take action
							controller->bRobotJobReady = FALSE;
							Ros_MotionServer_ClearQ_All(controller);
						}
						else // signal turned ON (rising edge)
							controller->bRobotJobReady = TRUE; //job is ready (even if other factors will prevent motion)
						break;
					}
#if (YRC1000||YRC1000u)
					case IO_ROBOTSTATUS_PFL_STOP: // PFL Stop
					case IO_ROBOTSTATUS_PFL_ESCAPE: //  PFL Escaping
					case IO_ROBOTSTATUS_PFL_AVOIDING: // PFL Avoidance
					{
						if (controller->bPFLEnabled && controller->bRobotJobReady && Ros_Controller_IsPflActive(controller))
						{
							// Job execution stopped by PFL take action
							controller->bPFLduringRosMove = TRUE; //force job to be restarted with new ROS_CMD_START_TRAJ_MODE command
							Ros_MotionServer_ClearQ_All(controller);
						}
						break;
					}
#endif
				}
			}
		}

		if (!prevReadyStatus && Ros_Controller_IsMotionReady(controller))
			printf("Robot job is ready for ROS commands.\r\n");

		return TRUE;
	}
	else
		return FALSE;
}



/**** Wrappers on MP standard function ****/

//-------------------------------------------------------------------
// Get I/O state on the controller
//-------------------------------------------------------------------
BOOL Ros_Controller_GetIOState(ULONG signal)
{
	MP_IO_INFO ioInfo;
	USHORT ioState;
	int ret;
	
	//set feedback signal
	ioInfo.ulAddr = signal;
	ret = mpReadIO(&ioInfo, &ioState, 1);
	if(ret != 0)
		printf("mpReadIO failure (%d)\r\n", ret);
		
	return (ioState != 0);
}


//-------------------------------------------------------------------
// Set I/O state on the controller
//-------------------------------------------------------------------
void Ros_Controller_SetIOState(ULONG signal, BOOL status)
{
	MP_IO_DATA ioData;
	int ret;
	
	//set feedback signal
	ioData.ulAddr = signal;
	ioData.ulValue = status;
	ret = mpWriteIO(&ioData, 1);
	if(ret != 0)
		printf("mpWriteIO failure (%d)\r\n", ret);
}


//-------------------------------------------------------------------
// Get the code of the first alarm on the controller
//-------------------------------------------------------------------
int Ros_Controller_GetAlarmCode()
{
	MP_ALARM_CODE_RSP_DATA alarmData;
	memset(&alarmData, 0x00, sizeof(alarmData));
	if(mpGetAlarmCode(&alarmData) == 0)
	{
		if(alarmData.usAlarmNum > 0)
			return(alarmData.AlarmData.usAlarmNo[0]);
		else
			return 0;
	}
	return -1;
}


//-------------------------------------------------------------------
// Convert error code to string
//-------------------------------------------------------------------
void Ros_Controller_ErrNo_ToString(int errNo, char errMsg[ERROR_MSG_MAX_SIZE], int errMsgSize)
{
	switch(errNo)
	{
		case 0x2010: memcpy(errMsg, "Robot is in operation", errMsgSize); break;
		case 0x2030: memcpy(errMsg, "In HOLD status (PP)", errMsgSize); break;
		case 0x2040: memcpy(errMsg, "In HOLD status (External)", errMsgSize); break;
		case 0x2050: memcpy(errMsg, "In HOLD status (Command)", errMsgSize); break;
		case 0x2060: memcpy(errMsg, "In ERROR/ALARM status", errMsgSize); break;
		case 0x2070: memcpy(errMsg, "In SERVO OFF status", errMsgSize); break;
		case 0x2080: memcpy(errMsg, "Wrong operation mode", errMsgSize); break;
		case 0x3040: memcpy(errMsg, "The home position is not registered", errMsgSize); break;
		case 0x3050: memcpy(errMsg, "Out of range (ABSO data", errMsgSize); break;
		case 0x3400: memcpy(errMsg, "Cannot operate MASTER JOB", errMsgSize); break;
		case 0x3410: memcpy(errMsg, "The JOB name is already registered in another task", errMsgSize); break;
		case 0x4040: memcpy(errMsg, "Specified JOB not found", errMsgSize); break;
		case 0x5200: memcpy(errMsg, "Over data range", errMsgSize); break;
		default: memcpy(errMsg, "Unspecified reason", errMsgSize); break;
	}
}


#ifdef DX100

void Ros_Controller_ListenForSkill(Controller* controller, int sl)
{
	SYS2MP_SENS_MSG skillMsg;
	STATUS apiRet;
	
	controller->bSkillMotionReady[sl - MP_SL_ID1] = FALSE;
	memset(&skillMsg, 0x00, sizeof(SYS2MP_SENS_MSG));
	
	FOREVER
	{
		//SKILL complete
		//This will cause the SKILLSND command to complete the cursor to move to the next line.
		//Make sure all preparation is complete to move.
		//mpEndSkillCommandProcess(sl, &skillMsg); 
		mpEndSkillCommandProcess(sl, &skillMsg);
		
		Ros_Sleep(4); //sleepy time
		
		//Get SKILL command
		//task will wait for a skillsnd command in INFORM
		apiRet = mpReceiveSkillCommand(sl, &skillMsg);
		
		if (skillMsg.main_comm != MP_SKILL_COMM)
		{
			printf("Ignoring command, because it's not a SKILL command\n");
			continue;
		}
		
		//Process SKILL command
		switch(skillMsg.sub_comm)
		{
		case MP_SKILL_SEND:
			if(strcmp(skillMsg.cmd, "ROS-START") == 0)
			{
				controller->bSkillMotionReady[sl - MP_SL_ID1] = TRUE;
			}
			else if(strcmp(skillMsg.cmd, "ROS-STOP") == 0)
			{
				controller->bSkillMotionReady[sl - MP_SL_ID1] = FALSE;
			}
			else
			{
				printf ("MP_SKILL_SEND(SL_ID=%d) - %s \n", sl, skillMsg.cmd);
			}
#ifdef DEBUG
			printf("controller->bSkillMotionReady[%d] = %d\n", (sl - MP_SL_ID1), controller->bSkillMotionReady[sl - MP_SL_ID1]);
#endif

			if(Ros_Controller_IsMotionReady(controller))
				printf("Robot job is ready for ROS commands.\r\n");
			break;
			
		case MP_SKILL_END:
			//ABORT!!!
			controller->bSkillMotionReady[sl - MP_SL_ID1] = FALSE;
			break;
		}
	}
}
#endif


#if DX100
// VxWorks 5.5 do not have vsnprintf, use vsprintf instead...
int vsnprintf(char *s, size_t sz, const char *fmt, va_list args)
{
	char tmpBuf[1024]; // Hopefully enough...
	size_t res;
	res = vsprintf(tmpBuf, fmt, args);
	tmpBuf[sizeof(tmpBuf) - 1] = 0;  // be sure ending \0 is there
	if (res >= sz)
	{
		// Buffer overrun...
		printf("Logging.. Error vsnprintf:%d max:%d, anyway:\r\n", (int)res, (int)sz);
		printf("%s", tmpBuf);
		res = -res;
	}
	strncpy(s, tmpBuf, sz);
	s[sz - 1] = 0;  // be sure ending \0 is there
	return res;
}

// VxWorks 5.5 do not have snprintf
int snprintf(char *s, size_t sz, const char *fmt, ...)
{
	size_t res;
	char tmpBuf[1024]; // Hopefully enough...
	va_list va;

	va_start(va, fmt);
	res = vsnprintf(tmpBuf, sz, fmt, va);
	va_end(va);

	strncpy(s, tmpBuf, sz);
	s[sz - 1] = 0;  // be sure ending \0 is there
	return res;
}
#endif

void motoRosAssert(BOOL mustBeTrue, ROS_ASSERTION_CODE subCodeIfFalse, char* msgFmtIfFalse, ...)
{
	const int MAX_MSG_LEN = 32;
	char msg[MAX_MSG_LEN];
	char subMsg[MAX_MSG_LEN];
	va_list va;

	if (!mustBeTrue)
	{
		memset(msg, 0x00, MAX_MSG_LEN);
		memset(subMsg, 0x00, MAX_MSG_LEN);

		va_start(va, msgFmtIfFalse);
		vsnprintf(subMsg, MAX_MSG_LEN, msgFmtIfFalse, va);
		va_end(va);

		snprintf(msg, MAX_MSG_LEN, "MotoROS:%s", subMsg); //add "MotoROS" to distinguish from other controller alarms

		Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
		Ros_Controller_SetIOState(IO_FEEDBACK_INITIALIZATION_DONE, FALSE);

		mpSetAlarm(8000, msg, subCodeIfFalse);

		while (TRUE) //forever
		{
			puts(msg);
			Ros_Sleep(5000);
		}
	}
}

void Db_Print(char* msgFormat, ...)
{
#ifdef DEBUG
	const int MAX_MSG_LEN = 128;
	char msg[MAX_MSG_LEN];
	va_list va;

	memset(msg, 0x00, MAX_MSG_LEN);

	va_start(va, msgFormat);
	vsnprintf(msg, MAX_MSG_LEN, msgFormat, va);
	va_end(va);

	printf(msg);
#endif
}

void Ros_Sleep(float milliseconds)
{
	mpTaskDelay(milliseconds / mpGetRtc()); //Tick length varies between controller models
}
