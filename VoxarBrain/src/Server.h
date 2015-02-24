/* Copyright(c) 2015 Voxar Labs (www.cin.ufpe.br/voxarlabs)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#pragma once
#include "RobotActivity.h"
#include "Message.h"
#include <Pillar\Pillar.h>
//#include <future>
#include <thread>

#define StartFollowMeFollowMeServerState (ServiceID::StartupShutdownServiceID) | (Service::FollowMeService << 8) | (SubService::FollowMeSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define EndFollowMeFollowMeServerState (ServiceID::StartupShutdownServiceID) | (Service::FollowMeService << 8) | (SubService::FollowMeSubService << 16) | (FollowMeDirection::Stop << 24)

#define StartObjRecTrainServerState (ServiceID::StartupShutdownServiceID) | (Service::ObjRecService << 8) | (SubService::TrainSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define EndObjRecTrainServerState (ServiceID::StartupShutdownServiceID) | (Service::ObjRecService << 8) | (SubService::TrainSubService << 16) | (FollowMeDirection::Stop << 24)

#define StartObjRecObjRecServerState (ServiceID::StartupShutdownServiceID) | (Service::ObjRecService << 8) | (SubService::ObjRecSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define EndObjRecObjRecServerState (ServiceID::StartupShutdownServiceID) | (Service::ObjRecService << 8) | (SubService::ObjRecSubService << 16) | (FollowMeDirection::Stop << 24)

#define StartGestRecTrainServerState (ServiceID::StartupShutdownServiceID) | (Service::GestRecService << 8) | (SubService::TrainSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define EndGestRecTrainServerState (ServiceID::StartupShutdownServiceID) | (Service::GestRecService << 8) | (SubService::TrainSubService << 16) | (FollowMeDirection::Stop << 24)

#define StartGestRecGestRecServerState (ServiceID::StartupShutdownServiceID) | (Service::GestRecService << 8) | (SubService::GestRecSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define EndGestRecGestRecServerState (ServiceID::StartupShutdownServiceID) | (Service::GestRecService << 8) | (SubService::GestRecSubService << 16) | (FollowMeDirection::Stop << 24)

#define ObjectRecTrainingTemporaryStoreServerState (ServiceID::ObjectTrainingServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define ObjectRecTrainingPermanentStoreServerState (ServiceID::ObjectTrainingServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24)

#define GestRecTrainingTemporaryStoreServerState (ServiceID::GestureTrainingServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24) | 0x80000000
#define GestRecTrainingPermanentStoreServerState (ServiceID::GestureTrainingServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24)

#define FollowMeForwardState (ServiceID::FollowMeServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Forward << 24)
#define FollowMeBackState (ServiceID::FollowMeServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Back << 24)
#define FollowMeRightState (ServiceID::FollowMeServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Right << 24)
#define FollowMeLeftState (ServiceID::FollowMeServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Left << 24)
#define FollowMeStopState (ServiceID::FollowMeServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24)

#define IdleServerState (ServiceID::NoServiceID) | (Service::NoService << 8) | (SubService::NoSubService << 16) | (FollowMeDirection::Stop << 24)

//enum ServerState
//{
//	StartFollowMeFollowMe, EndFollowMeFollowMe, StartObjRecTrain, EndObjRecTrain, StartObjRecObjRec, EndObjRecObjRec,
//	StartGestRecTrain, EndGestRecTrain, StartGestRecGestRec, EndGestRecGestRec, ObjectRecTraining, 
//	GestureRecTraining, Idle
//};

class Server
{
public:
	LibTools::Peer* peer;

	std::vector<vbByte> buffer;
	Message message;

	vbNatural state;

	int *voxarBrainState;
	
	thread sThread;

	Server(int *voxarBrainState)
	{
		this->voxarBrainState = voxarBrainState;
		peer = 0;
		state = IdleServerState;
		buffer.resize(256);
	}

	static void receiveCommands(Server* server)
	{
		server->peer = new LibTools::Peer(49002);
		server->peer->open();
		while (true)
		{
			Server::receiveCommand(server);
		}
	}
		
	static void receiveCommand(Server* server)
	{
		vbNatural lengthReceived = 0;
		server->peer->receive(server->buffer, lengthReceived);
		if (lengthReceived)
		{
			for (int i = 0; i < lengthReceived; i++) {
				printf("%02x ", server->buffer.at(i));
			}
			printf("\n");
			
			Message& message = server->message;
			message.set(&server->buffer.at(0));
			if (message.valid) {
				server->state = message.serviceId | (message.service << 8) | (message.mode << 16) | (message.followMeDirection << 24) | (message.storeTemporary << 31) | (message.actionTypeStart << 31);
				switch (server->state)
				{
				case StartFollowMeFollowMeServerState:
					//printf("StartFollowMeFollowMeServerState\n");
					*server->voxarBrainState = ROBOT_FOLLOW_ME;
					break;
				case EndFollowMeFollowMeServerState:
					//printf("EndFollowMeFollowMeServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				case StartObjRecTrainServerState:
					//printf("StartObjRecTrainServerState\n");
					break;
				case EndObjRecTrainServerState:
					//printf("EndObjRecTrainServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				case StartObjRecObjRecServerState:
					//printf("StartObjRecObjRecServerState\n");
					break;
				case EndObjRecObjRecServerState:
					//printf("EndObjRecObjRecServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				case StartGestRecTrainServerState:
					//printf("StartGestRecTrainServerState\n");
					break;
				case EndGestRecTrainServerState:
					//printf("EndGestRecTrainServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				case StartGestRecGestRecServerState:
					//printf("StartGestRecGestRecServerState\n");
					break;
				case EndGestRecGestRecServerState:
					//printf("EndGestRecGestRecServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				case ObjectRecTrainingTemporaryStoreServerState:
					//printf("ObjectRecTrainingTemporaryStoreServerState\n");
					break;
				case ObjectRecTrainingPermanentStoreServerState:
					//printf("ObjectRecTrainingPermanentStoreServerState\n");
					break;
				case GestRecTrainingTemporaryStoreServerState:
					//printf("GestRecTrainingTemporaryStoreServerState\n");
					break;
				case GestRecTrainingPermanentStoreServerState:
					//printf("GestRecTrainingPermanentStoreServerState\n");
					break;
				case FollowMeForwardState:
					//printf("FollowMeForwardState\n");
					break;
				case FollowMeBackState:
					//printf("FollowMeBackState\n");
					break;
				case FollowMeRightState:
					//printf("FollowMeRightState\n");
					break;
				case FollowMeLeftState:
					//printf("FollowMeLeftState\n");
					break;
				case FollowMeStopState:
					//printf("FollowMeStopState\n");
					break;
				case IdleServerState:
					//printf("IdleServerState\n");
					*server->voxarBrainState = ROBOT_IDLE;
					break;
				default:
					//printf("IdleServerState\n");
					//*this->voxarBrainState = ROBOT_IDLE;
					break;
				}
			}
			message.resetData();
			//server->printfState();
		}
		/*else
		{
			server->state = IdleServerState;
		}*/
		
	}

	void printfState()
	{
		switch (state)
		{
		case StartFollowMeFollowMeServerState:
			printf("StartFollowMeFollowMeServerState\n");
			*this->voxarBrainState = ROBOT_FOLLOW_ME;
			break;
		case EndFollowMeFollowMeServerState:
			printf("EndFollowMeFollowMeServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		case StartObjRecTrainServerState:
			printf("StartObjRecTrainServerState\n");
			break;
		case EndObjRecTrainServerState:
			printf("EndObjRecTrainServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		case StartObjRecObjRecServerState:
			printf("StartObjRecObjRecServerState\n");
			break;
		case EndObjRecObjRecServerState:
			printf("EndObjRecObjRecServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		case StartGestRecTrainServerState:
			printf("StartGestRecTrainServerState\n");
			break;
		case EndGestRecTrainServerState:
			printf("EndGestRecTrainServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		case StartGestRecGestRecServerState:
			printf("StartGestRecGestRecServerState\n");
			break;
		case EndGestRecGestRecServerState:
			printf("EndGestRecGestRecServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		case ObjectRecTrainingTemporaryStoreServerState:
			printf("ObjectRecTrainingTemporaryStoreServerState\n");
			break;
		case ObjectRecTrainingPermanentStoreServerState:
			printf("ObjectRecTrainingPermanentStoreServerState\n");
			break;
		case GestRecTrainingTemporaryStoreServerState:
			printf("GestRecTrainingTemporaryStoreServerState\n");
			break;
		case GestRecTrainingPermanentStoreServerState:
			printf("GestRecTrainingPermanentStoreServerState\n");
			break;
		case FollowMeForwardState:
			printf("FollowMeForwardState\n");
			break;
		case FollowMeBackState:
			printf("FollowMeBackState\n");
			break;
		case FollowMeRightState:
			printf("FollowMeRightState\n");
			break;
		case FollowMeLeftState:
			printf("FollowMeLeftState\n");
			break;
		case FollowMeStopState:
			printf("FollowMeStopState\n");
			break;
		case IdleServerState:
			printf("IdleServerState\n");
			*this->voxarBrainState = ROBOT_IDLE;
			break;
		default:
			printf("IdleServerState\n");
			//*this->voxarBrainState = ROBOT_IDLE;
			break;
		}
	}

	void startServer()
	{
		sThread = thread(Server::receiveCommands, this);
	}

	void receiveCommandSync()
	{
		Server::receiveCommand(this);
	}

	void receiveCommandAsync()
	{
		//std::async(Server::receiveCommand,this);
		//sThread = thread(Server::receiveCommand, this);
	}
	
	void syncServer()
	{
		sThread.join();
	}
	
	~Server()
	{
		//syncServer();
		if (peer)
			delete peer;
	}

};