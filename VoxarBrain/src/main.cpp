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

#pragma comment ( lib, "d2d1.lib" )

#include <stdio.h>
#include <vector>

#include <future>

#include "Client.h"

#include "RobotActivity.h"
#include "IdleActivity.h"
#include "FollowMeActivityOld.h"

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define BUFLEN 1024  //Max length of buffer
#define PORT 49002   //The port on which to listen for incoming data

using namespace std;
using namespace cv;

bool recognizing = true;
bool transparent = false;

vector<RobotActivity*> activities;

SOCKET s;
struct sockaddr_in server, si_other;
int slen, recv_len;
char buf[BUFLEN];
WSADATA wsa;

static int main2() {
	slen = sizeof(si_other);

	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	//Bind
	if (::bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	puts("Bind done");

	//keep listening for data
	while (1)
	{
		printf("Waiting for data...");
		fflush(stdout);

		//clear the buffer by filling null, it might have previously received data
		memset(buf, '\0', BUFLEN);

		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
		{
			printf("recvfrom() failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}

		//print details of the client/peer and the data received
		printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
		printf("Data: %s\n", buf);

		//now reply the client with the same data
		if (sendto(s, buf, recv_len, 0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
		{
			printf("sendto() failed with error code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
	}

	//closesocket(s);
	//WSACleanup();

	return 0;
}

void initKinect(RobotData &rdata) {
	int sensorCount = 0;
	HRESULT hr = NuiGetSensorCount(&sensorCount);

	if (sensorCount > 0) {
		hr = NuiCreateSensorByIndex(0, &rdata.sensor);
		if (FAILED(hr))
		{
			return;
		}

		hr = rdata.sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_AUDIO);
		if (FAILED(hr))
		{
			return;
		}
	}

	rdata.colorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	rdata.depthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	hr = rdata.sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, rdata.depthFrameEvent, &rdata.depthStreamHandle);
	if (FAILED(hr))
	{
		return;
	}

	hr = rdata.sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, rdata.colorFrameEvent, &rdata.colorStreamHandle);
	if (FAILED(hr))
	{
		return;
	}

	rdata.skeletonFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	hr = rdata.sensor->NuiSkeletonTrackingEnable(rdata.skeletonFrameEvent, 0);
	if (FAILED(hr))
	{
		return;
	}

	//depthImage.create(Size(width, height), CV_8UC3);
	//colorImage.create(Size(width * 2, height * 2), CV_8UC4);
	//depthBuffer = new unsigned short[width * height];
	//colorBuffer = new unsigned char[640 * 480 * 4];
}

void main() 
{	
	printf("COMMANDS: \n\t press 'i' to force IDLE state\n\t press 'f' to force FOLLOW ME state");

	cv::namedWindow("Cexar Robot", CV_WINDOW_NORMAL);
	cv::setWindowProperty("Cexar Robot", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	// init kinect and aux libraries
	Client client;

	RobotData rdata;
	rdata.screen.create(800, 1280, CV_8UC3);
	rdata.screen2.create(800, 1280, CV_8UC3);
	rdata.write2 = false;
		
	initKinect(rdata);
	unsigned char t[4];
	t[0] = 0xff;
	t[1] = 0x0f;
	t[2] = 0xf0;
	t[3] = 0xa8;

	// create and init activities
	// idle activity
	RobotActivity *idleActivity = new IdleActivity(&client);
	activities.push_back(idleActivity);
	// follow me activity
	RobotActivity *followMeActivity = new FollowMeActivity(&client);
	activities.push_back(followMeActivity);
	// locate speaker activity
	
	int previousActivity = ROBOT_IDLE;

	//Server2 selvinho(&recognizer->activity);

	while (true) 
	{
		rdata.isIdleCurrentActivity = (previousActivity == ROBOT_IDLE);

		// call current activity function
		activities[previousActivity]->update(rdata);

		imshow("Cexar Robot", rdata.screen);

		// wait for key
		int c = cv::waitKey(33);		
				
		if (c == 27) exit(0);
		switch (c) {
			case 'i':
				activities[previousActivity]->pause();
				previousActivity = ROBOT_IDLE;
				break;
			case'f':
				activities[previousActivity]->pause();
				previousActivity = ROBOT_FOLLOW_ME;
				break;
			default:
				break;
		}
	}
}
