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

#ifndef __ROBOT_ACTIVITY_H__
#define __ROBOT_ACTIVITY_H__

#include <opencv2/highgui/highgui.hpp>

#include <Windows.h>
#include <NuiApi.h>
#include "Client.h"

#define ROBOT_IDLE 0
#define ROBOT_FOLLOW_ME 1
#define ROBOT_LOCATE_SPEAKER 2
#define ROBOT_HUMOR_ANALYSIS 3
#define ROBOT_AIR_GUITAR 4
#define ROBOT_READING_PAPER 5
#define ROBOT_DANCING_BALLET 6
#define ROBOT_PERSONAL_TRAINER 7
#define ROBOT_OBJECT_FINDER 8
#define ROBOT_PERSON_FINDER 9
#define ROBOT_FOOD_INFO 10
#define ROBOT_EXPLORE 11
#define ROBOT_FALL_DETECTOR 12
#define ROBOT_CHOCOLATE 13

using namespace cv;


struct RobotData {
	Mat screen, screen2;
	INuiSensor* sensor;
	INuiAudioBeam* m_pNuiAudioSource;
	HANDLE colorFrameEvent;
	HANDLE depthFrameEvent;
	HANDLE colorStreamHandle;
	HANDLE depthStreamHandle;
	HANDLE skeletonFrameEvent;
	NUI_IMAGE_FRAME colorFrame;
	NUI_IMAGE_FRAME depthFrame;
	WAVEFORMATEX			m_wfxOut;
	bool write2;
	bool isIdleCurrentActivity;

};

class RobotActivity {

public:
	
	virtual bool update(RobotData &rdata) = 0;
	virtual bool pause() = 0;

	int activity;
	Client *client;

};

#endif