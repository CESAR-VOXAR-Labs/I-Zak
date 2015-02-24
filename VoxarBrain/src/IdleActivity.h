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

#ifndef __IDLE_ACTIVITY_H__
#define __IDLE_ACTIVITY_H__

#include <opencv2/highgui/highgui.hpp>
#include "RobotActivity.h"

using namespace cv;

class IdleActivity : public RobotActivity {

public:
	IdleActivity(Client *client) {
		this->activity = ROBOT_IDLE;
		this->client = client;
		this->refreshCounter = 0;

		this->capturer.open(robotFacePath);

		if (!this->capturer.isOpened()) {
			printf("Arquivo do video de idle do robo nao encontrado.\n");
			exit(1);
		}
	}	

	bool pause()
	{
		return true;
	}

	bool update(RobotData &rdata) {
		
		if (refreshCounter == 0) {
			
			int value = (rand() % 101);

			unsigned char gimbal[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x05, 0x4E, 0x00, 0x00 };
			gimbal[7] = value;
			gimbal[8] = 255 - ((gimbal[5] + gimbal[6] + gimbal[7]) % 256);

			// sendCommand
			client->sendCommand(gimbal, 9);
			printf("Move gimbal: %d\n", value);

			refreshCounter = 30 * (3 + (rand() % 8));
		}
		else {
			--refreshCounter;
		}

		if (!capturer.grab())
		{
			capturer.release();
			capturer.open(robotFacePath);
			capturer.grab();
		}
		if (rdata.write2) {
			capturer.retrieve(rdata.screen2);
		}
		else {
			capturer.retrieve(rdata.screen);
		}
		
		return 1;
	}

private:
	int refreshCounter;
	VideoCapture capturer;
	string robotFacePath = "rsc/idle/out%02d.png";

};



#endif