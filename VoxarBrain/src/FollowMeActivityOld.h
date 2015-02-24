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

#ifndef __FOLLOW_ME_ACTIVITY_H__
#define __FOLLOW_ME_ACTIVITY_H__

#include <opencv2/highgui/highgui.hpp>
#include "RobotActivity.h"

using namespace cv;

unsigned char backwardMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0x08, 0xA8 };
unsigned char forwardMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0x04, 0xAC };
unsigned char stopMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0x00, 0xB0 };
unsigned char rotateRightMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0x02, 0xAE };
unsigned char rotateLeftMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0x01, 0xAF };
unsigned char personFoundMessage[9] = { 0xFF, 0xFD, 0x01, 0x00, 0xFE, 0x01, 0x4E, 0xff, 0xAF };

class FollowMeActivity : public RobotActivity {

public:
	FollowMeActivity(Client *client) {
		
		this->activity = ROBOT_FOLLOW_ME;
		this->client = client;

		personFound = false;

		this->width = 320;
		this->height = 240;

		this->headx = -1;
		this->heady = -1;
		this->distance = 0;

		this->state = stopped;

		this->depthImage.create(Size(width, height), CV_8UC3);
		this->colorImage.create(Size(width * 2, height * 2), CV_8UC4);
		this->depthBuffer = new unsigned short[width * height];
		this->colorBuffer = new unsigned char[640 * 480 * 4];
	}

	bool pause()
	{
		return true;
	}

	bool update(RobotData &rdata) {
		
		HRESULT hr = rdata.sensor->NuiImageStreamGetNextFrame(rdata.colorStreamHandle, 300, &rdata.colorFrame);
		if (!FAILED(hr))
		{

			NUI_LOCKED_RECT lockedRect;
			rdata.colorFrame.pFrameTexture->LockRect(0, &lockedRect, NULL, 0);

			if (lockedRect.Pitch != 0)
			{
				// Copy image information into buffer
				BYTE* pBuffer = lockedRect.pBits;
				INT size = lockedRect.size;

				// Only reallocate memory if the buffer size has changed
				memcpy_s(this->colorBuffer, size, pBuffer, size);
				memcpy_s(this->colorImage.data, size, pBuffer, size);

			}

			rdata.colorFrame.pFrameTexture->UnlockRect(0);

			rdata.sensor->NuiImageStreamReleaseFrame(rdata.colorStreamHandle, &rdata.colorFrame);
		}

		hr = rdata.sensor->NuiImageStreamGetNextFrame(rdata.depthStreamHandle, 300, &rdata.depthFrame);
		if (FAILED(hr))
		{
			return 0;
		}

		NUI_LOCKED_RECT lockedRect;
		rdata.depthFrame.pFrameTexture->LockRect(0, &lockedRect, NULL, 0);

		// Check if image is valid
		if (lockedRect.Pitch != 0)
		{
			// Copy image information into buffer
			BYTE* pBuffer = lockedRect.pBits;
			INT size = lockedRect.size;

			// Only reallocate memory if the buffer size has changed
			memcpy_s(this->depthBuffer, size, pBuffer, size);
		}

		rdata.depthFrame.pFrameTexture->UnlockRect(0);

		rdata.sensor->NuiImageStreamReleaseFrame(rdata.depthStreamHandle, &rdata.depthFrame);

		NUI_SKELETON_FRAME skeletonFrame;

		rdata.sensor->NuiSkeletonGetNextFrame(300, &skeletonFrame);
		rdata.sensor->NuiTransformSmooth(&skeletonFrame, NULL);

		float tempfx = -1;
		float tempfy = -1;

		int theadx, theady;
		int tshoulderx, tshouldery;

		bool skeldetected = false;

		int countSkeletons = 0;
		for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
		{
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
			if (trackingState == NUI_SKELETON_TRACKED) {
				countSkeletons++;

				// debug
				for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
					Vector4 joint = skeletonFrame.SkeletonData[i].SkeletonPositions[j];
					NuiTransformSkeletonToDepthImage(joint, &tempfx, &tempfy);
					theadx = tempfx;
					theady = tempfy;
					//SHORT realDepth = NuiDepthPixelToDepth(depthBuffer[(int)tempfy * width + (int)tempfx]);
					if (theadx >= 0 && theadx < 320 && theady >= 0 && theady < 240) {
						NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, 0, tempfx, tempfy, depthBuffer[(int)theady * width + (int)theadx], &headx, &heady);
						circle(colorImage, Point(headx, heady), 2, Scalar(0, 0, 255), 3);
					}
				}
			}
		}

		if (countSkeletons == 1 && !personFound) {
			
			personFoundMessage[8] = (255 - (personFoundMessage[5] + personFoundMessage[6] + personFoundMessage[7])) % 256;
			this->client->sendCommand(personFoundMessage, 9);
			printf("pessoa encontrada!\n");
			personFound = true;
			Sleep(2000);
		}

		for (int i = 0; i < NUI_SKELETON_COUNT && countSkeletons == 1; ++i)
		{
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
			if (trackingState == NUI_SKELETON_TRACKED)
			{
				Vector4 headv = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
				Vector4 shouldercenterv = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER];
				Vector4 spinev = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SPINE];
				Vector4 hipcenterv = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER];
				
				if (headv.y > shouldercenterv.y && shouldercenterv.y > spinev.y && spinev.y > hipcenterv.y) {

					//printf("%f %f %f %f\n", headv.y, shouldercenterv.y, spinev.y, hipcenterv.y);

					NuiTransformSkeletonToDepthImage(headv, &tempfx, &tempfy);
					theadx = tempfx;// *width;
					theady = tempfy;// *height;

					NuiTransformSkeletonToDepthImage(shouldercenterv, &tempfx, &tempfy);
					tshoulderx = tempfx;// *width;
					tshouldery = tempfy;// *height;

					float dist = sqrtf((theadx - tshoulderx)*(theadx - tshoulderx) + (theady - tshouldery)*(theady - tshouldery));

					SHORT realDepth = NuiDepthPixelToDepth(depthBuffer[theady * width + theadx]);
					NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, 0, theadx, theady, depthBuffer[theady * width + theadx], &headx, &heady);
					distance = realDepth;

					if (distance != 0) {

						circle(colorImage, Point(headx, heady), dist * 2, Scalar(0, 255, 0), 3);

						//printf("%d %d %d\n", headx, heady, distance);
					}

					think(headx, heady, distance);
					skeldetected = true;

					//NuiTransformSkeletonToDepthImage(shouldercenterv, &tempfx, &tempfy);

					//radius = (headx - tempfx)*(headx - tempfx) + (heady - tempfy)*(heady - tempfy);
					//radius *= 1.25;
					break;
					//printf("%d\n", radius);
					//printf("%d %d\n", headx, heady);
				}

			}
		}

		if (!skeldetected && state != stopped) {
			state = stopped;
			this->client->sendCommand(stopMessage, 9);
			printf("STOP.\n");

		}

		//for (UINT y = 0; y < height; ++y)
		//{
		//	// Get row pointer for depth Mat
		//	Vec3b* pDepthRow = depthImage.ptr<Vec3b>(y);

		//	for (UINT x = 0; x < width; ++x)
		//	{
		//		unsigned char red, green, blue;

		//		//bool isHead = false;

		//		SHORT realDepth = NuiDepthPixelToDepth(depthBuffer[y * width + x]);
		//		USHORT playerIndex = NuiDepthPixelToPlayerIndex(depthBuffer[y * width + x]);

		//		// Convert depth info into an intensity for display
		//		BYTE b = 255 - static_cast<BYTE>(256 * realDepth / 0x0fff);

		//		int dist = (x - headx)*(x - headx) + (y - heady)*(y - heady);
		//		if (dist <= radius && playerIndex != 0) {

		//			long cx = 0, cy = 0;

		//			hr = NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, &colorFrame.ViewArea, x, y, depthBuffer[y * width + x], &cx, &cy);
		//			if (!FAILED(hr) && cx >= 0 && cx < 640 && cy >= 0 && cy < 480) {
		//				pDepthRow[x] = Vec3b(colorBuffer[(cy * 640 + cx) * 4 + 0], colorBuffer[(cy * 640 + cx) * 4 + 1], colorBuffer[(cy * 640 + cx) * 4 + 2]);

		//				Vector4 coord3d = NuiTransformDepthImageToSkeleton(x, y, depthBuffer[y * width + x]);
		//				/*f.x = coord3d.x;
		//				f.y = coord3d.y;
		//				f.z = coord3d.z;

		//				f.r = colorBuffer[(cy * 640 + cx) * 4 + 2] / 255.0f;
		//				f.g = colorBuffer[(cy * 640 + cx) * 4 + 1] / 255.0f;
		//				f.b = colorBuffer[(cy * 640 + cx) * 4 + 0] / 255.0f;*/
		//			}
		//			else {
		//				DepthShortToRgb(depthBuffer[y * width + x], &red, &green, &blue);
		//				pDepthRow[x] = Vec3b(b / 2, b / 2, b / 2);
		//			}

		//		}
		//		else {
		//			//DepthShortToRgb(depthBuffer[y * width + x], &red, &green, &blue);
		//			pDepthRow[x] = Vec3b(b / 2, b / 2, b / 2);
		//		}


		//	}
		//}
		rdata.screen = colorImage;
		//imshow("color", colorImage);
		//imshow("depth", depthImage);
		//int key = cvWaitKey(30);
		//if (key == 27) exit(0);
				
		return 1;
	}

private:
	int width;
	int height;
	long headx;
	long heady;
	short distance = 0;
	Mat colorImage;
	Mat depthImage;
	unsigned short *depthBuffer;
	unsigned char *colorBuffer;
	const int stopped = 0;
	const int rotating_left = 1;
	const int rotating_right = 2;
	const int walking = 3;
	bool personFound;
		
	int state;

	void think(int x, int y, int distance) {

		if (x < (320 - 80)) {

			if (state != rotating_left) {
				// send stop
				this->client->sendCommand(stopMessage, 9);
				printf("STOP.\n");
				// send rotate left
				this->client->sendCommand(rotateRightMessage, 9);
				printf("ROTATE RIGHT.\n");
				state = rotating_left;
			}

		}
		else if (x > 320 + 80) {

			if (state != rotating_right) {
				// send stop
				this->client->sendCommand(stopMessage, 9);
				printf("STOP.\n");
				// send rotate right
				this->client->sendCommand(rotateLeftMessage, 9);
				printf("ROTATE LEFT.\n");
				state = rotating_right;
			}

		}
		else {

			if (x > (320 - 40) && x < (320 + 40)) {
				if (state != walking && state != stopped) {
					// send stop
					this->client->sendCommand(stopMessage, 9);
					printf("STOP.\n");
					state = stopped;

				}
				else {
					if (distance > 1800 && state == stopped) {
						// send walk
						this->client->sendCommand(forwardMessage, 9);
						printf("WALK.\n");
						state = walking;
					}
					else if (distance < 1500 && state == walking) {
						// send stop
						this->client->sendCommand(stopMessage, 9);
						printf("STOP.\n");
						state = stopped;
					}
				}
			}

		}


	}
};

#endif