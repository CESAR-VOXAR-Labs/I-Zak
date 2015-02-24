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
#include "Defines.h"
#include <vector>

using namespace ROSProtocol;
using namespace std;

enum ServiceID
{
	StartupShutdownServiceID, ObjectTrainingServiceID, GestureTrainingServiceID, FollowMeServiceID, NoServiceID
};

enum FollowMeDirection
{
	Left, Right, Forward, Back, Stop
};

enum Service
{
	FollowMeService, ObjRecService, GestRecService, NoService
};

enum SubService
{
	FollowMeSubService, ObjRecSubService, GestRecSubService, TrainSubService, NoSubService
};

class Message
{
private:
	vbByte flag[2];
	vbByte length[2];
	vbByte checksumLength;
	vbByte serviceID[2];
	vbByte* data;
	vbByte checksumData;

	vbNatural dataSize;

	void identifyService(string& str, vbNatural& i)
	{
		i = 1;
		vbByte datum = data[i];
		while (datum != 0x00)
		{
			str.push_back(datum);
			++i;
			datum = data[i];
		}
		if (str == "followme")
		{
			service = Service::FollowMeService;
		}
		else if (str == "objrec")
		{
			service = Service::ObjRecService;
		}
		else if (str == "gestrec")
		{
			service = Service::GestRecService;
		}
		++i;
	}

	void identifySubService(string& str, vbNatural& i)
	{
		vbByte datum = data[i];
		while (datum != 0x00)
		{
			str.push_back(datum);
			++i;
			datum = data[i];
		}
		if (str == "followme")
		{
			mode = SubService::FollowMeSubService;
		}
		else if (str == "objrec")
		{
			mode = SubService::ObjRecSubService;
		}
		else if (str == "gestrec")
		{
			mode = SubService::GestRecSubService;
		}
		else if (str == "train")
		{
			mode = SubService::TrainSubService;
		}
	}

public:
	bool valid;

	bool actionTypeStart;
	Service service;
	SubService mode;
	ServiceID serviceId;

	vbNatural objectGesture;
	bool storeTemporary;

	FollowMeDirection followMeDirection;

	Message()
	{
		dataSize = 0;
		resetData();
	}

	~Message()
	{
		if (dataSize)
		{
			delete[] data;
		}			
	}

	void set(const vbByte* message)
	{
		flag[0] = message[0];
		flag[1] = message[1];
		length[0] = message[2];
		length[1] = message[3];
		
		vbNatural lo = static_cast<vbNatural>(length[0]);
		vbNatural hi = static_cast<vbNatural>(length[1]);

		vbNatural check = 255 - (lo + hi) % 256;

		checksumLength = message[4];
		vbNatural checkNaturalValue = static_cast<vbNatural>(checksumLength);

		valid = checkNaturalValue == check;
		
		if (!valid)
			return;

		serviceID[0] = message[5];
		serviceID[1] = message[6];

		dataSize = (lo & 0x000000ff) | ((hi << 8) & 0x0000ff00);

		data = new vbByte[dataSize];
		vbNatural count;
		const vbNatural offset = 7;
		vbNatural sum = 0;

		for (count = 0;count<dataSize; ++count)
		{
			data[count] = message[offset + count];
			sum += static_cast<vbNatural>(data[count]);
		}

		checksumData = message[dataSize + offset];
		checkNaturalValue = static_cast<vbNatural>(checksumData);

		lo = static_cast<vbNatural>(serviceID[0]);
		hi = static_cast<vbNatural>(serviceID[1]);

		check = 255 - (lo + hi + sum) % 256;

		valid = checkNaturalValue == check;

		if (!valid)
		{
			resetData();
			return;
		}

		if (serviceID[0] == 0x01 && serviceID[1] == 0x80)
		{
			serviceId = ServiceID::StartupShutdownServiceID;
		}
		else if (serviceID[0] == 0x01 && serviceID[1] == 0x8e)
		{
			serviceId = ServiceID::ObjectTrainingServiceID;
		}
		else if (serviceID[0] == 0x02 && serviceID[1] == 0x8e)
		{
			serviceId = ServiceID::GestureTrainingServiceID;
		}
		else if (serviceID[0] == 0x01 && serviceID[1] == 0x4e)
		{
			serviceId = ServiceID::FollowMeServiceID;
		}
		else
		{
			resetData();
			return;
		}

		if (serviceId == ServiceID::StartupShutdownServiceID)
		{
			actionTypeStart = data[0];
			string str;
			str.reserve(dataSize);
			vbNatural index;
			identifyService(str,index);
			if (service == Service::NoService)
			{
				resetData();
				return;
			}
			str.clear();
			identifySubService(str, index);
			if (mode == SubService::NoSubService)
			{
				resetData();
				return;
			}
		}
		else if (serviceId == ServiceID::ObjectTrainingServiceID || serviceId == ServiceID::GestureTrainingServiceID)
		{
			vbNatural byte0 = static_cast<vbNatural>(data[0]);
			vbNatural byte1 = static_cast<vbNatural>(data[1]);
			vbNatural byte2 = static_cast<vbNatural>(data[2]);
			vbNatural byte3 = static_cast<vbNatural>(data[3]);

			objectGesture = (byte0 & 0x000000ff) | ((byte1 << 8) & 0x0000ff00)
				| ((byte2 << 16) & 0x00ff0000) | ((byte3 << 24) & 0x7f000000);

			storeTemporary = byte3 >> 7;
		}
		else if (serviceId == ServiceID::FollowMeServiceID)
		{
			vbNatural direction = static_cast<vbNatural>(data[0]);
			switch (direction)
			{
			case 1:
				followMeDirection = FollowMeDirection::Left;
				break;
			case 2:
				followMeDirection = FollowMeDirection::Right;
				break;
			case 4:
				followMeDirection = FollowMeDirection::Forward;
				break;
			case 8:
				followMeDirection = FollowMeDirection::Back;
				break;
			case 0:
				followMeDirection = FollowMeDirection::Stop;
				break;
			default:
				followMeDirection = FollowMeDirection::Stop;
				break;
			};
		}
	}

	

	void resetData()
	{
		if (dataSize)
		{
			delete[] data;
		}
		valid = false;
		storeTemporary = false;
		objectGesture = 0;
		dataSize = 0;
		actionTypeStart = false;
		serviceId = ServiceID::NoServiceID;
		service = Service::NoService;
		mode = SubService::NoSubService;
		followMeDirection = FollowMeDirection::Stop;
	}
};