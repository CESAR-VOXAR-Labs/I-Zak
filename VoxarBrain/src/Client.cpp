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

#include <Pillar\Pillar.h>
#include "Client.h"

Client::Client()
{
	this->peer = new LibTools::Peer("192.168.1.65", 49001);
	//this->peer = new LibTools::Peer("127.0.0.1", 12345);
	this->peer->open();
}

void Client::sendCommand(unsigned char* message, unsigned int length)
{
	this->peer->send(message, length);
}

void Client::sendAudio(char *text) {
	int length = strlen(text);
	unsigned char *audioMessage = new unsigned char[length + 1 + 8];

	audioMessage[0] = 0xFF;
	audioMessage[1] = 0xFD;
	audioMessage[2] = (length + 1) & 0x00ff;
	audioMessage[3] = ((length + 1) & 0xff00) >> 8;
	audioMessage[4] = 255 - ((length + 1) % 256); // CHECKSUM MESSAGE LENGTH
	audioMessage[5] = 0x00;
	audioMessage[6] = 0x41;

	memcpy(&audioMessage[7], text, length + 1);

	unsigned long sum = 0x00 + 0x41;
	for (int i = 0; i < length; i++) {
		sum += (unsigned char)text[i];
	}

	audioMessage[length + 8] = 255 - (sum % 256); // CHECKSUM DATA

	/*for (int i = 0; i < length + 1 + 8; i++) {
		printf("%03d\t%02x\t%c\n", audioMessage[i], audioMessage[i], audioMessage[i]);
	}*/

	this->peer->send(audioMessage, length + 1 + 8);

	delete[] audioMessage;
}