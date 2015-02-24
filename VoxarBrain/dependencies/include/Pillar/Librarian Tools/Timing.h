/**
* The Pillar Library
* Copyright (c) 2013 by Voxar Labs.-UFPE
* Permission to use, copy, modify, distribute and sell this software for any
*     purpose is hereby granted without fee, provided that the above copyright
*     notice appear in all copies and that both that copyright notice and this
*     permission notice appear in supporting documentation.
* The author makes no representations about the
*     suitability of this software for any purpose. It is provided "as is"
*     without express or implied warranty.
*/
#ifndef __Timing_H__
#define __Timing_H__

namespace LibTools
{
	class Timer
	{
	private:

		std::chrono::high_resolution_clock::time_point begin;
		std::chrono::high_resolution_clock::time_point end;

		std::chrono::nanoseconds::rep nanoseconds;

		bool started;

	public:

		Timer() : nanoseconds(0), started(false) {}

		void start()
		{
			if (this->started)
				return;

			this->started = true;
			this->begin = std::chrono::high_resolution_clock::now();			
		}

		void stop()
		{
			if (!this->started)
				return;

			this->end = std::chrono::high_resolution_clock::now();
			this->nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
			
			this->started = false;
		}

		unsigned long long getElapsedTime()
		{
			if (this->started)
			{
				this->end = std::chrono::high_resolution_clock::now();
				this->nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(this->end - this->begin).count();
			}
			
			return this->nanoseconds;
		}
	};
}

#endif