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
#ifndef __Threading_H__
#define __Threading_H__

#if (_MSC_VER == 1800)
	using namespace std;
#else
	using namespace std::cv_status;
#endif

namespace LibTools
{
	/**
	* Standard mutex type abstraction.
	*/
	typedef std::mutex Mutex;
	
	/**
	* Standard unique_lock type abstraction.
	*/
	typedef std::unique_lock<std::mutex> MutexHold;

	/**
	* Limits the number of threads that can access a resource or pool of resources concurrently.
	*/
	class Semaphore
	{
	private:

		Mutex mtx;
		std::condition_variable cv;
		unsigned int count;

	public:

		/**
		* Constructor.
		* @param count_ The number of ticks necessary for the semaphore releases the resource access.
		*/
		Semaphore(unsigned int count_ = 0) :count(count_){}

		/**
		* Notifies the semaphore that the thread is no longer using the shared resource.
		* @param count_ The number of ticks held by the thread.
		*/
		void notify(unsigned int count_ = 1)
		{
			MutexHold lck(mtx);
			count += count_;
			cv.notify_one();
		}

		/**
		* Locks the current thread and waits a certain number of available ticks to continue.
		* @param count_ The number of ticks required for the thread.
		*/
		void wait(unsigned int count_ = 1)
		{
			MutexHold lck(mtx);

			while (count < count_)
			{
				cv.wait(lck);
			}
			count -= count_;
		}

		/**
		* Locks the current thread and waits a certain number of available ticks or a amount of time to continue.
		* @param ms The thread lock time (in milliseconds).
		* @param count_ The number of ticks required for the thread.
		* @return True if the semaphore released the number of ticks required. False if the time has ended.
		*/
		bool wait(std::chrono::milliseconds ms, unsigned int count_ = 1)
		{
			MutexHold lck(mtx);

			cv_status status = std::cv_status::no_timeout;

			while (count < count_)
			{
				std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

				status = cv.wait_for(lck, ms);

				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

				ms -= std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

				if (status == std::cv_status::timeout || ms.count() <= 0)
					return false;
			}
			count -= count_;

			return true;
		}

		/**
		* Resets the semaphore instance.
		*/
		void clear()
		{
			MutexHold lck(mtx);
			this->count = 0;
		}
	};

	/**
	* Defines an abstraction over std::thread.<br>	
	* Usage:
	* @code
	* class Foo
	* {
	*	public:
	*		void run()
	*		{
	*			//some code...
	*		}
	* }
	*
	* void main()
	* {
	*	LibTools::Thread<Foo> fooThread;
	*	fooThread.start();
	* }
	* @endcode
	* @tparam T Runnable type.
	* @tparam PtrToRun Runnable method pointer type. 
	* @tparam runCore Runnable method pointer instance.
	*/
	template<typename T, typename PtrToRun = decltype(&T::run), PtrToRun runCore = &T::run>
	class Thread : public T
	{
	private:

		std::thread thread_;

	public:

		/**
		* Constructor.		
		*/
		template<typename ... Args>
		Thread(Args& ... args) : T(args...)
		{
		}

		/**
		* Starts the thread instance.
		*/
		template<typename ... Args>
		void start(Args& ... args)
		{
			this->thread_ = std::thread(runCore, this, std::ref(args)...);
		}

		/**
		* Calls the std::thread::detach method.
		*/
		void detach()
		{
			this->thread_.detach();
		}

		/**
		* Calls the std::thread::join method.
		*/
		void join()
		{
			if (this->thread_.joinable())
				this->thread_.join();
		}

		/**
		* Calls the std::thread::get_id method.
		* @return The thread id.
		*/
		std::thread::id getId() const
		{
			return this->thread_.get_id();
		}

		/**
		* Calls the std::thread::joinable method.
		* @return True if it is joinable.
		*/
		bool isJoinable() const
		{
			return this->thread_.joinable();
		}

		/**
		* Calls the std::thread::native_handle method.
		* @return The thread handle.
		*/
		std::thread::native_handle_type getHandle()
		{
			return this->thread_.native_handle();
		}
	};
}

#endif