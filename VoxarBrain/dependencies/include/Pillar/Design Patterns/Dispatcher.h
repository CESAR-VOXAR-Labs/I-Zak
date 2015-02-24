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

#ifndef __Dispatcher_H__
#define __Dispatcher_H__

namespace DesignPatterns
{
	/** \cond PRIVATE */
	namespace Private
	{
		template<typename EventType>
		class EventHolder
		{
		public:
			int priority;
			EventType evt;

			EventHolder(const EventType& evt, int priority) : evt(evt), priority(priority)
			{}

			bool operator<(const EventHolder<EventType>& evt) const
			{
				return this->priority < evt.priority;
			}
		};

		class EventDispatcherInterface
		{
		public:

			virtual void dispatch() = 0;
		};

		template<typename EventType, typename ObservableType>
		class EventDispatcher : public EventDispatcherInterface
		{
		private:

			std::priority_queue<EventHolder<EventType>> events;

		public:

			typedef EventDispatcher<EventType, ObservableType> Self;

			ObservableType* observable;

			EventDispatcher() : observable(0)
			{}

			void schedule(const EventType& evt, int priority)
			{
				this->events.push(EventHolder<EventType>(evt, priority));
			}

			void dispatch()
			{
				observable->notify(this->events.top().evt);
				this->events.pop();
			}
		};

		template<typename ... Events>
		class EventScheduler : public Observable< Events... >,
							   public EventDispatcher< Events, Observable<Events...> >...
		{
		private:

			class DispatcherHolder
			{
			public:
				int priority;
				EventDispatcherInterface* dispatcher;

				DispatcherHolder(EventDispatcherInterface* dispatcher, int priority) : dispatcher(dispatcher), priority(priority)
				{}

				bool operator<(const DispatcherHolder& dispatcherHolder) const
				{
					return this->priority < dispatcherHolder.priority;
				}
			};

			template<typename ... Ts>
			class Init;

			template<typename T, typename ... U>
			class Init<T, U...>
			{
			public:

				template<typename FullType>
				static void init(EventDispatcher<T, Observable<Events...>>* dispatcher, FullType* full)
				{
					dispatcher->observable = full;
					Init<U...>::init(full, full);
				}
			};

			template<typename T>
			class Init<T>
			{
			public:

				template<typename FullType>
				static void init(EventDispatcher<T, Observable<Events...>>* dispatcher, FullType* full)
				{
					dispatcher->observable = full;
				}
			};

			std::priority_queue<DispatcherHolder> dispatchers;

			LibTools::Mutex mutex;
			LibTools::Semaphore semaphore;

			bool stopFlag;
			
		public:

			EventScheduler() : stopFlag(false)
			{
				Init<Events...>::init(this, this);
			}

			template<typename EventType>
			void schedule(const EventType& evt, int priority = 0)
			{
				this->mutex.lock();

				EventDispatcher<EventType, Observable<Events...>>* dispatcher = (EventDispatcher<EventType, Observable<Events...>>*)this;

				dispatcher->schedule(evt, priority);

				this->dispatchers.push(DispatcherHolder((EventDispatcherInterface*)dispatcher, priority));

				this->mutex.unlock();
				this->semaphore.notify();
			}

			void dispatchAll()
			{
				this->semaphore.wait();
				this->mutex.lock();

				while (!this->dispatchers.empty())
				{
					EventDispatcherInterface* i = this->dispatchers.top().dispatcher;
					i->dispatch();
					this->dispatchers.pop();
				}

				this->semaphore.clear();
				this->mutex.unlock();
			}

			void stop()
			{
				this->stopFlag = true;
			}

			void run()
			{
				while (!this->stopFlag)
				{
					this->dispatchAll();
				}

				this->stopFlag = false;
			}
		};
	}
	/** \endcond */

	/**
	* Template class that implements the Dispatcher Design Pattern (using Observable as base class).<br>
	* An event scheduler is a thread responsible for dispatch events to their respectives observers following a priority order.
	* @tparam Events... Variadic type pack of event types that can be scheduled by the scheduler.
	*/
	template<typename ... Events>
	class EventScheduler : public LibTools::Thread< Private::EventScheduler<Events...> >
	{
	private:

		typedef LibTools::Thread< Private::EventScheduler<Events...> > Parent;

	public:

		/**
		* Schedules an event into the schedule queue.
		* @param evt The event to be scheduled.
		* @param priority The event priority. As higher the priority, sooner the event will be dispatched.
		*/
		template<typename EventType>
		void schedule(const EventType& evt, int priority = 0)
		{
			Parent::schedule(evt, priority);
		}

		/**
		* Stops the event scheduler.
		*/
		void stop()
		{
			Parent::stop();
		}

		/**
		* Method that registers a new EventType's Observer to the EventScheduler instance.
		* @param observer The new observer to be registered.
		*/
		template<typename EventType>
		void attach(Observer<EventType>* observer)
		{
			Parent::attach(observer);
		}

		/**
		* Method that unregister the EventType's Observer from the EventScheduler instance.
		* @param observer The observer to be unregistered.
		*/
		template<typename EventType>
		void detach(Observer<EventType>* observer)
		{
			Parent::detach(observer);
		}
	};
}

#endif