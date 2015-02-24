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

#ifndef __Observer_H__
#define __Observer_H__

namespace DesignPatterns
{
	/** \cond PRIVATE */
	namespace Private
	{
		template< typename EventType >
		class ObservableHandler;
	}
	/** \endcond  */

	/**
	* Template class responsible for implement the Observer object of Observer Design Pattern.
	* @tparam EventType... Events to be observed by the class.
	*/
	template< typename EventType >
	class Observer
	{
	private:

		/** \cond PRIVATE */

		friend class Private::ObservableHandler< EventType >;

		/** Detach Command used to notify the Observable that the observer has been destroyed. */
		DesignPatterns::Functor<void, Observer*>* detachCommand;

		/** \endcond */

	protected:
		
		/** \cond PRIVATE */

		/**
		* Default Destructor
		*/
		virtual ~Observer()
		{
			Observer* obs = this;
			//if there is a <i>detachCommand</i>, execute it before destroy the object.
			if(detachCommand)
				(*(this->detachCommand))(obs);
		}

		/** \endcond */

	public:

		/**
		* Virtual method that notifies the observer an EventType event occurrence.
		* @param evt The event that has been occurred.
		*/
		virtual void observe(const EventType& evt) = 0;
	};

	/** \cond PRIVATE */
	namespace Private
	{
		/**
		* Template class used to defining an EventType's Observable class of Observer Design Pattern.
		*/
		template< typename EventType >
		class ObservableHandler
		{
		private:
	
			friend class Observer< EventType >;
	
			/** List of registered EventType's observers */
			std::set< Observer< EventType >* > observers;
	
			/** Detach Command used by the registered observers to notify its destruction to the Observable */
			DesignPatterns::Functor< void, Observer< EventType >* >* detachCommand;
	
		public:

			typedef ObservableHandler<EventType> Self;
	
			/**
			* Default Constructor
			*/
			ObservableHandler() : detachCommand(new DesignPatterns::Functor< void, Observer<EventType>* >(this, &ObservableHandler<EventType>::dettach))
			{
			}
	
			/**
			* Method that registers a new EventType's Observer to the Observable instance.
			* @param observer The new observer to be registered.
			*/
			void attach(Observer< EventType >* observer)
			{
				observer->detachCommand = this->detachCommand;
				this->observers.insert(observer);
			}
			
			/**
			* Method that unregistered the EventType's Observer from the Observable instance.
			* @param observer The observer to be unregistered.
			*/
			void dettach(Observer< EventType >* observer)
			{
				observer->detachCommand = 0;
				this->observers.erase( this->observers.find(observer) );
			}
	
			/**
			* Method that notifies all registered observers the occurrence of a EventType event.
			* @param evt The event that just occurred.
			*/
			void notify(const EventType& evt)
			{
				for (Observer< EventType >* observer : this->observers)
				{
					observer->observe(evt);
				}
			}
		};
	}
	/** \endcond */

	/**
	* Template class used to defining an Observable class of Observer Design Pattern.
	* @tparam EventTypes... Events to be notified by the class.
	*/
	template<typename ... EventTypes>
	class Observable : public Private::ObservableHandler<EventTypes>...
	{
	public:

		/**
		* Method that registers a new EventType's Observer to the Observable instance.
		* @param observer The new observer to be registered.
		*/
		template<typename EventType>
		void attach(Observer<EventType>* observer)
		{
			((Private::ObservableHandler<EventType>*)this)->attach(observer);
		}

		/**
		* Method that unregister the EventType's Observer from the Observable instance.
		* @param observer The observer to be unregistered.
		*/
		template<typename EventType>
		void detach(Observer<EventType>* observer)
		{
			((Private::ObservableHandler<EventType>*)this)->detach(observer);
		}

		/**
		* Method that notifies all registered observers the occurrence of a EventType event.
		* @param evt The event that just occurred.
		*/
		template<typename EventType>
		void notify(const EventType& evt)
		{
			((Private::ObservableHandler<EventType>*)this)->notify(evt);
		}
	};
}

#endif