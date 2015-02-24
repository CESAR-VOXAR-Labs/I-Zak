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
#ifndef __Any_H__
#define __Any_H__

namespace TypeTools
{
	/**
	* Class used to hold a value/pointer of any type.
	* Useful for defining an "user data object" in libraries and frameworks.
	* <br>
	* Usage:
	* @code
	* class Foo
	* {
	*	public:
	*		Any userData;
	* };
	*
	* void main()
	* {
	*	Foo f;
	*   
	*	//assign an integer
	*	f.userData = 20;
	*	std::cout << (int)f.userData << std::endl;
	*
	*	//assign a string
	*	f.userData = std::string("just a string");
	*	std::cout << (std::string)f.userData << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* 20
	* just a string
	* @endcode
	*/
	class Any
	{
	private:
	
		/**Internal interface used to implement the concrete value/pointer's holder.*/
		class AnyImpl
		{
		public:
			
			virtual ~AnyImpl(){}

			virtual const type_info& getType() = 0;
		};
	
		/**Template concrete value/pointer's holder.*/
		template<typename T>
		class AnyHolder : public AnyImpl
		{
		public:

			T obj;

			AnyHolder(){}
	
			AnyHolder(const T& obj) : obj(obj){}
	
			virtual ~AnyHolder(){}

			const type_info& getType()
			{
				return typeid(T);
			}
		};

		/**Holder interface instance.*/
		std::shared_ptr<AnyImpl> anyImp;
	
	public:
	
		/**Default constructor.
		* @return Any instance.
		*/
		Any() : anyImp(0)
		{
		}
	
		/**
		* Constructor.
		* @tparam T The value/pointer type.
		* @param t The value/pointer instance to be held by Any.
		* @return Any instance.
		*/
		template<typename T>
		Any(T t) : anyImp(new AnyHolder<T>(t))
		{
		}
	
		/**Default destructor.*/
		virtual ~Any()
		{			
		}
		
		/**
		* Overloaded <i>operator ()</i> used to retrieve the held value/pointer.
		* @tparam T The lvalue instance type.
		* @return the value/pointer held.
		*/
		template<typename T>
		operator T& ()
		{
			if (!this->anyImp.get())
				this->anyImp = std::shared_ptr<AnyImpl>(new AnyHolder<T>());

			return static_cast< AnyHolder<T>* >(this->anyImp.get())->obj;			
		}
		
		/**
		* Overloaded <i>operator =</i> used to assign a new value/pointer to the Any instance
		* @tparam T The value/pointer type.
		* @param t Value/pointer to be held.
		* @return The updated Any instance.
		*/
		template<typename T>
		Any& operator=(T t)
		{
			/*if <i>t</i> has the same type of the held value/pointer, overwrite only the held value/pointer*/
			if(dynamic_cast< AnyHolder<T>* >(this->anyImp.get()))
			{
				static_cast< AnyHolder<T>* >(this->anyImp.get())->obj = t;
			}
			/*else, overwrite the whole holder instance.*/
			else
			{
				this->anyImp = std::shared_ptr<AnyImpl>(new AnyHolder<T>(t));				
			}

			return *this;
		}

		/**
		* Method that returns the held type's <a class="el" href="http://www.cplusplus.com/reference/typeinfo/type_info/">type_info</a>.
		* @return The held type's <a class="el" href="http://www.cplusplus.com/reference/typeinfo/type_info/">type_info</a>.
		*/
		const std::type_info& getType() const
		{
			return this->anyImp->getType();
		}
	};
}

#endif