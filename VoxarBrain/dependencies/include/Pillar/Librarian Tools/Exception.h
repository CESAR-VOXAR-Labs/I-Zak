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
#ifndef __Exception_H__
#define __Exception_H__

namespace LibTools
{
	/**
	* Template exception class.
	*/
	template< typename T >
	class Exception : public std::exception
	{
	private:

		T obj;
		std::string message;

	public:
		
		/**
		* Constructor.
		* @param obj The object responsible for the exception thrown.
		* @param message Exception's message.
		* @return Exception<T> instance.
		*/
		Exception(const T& obj, const std::string& message) : obj(obj), message(message){}
		
		/**
		* Default destructor.
		*/
		virtual ~Exception(){}

		/**
		* Method that returns the object responsible for the exception thrown.
		* @return T The object.
		*/
		T& getObject() const
		{
			return this->obj;
		}

		/**
		* Method that returns the exception's message.
		* @return T The message.
		*/
		virtual const char* what() const throw()
		{
			return this->message.c_str();
		}
	};
}
#endif