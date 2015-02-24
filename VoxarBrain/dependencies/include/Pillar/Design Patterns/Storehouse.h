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

#ifndef __Storehouse_H__
#define __Storehouse_H__

namespace DesignPatterns
{
	/**
	* Template class that combines the Pool and the Factory Design Patterns.
	* @tparam ObjectType The object base type created by the factory.
	* @tparam CreatorKeyType The key type used to store the object creators.
	* @tparam Keys... Variadic set of keys (Private::Key) used to index the stored objects.
	*/
	template<typename ObjectType, typename CreatorKeyType, typename ... Keys>
	class Storehouse : public Factory< ObjectType, CreatorKeyType >,
					   public Repository< ObjectType, Keys... >
	{
	public:


		/**
		* Method that calls the specific creator identified by the key to create an object.
		* @tparam ReturnType The created object type.
		* @param key The key used to index the creator.
		* @param args... The creator arguments.
		* @return The object created.
		*/
		template< typename ReturnType,  typename ... Args >
		ReturnType create(CreatorKeyType key, Args ... args)
		{
			return Repository::getById(Repository::insert(Factory::create(key, args...)));
		}

		/**
		* Method that calls the specific creator identified by the key to create an object.
		* @param key The key used to index the creator.
		* @param args... The creator arguments.
		* @return The object created.
		*/
		template< typename ... Args >
		ObjectType create(CreatorKeyType key, Args ... args)
		{
			return Repository::getById(Repository::insert(Factory::create(key, args...)));
		}
	};
}

#endif