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

#ifndef __Factory_H__
#define __Factory_H__

namespace DesignPatterns
{
	/**
	* Template class that implements the Object Factory Design Pattern.
	* @tparam ObjectType The object base type created by the factory.
	* @tparam KeyType The key type used to store the object creators.
	*/
	template<typename ObjectType, typename KeyType>
	class Factory
	{
	private:

		/**
		* std::map that contains all object creators used by the factory.<br>
		* The KeyType represents the type used to index all the creators. The Any object is used to hold the actual functor used as creator.
		*/
		std::map<KeyType, TypeTools::Any> creators;

	public:

		/**
		* Method that adds a creator to the factory.
		* @param key The key used to index the new creator.
		* @param creator The function pointer used by the factory to create objects.
		*/
		template<typename CreatorType>
		void addCreator(KeyType key, CreatorType creator)
		{
			typedef FunctorType<CreatorType>::Result FunctorType;

			if(this->creators.count(key) == 0)
				this->creators.insert(std::pair<KeyType, FunctorType >(key, creator));
			else
				throw LibTools::Exception<KeyType>(key, "There is a creator identified by this key");
		}

		/**
		* Method that adds a creator to the factory.
		* @param key The key used to index the new creator.
		* @param caller The object pointer used to call the creator held by the factory.
		* @param creator The method pointer used by the factory to create objects.
		*/
		template<typename CreatorCaller, typename CreatorType>
		void addCreator(KeyType key, CreatorCaller* caller, CreatorType creator)
		{
			typedef FunctorType<CreatorType>::Result FunctorType;

			if(this->creators.count(key) == 0)
				this->creators.insert(std::pair<KeyType, FunctorType >(key, FunctorType(caller, creator)));
			else
				throw LibTools::Exception<KeyType>(key, "There is a creator identified by this key");
		}

		/**
		* Method that removes a creator.
		* @param key The used to index the creator to be removed.
		*/
		void removeCreator(KeyType key)
		{
			this->creators.erase(key); 			
		}

		/**
		* Method that creates an object instance.
		* @tparam ReturnType The created object type.
		* @param key The key used to index the creator.
		* @param args... The creator arguments.
		* @return The object created.
		*/
		template<typename ReturnType, typename ... Args>
		ReturnType create(KeyType key, Args& ... args)
		{
			if(this->creators.count(key))
				return ((Functor<ReturnType, typename std::decay<Args>::type...>&)this->creators[key])(args...);
			else
				throw LibTools::Exception<KeyType>(key, "There is no creator identified by this key");
		}

		/**
		* Method that creates an object instance.
		* @param key The key used to index the creator.
		* @param args... The creator arguments.
		* @return The object created.
		*/
		template<typename ... Args>
		ObjectType create(KeyType key, Args& ... args)
		{
			if(this->creators.count(key))
				return ((Functor<ObjectType, typename std::decay<Args>::type...>&)this->creators[key])(args...);
			else
				throw LibTools::Exception<KeyType>(key, "There is no creator identified by this key");
		}
	};
}

#endif