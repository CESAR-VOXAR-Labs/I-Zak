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

#ifndef __Reflexion_H__
#define __Reflexion_H__

namespace ArchPatterns
{
	/**
	* Enumeration that represents the member type (Ex: Attribute, Static Method)
	*/
	enum MemberInfoType
	{
		CONSTRUCTOR,
		ATTRIBUTE,
		METHOD,
		STATIC_ATTRIBUTE,
		STATIC_METHOD
	};

	/**
	* Class that holds a member instance.<br>
	* Used by the Reflectable class to manage the registered members.
	*/
	class MemberInfo
	{
	public:

		/** Member type flag. */
		MemberInfoType type;
		/** Member name. */
		std::string name;
		/** Member id. */
		unsigned int id;
		/** Member instance held by an Any object.*/
		TypeTools::Any member;

		/** Default constructor.*/
		MemberInfo()
		{
		}

		/**
		* Constructor.
		* @param type Member type flag.
		* @param member Member instance.
		* @param id Member id.
		* @param name Member name.
		*/
		template<typename MemberType>
		MemberInfo(MemberInfoType type, MemberType member, unsigned int id, const std::string& name) : member(member), type(type), id(id), name(name)
		{}

		bool operator==(const MemberInfo& other) const
		{
			return DataTools::StringTools::match(this->name, other.name) && this->member.getType() == other.member.getType();
		}
	};

	/**
	* Class used to implement a "pseudo-reflection" capability.<br>
	* As a pointer to a constructor cannot be held, the list of constructors is actually a list of static methods that work as constructors
	*/
	class Reflectable
	{
	private:

		/**
		* Loads all the members from other Reflectable instance.
		* @param other The other Reflectable instance.
		*/
		template<typename ReflectableType>
		void internalLoadMembersFrom(ReflectableType* other)
		{
			for (auto pairs : other->memberMap)
			{
				this->registerMember(pairs.second);
			}
		}

		/**
		* Loads all the members from all Reflectable instances.
		* @param reflectables Reflectable instance pack.
		*/
		template<typename ReflectableType, typename ... Reflectables>
		void internalLoadMembersFrom(ReflectableType* reflectable, Reflectables* ... reflectables)
		{
			this->internalLoadMembersFrom(reflectable);
			this->internalLoadMembersFrom(reflectables...);
		}

		template<typename ReflectableType>
		void internalUnloadMembersFrom(ReflectableType* other)
		{
			for (auto pairs : other->memberMap)
			{
				this->unregisterMember(pairs.second);
			}
		}

		template<typename ReflectableType, typename ... Reflectables>
		void internalUnloadMembersFrom(ReflectableType* reflectable, Reflectables* ... reflectables)
		{
			this->internalUnloadMembersFrom(reflectable);
			this->internalUnloadMembersFrom(reflectables...);
		}

	protected:

		/** Members map.*/
		std::multimap<std::string, MemberInfo> memberMap;

		/** Number of constructors. */
		unsigned int nbConstructors;
		/** List of constructors. */
		MemberInfo* constructors;

		/** Number of attributes. */
		unsigned int nbAttributes;
		/** List of attributes. */
		MemberInfo* attributes;

		/** Number of methods. */
		unsigned int nbMethods;
		/** List of methods. */
		MemberInfo* methods;

		/** Number of static attributes. */
		unsigned int nbStaticAttributes;
		/** List of static attributes. */
		MemberInfo* staticAttributes;

		/** Number of static methods. */
		unsigned int nbStaticMethods;
		/** List of static methods */
		MemberInfo* staticMethods;

		template<typename T>
		struct MemberHelper;

		template<typename ObjectType, typename ReturnType, typename ... Args>
		struct MemberHelper< ReturnType(ObjectType::*)(Args...) >
		{
			typedef ReturnType(ObjectType::*Method)(Args...);

			static void registerMember(Reflectable* instance, const Method& method, const std::string& name)
			{
				unsigned int id = instance->nbMethods;
				instance->nbMethods++;

				DesignPatterns::FunctorType<Method>::Result func((ObjectType*)instance, method);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::METHOD, func, id, name)));
			}
		};

		template<typename ObjectType, typename ReturnType, typename ... Args>
		struct MemberHelper< ReturnType(ObjectType::*)(Args...) const >
		{
			typedef ReturnType(ObjectType::*Method)(Args...) const;

			static void registerMember(Reflectable* instance, const Method& method, const std::string& name)
			{
				unsigned int id = instance->nbMethods;
				instance->nbMethods++;

				DesignPatterns::FunctorType<Method>::Result func((ObjectType*)instance, method);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::METHOD, func, id, name)));
			}
		};

		template<typename ObjectType, typename ReturnType, typename ... Args>
		struct MemberHelper< ReturnType(ObjectType::*)(Args...) volatile>
		{
			typedef ReturnType(ObjectType::*Method)(Args...) volatile;

			static void registerMember(Reflectable* instance, const Method& method, const std::string& name)
			{
				unsigned int id = instance->nbMethods;
				instance->nbMethods++;

				DesignPatterns::FunctorType<Method>::Result func((ObjectType*)instance, method);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::METHOD, func, id, name)));
			}
		};

		template<typename ObjectType, typename ReturnType, typename ... Args>
		struct MemberHelper< ReturnType(ObjectType::*)(Args...) const volatile >
		{
			typedef ReturnType(ObjectType::*Method)(Args...) const volatile;

			static void registerMember(Reflectable* instance, const Method& method, const std::string& name)
			{
				unsigned int id = instance->nbMethods;
				instance->nbMethods++;

				DesignPatterns::FunctorType<Method>::Result func((ObjectType*)instance, method);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::METHOD, func, id, name)));
			}
		};

		template<typename ObjectType, typename ReturnType>
		struct MemberHelper< ReturnType(ObjectType::*) >
		{
			typedef ReturnType(ObjectType::*Attribute);

			static void registerMember(Reflectable* instance, const Attribute& attribute, const std::string& name)
			{
				unsigned int id = instance->nbAttributes;
				instance->nbAttributes++;

				DesignPatterns::FunctorType<Attribute>::Result func((ObjectType*)instance, attribute);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::ATTRIBUTE, func, id, name)));
			}
		};

		template<typename ReturnType, typename ... Args>
		struct MemberHelper< ReturnType(*)(Args...) >
		{
			typedef ReturnType(*Method)(Args...);

			static void registerMember(Reflectable* instance, const Method& method, const std::string& name)
			{
				unsigned int id = instance->nbStaticMethods;
				instance->nbStaticMethods++;

				DesignPatterns::FunctorType<Method>::Result func(method);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::STATIC_METHOD, func, id, name)));
			}
		};

		template<typename ReturnType>
		struct MemberHelper< ReturnType(*) >
		{
			typedef ReturnType(*Atribbute);

			static void registerMember(Reflectable* instance, const Atribbute& attribute, const std::string& name)
			{
				unsigned int id = instance->nbStaticAttributes;
				instance->nbStaticAttributes++;

				DesignPatterns::FunctorType<Atribbute>::Result func(attribute);
				instance->memberMap.insert(std::pair< std::string, MemberInfo >(name, MemberInfo(MemberInfoType::STATIC_ATTRIBUTE, func, id, name)));
			}
		};

		/**
		* Registers a member (method, attribute, static method, static attribute or "constructor").
		* @param member The member to be registered.
		* @param name The member name.
		*/
		template<typename MemberType>
		void registerMember(const MemberType& member, const std::string& name)
		{
			MemberHelper<MemberType>::registerMember(this, member, name);
		}

		/**
		* Registers a member.
		* @param memberInfo The MemberInfo instance to be registered.
		*/
		void registerMember(const MemberInfo& memberInfo)
		{
			unsigned int id = 0;
			switch (memberInfo.type)
			{
				case MemberInfoType::ATTRIBUTE:
				{
					id = this->nbAttributes;
					this->nbAttributes++;
					break;
				}
				case MemberInfoType::METHOD:
				{
					id = this->nbMethods;
					this->nbMethods++;
					break;
				}
				case MemberInfoType::CONSTRUCTOR:
				{
					id = this->nbConstructors;
					this->nbConstructors++;
					break;
				}
				case MemberInfoType::STATIC_ATTRIBUTE:
				{
					id = this->nbStaticAttributes;
					this->nbStaticAttributes++;
					break;
				}
				case MemberInfoType::STATIC_METHOD:
				{
					id = this->nbStaticMethods;
					this->nbStaticMethods++;
					break;
				}
			}

			MemberInfo m = memberInfo;
			m.id = id;
			this->memberMap.insert(std::pair< std::string, MemberInfo >(m.name, m));
		}

		void unregisterMember(const MemberInfo& memberInfo)
		{
			for (auto it = this->memberMap.begin(); it != this->memberMap.end(); it++)
			{
				if (memberInfo == it->second)
				{
					switch (memberInfo.type)
					{
						case MemberInfoType::ATTRIBUTE:
						{
							this->nbAttributes--;
							break;
						}
						case MemberInfoType::METHOD:
						{
							this->nbMethods--;
							break;
						}
						case MemberInfoType::CONSTRUCTOR:
						{
							this->nbConstructors--;
							break;
						}
						case MemberInfoType::STATIC_ATTRIBUTE:
						{
							this->nbStaticAttributes--;
							break;
						}
						case MemberInfoType::STATIC_METHOD:
						{							
							this->nbStaticMethods--;
							break;
						}
					}

					this->memberMap.erase(it);
					return;
				}
			}
		}

		/**
		* Performs the Reflection initial configuration.<br>
		* This method must be called after register all desired members.
		*/
		void setup()
		{
			delete[] this->constructors;
			delete[] this->attributes;
			delete[] this->methods;
			delete[] this->staticAttributes;
			delete[] this->staticMethods;

			this->constructors = 0;
			this->attributes = 0;
			this->methods = 0;
			this->staticAttributes = 0;
			this->staticMethods = 0;

			if (nbConstructors)
				this->constructors = new MemberInfo[nbConstructors];

			if (nbAttributes)
				this->attributes = new MemberInfo[nbAttributes];

			if (nbMethods)
				this->methods = new MemberInfo[nbMethods];

			if (nbStaticAttributes)
				this->staticAttributes = new MemberInfo[nbStaticAttributes];

			if (nbStaticMethods)
				this->staticMethods = new MemberInfo[nbStaticMethods];

			for (auto entry : this->memberMap)
			{
				switch (entry.second.type)
				{
					case MemberInfoType::CONSTRUCTOR:
					{
						this->constructors[entry.second.id] = entry.second;
						break;
					}
					case MemberInfoType::ATTRIBUTE:
					{
						this->attributes[entry.second.id] = entry.second;
						break;
					}
					case MemberInfoType::METHOD:
					{
						this->methods[entry.second.id] = entry.second;
						break;
					}
					case MemberInfoType::STATIC_ATTRIBUTE:
					{
						this->staticAttributes[entry.second.id] = entry.second;
						break;
					}
					case MemberInfoType::STATIC_METHOD:
					{
						this->staticMethods[entry.second.id] = entry.second;
						break;
					}
				}
			}
		}

		template<typename ... Reflectables>
		void loadMembersFrom(Reflectables* ... reflectables)
		{
			this->internalLoadMembersFrom(reflectables...);
			this->setup();
		}

		template<typename ... Reflectables>
		void unloadMembersFrom(Reflectables* ... reflectables)
		{
			this->internalUnloadMembersFrom(reflectables...);
			this->setup();
		}

	public:

		/**
		* Constructor.
		* @param nbConstructors Number of "constructors" to be stored.
		* @param nbAttributes Number of attributes to be stored.
		* @param nbMethods Number of methods to be stored.
		* @param nbStaticAttributes Number of static attributes to be stored.
		* @param nbStaticMethods Number of static methods to be stored.
		*/
		Reflectable(const unsigned int nbConstructors = 0, const unsigned int nbAttributes = 0, const unsigned int nbMethods = 0,
			const unsigned int nbStaticAttributes = 0, const unsigned int nbStaticMethods = 0)
			: nbConstructors(nbConstructors), nbAttributes(nbAttributes), nbMethods(nbMethods),
			nbStaticAttributes(nbStaticAttributes), nbStaticMethods(nbStaticMethods),
			constructors(0), attributes(0), methods(0), staticMethods(0), staticAttributes(0)
		{
			if (nbConstructors)
				this->constructors = new MemberInfo[nbConstructors];

			if (nbAttributes)
				this->attributes = new MemberInfo[nbAttributes];

			if (nbMethods)
				this->methods = new MemberInfo[nbMethods];

			if (nbStaticAttributes)
				this->staticAttributes = new MemberInfo[nbStaticAttributes];

			if (nbStaticMethods)
				this->staticMethods = new MemberInfo[nbStaticMethods];
		}
		
		/**
		* Resturns the total of members registered.
		* @return The total of members registered.
		*/
		const unsigned int getMemberCount() const
		{
			return this->nbStaticMethods + this->nbStaticAttributes + this->nbMethods + this->nbAttributes + this->nbConstructors;
		}

		/**
		* Resturns the total of members registered with the same key.
		* @param name Key used to register the member.
		* @return The total of members registered with the same key.
		*/
		const unsigned int getMemberCount(const std::string& name) const
		{
			return this->memberMap.count(name);
		}
		
		/**
		* Resturns the stored member instance.
		* @param name Key used to register the member.
		* @return The iterator containing all members registered with the same key.
		*/
		auto getMember(const std::string& name) const -> decltype(memberMap.find(name))
		{
			return this->memberMap.find(name);
		}

		/**
		* Method for retrieving an attribute reference.
		* @tparam index Index of the attribute.
		* @return The attribute of type <i>AttributeType</i>.
		*/
		template<const unsigned int index, typename AttributeType>
		AttributeType& retrieveAttribute()
		{
			return ((DesignPatterns::Functor<AttributeType&>&)this->attributes[index].member)();
		}

		/**
		* Method for retrieving a static attribute reference.
		* @tparam index Index of the static attribute.
		* @return The static attribute of type <i>AttributeType</i>.
		*/
		template<const unsigned int index, typename AttributeType>
		AttributeType& retrieveStaticAttribute()
		{
			return ((DesignPatterns::Functor<AttributeType&>&)this->staticAttributes[index].member)();
		}

		/**
		* Method for retrieving a attribute (static or not) referenced by its name.
		* @param name The name of the attribute.
		* @return The attribute of type <i>AttributeType</i>.
		*/
		template<typename AttributeType>
		AttributeType& retrieveAttributeByName(const std::string& name)
		{
			return ((DesignPatterns::Functor<AttributeType&>&)this->memberMap.find(name)->second.member)();
		}

		/**
		* Method for invoking a stored constructor.
		* @tparam index Index of the constructor.
		* @return A new instance of the class cast to Reflectable*.
		*/
		template<const unsigned int index, typename ... Args>
		Reflectable* invokeConstructor(Args& ... args)
		{
			return ((DesignPatterns::Functor<Reflectable*, Args...>&)this->constructors[index].member)(args...);
		}

		/**
		* Method for invoking a stored constructor by its name.
		* @param name Constructor name.
		* @param args Constructor arguments.
		* @return A new instance of the class cast to Reflectable*.
		*/
		template<typename ... Args>
		Reflectable* invokeConstructorByName(const std::string& name, Args& ... args)
		{
			return ((DesignPatterns::Functor<Reflectable*, Args...>&)this->memberMap.find(name)->second.member)(args...);
		}

		/**
		* Method for invoking a stored method.
		* @tparam index Index of the method.
		* @param args Method arguments.
		* @return The method result of type <i>ReturnType</i>.
		*/
		template<const unsigned int index, typename ReturnType, typename ... Args>
		ReturnType invokeMethod(Args& ... args)
		{
			return ((DesignPatterns::Functor<ReturnType, Args...>&)this->methods[index].member)(args...);
		}

		/**
		* Method for invoking a stored static method.
		* @tparam index Index of the method.
		* @param args Method arguments.
		* @return The method result of type <i>ReturnType</i>.
		*/
		template<const unsigned int index, typename ReturnType, typename ... Args>
		ReturnType invokeStaticMethod(Args& ... args)
		{
			return ((DesignPatterns::Functor<ReturnType, Args...>&)this->staticMethods[index].member)(args...);
		}

		/**
		* Method for invoking a stored method (static or not) identified by its name.
		* @param name The method name
		* @param args Method arguments.
		* @return The method result of type <i>ReturnType</i>.
		*/
		template<typename ReturnType, typename ... Args>
		ReturnType invokeMethodByName(const std::string& name, Args& ... args)
		{
			return ((DesignPatterns::Functor<ReturnType, Args...>&)this->memberMap.find(name)->second.member)(args...);
		}
	};

#define PILLAR_QUOTE(name) #name
#define PILLAR_STR(macro) PILLAR_QUOTE(macro)

#define PILLAR_REFLECTABLE_MEMBER(member) registerMember(member, DataTools::StringTools::split(PILLAR_STR(member), "::")[1])
}
#endif