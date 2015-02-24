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

#ifndef __Command_H__
#define __Command_H__

namespace DesignPatterns
{
	/** \cond PRIVATE */
	namespace Private
	{
		template<typename ... Args>
		class FunctorImpl;

		template<typename ReturnType, typename ... Args>
		class FunctorImpl<ReturnType, Args...>
		{
		public:
			virtual ReturnType operator()(Args& ... args) = 0;
			virtual ~FunctorImpl() {}
		};

		template<typename ReturnType>
		class FunctorImpl<ReturnType>
		{
		public:
			virtual ReturnType operator()() = 0;
			virtual ~FunctorImpl() {}
		};

		template<typename ReturnType>
		class FunctorHandler;

		template<typename ReturnType>
		class FunctorHandler< ReturnType(*) > : public FunctorImpl< typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
																			 ReturnType&,
																			 ReturnType
																		   >::type >
		{
		public:

			typedef ReturnType(*PtrToMember);
			PtrToMember ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(PtrToMember ptr) : ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : ptr_(fun.ptr_)
			{
			}

			typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
				ReturnType&,
				ReturnType
			>::type operator()()
			{
				return (*ptr_);
			}
		};

		template<typename ReturnType, typename ... Args>
		class FunctorHandler< ReturnType(*)(Args...) > : public FunctorImpl<ReturnType, typename std::decay<Args>::type...>
		{
		public:

			typedef ReturnType(*PtrToMember)(Args...);
			PtrToMember ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(PtrToMember ptr) : ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : ptr_(fun.ptr_)
			{
			}

			ReturnType operator()(typename std::decay<Args>::type&... args)
			{
				return (*ptr_)(args...);
			}

		};

		template<typename ReturnType, typename Object>
		class FunctorHandler< ReturnType(Object::*) > : public FunctorImpl< typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
																			 ReturnType&,
																			 ReturnType
																		   >::type >
		{
		public:

			Object* obj;
			typedef ReturnType(Object::*PtrToMember);
			PtrToMember ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(Object* obj, PtrToMember ptr) : obj(obj), ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : obj(fun.obj), ptr_(fun.ptr_)
			{
			}

			typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
				ReturnType&,
				ReturnType
			>::type operator()()
			{
				return (obj->*ptr_);
			}
		};

		template< typename ReturnType, typename Object, typename ... Args >
		class FunctorHandler< ReturnType(Object::*)(Args...) > : public FunctorImpl<ReturnType, typename std::decay<Args>::type... >
		{
		public:
			Object* obj;
			typedef ReturnType(Object::*PtrToMethod)(Args...);
			PtrToMethod ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(Object* obj, PtrToMethod ptr) : obj(obj), ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : obj(fun.obj), ptr_(fun.ptr_)
			{
			}

			ReturnType operator()(typename std::decay<Args>::type& ... args)
			{
				return (obj->*ptr_)(args...);
			}
		};

		template< typename ReturnType, typename Object, typename ... Args >
		class FunctorHandler< ReturnType(Object::*)(Args...) const > : public FunctorImpl<ReturnType, typename std::decay<Args>::type... >
		{
		public:
			Object* obj;
			typedef ReturnType(Object::*PtrToMethod)(Args...) const;
			PtrToMethod ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(Object* obj, PtrToMethod ptr) : obj(obj), ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : obj(fun.obj), ptr_(fun.ptr_)
			{
			}

			ReturnType operator()(typename std::decay<Args>::type& ... args)
			{
				return (obj->*ptr_)(args...);
			}
		};

		template< typename ReturnType, typename Object, typename ... Args >
		class FunctorHandler< ReturnType(Object::*)(Args...) volatile > : public FunctorImpl<ReturnType, typename std::decay<Args>::type... >
		{
		public:
			Object* obj;
			typedef ReturnType(Object::*PtrToMethod)(Args...) volatile;
			PtrToMethod ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(Object* obj, PtrToMethod ptr) : obj(obj), ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : obj(fun.obj), ptr_(fun.ptr_)
			{
			}

			ReturnType operator()(typename std::decay<Args>::type& ... args)
			{
				return (obj->*ptr_)(args...);
			}
		};

		template< typename ReturnType, typename Object, typename ... Args >
		class FunctorHandler< ReturnType(Object::*)(Args...) const volatile > : public FunctorImpl< ReturnType, typename std::decay<Args>::type... >
		{
		public:
			Object* obj;
			typedef ReturnType(Object::*PtrToMethod)(Args...) const volatile;
			PtrToMethod ptr_;

			FunctorHandler()
			{
			}

			FunctorHandler(Object* obj, PtrToMethod ptr) : obj(obj), ptr_(ptr)
			{
			}

			FunctorHandler(const FunctorHandler& fun) : obj(fun.obj), ptr_(fun.ptr_)
			{
			}

			ReturnType operator()(typename std::decay<Args>::type& ... args)
			{
				return (obj->*ptr_)(args...);
			}
		};
	}
	/** \endcond */

	/**
	* Template class that defines a Functor ("functions with a state"), used to implement the Command Design Pattern.
	* @tparam ReturnType The command return type.
	* @tparam Args... The command arguments.
	*/	
	template<typename ReturnType = void, typename ... Args>
	class Functor
	{
	private:

		//"Callable" object type definition.
		typedef Private::FunctorImpl<ReturnType, typename std::decay<Args>::type...> Impl;

		friend class Functor;

		//Attribute that holds the actual "callable" object.
		std::shared_ptr<Impl> spImpl_;

	public:

		/**
		* Default Constructor
		*/
		Functor()
		{
		}

		/**
		* Constructor that receives a function or static method pointer.
		* @param t Function or static method pointer.
		*/
		template<typename T>
		Functor(T t) : spImpl_(new Private::FunctorHandler<T>(t))
		{
		}

		/**
		* Constructor that receives an object pointer and a object's method pointer.
		* @param t Object pointer.
		* @param u Method pointer.
		*/
		template<typename T, typename U>
		Functor(T* t, U u) : spImpl_(new Private::FunctorHandler<U>(t, u))
		{
		}

		/**
		* Copy Constructor.
		*/
		Functor(const Functor& fun) : spImpl_(fun.spImpl_)
		{
		}

		/**
		* Overloaded assign operator.
		* @param fun Functor to be assigned.
		* @return Assigned functor reference.
		*/
		Functor& operator=(const Functor& fun)
		{
			this->spImpl_ = fun.spImpl_;
			return *this;
		}

		/**
		* Default destructor.
		*/
		virtual ~Functor()
		{
		}

		/**
		* Overloaded operator '()' used to call the respective "command",
		* passing as arguments elements respecting the types informed in the Functor definiton.
		* @return An element of ReturnType (also informed in the Functor definition).
		*/
		ReturnType operator()(typename std::decay<Args>::type& ... args)
		{
			return (*spImpl_)(args...);
		}
	};

	/**
	* Returns, base on a pointer (to attribute, to method or to function) type, an equivalent functor type.<br>
	* Usage:
	* @code
	* class Foo
	* {
	* public:
	* 	
	* 	const char* foo(double& d, const int& i)
	* 	{
	* 		return "foo";
	* 	}
	* };
	*
	* typedef typename FunctorType< decltype(&Foo::foo) >::Result FooFunctor;
	*
	* void main()
	* {
	*	//In this case, the output is "Functor< const char*, double, int >"
	*	std::cout << typeid(FooFunctor).name() << std::endl;
	* }
	* @endcode
	* @code
	* output: Functor< const char*, double, int >
	* @endcode
	*/
	template<typename T>
	struct FunctorType
	{
	private:
	
		template<typename T>
		struct MemberToTypePack;

		template< typename ReturnType >
		struct MemberToTypePack< ReturnType(*) >
		{
			typedef typename TypeTools::TypePack< typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
																			 ReturnType&,
																			 ReturnType
																		   >::type
												> Result;
		};
	
		template< typename ReturnType >
		struct MemberToTypePack< ReturnType(*)() >
		{
			typedef typename TypeTools::TypePack<ReturnType> Result;
		};
	
		template< typename ReturnType, typename ... Args >
		struct MemberToTypePack< ReturnType(*)(Args...) >
		{
			typedef typename TypeTools::TypePack<ReturnType, typename std::decay<Args>::type...> Result;
		};
	
		template< typename ReturnType, typename Object>
		struct MemberToTypePack< ReturnType(Object::*) >
		{
			typedef typename TypeTools::TypePack< typename std::conditional< !std::is_reference< ReturnType >::value && !std::is_pointer< ReturnType >::value,
																			 ReturnType&,
																			 ReturnType
																		   >::type
												> Result;
		};
	
		template< typename ReturnType, typename Object, typename ... Args >
		struct MemberToTypePack< ReturnType(Object::*)(Args...) >
		{
			typedef typename TypeTools::TypePack<ReturnType, typename std::decay<Args>::type...> Result;
		};
	
		template< typename ReturnType, typename Object, typename ... Args >
		struct MemberToTypePack< ReturnType(Object::*)(Args...) const >
		{
			typedef typename TypeTools::TypePack<ReturnType, typename std::decay<Args>::type...> Result;
		};
	
		template< typename ReturnType, typename Object, typename ... Args >
		struct MemberToTypePack< ReturnType(Object::*)(Args...) volatile >
		{
			typedef typename TypeTools::TypePack<ReturnType, typename std::decay<Args>::type...> Result;
		};
	
		template< typename ReturnType, typename Object, typename ... Args >
		struct MemberToTypePack< ReturnType(Object::*)(Args...) const volatile >
		{
			typedef typename TypeTools::TypePack<ReturnType, typename std::decay<Args>::type...> Result;
		};
	
		template<typename TPack>
		struct FunctorFromTypePack;
	
		template<typename Head, typename ... Tail>
		struct FunctorFromTypePack< TypeTools::TypePack< Head, Tail... > >
		{
			typedef Functor<Head, Tail...> Result;
		};
	
		template<typename Head>
		struct FunctorFromTypePack< TypeTools::TypePack< Head > >
		{
			typedef Functor<Head> Result;
		};
	
	public:
		/** \cond PRIVATE */
		typedef typename FunctorFromTypePack< 
												typename MemberToTypePack<T>::Result
											>::Result Result;
		/** \endcond */
	};	
}

#endif