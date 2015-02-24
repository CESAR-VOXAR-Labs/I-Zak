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
#ifndef __TypePack_H__
#define __TypePack_H__

namespace TypeTools
{
	/**
	* The building block of type packs (type lists using variadic templates) of any length.<br>
	* All operators defined for type lists (including TypeTools::TypeList) can be applied on TypePack.<br>
	* Usage:
	* @code
	* typedef TypeTools::TypePack< std::string, double, int > FooPack;	
	*
	* void main()
	* {
	*	//In this case, FooPack::Head is "std::string" and FooPack::Tail::Tail is "int".
	*	std::cout << typeid(FooPack::Head).name() << std::endl; << typeid(FooPack::Tail::Tail).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* std::string
	* int
	* @endcode
	* @tparam U... Variadic set of types used to define the type pack.
	*/
	template<typename ... U>
	struct TypePack;
	
	/** \cond PRIVATE */
	template<typename T, typename ... U>
	struct TypePack<T, U...>
	{
		/** First element, a non type list type by convention. */
		typedef T Head;
		/** Second element, a non type list type by convention. */
		typedef TypePack<U...> Tail;
	};
	
	template<typename T>
	struct TypePack<T>
	{
		typedef T Head;
		typedef NullType Tail;
	};

	template<>
	struct TypePack<>
	{
		typedef NullType Head;
		typedef NullType Tail;
	};
	/** \endcond */

	/**
	* Used to convert a type list into a TypePack.<br>
	* Usefull when it is necessary to retrieve all types present in a type list.
	* Usage:
	* @code
	* typedef TypeTools::TypeList<int, TypeTools::TypeList<float, TypeTools::TypeList<char, TypeTools::NullType> > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypePack<int, float, char>".
	*	std::cout << typeid(TypeListToTypePack<FooList>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypePack<int, float, char>
	* @endcode
	* @tparam TList The type list to be converted.
	* @tparam EndType Optional end list type.
	*/
	template<typename TList, typename EndType = NullType>
	struct TypeListToTypePack
	{
	private:

		template<typename TPack, typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct FillTypePack;

		template<typename Tail, typename T>
		struct FillTypePack<TypePack<T>, Tail, true>
		{
			typedef typename FillTypePack<TypePack<T, typename Tail::Head>, typename Tail::Tail>::Result Result;
		};

		template<typename Tail, typename ... Ts>
		struct FillTypePack<TypePack<Ts...>, Tail, true>
		{
			typedef typename FillTypePack<TypePack<Ts..., typename Tail::Head>, typename Tail::Tail>::Result Result;
		};

		template<typename Tail, typename ... Ts>
		struct FillTypePack<TypePack<Ts...>, Tail, false>
		{
			typedef typename std::conditional< std::is_same<Tail, EndType>::value,
											   TypePack<Ts...>,
											   TypePack<Ts..., Tail>
											 >::type Result;
		};

	public:

		/** \cond PRIVATE */
		typedef typename FillTypePack<TypePack<typename TList::Head>, typename TList::Tail>::Result Result;
		/** \endcond */
	};

	template<unsigned int begin, unsigned int end, typename TPack>
	struct SliceTypePack
	{
	private:
		
		template<unsigned int end, typename TPack>
		struct TrimTypePackEnd
		{
		private:

			template<typename TPack1, typename TPack2, unsigned int length>
			struct CopyTo;

			template<typename ... Args1, typename Arg2, typename ... Args2, unsigned int length>
			struct CopyTo<TypePack<Args1...>, TypePack<Arg2, Args2...>, length>
			{
				typedef typename std::conditional< length == 0,
												   TypePack<Args1..., Arg2>,
												   typename CopyTo< TypePack<Args1..., Arg2>, TypePack<Args2...>, length - 1 >::Result
												 >::type Result;
			};

			template<typename TPack1, unsigned int length>
			struct CopyTo< TPack1, TypePack<>, length >
			{
				typedef TypePack<> Result;
			};

		public:

			typedef typename CopyTo<TypePack<>, TPack, end>::Result Result;

		};

		template<unsigned int begin, typename TPack>
		struct TrimTypePackBegin
		{
			typedef typename SublistAt<TPack, begin>::Result Result;
		};

	public:

		typedef typename TrimTypePackBegin<begin, typename TrimTypePackEnd<end, TPack>::Result >::Result Result;
	};

	template<typename TPack>
	struct TypePackToTuple;

	template<typename ... Args>
	struct TypePackToTuple< TypePack<Args...> >
	{
		typedef std::tuple< Args... > Result;
	};
	
	template<typename Tuple>
	struct TupleToTypePack;
	
	template<typename ... Args>
	struct TupleToTypePack< std::tuple< Args... > >
	{
		typedef TypePack< Args... > Result;
	};

	/**
	* The building block of value packs (a set of values using variadic templates) of any length.
	* @tparam values... Variadic integral value set.	
	*/
	template<unsigned int ... values>
	struct ValuePack{};

	/**
	* Used to retrieve a filled ValuePack with ordered <i>size</i> elements.<br>
	* Usage:
	* @code
	* typedef TypeTools::FillValuePack<5>::Result FooValues;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::ValuePack<0,1,2,3,4>".
	*	std::cout << typeid(FooValues).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* TypeTools::ValuePack<0,1,2,3,4>
	* @endcode
	* @tparam size The number of elements that must be inserted in the ValuePack.
	* @tparam VList Optional ValuePack to insert the values. The default is ValuePack<>
	*/	
	template<unsigned int size, typename VList = ValuePack<>>
	struct FillValuePack;

	/** \cond PRIVATE */
	template<unsigned int ... values, unsigned int size>
	struct FillValuePack<size, ValuePack<values...>>
	{
		typedef typename FillValuePack<size - 1, ValuePack< size - 1, values... >>::Result Result;
	};

	template<unsigned int ... values>
	struct FillValuePack<0, ValuePack<values...>>
	{
		typedef ValuePack<values...> Result;
	};
	/** \endcond */
	
	template<unsigned int first, unsigned int last, typename VList = ValuePack<>>
	struct IntervalToValuePack;

	template<unsigned int first>
	struct IntervalToValuePack<first, first, ValuePack<>>
	{
		typedef ValuePack<first> Result;
	};

	template<unsigned int ... values, unsigned int last>
	struct IntervalToValuePack<last, last, ValuePack<values...>>
	{
		typedef ValuePack<values..., last> Result;
	};

	template<unsigned int first, unsigned int last>
	struct IntervalToValuePack<first, last, ValuePack<>>
	{
		typedef typename IntervalToValuePack<first + 1, last, ValuePack< first >>::Result Result;
	};

	template<unsigned int ... values, unsigned int first, unsigned int last>
	struct IntervalToValuePack<first, last, ValuePack<values...>>
	{
		typedef typename IntervalToValuePack<first + 1, last, ValuePack< values..., first >>::Result Result;
	};
	
	template<typename CommandType>
	struct Decompose;

	template<typename ObjectType, typename ReturnType, typename ... Args>
	struct Decompose< ReturnType (ObjectType::*)(Args&...) >
	{
		typedef ReturnType ReturnType;
		typedef ObjectType ObjectType;
		typedef TypeTools::TypePack< Args... > ArgsPack;
	};
	
	template<typename ObjectType, typename ReturnType, typename ... Args>
	struct Decompose< ReturnType (ObjectType::*)(Args&...) const>
	{
		typedef ReturnType ReturnType;
		typedef ObjectType ObjectType;
		typedef TypeTools::TypePack< Args... > ArgsPack;
	};
	
	template<typename ObjectType, typename ReturnType, typename ... Args>
	struct Decompose< ReturnType (ObjectType::*)(Args&...) volatile>
	{
		typedef ReturnType ReturnType;
		typedef ObjectType ObjectType;
		typedef TypeTools::TypePack< Args... > ArgsPack;
	};
	
	template<typename ObjectType, typename ReturnType, typename ... Args>
	struct Decompose< ReturnType (ObjectType::*)(Args&...) const volatile>
	{
		typedef ReturnType ReturnType;
		typedef ObjectType ObjectType;
		typedef TypeTools::TypePack< Args... > ArgsPack;
	};
	
	template<typename ReturnType, typename ... Args>
	struct Decompose< ReturnType (*)(Args&...) >
	{
		typedef ReturnType ReturnType;
		typedef TypeTools::TypePack< Args... > ArgsPack;
	};
}  //namespace TypeTools

#endif //end file guardian