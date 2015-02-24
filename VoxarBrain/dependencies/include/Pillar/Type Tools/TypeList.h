////////////////////////////////////////////////////////////////////////////////
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
#ifndef __TypeList_H__
#define __TypeList_H__

namespace TypeTools
{
	/**
	* The building block of type lists of any length.<br>
	* Usage:
	* @code
	* typedef TypeTools::TypeList< std::string, double > FooTail;
	* typedef TypeTools::TypeList< int, FooTail> FooList;
	*
	* void main()
	* {
	*	//In this case, FooList::Head is "int" and FooList::Tail::Tail is "double".
	*	std::cout << typeid(FooList::Head).name() << std::endl; << typeid(FooList::Tail::Tail).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* int
	* double
	* @endcode
	* @tparam T Head type.
	* @tparam U Tail type.
	*/
	template <class T, class U>		
	struct TypeList
	{
		/** First element, a non type list type by convention. */
		typedef T Head;
		/** Second element, can be another type list. */
		typedef U Tail;
	};	
	
	/**
	* Checks if a certain type <i>T</i> has a nested type called <i>Head</i>.<br>
	* Usage:
	* @code
	* struct Foo
	* {
	*	typedef char Head;
	* };
	* 
	* struct FooHeadLess
	* {
	*	typedef int NotAHead;
	* };
	*
	* typedef TypeTools::TypeList< std::string, std::map<int, const char*> > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "true", "false" and "true".
	*	std::cout << TypeTools::HasHead<Foo>::value << std::endl; << TypeTools::HasHead<FooHeadLess>::value << std::endl; << TypeTools::HasHead<FooList>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* true
	* false
	* true
	* @endcode
	* @tparam T The type to check the possess the <t>Head<t/> nested type.
	*/
	template<typename T>		
	struct HasHead
	{
	private:
		typedef char                      yes;
		typedef struct { char array[2]; } no;
	
		template<typename C> static yes test(typename C::Head*);
		template<typename C> static no  test(...);
	public:
		/** \cond PRIVATE */
		static const bool value = sizeof(test<T>(0)) == sizeof(yes);
		/** \endcond */
	};
	
	/**
	* Checks if a certain type <i>T</i> has a nested type called <i>Tail</i>.<br>
	* Usage:
	* @code
	* struct Foo
	* {
	*	typedef char Tail;
	* };
	* 
	* struct FooTailLess
	* {
	*	typedef int NotATail;
	* };
	*
	* typedef TypeTools::TypeList< std::string, std::map<int, const char*> > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "true", "false" and "true".
	*	std::cout << TypeTools::HasTail<Foo>::value << std::endl; << TypeTools::HasTail<FooTailLess>::value << std::endl; << TypeTools::HasTail<FooList>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* true
	* false
	* true
	* @endcode
	* @tparam T The type to check the possess the <t>Tail<t/> nested type.
	*/
	template<typename T>		
	struct HasTail
	{
	private:
		typedef char                      yes;
		typedef struct { char array[2]; } no;
	
		template<typename C> static yes test(typename C::Head*);
		template<typename C> static no  test(...);
	public:
		/** \cond PRIVATE */
		static const bool value = sizeof(test<T>(0)) == sizeof(yes);
		/** \endcond */
	};
	
	/**
	* Checks if a certain type <i>T</i> has nested types called <i>Head</i> and <i>Tail</i>.<br>
	* Usage:
	* @code
	* struct Foo
	* {
	*	typedef char Tail;
	* };
	* 
	* struct FooTailLess
	* {
	*	typedef int Head;
	* };
	*
	* typedef TypeTools::TypeList< std::string, std::map<int, const char*> > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "false", "false" and "true".
	*	std::cout << TypeTools::IsTypeList<Foo>::value << std::endl; << TypeTools::IsTypeList<FooTailLess>::value << std::endl; << TypeTools::IsTypeList<FooList>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* false
	* false
	* true
	* @endcode
	* @tparam T The type to check if it is a type list.
	*/
	template<typename T>
	struct IsTypeList
	{
	public:
		/** \cond PRIVATE */
		static const bool value = HasHead<T>::value && HasTail<T>::value;
		/** \endcond */
	};	
	
	/**
	* Computes the length of a type list.<br>
	* Usage:
	* @code
	* typedef TypeTools::TypeList< std::string, std::map<int, const char*> > FooList;
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::NullType> > FoolerList;
	* typedef TypeTools::TypeList< std::string, double > FoolestList;
	*
	* void main()
	* {
	*	//In this case, the output is "2", "2" and "1".
	*	std::cout << TypeTools::Length<FooList>::value << std::endl; << TypeTools::Length<FoolerList>::value << std::endl; << TypeTools::Length<FoolestList, double>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* 2
	* 2
	* 1
	* @endcode
	* @tparam TList The type list.
	* @tparam EndType Optional end list type.
	*/	
	template <typename TList, typename EndType = NullType> 
	struct Length
	{
	private:

		template<typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct TailLength
		{
			enum{ value = Length<Tail>::value };
		};

		template<typename Tail>
		struct TailLength<Tail, false>
		{
			enum{ value = (std::is_same<Tail, EndType>::value ? 0 : 1) };
		};		
		
	public:

		/** \cond PRIVATE */
		enum { value = 1 + TailLength<typename TList::Tail>::value };
		/** \endcond */
	};	
	
	/**
	* Finds the type at a given index in a type list.<br>
	* If it is passed an out-of-bounds index or a non type list, the result is a compile-time error.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "double".
	*	std::cout << typeid(TypeTools::TypeAt<FooList, 2>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* double
	* @endcode
	* @tparam TList The type list.
	* @tparam index The desired type index.
	*/
	template< typename TList, unsigned int index>
	struct TypeAt
	{
		/** \cond PRIVATE */
		//Error ('out of bounds' or 'not typelist') if TList does not have nested type Tail.
		typedef typename TypeAt<typename TList::Tail, index-1>::Result Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template< typename TList >
	struct TypeAt<TList, 0>
	{
	private:

		template<typename T, bool isTypeList>
		struct TListHead
		{
			typedef typename T::Head Result;
		};

		template<typename T>
		struct TListHead<T, false>
		{
			typedef T Result;
		};

	public:

		typedef typename TListHead<TList, IsTypeList<TList>::value>::Result Result;
	};

	/** \endcond */
	
	
	/**
	* Finds the type at a given index in a type list.<br>
	* If it is passed an out-of-bounds index or a non type list, the result is the <i>DefaultType</i>.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "double" and "void".
	*	std::cout << typeid(TypeTools::TypeAtNonStrict<FooList, 2>::Result).name() << std::endl;
	*	std::cout << typeid(TypeTools::TypeAtNonStrict<FooList, 20, void>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* double
	* void
	* @endcode
	* @tparam TList The type list.
	* @tparam index The desired type index.
	* @tparam DefaultType The type returned if it is passed an out-of-bounds index or a non type list.
	*/
	template< typename TList, unsigned int index, typename DefaultType = NullType>
	struct TypeAtNonStrict
	{
	private:

		//Auxiliar struct used to retrieve a type typename TypeAtNonStrict<Tail, index - 1, DefaultType>::Result,
		//if Tail is a typelist, or DefaultType, if it is not.
		template<typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct TailOrDefault
		{
			typedef typename TypeAtNonStrict<typename TList::Tail, index - 1, DefaultType>::Result Result;
		};

		template<typename Tail>
		struct TailOrDefault<Tail, false>
		{
			typedef typename std::conditional< index - 1 == 0,
											   Tail,
											   DefaultType
											 >::type Result;
		};

	public:
		/** \cond PRIVATE */
		//If index == 0, return TList::Head
		//Else if TList::Tail is typelist, execute TypeAtNonStrict over it
		//Else, return DefaultType
		typedef typename std::conditional< index == 0,
										   typename TList::Head,
										   typename TailOrDefault<typename TList::Tail>::Result
										 >::type Result;
		/** \endcond */
	};
	
	/**
	* Finds the sub list at a given index in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< double, TypeTools::NullType>".
	*	std::cout << typeid(TypeTools::SublistAt<FooList, 2>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< double, TypeTools::NullType>
	* @endcode
	* @tparam TList The type list.
	* @tparam index The desired sub list index.
	*/	
	template< typename TList, int index >
	struct SublistAt
	{
		/** \cond PRIVATE */
		//Error ('out of bounds' or 'not typelist') if TList does not have nested type Tail.
		typedef typename SublistAt<typename TList::Tail, index-1>::Result Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template< typename TList >
	struct SublistAt<TList, 0>
	{
	public:
	
		typedef TList Result;
	};
	/** \endcond */
	
	/**
	* Finds the sub list at a given index in a type list.<br>
	* If it is passed an out-of-bounds index or a non type list, the result is the <i>DefaultType</i>.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< double, TypeTools::NullType>" and "void".
	*	std::cout << typeid(TypeTools::SublistAtNonStrict<FooList, 2>::Result).name() << std::endl;
	*	std::cout << typeid(TypeTools::SublistAtNonStrict<FooList, 20, void>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< double, TypeTools::NullType>
	* void
	* @endcode
	* @tparam TList The type list.
	* @tparam index The desired sub list index.
	* @tparam DefaultType The type returned if it is passed an out-of-bounds index or a non type list.
	*/	
	template< typename TList, unsigned int index, typename DefaultType = NullType>
	struct SublistAtNonStrict
	{
	private:
		//Auxiliar struct used to retrieve a type typename SublistAtNonStrict<Tail, index - 1, DefaultType>::Result, if Tail is a typelist, or DefaultType, if it is not.
		template<typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct TailOrDefault
		{
			typedef typename SublistAtNonStrict<typename TList::Tail, index - 1, DefaultType>::Result Result;
		};

		template<typename Tail>
		struct TailOrDefault<Tail, false>
		{
			typedef typename std::conditional< index - 1 == 0,
											   Tail,
											   DefaultType
											 >::type Result;
		};

	public:
		/** \cond PRIVATE */
		//If index == 0, return TList
		//Else if TList::Tail is typelist, execute SublistAtNonStrict over it
		//Else, return DefaultType
		typedef typename std::conditional< index == 0,
										   typename TList,
										   typename TailOrDefault<typename TList::Tail>::Result
										 >::type Result;
		/** \endcond */
	};
	
	/**
	* Finds the index of a type in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "0" and "-1".
	*	std::cout << TypeTools::IndexOf<FooList, std::string>::value << std::endl;
	*	std::cout << TypeTools::IndexOf<FooList, char>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* 0
	* -1
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to locate in the type list.
	*/	
	template <class TList, class T>
	struct IndexOf
	{
	private:

		//Auxiliar struct used to retrieve a value typename IndexOf<Tail, T>::value, if Tail is a typelist, or -1, if it is not.
		template<typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct TailOrNegative
		{
			enum{ temp = IndexOf<Tail, T>::value };

			enum{ value = temp == -1 ? -1 : 1 + temp };
		};

		template<typename Tail>
		struct TailOrNegative<Tail, false>
		{
			enum{ value = (std::is_same<Tail, T>::value ? 0 : -1) };
		};

	public:

		/** \cond PRIVATE */
		enum{ value = (std::is_same<typename TList::Head, T>::value ?
					   0 : 
					   TailOrNegative<typename TList::Tail>::value)
		};
		/** \endcond */
	};
	
	/**
	* Finds the index of a sub list in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< std::map<int, const char*>, TypeTools::TypeList< double, TypeTools::NullType> > > FooList;	
	*
	* void main()
	* {
	*	//In this case, the output is "2" and "-1".
	*	std::cout << TypeTools::IndexOfSublist<FooList, TypeTools::TypeList< double, TypeTools::NullType>>::value << std::endl;
	*	std::cout << TypeTools::IndexOfSublist<FooList, char>::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* 2
	* -1
	* @endcode
	* @tparam TList The type list.
	* @tparam TSub The sub list to locate in the type list.
	*/
	template <class TList, class TSub>
	struct IndexOfSublist
	{
	private:

		//Auxiliar struct used to retrieve a value <i>typename IndexOfSublist<Tail, TSub>::value</i>, if <i>Tail</i> is a typelist, or -1, if it is not.
		template<typename Tail, bool isTypeList = IsTypeList<Tail>::value>
		struct TailOrNegative
		{
			enum{ temp = IndexOfSublist<Tail, TSub>::value };

			enum { value = temp == -1 ? (std::is_same<typename TList::Head, typename TSub::Head>::value &&
										 std::is_same<typename TList::Tail, typename TSub::Tail>::value) -1
								      : 1 + IndexOfSublist<Tail, TSub>::value };
		};

		template<typename Tail>
		struct TailOrNegative<Tail, false>
		{
			enum{ value = -1 };
		};

	public:

		/** \cond PRIVATE */
		enum{ value = (std::is_same<typename TList::Head, typename TSub::Head>::value &&
				       std::is_same<typename TList::Tail, typename TSub::Tail>::value ?
					   0 :
					   TailOrNegative<typename TList::Tail>::value)
		};
		/** \endcond */
	};
	
	/**
	* Appends a type or a type list to another.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::NullType > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > >".
	*	std::cout << typeid(TypeTools::Append<FooList, int>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > >
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to be appended.
	* @tparam EndType Optional end list type.
	*/
	template <typename TList, typename T, typename EndType = NullType>
	struct Append
	{
	private:

		template<typename Tail, typename EndType, bool isEndType = std::is_same<Tail, EndType>::value>
		struct TailToAppend
		{
			typedef TypeList< T, EndType > Result;
		};

		template<typename Tail, typename EndType>
		struct TailToAppend<Tail, EndType, false>
		{
			typedef TypeList<typename Tail::Head, typename TailToAppend<typename Tail::Tail, EndType>::Result> Result;
		};

	public:
		
		/** \cond PRIVATE */
		typedef TypeList< typename TList::Head, 
						  typename TailToAppend<typename TList::Tail, EndType>::Result
						> Result;
		/** \endcond */
	};
	
	/**
	* Erases the first occurrence, if any, of a type in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::NullType >".
	*	std::cout << typeid(TypeTools::Erase<FooList, int>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::NullType >
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to be erased.
	* @tparam EndType Optional end list type.
	*/
	template <typename TList, typename T, typename EndType = NullType>
	struct Erase
	{
		/** \cond PRIVATE */
		typedef typename std::conditional< std::is_same<typename TList::Head, T>::value,
										   typename TList::Tail,
										   TypeList<typename TList::Head, typename Erase<typename TList::Tail, T, EndType>::Result>
										 >::type Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <typename T, typename EndType>
	struct Erase<EndType, T, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond*/
	
	/**
	* Erases all occurrences, if any, of a type in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< int, TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::NullType >".
	*	std::cout << typeid(TypeTools::EraseAll<FooList, int>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::NullType >
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to be erased.
	* @tparam EndType Optional end list type.
	*/
	template <typename TList, typename T, typename EndType = NullType>
	struct EraseAll
	{
		/** \cond PRIVATE */
		typedef typename std::conditional< std::is_same<typename TList::Head, T>::value,
										   typename EraseAll<typename TList::Tail, T>::Result,
										   TypeList< typename TList::Head, typename EraseAll<typename TList::Tail, T, EndType>::Result >
										 >::type Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <typename T, typename EndType>
	struct EraseAll<EndType, T, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond */
	
	/**
	* Removes all duplicate types in a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< int, TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::NullType >".
	*	std::cout << typeid(TypeTools::NoDuplicates<FooList>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::NullType >
	* @endcode
	* @tparam TList The type list.
	*/
	template <typename TList, typename EndType = NullType>
	struct NoDuplicates
	{
	private:
	
		typedef typename NoDuplicates<typename TList::Tail, EndType>::Result L1;
		typedef typename Erase<L1, typename TList::Head, EndType>::Result L2;
		
	public:
		/** \cond PRIVATE */
		typedef TypeList<typename TList::Head, L2> Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <typename EndType>
	struct NoDuplicates<EndType, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond*/
	
	/**
	* Replaces the first occurrence, if any, of a type in a type list by another.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::TypeList< char, TypeTools::NullType > >".
	*	std::cout << typeid(TypeTools::Replace<FooList, int, char>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::TypeList< char, TypeTools::NullType > >
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to be replaced.
	* @tparam U The type to replace the first <i>T</i> found.
	* @tparam EndType Optional end list type.
	*/	
	template <typename TList, typename T, typename U, typename EndType = NullType>
	struct Replace
	{
		/** \cond PRIVATE */
		typedef typename std::conditional<
										  std::is_same<typename TList::Head, T>::value,
										  typename TypeList<U, typename TList::Tail>,
										  typename TypeList<typename TList::Head, typename Replace<typename TList::Tail, T, U, EndType>::Result>								
										 >::type Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <typename T, typename U, typename EndType>
	struct Replace<EndType, T, U, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond */
	
	/**
	* Replaces all occurrences, if any, of a type in a type list by another.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< int, TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< char, TypeTools::TypeList< std::string, TypeTools::TypeList< char, TypeTools::NullType > >".
	*	std::cout << typeid(TypeTools::ReplaceAll<FooList, int, char>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< char, TypeTools::TypeList< std::string, TypeTools::TypeList< char, TypeTools::NullType > >
	* @endcode
	* @tparam TList The type list.
	* @tparam T The type to be replaced.
	* @tparam U The type to replace all <i>T</i> found.
	* @tparam EndType Optional end list type.
	*/
	template <typename TList, typename T, typename U, typename EndType = NullType>
	struct ReplaceAll
	{
		/** \cond PRIVATE */
		typedef typename std::conditional<
										  std::is_same<typename TList::Head, T>::value,
										  typename TypeList<U, typename ReplaceAll< typename TList::Tail, T, U, EndType >::Result>,
										  typename TypeList<typename TList::Head, typename ReplaceAll<typename TList::Tail, T, U, EndType>::Result>
										 >::type Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <typename T, typename U, typename EndType>
	struct ReplaceAll<EndType, T, U, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond */
	
	/**
	* Reverses a type list.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< int, TypeTools::TypeList< std::string, TypeTools::NullType > > FooList;
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > >".
	*	std::cout << typeid(TypeTools::Reverse<FooList>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::string, TypeTools::TypeList< int, TypeTools::NullType > >
	* @endcode
	* @tparam TList The type list.
	*/
	template <typename TList, typename EndType = NullType>
	struct Reverse
	{
		/** \cond PRIVATE */
		typedef typename Append< typename Reverse<typename TList::Tail, EndType>::Result, 
								 typename TList::Head,
								 EndType
							   >::Result Result;
		/** \endcond */
	};

	/** \cond PRIVATE */
	template <template <typename...> class TList, typename T, typename EndType>
	struct Reverse< TList<T, EndType>, EndType >
	{
		typedef TypeList<T, EndType> Result;
	};

	template <template <typename...> class TList, typename T, typename EndType>
	struct Reverse< TList<T>, EndType >
	{
		typedef TypeList<T, EndType> Result;
	};
	/** \endcond */
	
	/**
	* Used to modify all types present in a type list with a given template class.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::NullType > FooList;
	*
	* tenmplate<typename T>
	* class Foo{};
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< Foo<std::string>, TypeTools::NullType >".
	*	std::cout << typeid(TypeTools::Apply<FooList, Foo>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< Foo<std::string>, TypeTools::NullType >
	* @endcode
	* @tparam TList The type list.
	* @tparam Modifier The modifier class to be applied.
	* @tparam EndType Optional end list type.
	*/
	template<typename TList, template <typename...> class Modifier, typename EndType = NullType>
	struct Apply
	{
		/** \cond PRIVATE */
		typedef TypeList< typename Modifier<typename TList::Head>, 
						  typename Apply<typename TList::Tail, Modifier, EndType>::Result
						> Result;
		/** \endcond */
	};
	
	/** \cond PRIVATE */
	template <template <typename...> class Modifier, typename EndType>
	struct Apply<EndType, Modifier, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond */
	
	/**
	* Used to modify all types present in a type list with a given template class, using it's <i>Result</i> nested type to replace the type list content.<br>
	* Usage:
	* @code	
	* typedef TypeTools::TypeList< std::string, TypeTools::NullType > FooList;
	*
	* tenmplate<typename T>
	* class Foo
	* {
	* public:
	*		typedef std::vector<T> Result;
	* };
	*
	* void main()
	* {
	*	//In this case, the output is "TypeTools::TypeList< std::vector<std::string>, TypeTools::NullType >".
	*	std::cout << typeid(TypeTools::ApplyResult<FooList, Foo>::Result).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code
	* TypeTools::TypeList< std::vector<std::string>, TypeTools::NullType >
	* @endcode
	* @tparam TList The type list.
	* @tparam Modifier The modifier class to be applied.
	* @tparam EndType Optional end list type.
	*/
	template<typename TList, template <typename...> class Modifier, typename EndType = NullType>
	struct ApplyResult
	{
		/** \cond PRIVATE */
		typedef TypeList< typename Modifier<typename TList::Head>::Result, 
						  typename ApplyResult<typename TList::Tail, Modifier, EndType>::Result
						> Result;
		/** \endcond */
	};

	/** \cond PRIVATE */
	template <template <typename...> class Modifier, typename EndType>
	struct ApplyResult<EndType, Modifier, EndType>
	{
		typedef EndType Result;
	};
	/** \endcond */
}  //namespace TypeTools

#endif //end file guardian