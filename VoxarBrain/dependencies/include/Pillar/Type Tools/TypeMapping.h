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
#ifndef __TypeMapping_H__
#define __TypeMapping_H__

namespace TypeTools
{
	/**
	* Struct used to map some integral constant into a unique type.
	* Useful when you need to get a specific type at compilation time.<br>
	* Usage (this struct must be specialized):
	* @code
	* template<>
	* struct TypeTools::IntToType< 20 >
	* {
	*	static const unsigned int value = 20;
	*	typedef bool Type;
	* };
	*
	* typedef TypeTools::IntToType< 20 > Foo;
	*
	* void main()
	* {
	*	//In this case, Foo::Type is "bool" and Foo::value is "20".
	*	std::cout << typeid(Foo::Type).name() << std::endl << Foo::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* bool
	* 20
	* @endcode
	* @tparam v Integral constant used to map a type.
	*/
	template <unsigned int v>
	struct IntToType
	{
		/** \cond PRIVATE */
		static const unsigned int value = v;
		typedef void Type;
		/** \endcond*/
	};

	/**
	* Maps some type into a unique integral constant.
	* Useful when you need to get a specific value at compilation time.<br>
	* Usage (this struct must be specialized):
	* @code
	* template<>
	* struct TypeTools::TypeToInt< const char* >
	* {
	*	typedef const char* Type;
	*	static const unsigned int value = 15;
	* };
	*
	* typedef TypeTools::TypeToInt< const char* > Foo;
	*
	* void main()
	* {
	*	//In this case, Foo::Type is "const char*" and Foo::value is "15".
	*	std::cout << typeid(Foo::Type).name() << std::endl << Foo::value << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* const char*
	* 15
	* @endcode
	* @tparam T Type used to map into an integral constant.
	*/
	template <typename T>
	struct TypeToInt
	{
		/** \cond PRIVATE */
		typedef T Type;
		static const unsigned int value = (unsigned int)(-1);
		/** \endcond*/
	};
	
	/**
	* Maps some type into another type.
	* Useful when you need to get a specific type at compilation time.<br>
	* Usage (this struct must be specialized):
	* @code
	* template<>
	* struct TypeTools::TypeToType< const char* >
	* {
	*	typedef const char* OriginalType;
	*	typedef int* MappedType;
	* };
	*
	* typedef TypeTools::TypeToType< const char* > Foo;
	*
	* void main()
	* {
	*	//In this case, Foo::OriginalType is "const char*" and Foo::MappedType is "int*".
	*	std::cout << typeid(Foo::OriginalType).name() << std::endl << typeid(Foo::MappedType).name() << std::endl;
	* }
	* @endcode
	* Output:
	* @code	
	* const char*
	* int*
	* @endcode
	* @tparam T Type used to map into another type.
	*/
	template <typename T>
	struct TypeToType
	{
		/** \cond PRIVATE */
		typedef T OriginalType;
		typedef void MappedType;
		/** \endcond*/
	};	
}// namespace TypeTools

 /**
  * \def      PILLAR_MAP_INT_TYPE(val, T)
  * \brief    Macro for defining a 2-way int-to-type association
  */
#define PILLAR_MAP_INT_TYPE(val, T) template<>										\
									struct TypeTools::IntToType<val>				\
									{												\
									public:											\
										static const unsigned int value = val;		\
										typedef T MappedType;						\
									};												\
																					\
									template<> 										\
									struct TypeTools::TypeToInt<T>					\
									{												\
									public:											\
										typedef T MappedType;						\
										static const unsigned int value = val;		\
									};												\
																			
 /**
  * \def PILLAR_MAP_TYPE_TYPE(T, U)
  * \brief Macro for defining a 2-way type-to-type association
  */
#define PILLAR_MAP_TYPE_TYPE(T, U)  template<>									\
									struct TypeTools::TypeToType<T>				\
									{											\
									public:										\
										typedef T OriginalType;					\
										typedef U MappedType;					\
									};											\
																				\
									template<> 									\
									struct TypeTools::TypeToType<U>				\
									{											\
									public:										\
										typedef U OriginalType;					\
										typedef T MappedType;					\
									};											
#endif // end file guardian