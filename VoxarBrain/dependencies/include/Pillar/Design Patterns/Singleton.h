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

#ifndef __Singleton_H__
#define __Singleton_H__

namespace DesignPatterns
{
	/**
	  * Template class for implement the Singleton Design Pattern.	  
	  *	@code
	  *		#include <Singleton.h>
      *
	  *		class Object : public DesignPatterns::Singleton< Object >
	  *		{
	  *		private:
	  *			friend class DesignPatterns::Singleton< Object >;
	  *			
	  *			...
	  *		};
	  *	@endcode
	  * @tparam T Type to be used as Singleton.
	  */
	template <typename T>
	class Singleton
	{
	protected:

		/** \cond PRIVATE */
		/**
		 * Default constructor.
		 */
		Singleton( void )
		{
		}

		/**
		 * Default destructor.
		 */
		virtual ~Singleton( void )
		{
		}

		/**
		 * Method that creates the Singleton instance.
		 * Internal.
		 * @return Singleton instance.
		 */
		static T* internalInstance( void )
		{
			static T* inst = new T();
			return inst;
		}
		/** \endcond */

	public:

		/**
		 * Static method that returns a reference to the Singleton instance.
		 * @return Singleton reference.
		 */
		static T& getInstanceRef( void )
		{
			return *internalInstance();
		}

		/**
		 * Static method that returns a pointer to the Singleton instance.
		 * @return Singleton pointer.
		 */
		static T* getInstancePtr( void )
		{
			return internalInstance();
		}
	};
}
#endif