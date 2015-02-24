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
#ifndef __Parser_H__
#define __Parser_H__

namespace DataTools
{
	/**
	 * Generic parser.<br>
	 * It should be specialized for each pair T - U.
	 * @tparam T Element type to be serialized.
	 * @tparam U Element type to be parsed.
	 */
	template <typename T, typename U>
	class Parser
	{
	public:

		/**
		 * Converts serialized data U into a element T.
		 * @param data The serialized data.
		 * @return The parsed element T.
		 */
		static T parse(const U& data);

		/**
		 * Converts an element T into serialized data U.
		 * @param element The element T.
		 * @return The serialized U.
		 */
		static U serialize(const T& element);
	};
	
	/**
	 * Generic Parser interface.<br>
	 * Useful when exist many ways of converting pairs T - U.
	 * @tparam T Element type to be serialized.
	 * @tparam U Element type to be parsed.
	 */
	 template<typename T, typename U>
	 class ParserInterface
	 {
	 public:
	 
		/**
		 * Converts serialized data U into a element T.
		 * @param data The serialized data.
		 * @return The parsed element T.
		 */
		virtual T parse(const U& data) = 0;

		/**
		 * Converts an element T into serialized data U.
		 * @param element The element T.
		 * @return The serialized element U.
		 */
		virtual U serialize(const T& element) = 0;
	 };
}
#endif