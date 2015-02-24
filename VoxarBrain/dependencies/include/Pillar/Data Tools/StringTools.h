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
#ifndef __StringTools_H__
#define __StringTools_H__

#ifdef _MSC_VER
#pragma warning( push ) 
#pragma warning( disable : 4996 )
#endif

namespace DataTools
{
	/**
	* String related tools.
	*/
	class StringTools
	{
	public:
		/**
		* Gets whether a given string matches a pattern.<br>
		* When wildcards are not present, source and pattern can only match if they have the same size.
		* @param source The given string.
		* @param pattern The pattern. It allows wildcards.
		* @param caseSensitive Are source and pattern case-sensitive?
		* @return Does the source match the pattern?
		*/
		static bool match(const std::string& source, const std::string& pattern, bool caseSensitive = true)
		{
			if (source != pattern)
			{
				std::string fSource, fPattern;
				if (caseSensitive) 
				{
					fSource = source;
					fPattern = pattern;
				}
				else
				{
					fSource = toLower(source);
					fPattern = toLower(pattern);
				}
				if (pattern.find('*') == std::string::npos)  // If a wildcard is not found in source
				{
					if (fSource != fPattern) 
					{
						return false;
					}
				} 
				else
				{
					std::vector<std::string> tokens;
					split(fPattern, "*", tokens);
					for (unsigned int i = 0, pos = 0; i < tokens.size(); i++) 
					{
						pos = fSource.find(tokens[i], pos);
						if (pos != std::string::npos) 
						{
							pos += tokens[i].size();
						}
						else
						{
							return false;
						}
					}
				}
			}
			return true;
		}

		/**
		* Finds a substring inside a given string and replaces it by another substring.
		* @param source The given string.
		* @param what The substring to find.
		* @param with The substring to replace with.
		* @param caseSensitive Are source and what case-sensitive?
		* @return The replaced string.
		*/
		static std::string replace(const std::string& source, const std::string& what, const std::string& with, bool caseSensitive = true)
		{
			unsigned int p2 = source.find(what);
			if (p2 == std::string::npos && caseSensitive) // If "what" is not found in source
			{
				return source;
			} 
			else
			{
				std::string fSource, fWhat;
				if (caseSensitive) 
				{
					fSource = source;
					fWhat = what;
				}
				else
				{
					fSource = toLower(source);
					fWhat = toLower(what);
				}
				std::stringstream ss;
				unsigned int p1 = 0;
				while (p2 != std::string::npos) 
				{
					ss << source.substr(p1, p2 - p1) << with;
					p1 = p2 + what.size();
					p2 = fSource.find(fWhat, p1);
				}
				ss << source.substr(p1);
				return ss.str();
			}
		}

		/**
		* Splits a string.<br>
		* For example, if source is "*ab**cd*", delim is "*", and tokens is empty, tokens will be {"ab", "cd"}.
		* @param source The given string.
		* @param delim The delimiter characters.
		* @param[out] tokens The token container.
		*/
		static void split(const std::string& source, const std::string& delim, std::vector<std::string>& tokens)
		{
			unsigned int p1 = source.find_first_not_of(delim), p2 = 0;
			while (p1 != std::string::npos) 
			{
				p2 = source.find_first_of(delim, p1);
				tokens.push_back(source.substr(p1, p2 - p1));
				p1 = source.find_first_not_of(delim, p2);
			}
		}
		
		/**
		* Splits a string.<br>
		* For example, if source is "*ab**cd*", delim is "*", tokens will be {"ab", "cd"}.
		* @param source The given string.
		* @param delim The delimiter characters.
		* @return The tokens.
		*/
		static std::vector<std::string> split(const std::string& source, const std::string& delim)
		{
			std::vector<std::string> tokens;
			
			unsigned int p1 = source.find_first_not_of(delim), p2 = 0;
			while (p1 != std::string::npos) 
			{
				p2 = source.find_first_of(delim, p1);
				tokens.push_back(source.substr(p1, p2 - p1));
				p1 = source.find_first_not_of(delim, p2);
			}
			
			return tokens;
		}

		/**
		* Converts every character in a string to lower case.
		* @param source The given string.
		* @return The string converted.
		*/
		static std::string toLower(const std::string& source)
		{
			std::string lower = source;
			for (unsigned int i = 0; i < source.size(); i++) 
			{
				lower[i] = (char)tolower(source[i]);
			}
			return lower;
		}

		/**
		* Converts every character in a string to upper case.
		* @param source The given string.
		* @return The string converted.
		*/
		static std::string toUpper(const std::string& source)
		{
			std::string upper = source;
			for (unsigned int i = 0; i < source.size(); i++) 
			{
				upper[i] = (char)toupper(source[i]);
			}
			return upper;
		}

		/**
		* Converts an element of type T to a string.<br>
		* The method uses std::stringstream's features.
		* @tparam T Element type to be converted.
		* @param t The element.
		* @return The element's string representation.
		*/
		template<class T>
		static std::string toString(T t)
		{
			std::stringstream ss;
			ss << t;
			return ss.str();
		}

		/**
		* Converts a double to a string.
		* @param t The double.
		* @param precision The number of decimal places that the double must be represented.
		* @return The string.
		*/
		static std::string toString(const double t, const unsigned int precision)
		{
			std::string s1;
			char resp[310];

			s1 = "%.";
			s1.append(toString(precision));
			s1.append("f\0");

			sprintf(resp, s1.c_str(), t);

			return resp;
		}

		/**
		* Converts a float to a string.
		* @param t The float.
		* @param precision The number of decimal places that the double must be represented.
		* @return The string.
		*/
		static std::string toString(const float t, const unsigned int precision)
		{
			std::string s1;
			char resp[310];

			s1 = "%.";
			s1.append(toString(precision));
			s1.append("f\0");

			sprintf(resp, s1.c_str(), t);

			return resp;
		}

		/**
		* Converts a string to an element of type T.<br>
		* The method uses std::stringstream's features.
		* @tparam T Element type to be returned.
		* @param s The string.
		* @return The converted element.
		*/
		template<class T>
		static T parseString(const std::string& s)
		{
			T t;
			std::stringstream ss;
			ss << s;
			ss >> t;
			return t;
		}

		/**
		* Trims a string.<br>
		* The blank characters are removed from the given string.
		* @param source The given string.
		* @param blank The blank characters.
		* @return The trimmed string.
		*/
		static std::string trim(const std::string& source, const std::string& blank = " ")
		{
			unsigned int p1 = source.find_first_not_of(blank);
			if (p1 != std::string::npos) 
			{
				unsigned int p2 = source.find_last_not_of(blank);
				return source.substr(p1, p2 - p1 + 1);
			}
			else
			{
				return "";
			}
		}
	};
}

#ifdef _MSC_VER
#pragma warning( pop ) 
#endif

#endif