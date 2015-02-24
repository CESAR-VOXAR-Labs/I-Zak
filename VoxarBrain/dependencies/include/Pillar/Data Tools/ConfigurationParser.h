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
#ifndef __ConfigurationParser_H__
#define __ConfigurationParser_H__

namespace DataTools
{
	/**
	 * Parser specialization of Configuration.
	 */
	template<>
	class Parser<Configuration, std::string> 
	{
	public:

		/**
		 * Converts a string into a Configuration.<br>
		 * The string lines are expected to follow the syntax "Name = Value1, Value2".
		 * @param text The string.
		 * @return The created Configuration.
		 */
		static Configuration parse(const std::string& text) 
		{
			Configuration object;
			std::vector<std::string> t1;
			StringTools::split(text, "\n\r", t1);
			std::string blank = "\t ";
			for (unsigned int i = 0; i < t1.size(); i++) 
			{
				std::string line = StringTools::trim(t1[i], blank);
				if (line != "" && line[0] != '#') 
				{
					std::vector<std::string> t2;
					StringTools::split(line, ",=", t2);
					std::string name = StringTools::trim(t2[0], blank);
					for (unsigned int j = 1; j < t2.size(); j++) 
					{
						object.addConfig(name, StringTools::trim(t2[j], blank));
					}						
				}
			}
			return object;
		}

		/**
		 * Converts a Configuration into a string.
		 * @param object The Configuration.
		 * @return The string.
		 */
		static std::string serialize(const Configuration& object) 
		{
			std::stringstream text;
			std::vector<std::string> names = object.getNames();
			for (unsigned int i = 0; i < names.size(); i++) 
			{
				text << names[i] << " = ";
				std::vector<std::string> values = object.getValues(names[i]);
				for (unsigned int j = 0, size = values.size(); j < size; j++) 
				{
					text << values[j];
					if (j < size - 1) 
					{
						text << ", ";
					}
				}
				text << '\n';
			}
			return text.str();
		}
	};

	/**
	 * Parser specialization of Configuration*.
	 */
	template<>
	class Parser<Configuration*, std::string> 
	{
	public:

		/**
		 * Converts a string into a Configuration*.
		 * The string lines are expected to follow the syntax "Name = Value1, Value2".
		 * @param text The string.
		 * @return The created Configuration.
		 */
		static Configuration* parse(const std::string& text) 
		{
			Configuration* object = new Configuration();
			std::vector<std::string> t1;
			StringTools::split(text, "\n\r", t1);
			std::string blank = "\t ";
			for (unsigned int i = 0; i < t1.size(); i++) 
			{
				std::string line = StringTools::trim(t1[i], blank);
				if (line != "" && line[0] != '#') 
				{
					std::vector<std::string> t2;
					StringTools::split(line, ",=", t2);
					std::string name = StringTools::trim(t2[0], blank);
					for (unsigned int j = 1; j < t2.size(); j++) 
					{
						object->addConfig(name, StringTools::trim(t2[j], blank));
					}						
				}
			}
			return object;
		}

		/**
		 * Converts a Configuration* into a string.
		 * @param object The Configuration.
		 * @return The string.
		 */
		static std::string serialize(const Configuration* object) 
		{
			std::stringstream text;
			std::vector<std::string> names = object->getNames();
			for (unsigned int i = 0; i < names.size(); i++) 
			{
				text << names[i] << " = ";
				std::vector<std::string> values = object->getValues(names[i]);
				for (unsigned int j = 0, size = values.size(); j < size; j++) 
				{
					text << values[j];
					if (j < size - 1) 
					{
						text << ", ";
					}
				}
				text << '\n';
			}
			return text.str();
		}
	};
}
#endif