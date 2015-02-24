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
#ifndef __Configuration_H__
#define __Configuration_H__

namespace DataTools
{
	/**
	 * Represents a container of names and associated values (configurations).
	 */
	class Configuration
	{
	private:

		/**
		* Container with configurations (std::string) and sets of values (std::vector<std::string>)
		*/
		std::map<std::string, std::vector<std::string> > configs;

	public:		

		/**
		 * Adds a configuration.
		 * @param name The configuration name. It is case-sensitive.
		 * @param value The configuration value.
		 */
		void addConfig(const std::string& name, const std::string& value)
		{
			if (name != "") 
			{
				configs[name].push_back(value);
			}
		}

		/**
		 * Gets all configuration names.
		 * @return The names.
		 */
		std::vector<std::string> getNames() const
		{
			std::vector<std::string> names;
			std::map<std::string, std::vector<std::string> >::const_iterator i = this->configs.cbegin();
			for (; i != this->configs.cend(); i++) 
			{
				names.push_back(i->first);
			}
			return names;
		}

		/**
		 * Gets the values of a configuration.
		 * @param name The configuration name. It is case-sensitive.
		 * @return The found configuration values.
		 */
		const std::vector<std::string>& getValues(const std::string& name) const
		{
			if (configs.count(name) > 0) 
			{
				return configs.at(name);
			} 
			else
			{
				static const std::vector<std::string> dummy;
				return dummy;
			}
		}
	};
}
#endif