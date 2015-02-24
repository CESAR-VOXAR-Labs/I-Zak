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

#ifndef __Plugin_H__
#define __Plugin_H__

namespace DesignPatterns
{
	/**
	*Plugin's interface definition.
	*/
	class Plugin
	{
	private:

#ifdef _WIN32
		HINSTANCE hdll;
#endif

	protected:
		/**
		*Default destructor.
		*/
		virtual ~Plugin(){}

	public:
	
		/**
		* Abstract method that performs the plugin's initial configuration.
		*/
		virtual void initialize() = 0;

		/**
		* Abstract method that performs the plugin`s releasing steps.
		*/
		virtual void release() = 0;

		/** Method that loads a plugin's (.dll) instance.
		 *@param path .dll's path
		 *@return A Plugin's instance
		 */
		 template<typename T>
		static T* loadPlugin(const std::string& path, const std::string& entryPointName)
		{
			std::string lPath = DataTools::StringTools::toLower(path);

#ifdef _WIN32
			wchar_t* wcPath = new wchar_t[lPath.size() + 1];
			mbstowcs(wcPath, lPath.c_str(), lPath.size() + 1);
			HINSTANCE hdll = LoadLibrary(wcPath); // Loads the DLL
			delete[] wcPath;

			typedef T* (*entryPoint)();
			entryPoint pluginConstructor = (entryPoint)GetProcAddress(hdll, entryPointName.data());

			if ( pluginConstructor != 0 ) 
			{
				T* plugin = pluginConstructor();
				plugin->hdll = hdll;
				return plugin;
			}
			else
			{
				throw LibTools::Exception<const std::string>(path, "The Plugin \"" + path + "\" is not valid");
			}
#endif
		}		

		/** Method that unloads a pluggin by its instance.
		 * @param plugin Plugin's instance.
		 */
		 template<typename T>
		static void unloadPlugin(T* plugin)
		{
			plugin->release();
#ifdef _WIN32
			FreeLibrary(plugin->hdll);
#endif
		}
	};
}

/** Macro for defining, based on a class <i>DerivedType</i> and its interface (<i>BaseType</i>), the plugin's entry point.*/
#define PILLAR_PLUGIN_ENTRY(BaseType, DerivedType, entryPoint)						 \
	C_NAME EXPORT BaseType* entryPoint()					 						 \
	{																				 \
		return new DerivedType();													 \
	}																				 \

#endif