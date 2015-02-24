
#ifdef _MSC_VER
#pragma warning ( disable: 4503 )
#endif

//Dependencies Includes

#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#include <asio\include\asio.hpp>
#endif

//end of Dependencies Includes

//System Includes

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <typeindex>

#include <memory>
#include <iostream>
#include <sstream>
#include <io.h>
#include <direct.h>
#include <fstream>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif

//end of System Includes

//Library Includes

//Type Tools module (namespace TypeTools)
#include "Type Tools\NullType.h"
#include "Type Tools\TypeMapping.h"
#include "Type Tools\TypeList.h"
#include "Type Tools\TypePack.h"
#include "Type Tools\Any.h"

//Librarian Tools module (namespace LibTools)
#include "Librarian Tools\LibConfiguration.h"
#include "Librarian Tools\Exception.h"
#include "Librarian Tools\Timing.h"
#include "Librarian Tools\Threading.h"
#include "Librarian Tools\Networking.h"
#include "Librarian Tools\File.h"

//Data Tools module (namespace DataTools)
#include "Data Tools\StringTools.h"
#include "Data Tools\Parser.h"
#include "Data Tools\Configuration.h"
#include "Data Tools\ConfigurationParser.h"

//Design Patterns module (namespace DesignPatterns)
#include "Design Patterns\Pool.h"
#include "Design Patterns\Factory.h"
#include "Design Patterns\Storehouse.h"
#include "Design Patterns\Command.h"
#include "Design Patterns\Observer.h"
#include "Design Patterns\Dispatcher.h"
#include "Design Patterns\Plugin.h"

//Architectural Patterns module (namespace ArchPatterns)
#include "Architectural Patterns\Reflection.h"
#include "Architectural Patterns\PipesAndFilters.h"

//end of Library Includes