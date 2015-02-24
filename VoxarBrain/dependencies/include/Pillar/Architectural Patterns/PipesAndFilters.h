#ifndef __PipesAndFilters_H__
#define __PipesAndFilters_H__

namespace ArchPatterns
{
	template<unsigned int ... index>
	using CropData = TypeTools::ValuePack<index...>;

	namespace Private
	{
		class DataHelper
		{
		public:

			template<unsigned int index, typename T, typename ... Ts>
			static void setVectorElements(std::vector< TypeTools::Any >& v, T& t, Ts& ... ts)
			{
				v[index] = t;
				setVectorElements<index + 1>(v, ts...);
			}

			template<unsigned int index, typename T>
			static void setVectorElements(std::vector< TypeTools::Any >& v, T& t)
			{
				v[index] = t;
			}

			template<typename T, unsigned int ... index>
			static void fillVector(std::vector< TypeTools::Any >& v, T& t, TypeTools::ValuePack< index... >)
			{
				setVectorElements<0>(v, std::get<index>(t)...);
			}

			template<unsigned int index, typename T, typename ... Ts>
			static void setTupleElements(std::vector< TypeTools::Any >& v, T& t, Ts& ... ts)
			{
				t = (T&)v[index];
				setTupleElements<index + 1>(v, ts...);
			}

			template<unsigned int index, typename T>
			static void setTupleElements(std::vector< TypeTools::Any >& v, T& t)
			{
				t = (T&)v[index];
			}

			template<typename T, unsigned int ... index>
			static void fillTuple(std::vector< TypeTools::Any >& v, T& t, TypeTools::ValuePack< index... >)
			{
				setTupleElements<0>(v, std::get<index>(t)...);
			}

			template<typename ... Args>
			static void fillInfo(std::vector< std::type_index >& info, TypeTools::TypePack< Args... >)
			{
				info.insert(info.end(), { typeid(Args)... });
			}

			template<typename ... Args>
			static void fillInfo(std::vector< std::type_index >& info, std::tuple< Args... >)
			{
				info.insert(info.end(), { typeid(Args)... });
			}
		};

		template<bool isSource, typename TPack>
		struct MergeDataTypes;

		template<bool isSource, typename ... DataHolders>
		struct MergeDataTypes< isSource, TypeTools::TypePack< DataHolders... > >
		{
		private:

			template<typename Tuple, typename T, bool flag>
			struct ExtractDataType;

			template<typename ... Args, typename T, bool flag>
			struct ExtractDataType<std::tuple<Args...>, T, flag>
			{
				typedef typename std::tuple<Args..., typename TypeTools::Decompose< decltype(&T::captureData) >::ArgsPack::Head > Result;
			};

			template<typename ... Args, typename T>
			struct ExtractDataType<std::tuple< Args... >, T, false>
			{
			private:

				template<typename Tuple, typename Pack>
				struct Merge;

				template<typename ... TArgs, typename ... PArgs>
				struct Merge< std::tuple< TArgs... >, TypeTools::TypePack< PArgs... > >
				{
					typedef std::tuple< TArgs..., PArgs... > Result;
				};

			public:

				typedef typename Merge< std::tuple<Args...>, typename TypeTools::Decompose< decltype(&T::setData) >::ArgsPack >::Result Result;
			};

			template<typename Tupple, typename ... Holders>
			struct UnpackDataTypes;

			template<typename T, typename ... U, unsigned int ... index, typename ... TuppleArgs>
			struct UnpackDataTypes<std::tuple<TuppleArgs...>, CropData<index...>, T, U...>
			{
				typedef typename UnpackDataTypes< std::tuple< TuppleArgs...>, T, U... >::Result Result;
			};

			template<typename T, typename ... U, typename ... TuppleArgs>
			struct UnpackDataTypes<std::tuple<TuppleArgs...>, T, U...>
			{
				typedef typename UnpackDataTypes< typename ExtractDataType< std::tuple< TuppleArgs...>, T, isSource >::Result, U... >::Result Result;
			};

			template<typename T, typename ... TuppleArgs>
			struct UnpackDataTypes<std::tuple<TuppleArgs...>, T>
			{
				typedef typename ExtractDataType< std::tuple< TuppleArgs...>, T, isSource >::Result Result;
			};

		public:
			typedef typename UnpackDataTypes<std::tuple<>, DataHolders...>::Result Result;
		};
	}
}

#include "Architectural Patterns\Filter.h"
#include "Architectural Patterns\Pipe.h"
#include "Architectural Patterns\Source.h"
#include "Architectural Patterns\Sink.h"

#endif