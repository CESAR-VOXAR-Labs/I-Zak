#ifndef __Sink_H__
#define __Sink_H__

namespace ArchPatterns
{
	class AbstractDataSink
	{
	public:

		virtual void setData(std::vector<TypeTools::Any>& data) = 0;
	};

	template<typename ... DataTypes>
	class DataSinkInterface : public AbstractDataSink
	{
	public:

		virtual void setData(DataTypes& ... data) = 0;
	};

	namespace Private
	{
		template< typename DataSinkPolicy, typename DataType, typename CropDataType>
		class DataSinkCroped;

		template<typename DataSinkPolicy, typename ... DataTypes, unsigned int ... index>
		class DataSinkCroped< DataSinkPolicy, std::tuple< DataTypes... >, CropData< index... > > : public DataSinkPolicy
		{
		public:

			void setData(DataTypes& ... data)
			{
				auto dataTuple = std::tie(data...);

				DataSinkPolicy::setData(std::get<index>(dataTuple)...);
			}
		};

		template<typename DataType, typename DataSinkPack>
		struct DataSinkPolicyPack;

		template<typename DataType, typename ... DataSinkPolicies>
		struct DataSinkPolicyPack<DataType, TypeTools::TypePack< DataSinkPolicies... >>
		{
		private:

			template<typename TPack, typename ... FilterPolicies >
			struct PolicyPack;

			template<typename T, typename ... U>
			struct PolicyPack< TypeTools::TypePack<>, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack< T >, U... >::Result Result;
			};

			template<typename ... Args, typename T, typename ... U>
			struct PolicyPack< TypeTools::TypePack<Args...>, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack< Args..., T >, U... >::Result Result;
			};

			template<typename ... Args, typename T>
			struct PolicyPack< TypeTools::TypePack<Args...>, T>
			{
				typedef TypeTools::TypePack< Args..., T > Result;
			};

			template<typename T, unsigned int ... index, typename ... U>
			struct PolicyPack< TypeTools::TypePack<>, CropData< index... >, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack<>, DataSinkCroped<T, DataType, CropData<index...>>, U... >::Result Result;
			};

			template<typename T, typename ... Args, unsigned int ... index, typename ... U>
			struct PolicyPack< TypeTools::TypePack<Args...>, CropData< index... >, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack<Args...>, DataSinkCroped<T, DataType, CropData<index...>>, U... >::Result Result;
			};

			template<typename T>
			struct PolicyPack< TypeTools::TypePack<>, T>
			{
				typedef TypeTools::TypePack< T > Result;
			};

		public:

			typedef typename PolicyPack< TypeTools::TypePack<>, DataSinkPolicies... >::Result Result;
		};

		template<
			typename DataSinkPack,
			typename DataType = typename MergeDataTypes< false, DataSinkPack >::Result,
			typename ActualDataSinkPack = typename DataSinkPolicyPack< DataType, DataSinkPack >::Result
		>
		class DataSink;

		template<typename DataSinkPoliciesPack, typename DataSinkPolicy, typename ... DataTypes>
		class DataSink< DataSinkPoliciesPack,
						std::tuple< DataTypes... >,
						TypeTools::TypePack< DataSinkPolicy >
					  > : public DataSinkPolicy,
						  public DataSinkInterface< DataTypes... >
		{
		private:

			template<unsigned int ... index, typename ... Ts>
			void internalSetData(TypeTools::ValuePack<index...>, std::tuple< Ts... > tuple)
			{
				DataSinkPolicy::setData(std::get<index>(tuple)...);
			}

		public:

			typedef TypeTools::TypePack< DataTypes... > DataType;

			void setData(DataTypes& ... data)
			{
				DataSinkPolicy::setData(data...);
			}

			void setData(std::vector< TypeTools::Any >& data)
			{
				std::tuple< DataTypes... > dataTuple;
				Private::DataHelper::fillTuple(data, dataTuple, typename TypeTools::FillValuePack< sizeof...(DataTypes) >::Result());

				this->internalSetData(typename TypeTools::FillValuePack< sizeof...(DataTypes) >::Result(), dataTuple);
			}
		};

		template<typename DataSinkPoliciesPack, typename ... DataSinkPolicies, typename ... DataTypes>
		class DataSink< typename DataSinkPoliciesPack,
						std::tuple< DataTypes... >,
						TypeTools::TypePack< DataSinkPolicies... >
					  > : public DataSinkPolicies...,
						  public DataSinkInterface< DataTypes... >
		{
		private:

			typedef TypeTools::TypePack< DataSinkPolicies... > DataSinkPack;
			typedef typename TypeTools::FillValuePack< sizeof...(DataSinkPolicies) >::Result IndexPack;

			template<typename T>
			class SinkRunnable
			{
			public:

				T* sink;

				SinkRunnable()
				{
				}

				SinkRunnable(T* t) : sink(t)
				{
				}

				void run(DataTypes& ... data)
				{
					sink->setData(data...);
				}
			};

			template<typename SinkPack, unsigned int ... index, typename ... Ts>
			void internalSetData(SinkPack sPack, TypeTools::ValuePack<index...>, std::tuple< Ts... > tuple)
			{
				this->internalSetData(sPack, std::get<index>(tuple)...);
			}

			template<typename T, typename ... Ts>
			void internalSetData(TypeTools::TypePack< T, Ts... >, DataTypes& ... data)
			{
				LibTools::Thread< SinkRunnable<T> > thread(this);
				thread.start(data...);

				this->internalSetData(TypeTools::TypePack< Ts... >(), data...);

				thread.join();
			}

			template<typename T>
			void internalSetData(TypeTools::TypePack< T >, DataTypes& ... data)
			{
				LibTools::Thread< SinkRunnable<T> > thread(this);
				thread.start(data...);
				thread.join();
			}

		public:

			typedef TypeTools::TypePack< DataTypes... > DataType;

			void setData(DataTypes& ... data)
			{
				this->internalSetData(DataSinkPack(), data...);
			}

			void setData(std::vector< TypeTools::Any >& data)
			{
				std::tuple< DataTypes... > dataTuple;
				Private::DataHelper::fillTuple(data, dataTuple, typename TypeTools::FillValuePack< sizeof...(DataTypes) >::Result());

				this->internalSetData(DataSinkPack(), typename TypeTools::FillValuePack< sizeof...(DataTypes) >::Result(), dataTuple);
			}
		};
	}

	template<
		typename DataSinkCommandType,
		DataSinkCommandType,
		typename DataSinkPolicyType = typename TypeTools::Decompose< DataSinkCommandType >::ObjectType,
		typename DataSinkCommandArgs = typename TypeTools::Decompose< DataSinkCommandType >::ArgsPack
	>
	class DataSinkPolicy;

	template<
		typename DataSinkCommandType,
		DataSinkCommandType command,
		typename DataSinkPolicyType,
		typename ... Args
	>
	class DataSinkPolicy<DataSinkCommandType, command, DataSinkPolicyType, TypeTools::TypePack< Args... >> : public DataSinkPolicyType
	{
	public:

		void setData(Args& ... args)
		{
			(this->*command)(args...);
		}
	};

	template<typename ... DataSinkPolicies>
	class DataSink : public Private::DataSink< TypeTools::TypePack< DataSinkPolicies... > >
	{
	private:

		typedef Private::DataSink< TypeTools::TypePack< DataSinkPolicies... > > Parent;

	public:

		typedef typename Parent::DataType DataType;

		template<typename ... Args>
		void setData(Args& ... args)
		{
			Parent::setData(args...);
		}
	};

	template<>
	class DataSink<> : public AbstractDataSink
	{
	private:

		std::vector< AbstractDataSink* > dataSinks;

	public:

		void addDataSink(AbstractDataSink* dataSink)
		{
			this->dataSinks.push_back(dataSink);
		}

		void removeDataSink(unsigned int index)
		{
			this->dataSinks.erase(this->dataSinks.begin() + index);
		}

		void setData(std::vector< TypeTools::Any >& data)
		{
			for (unsigned int i = 0; i < this->dataSinks.size(); i++)
			{
				this->dataSinks[i]->setData(data);
			}
		}
	};
}

#define PILLAR_DATA_SINK(DataSinkCommand) ArchPatterns::DataSinkPolicy< decltype(DataSinkCommand), DataSinkCommand >

#endif