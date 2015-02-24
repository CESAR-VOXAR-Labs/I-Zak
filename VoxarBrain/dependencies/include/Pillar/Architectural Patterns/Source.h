#ifndef __Source_H__
#define __Source_H__

namespace ArchPatterns
{
	class AbstractDataSource
	{
	protected:

		virtual ~AbstractDataSource(){}

	public:

		virtual void captureData() = 0;

		virtual void captureData(TypeTools::Any& data) = 0;

		virtual void getData(TypeTools::Any& data) = 0;

		virtual void start() = 0;

		virtual void start(bool runOnce) = 0;

		virtual void start(TypeTools::Any& data) = 0;

		virtual void start(TypeTools::Any& data, bool runOnce) = 0;

		virtual void stop() = 0;

		virtual void join() = 0;

		virtual bool isRunning() = 0;

		virtual double getRate() = 0;

		virtual const std::vector<std::type_index>& getDataInfo() = 0;
	};

	namespace Private
	{
		template<typename ... Args>
		class DataSourceRunnable;

		template<typename DataType>
		class DataSourceRunnable<DataType> : public AbstractDataSource
		{
		protected:

			std::vector< std::type_index > info;

			bool running;
			double rate;

			LibTools::Timer timer;
			LibTools::Mutex mutex;

			DataType data;

			virtual void internalCapture(DataType& data) = 0;

		public:

			typedef DataType DataType;

			DataSourceRunnable() : running(false), rate(0.0)
			{
			}

			virtual ~DataSourceRunnable(){}

			void captureData()
			{
				this->captureData(this->data);
			}

			void captureData(DataType& data)
			{
				LibTools::MutexHold lck(this->mutex);

				this->timer.start();

				this->internalCapture(data);

				this->timer.stop();

				this->rate = 1000000000.0 / this->timer.getElapsedTime();
			}

			void captureData(TypeTools::Any& data)
			{
				this->captureData((DataType&)data);
			}

			DataType getData()
			{
				LibTools::MutexHold lck(this->mutex);
				return this->data;
			}

			void getData(DataType& data)
			{
				LibTools::MutexHold lck(this->mutex);
				data = this->data;
			}

			void getData(TypeTools::Any& data)
			{
				LibTools::MutexHold lck(this->mutex);
				data = this->data;
			}

			double getRate()
			{
				return this->rate;
			}

			virtual void run(DataType& data, bool runOnce)
			{
				if (this->running)
					return;

				if (runOnce)
				{
					this->captureData(data);
					return;
				}

				this->running = true;

				while (this->running)
				{
					this->captureData(data);
				}
			}

			void stop()
			{
				this->running = false;
			}

			bool isRunning()
			{
				return this->running;
			}

			const std::vector< std::type_index >& getDataInfo()
			{
				return this->info;
			}
		};

		template<typename DataSourcePolicy, typename DataType>
		class DataSourceRunnable< DataSourcePolicy, void(DataSourcePolicy::*)(DataType&) > : public DataSourceRunnable<DataType>
		{
		private:

			DataSourcePolicy* source;

			void internalCapture(DataType& data)
			{
				this->source->captureData(data);
			}

		public:

			typedef DataType DataType;

			DataSourceRunnable(DataSourcePolicy* source) : source(source){}
		};

		template<typename ... Ts>
		class DataSourceThread : public LibTools::Thread< DataSourceRunnable< Ts... > >
		{
		private:

			typedef LibTools::Thread< DataSourceRunnable< Ts... > > Parent;

		public:

			typedef typename Parent::DataType DataType;

			DataSourceThread(){}

			template<typename T>
			DataSourceThread(T* t) : LibTools::Thread< DataSourceRunnable< Ts... > >(t){}

			virtual ~DataSourceThread()
			{
				if (this->running)
				{
					this->stop();
					this->join();
				}
			}

			void start()
			{
				bool runOnce = false;
				this->start(this->data, runOnce);
			}

			void start(bool runOnce)
			{
				this->start(this->data, runOnce);
			}

			void start(TypeTools::Any& data)
			{
				bool runOnce = false;
				this->start((DataType&)data, runOnce);
			}

			void start(TypeTools::Any& data, bool runOnce)
			{
				this->start((DataType&)data, runOnce);
			}

			void start(DataType& data)
			{
				bool runOnce = false;
				this->start(data, runOnce);
			}

			void start(DataType& data, bool runOnce)
			{
				Parent::start(data, runOnce);
			}

			void join()
			{
				Parent::join();
			}
		};

		template<typename ... DataSources>
		struct DataSourceThreadsTuple
		{
		private:

			template<typename Tuple, typename ... Args>
			struct FillTuple;

			template<typename DataSourcePolicy>
			struct FillTuple< std::tuple<>, DataSourcePolicy >
			{
				typedef std::tuple< DataSourceThread< DataSourcePolicy, decltype(&DataSourcePolicy::grabData) >* > Result;
			};

			template<typename DataSourcePolicy, typename ... DataSourcePolicies>
			struct FillTuple< std::tuple<>, DataSourcePolicy, DataSourcePolicies... >
			{
				typedef typename FillTuple< std::tuple< DataSourceThread< DataSourcePolicy, decltype(&DataSourcePolicy::grabData) >* >, DataSourcePolicies... >::Result Result;
			};

			template<typename DataSourcePolicy, typename ... DataSourcePolicies, typename ... Args>
			struct FillTuple< std::tuple<Args...>, DataSourcePolicy, DataSourcePolicies... >
			{
				typedef typename FillTuple< std::tuple< Args..., DataSourceThread< DataSourcePolicy, decltype(&DataSourcePolicy::grabData) >* >, DataSourcePolicies... >::Result Result;
			};

			template<typename DataSourcePolicy, typename ... Args>
			struct FillTuple< std::tuple<Args...>, DataSourcePolicy >
			{
				typedef std::tuple< Args..., DataSourceThread< DataSourcePolicy, decltype(&DataSourcePolicy::grabData) >* > Result;
			};

		public:

			typedef typename FillTuple< std::tuple<>, DataSources... >::Result Result;
		};
	}

	template<
		typename DataSourceCommandType,
		DataSourceCommandType,
		typename DataSourcePolicyType = typename TypeTools::Decompose< DataSourceCommandType >::ObjectType,
		typename DataSourceCommandArgs = typename TypeTools::Decompose< DataSourceCommandType >::ArgsPack::Head
	>
	class DataSourcePolicy;

	template<
		typename DataSourceCommandType,
		DataSourceCommandType command,
		typename DataSourcePolicyType,
		typename DataType
	>
	class DataSourcePolicy<DataSourceCommandType, command, DataSourcePolicyType, TypeTools::TypePack< DataType >> : public DataSourcePolicyType
	{
	public:

		void captureData(DataType& data)
		{
			(this->*command)(data);
		}
	};

	template<typename DataType>
	class DataSourceInterface : public Private::DataSourceThread< DataType >
	{
	private:

		typedef Private::DataSourceThread< DataType > Parent;

	protected:

		virtual ~DataSourceInterface(){}

	public:

		typedef DataType DataType;

		void captureData()
		{
			Parent::captureData();
		}

		void captureData(DataType& data)
		{
			Parent::captureData(data);
		}

		void captureData(TypeTools::Any& data)
		{
			Parent::captureData(data);
		}

		DataType getData()
		{
			return Parent::getData();
		}

		void getData(DataType& data)
		{
			Parent::getData(data);
		}

		void getData(TypeTools::Any& data)
		{
			Parent::getData(data);
		}

		void start()
		{
			Parent::start();
		}

		void start(bool runOnce)
		{
			Parent::start(runOnce);
		}

		void start(DataType& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(DataType& data)
		{
			Parent::start(data);
		}

		void start(TypeTools::Any& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(TypeTools::Any& data)
		{
			Parent::start(data);
		}

		void stop()
		{
			Parent::stop();
		}

		void join()
		{
			Parent::join();
		}

		bool isRunning()
		{
			return Parent::isRunning();
		}

		double getRate()
		{
			return Parent::getRate();
		}
	};

	template<typename ... DataSourcePolicies>
	class DataSource;

	template<typename DataSourcePolicy>
	class DataSource<DataSourcePolicy> : public DataSourcePolicy,
										 public DataSourceInterface< typename TypeTools::Decompose< decltype(&DataSourcePolicy::captureData) >::ArgsPack::Head >
	{
	private:

		typedef DataSourceInterface< typename TypeTools::Decompose< decltype(&DataSourcePolicy::captureData) >::ArgsPack::Head > Parent;

		void internalCapture(DataType& data)
		{
			DataSourcePolicy::captureData(data);
		}

	public:

		typedef typename Parent::DataType DataType;

		DataSource()
		{
			Private::DataHelper::fillInfo(this->info, TypeTools::TypePack< DataType >());
		}

		virtual ~DataSource(){}

		typedef typename Parent::DataType DataType;

		void captureData()
		{
			Parent::captureData();
		}

		void captureData(DataType& data)
		{
			Parent::captureData(data);
		}

		void captureData(TypeTools::Any& data)
		{
			Parent::captureData(data);
		}

		DataType getData()
		{
			return Parent::getData();
		}

		void getData(DataType& data)
		{
			Parent::getData(data);
		}

		void getData(TypeTools::Any& data)
		{
			Parent::getData(data);
		}

		void start()
		{
			Parent::start();
		}

		void start(bool runOnce)
		{
			Parent::start(runOnce);
		}

		void start(DataType& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(DataType& data)
		{
			Parent::start(data);
		}

		void start(TypeTools::Any& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(TypeTools::Any& data)
		{
			Parent::start(data);
		}

		void stop()
		{
			Parent::stop();
		}

		void join()
		{
			Parent::join();
		}

		bool isRunning()
		{
			return Parent::isRunning();
		}

		double getRate()
		{
			return Parent::getRate();
		}

		const std::vector< std::type_index >& getDataInfo()
		{
			return Parent::getDataInfo();
		}
	};

	template<typename ... DataSourcePolicies>
	class DataSource<DataSourcePolicies...> : public DataSourcePolicies...,
											  public DataSourceInterface< typename Private::MergeDataTypes<true, DataSourcePolicies...>::Result >
	{
	private:

		typedef TypeTools::TypePack< DataSourcePolicies... > PoliciesPack;

		typedef DataSourceInterface< typename Private::MergeDataTypes<true, DataSourcePolicies...>::Result > Parent;

		typedef typename Private::DataSourceThreadsTuple< DataSourcePolicies... >::Result DataSourceThreads;

		DataSourceThreads threads;

		template< unsigned int index >
		void initThreads()
		{
			typedef typename std::remove_pointer<typename std::tuple_element<index, DataSourceThreads>::type>::type CurrentThread;

			std::get<index>(this->threads) = new CurrentThread(this);

			this->initThreads< index + 1 >();
		}

		template<>
		void initThreads< sizeof...(DataSourcePolicies)-1 >()
		{
			typedef typename std::remove_pointer<typename std::tuple_element<sizeof...(DataSourcePolicies)-1, DataSourceThreads>::type>::type CurrentThread;

			std::get<sizeof...(DataSourcePolicies)-1>(this->threads) = new CurrentThread(this);
		}

		template<unsigned int index>
		void deleteThreads()
		{
			delete std::get<index>(this->threads);
			this->deleteThreads<index + 1>();
		}

		template<>
		void deleteThreads<sizeof...(DataSourcePolicies)-1>()
		{
			delete std::get<sizeof...(DataSourcePolicies)-1>(this->threads);
		}

		template<unsigned int index>
		void startAll(DataType& data, bool runOnce)
		{
			std::get<index>(this->threads)->start(std::get<index>(data), runOnce);

			this->startAll< index + 1 >(data, runOnce);

			std::get<index>(this->threads)->join();
		}

		template<>
		void startAll< sizeof...(DataSourcePolicies)-1 >(DataType& data, bool runOnce)
		{
			std::get< sizeof...(DataSourcePolicies)-1 >(this->threads)->start(std::get<sizeof...(DataSourcePolicies)-1>(data), runOnce);

			std::get< sizeof...(DataSourcePolicies)-1 >(this->threads)->join();
		}

		template<unsigned int index>
		void stopAll()
		{
			std::get<index>(this->threads)->stop();
			this->stopAll<index + 1>();
		}

		template<>
		void stopAll< sizeof...(DataSourcePolicies)-1 >()
		{
			std::get< sizeof...(DataSourcePolicies)-1 >(this->threads)->stop();
		}

		template<unsigned int index>
		void getThreadRate(double* rate)
		{
			rate[index] = std::get<index>(this->threads)->getRate();

			this->getThreadRate<index + 1>(fps);
		}

		template<>
		void getThreadRate< sizeof...(DataSourcePolicies)-1 >(double* rate)
		{
			rate[sizeof...(DataSourcePolicies)-1] = std::get<sizeof...(DataSourcePolicies)-1>(this->threads)->getRate();
		}

		void internalCapture(DataType& data)
		{
			this->startAll<0>(data, true);
		}

	public:

		typedef typename Parent::DataType DataType;

		DataSource()
		{
			Private::DataHelper::fillInfo(this->info, DataType());
			this->initThreads<0>();
		}

		virtual ~DataSource()
		{
			if (this->running)
			{
				this->stop();
				this->join();
				this->deleteThreads<0>();
			}
		}

		void run(DataType& data, bool runOnce)
		{
			if (!this->running)
			{
				this->running = true;

				this->startAll<0>(data, runOnce);

				this->running = false;
			}
		}

		double getRate()
		{
			if (this->running)
			{
				static double rateSet[sizeof...(DataSourcePolicies)];
				std::memset(rateSet, 0, sizeof(rateSet));

				this->getThreadRate<0>(rateSet);

				this->rate = *std::min_element(rateSet, rateSet + sizeof...(DataSourcePolicies));
			}

			return this->rate;
		}

		void captureData()
		{
			Parent::captureData();
		}

		void captureData(DataType& data)
		{
			Parent::captureData(data);
		}

		void captureData(TypeTools::Any& data)
		{
			Parent::captureData(data);
		}

		DataType getData()
		{
			return Parent::getData();
		}

		void getData(DataType& data)
		{
			Parent::getData(data);
		}

		void getData(TypeTools::Any& data)
		{
			Parent::getData(data);
		}

		void start()
		{
			Parent::start();
		}

		void start(bool runOnce)
		{
			Parent::start(runOnce);
		}

		void start(DataType& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(DataType& data)
		{
			Parent::start(data);
		}

		void start(TypeTools::Any& data, bool runOnce)
		{
			Parent::start(data, runOnce);
		}

		void start(TypeTools::Any& data)
		{
			Parent::start(data);
		}

		void join()
		{
			Parent::join();
		}

		bool isRunning()
		{
			return Parent::isRunning();
		}

		void stop()
		{
			this->stopAll<0>();
		}

		const std::vector< std::type_index >& getDataInfo()
		{
			return Parent::getDataInfo();
		}
	};

	template<>
	class DataSource<> : public DataSourceInterface< std::vector< TypeTools::Any > >
	{
	private:

		typedef DataSourceInterface< std::vector< TypeTools::Any > > Parent;

		std::vector< AbstractDataSource* > sources;

		void startAll(DataType& data, bool runOnce)
		{
			data.resize(this->sources.size());

			for (unsigned int i = 0; i < this->sources.size(); i++)
			{
				this->sources[i]->start(data[i], runOnce);
			}

			for (unsigned int i = 0; i < this->sources.size(); i++)
			{
				this->sources[i]->join();
			}
		}

		void stopAll()
		{
			for (unsigned int i = 0; i < this->sources.size(); i++)
			{
				this->sources[i]->stop();
			}
		}

		void internalCapture(DataType& data)
		{
			this->startAll(data, true);
		}

	public:

		typedef Parent::DataType DataType;

		void addDataSource(AbstractDataSource* source)
		{
			LibTools::MutexHold lck(this->mutex);

			this->info.clear();

			bool flag = this->running;

			if (flag)
			{
				this->stop();
				this->join();
			}

			this->sources.push_back(source);
			this->data.push_back(TypeTools::Any());

			for (unsigned int i = 0; i < this->sources.size(); i++)
			{
				std::vector< std::type_index > sourceInfo = this->sources[i]->getDataInfo();

				for (unsigned int j = 0; j < sourceInfo.size(); j++)
				{
					this->info.push_back(sourceInfo[j]);
				}
			}

			if (flag)
				this->start();
		}

		void removeDataSource(unsigned int index)
		{
			LibTools::MutexHold lck(this->mutex);

			this->info.clear();

			bool flag = this->running;

			if (flag)
			{
				this->stop();
				this->join();
			}

			auto it = this->sources.begin() + index;

			this->sources.erase(it);
			this->data.erase(this->data.begin() + index);

			for (unsigned int i = 0; i < this->sources.size(); i++)
			{
				std::vector< std::type_index > sourceInfo = this->sources[i]->getDataInfo();

				for (unsigned int j = 0; j < sourceInfo.size(); j++)
				{
					this->info.push_back(sourceInfo[j]);
				}
			}

			if (flag)
				this->start();
		}

		DataSource() {}

		virtual ~DataSource(){}

		void run(DataType& data, bool runOnce)
		{
			if (!this->running)
			{
				this->running = true;

				this->startAll(data, runOnce);

				this->running = false;
			}
		}

		double getRate()
		{
			if (this->running)
			{
				double* rateSet = new double[this->sources.size()];
				std::memset(rateSet, 0, sizeof(rateSet));

				for (unsigned int i = 0; i < this->sources.size(); i++)
				{
					rateSet[i] = this->sources[i]->getRate();
				}

				this->rate = *std::min_element(rateSet, rateSet + this->sources.size());

				delete[] rateSet;
			}

			return this->rate;
		}

		void stop()
		{
			this->stopAll();
		}

		const std::vector<std::type_index>& getDataInfo()
		{
			LibTools::MutexHold lck(this->mutex);
			return Parent::getDataInfo();
		}
	};
}

#define PILLAR_DATA_SOURCE(DataSourceCommand) ArchPatterns::DataSourcePolicy< decltype(DataSourceCommand), DataSourceCommand >

#endif