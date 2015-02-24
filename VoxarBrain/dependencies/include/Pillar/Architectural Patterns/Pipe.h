namespace ArchPatterns
{
	enum PipeType
	{
		SINGLE,
		LOOP,
		SEQUENTIAL,
		PARALLEL
	};

	class AbstractPipe : public AbstractFilter
	{
	protected:

		std::vector< AbstractFilter* > filters;

	public:

		virtual void addFilter(AbstractFilter* filter)
		{
			this->filters.push_back(filter);
		}

		template<typename T>
		void addFilterPolicy()
		{
			this->addFilter(new Filter<T>);
		}

		void removeFilterAt(const unsigned int index, bool destroy = false)
		{
			if (destroy)
				delete this->filters[index];

			this->filters.erase(this->filters.begin() + index);
		}

		void removeAllFilters(bool destroy = false)
		{
			if (destroy)
			{
				for (unsigned int i = 0; i < this->filters.size(); i++)
				{
					delete this->filters[i];
				}
			}

			this->filters.clear();
		}
	};

	template<typename ... FilterPolicies>
	class SequentialPipe;

	template<typename ... FilterPolicies>
	class ParallelPipe;

	namespace Private
	{
		template<
			typename FilterPack,
			unsigned int nbFilters = TypeTools::Length<FilterPack>::value,
			typename LastOutput = typename TypeTools::TypeAt< FilterPack, nbFilters - 1 >::Result::Output,
			typename FirstInput = typename TypeTools::TypeAt< FilterPack, 0 >::Result::Input
		>
		class SequentialPipe;

		template<typename ... Filters, unsigned int nbFilters, typename ... Outputs, typename ... Inputs>
		class SequentialPipe<
			TypeTools::TypePack<Filters...>,
			nbFilters,
			std::tuple< Outputs... >,
			std::tuple< Inputs... >
							> : public FilterInterface<Outputs..., Inputs...>
		{
		private:

			typedef typename TypeTools::TypePack<Filters...> FilterPack;

			typedef typename TypeTools::TypeAt< FilterPack, 0 >::Result FirstFilter;
			typedef typename TypeTools::TypeAt< FilterPack, nbFilters - 1 >::Result LastFilter;

			typedef typename TypeTools::FillValuePack< sizeof...(Outputs) >::Result OutputIndex;
			typedef typename TypeTools::FillValuePack< sizeof...(Inputs) >::Result InputIndex;			

			template<unsigned int filterIndex, typename CurrentOutput, typename CurrentInput>
			void internalFilter(std::tuple<Outputs...>& output, CurrentOutput& interOutput, CurrentInput& input)
			{
				typedef typename TypeTools::TypeAt<FilterPack, filterIndex + 1>::Result::Output NextOutput;
				NextOutput nextOutput;

				std::get<filterIndex>(this->filters).filter(interOutput, input);

				this->internalFilter<filterIndex + 1>(output, nextOutput, interOutput);
			}

			template<>
			void internalFilter< nbFilters - 1, std::tuple<Outputs...>, typename LastFilter::Input >(std::tuple<Outputs...>& output, std::tuple<Outputs...>&, typename LastFilter::Input& input)
			{
				std::get<nbFilters - 1>(this->filters).filter(output, input);
			}

		protected:

			std::tuple<Filters...> filters;

		public:

			SequentialPipe()
			{
				DataHelper::fillInfo(this->outputsInfo, TypeTools::TypePack< Outputs... >());
				DataHelper::fillInfo(this->inputsInfo, TypeTools::TypePack< Inputs... >());
			}

			typedef std::tuple< Outputs... > Output;
			typedef std::tuple< Inputs... > Input;

		protected:

			void filter(Outputs& ... outputs, Inputs& ... inputs)
			{
				typename FirstFilter::Output firstOutput;

				std::tuple<Outputs...> output;
				std::tuple<Inputs...> input(inputs...);

				this->internalFilter<0>(output, firstOutput, input);

				std::tie(outputs...) = output;
			}

			void filter(std::tuple< Outputs... >& output, std::tuple< Inputs... >& Input)
			{
				typename FirstFilter::Output firstOutput;

				this->internalFilter<0>(output, firstOutput, input);
			}

			void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
			{
				typename FirstFilter::Output firstOutput;
				std::tuple< Outputs... > lastOutput;

				Input actualInput;

				DataHelper::fillTuple(input, actualInput, InputIndex());

				this->internalFilter<0>(lastOutput, firstOutput, actualInput);

				DataHelper::fillVector(output, lastOutput, OutputIndex());
			}
		};

		template<typename FilterPack>
		struct MergeOutputs
		{
		private:

			template<typename FPack>
			struct ExtractFirstOutput;

			template<typename T, typename ... U>
			struct ExtractFirstOutput< TypeTools::TypePack< T, U... > >
			{
				typedef TypeTools::TypePack< typename T::Output, U... > Result;
			};

			template<typename TPack, typename FPack>
			struct Merge;

			template<typename ... Args, typename ... TupleArgs>
			struct Merge<TypeTools::TypePack< Args... >, TypeTools::TypePack< std::tuple<TupleArgs...> > >
			{
				typedef std::tuple< Args..., TupleArgs... > Result;
			};

			template<typename ... Args, typename ... TupleArgs, typename Output>
			struct Merge< TypeTools::TypePack< Args... >, TypeTools::TypePack< TypeTools::TypePack< std::tuple< TupleArgs... >, Output > > >
			{
				typedef typename Merge< TypeTools::TypePack< Args..., TupleArgs... >, TypeTools::TypePack< typename Output::Output > >::Result Result;
			};

			template<typename ... Args, typename ... TupleArgs, typename Output, typename ... Outputs>
			struct Merge< TypeTools::TypePack< Args... >, TypeTools::TypePack< std::tuple< TupleArgs... >, Output, Outputs... > >
			{
				typedef typename Merge< TypeTools::TypePack< Args..., TupleArgs... >, TypeTools::TypePack< typename Output::Output, Outputs... > >::Result Result;
			};

		public:

			typedef typename Merge< TypeTools::TypePack<>, typename ExtractFirstOutput<FilterPack>::Result >::Result Result;
		};

		template<
			typename FilterPack,
			typename Output = typename MergeOutputs< FilterPack >::Result,
			typename Input = typename TypeTools::TypeAt< FilterPack, 0 >::Result::Input,
			unsigned int nbFilters = TypeTools::Length< FilterPack >::value,
			unsigned int nbOutputs = std::tuple_size<Output>::value
		>
		class ParallelPipe;

		template<typename ... Filters, typename ... Outputs, typename ... Inputs, unsigned int nbFilters, unsigned int nbOutputs>
		class ParallelPipe<
			TypeTools::TypePack<Filters...>,
			std::tuple< Outputs... >,
			std::tuple< Inputs... >,
			nbFilters,
			nbOutputs
						  > : public FilterInterface<Outputs..., Inputs...>
		{
		private:

			template<typename T, typename OutIndex, typename InIdex>
			class FilterRunnable;

			template<typename T, unsigned int ... OIndex, unsigned int ... IIndex>
			class FilterRunnable<T, TypeTools::ValuePack<OIndex...>, TypeTools::ValuePack<IIndex...> >
			{
			public:

				T& t;
				std::tuple<Outputs...>& outs;
				std::tuple<Inputs...>& ins;

				FilterRunnable(T& t, std::tuple<Outputs...>& outs, std::tuple<Inputs...>& ins) : t(t), outs(outs), ins(ins)
				{}

				FilterRunnable& operator=(const FilterRunnable& other)
				{
					this->t = other.t;
					this->outs = other.outs;
					this->ins = other.ins;

					return *this;
				}

				void run()
				{
					t.filter(std::get<OIndex>(outs)..., std::get<IIndex>(ins)...);
				}
			};

			typedef TypeTools::TypePack<Filters...> FilterPack;

			typedef typename TypeTools::FillValuePack< sizeof...(Outputs) >::Result OutputIndex;
			typedef typename TypeTools::FillValuePack< sizeof...(Inputs) >::Result InputIndex;

			std::tuple<Filters...> filters;

			template< unsigned int filterIndex, unsigned int nextOutputIndex >
			void internalFilter(std::tuple<Outputs...>& output, std::tuple<Inputs...>& input)
			{
				typedef typename TypeTools::TypeAt< FilterPack, filterIndex >::Result CurrentFilter;

				typedef typename TypeTools::IntervalToValuePack<
					nextOutputIndex,
					std::tuple_size< typename CurrentFilter::Output >::value - 1 + nextOutputIndex
				>::Result CurrentOutputIndex;

				typedef LibTools::Thread<
					FilterRunnable<
					CurrentFilter,
					CurrentOutputIndex,
					InputIndex
					>
				> FilterThread;

				FilterThread thread(std::get<filterIndex>(this->filters), output, input);

				thread.start();

				this->internalFilter<
					filterIndex + 1,
					filterIndex + 1 != nbFilters ? std::tuple_size< typename CurrentFilter::Output >::value + nextOutputIndex : 0
				>(output, input);

				thread.join();
			}

			/**
			* Dummy method just for end the recursion.
			*/
			template<>
			void internalFilter< nbFilters, 0 >(std::tuple<Outputs...>&, std::tuple<Inputs...>&)
			{
			}

		public:

			ParallelPipe()
			{
				DataHelper::fillInfo(this->outputsInfo, TypeTools::TypePack< Outputs... >());
				DataHelper::fillInfo(this->inputsInfo, TypeTools::TypePack< Inputs... >());
			}

			typedef std::tuple< Outputs... > Output;
			typedef std::tuple< Inputs... > Input;

		protected:

			void filter(Outputs& ... outputs, Inputs& ... inputs)
			{
				std::tuple<Outputs...> output;
				std::tuple<Inputs...> input(inputs...);

				this->internalFilter<0, 0>(output, input);

				std::tie(outputs...) = output;
			}

			void filter(std::tuple< Outputs... >& output, std::tuple< Inputs... >& input)
			{
				this->internalFilter<0, 0>(output, input);
			}

			void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
			{
				std::tuple< Outputs... > lastOutput;

				Input actualInput;

				DataHelper::fillTuple(input, actualInput, InputIndex());

				this->internalFilter<0, 0>(lastOutput, actualInput);

				DataHelper::fillVector(output, lastOutput, OutputIndex());
			}
		};

		template<typename ... FilterPolicies>
		struct FilterPolicyPack
		{
		private:

			template<typename TPack, typename ... FilterPolicies >
			struct PolicyPack;

			template<typename T, typename ... U>
			struct PolicyPack< TypeTools::TypePack<>, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack< typename FilterPolicyModifier<T>::Result >, U... >::Result Result;
			};

			template<typename ... Args, typename T, typename ... U>
			struct PolicyPack< TypeTools::TypePack<Args...>, T, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack< Args..., typename FilterPolicyModifier<T>::Result >, U... >::Result Result;
			};

			template<typename ... Args, typename T>
			struct PolicyPack< TypeTools::TypePack<Args...>, T>
			{
				typedef TypeTools::TypePack< Args..., typename FilterPolicyModifier<T>::Result > Result;
			};

			template<typename T, unsigned int ... index, typename ... U>
			struct PolicyPack< TypeTools::TypePack<>, T, CropData< index... >, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack<>, T, ArchPatterns::OutputFilter<T, index...>, U... >::Result Result;
			};

			template<typename ... Args, typename T, unsigned int ... index, typename ... U>
			struct PolicyPack< TypeTools::TypePack<Args...>, T, CropData< index... >, U... >
			{
				typedef typename PolicyPack< TypeTools::TypePack< Args... >, T, ArchPatterns::OutputFilter<T, index...>, U... >::Result Result;
			};

			template<typename T>
			struct PolicyPack< TypeTools::TypePack<>, T>
			{
				typedef TypeTools::TypePack< typename FilterPolicyModifier<T>::Result > Result;
			};

		public:

			typedef typename PolicyPack< TypeTools::TypePack<>, FilterPolicies... >::Result Result;
		};

		template<PipeType type, typename ... Filters>
		struct SelectPipe
		{
		private:

			template<PipeType t, typename ... Args>
			struct InternalSelectPipe;

			template<typename Arg, typename ... Args>
			struct InternalSelectPipe<SINGLE, Arg, Args...>
			{
				typedef ArchPatterns::Filter<Arg> Result;
			};

			template<typename ... Args>
			struct InternalSelectPipe<LOOP, Args...>
			{
				typedef ArchPatterns::LoopFilter<Args...> Result;
			};

			template<typename ... Args>
			struct InternalSelectPipe<SEQUENTIAL, Args...>
			{
				typedef ArchPatterns::SequentialPipe<Args...> Result;
			};

			template<typename ... Args>
			struct InternalSelectPipe<PARALLEL, Args...>
			{
				typedef ArchPatterns::ParallelPipe<Args...> Result;
			};

		public:

			typedef typename InternalSelectPipe<type, Filters...>::Result Result;
		};
	}

	template<typename ... FilterPolicies>
	class SequentialPipe : public Private::SequentialPipe< typename Private::FilterPolicyPack<FilterPolicies...>::Result >
	{
	private:

		typedef Private::SequentialPipe< typename Private::FilterPolicyPack< FilterPolicies... >::Result > Parent;

	public:

		template<typename ... Args>
		void filter(Args& ... args)
		{
			Parent::filter(args...);
		}

		const std::vector< std::type_index >& getOutputsInfo() const
		{
			return Parent::outputsInfo;
		}

		const std::vector< std::type_index >& getInputsInfo() const
		{
			return Parent::inputsInfo;
		}
	};

	template<>
	class SequentialPipe<> : public AbstractPipe
	{
	private:

		void configure()
		{
			this->inputsInfo = this->filters[0]->getInputsInfo();
			this->outputsInfo = this->filters[this->filters.size() - 1]->getOutputsInfo();
		}

	public:

		void addFilter(AbstractFilter* filter)
		{
			this->filters.push_back(filter);
			this->configure();
		}

		void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
		{
			std::vector<TypeTools::Any> output_;
			std::vector<TypeTools::Any> input_;

			this->filters[0]->allocateOutputs(output_, false);
			this->filters[0]->filter(output_, input);

			for (unsigned int i = 1; i < this->filters.size() - 1; i++)
			{
				input_ = output_;
				this->filters[i]->allocateOutputs(output_, false);
				this->filters[i]->filter(output_, input_);
			}

			this->filters[this->filters.size() - 1]->filter(output, output_);
		}
	};

	template<typename ... FilterPolicies>
	class ParallelPipe : public Private::ParallelPipe< typename Private::FilterPolicyPack< FilterPolicies... >::Result >
	{
	private:

		typedef Private::ParallelPipe< typename Private::FilterPolicyPack< FilterPolicies... >::Result > Parent;

	public:

		template<typename ... Args>
		void filter(Args& ... args)
		{
			Parent::filter(args...);
		}

		const std::vector< std::type_index >& getOutputsInfo() const
		{
			return Parent::outputsInfo;
		}

		const std::vector< std::type_index >& getInputsInfo() const
		{
			return Parent::inputsInfo;
		}
	};

	template<>
	class ParallelPipe<> : public AbstractPipe
	{
	private:

		class FilterRunnable
		{
		public:

			AbstractFilter* filter;
			std::vector<TypeTools::Any>& output;
			std::vector<TypeTools::Any>& input;

			FilterRunnable(AbstractFilter* filter, std::vector<TypeTools::Any>& output) :
				filter(filter), output(output), input(output)
			{
			}

			FilterRunnable& operator=(const FilterRunnable& other)
			{
				this->filter = other.filter;
				this->output = other.output;
				this->input = other.input;
			}

			void run()
			{
				this->filter->filter(this->output, this->input);
			}
		};

		std::vector< std::vector<TypeTools::Any> > internalOutputs;
		std::vector< LibTools::Thread<FilterRunnable>* > filterThreads;

		void configure()
		{
			this->inputsInfo.clear();
			this->outputsInfo.clear();

			this->internalOutputs.resize(this->filters.size(), std::vector<TypeTools::Any>());

			for (unsigned int i = 0; i < this->filterThreads.size(); i++)
			{
				delete filterThreads[i];
			}

			this->filterThreads.resize(this->filters.size(), 0);

			for (unsigned int i = 0; i < this->filterThreads.size(); i++)
			{
				this->filters[i]->allocateOutputs(internalOutputs[i]);

				filterThreads[i] = new LibTools::Thread<FilterRunnable>(this->filters[i], internalOutputs[i]);

				std::vector< std::type_index > outInfo = this->filters[i]->getOutputsInfo();

				for (unsigned int j = 0; j < outInfo.size(); j++)
				{
					this->outputsInfo.push_back(outInfo[j]);
				}
			}

			this->inputsInfo = this->filters[0]->getInputsInfo();
		}

	public:

		void addFilter(AbstractFilter* filter)
		{
			this->filters.push_back(filter);
			this->configure();
		}

		void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
		{
			for (unsigned int i = 0; i < this->filters.size(); i++)
			{
				filterThreads[i]->input = input;
				filterThreads[i]->start();
			}

			for (unsigned int i = 0; i < this->filters.size(); i++)
			{
				filterThreads[i]->join();
			}

			unsigned int k = 0;
			for (unsigned int i = 0; i < this->internalOutputs.size(); i++)
			{
				for (unsigned int j = 0; j < internalOutputs[i].size(); j++)
				{
					output[k] = internalOutputs[i][j];
					k++;
				}
			}
		}
	};

	template<PipeType type, typename ... Filters>
	using Pipe = typename Private::SelectPipe<type, Filters...>::Result;
}