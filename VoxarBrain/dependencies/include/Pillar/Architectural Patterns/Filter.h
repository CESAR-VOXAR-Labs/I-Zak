#ifndef __Filter_H__
#define __Filter_H__

namespace ArchPatterns
{
	class AbstractLoopChecker
	{
	protected:

		std::vector< std::type_index > outputsInfo;

	public:

		virtual bool hasEnded(std::vector<TypeTools::Any>& output) = 0;

		const std::vector< std::type_index >& getOutputsInfo() const
		{
			return this->outputsInfo;
		}
	};

	template<typename ... Outputs>
	class LoopCheckerInterface : public AbstractLoopChecker
	{
	public:

		LoopCheckerInterface()
		{
			Private::DataHelper::fillInfo(this->outputsInfo, TypeTools::TypePack< Outputs... >());
		}

		virtual bool hasEnded(Outputs& ... outputs) = 0;
	};

	class AbstractFilter
	{
	protected:

		std::vector< std::type_index > outputsInfo;
		std::vector< std::type_index > inputsInfo;

	public:

		virtual void filter(std::vector< TypeTools::Any >& output, std::vector< TypeTools::Any >& input) = 0;

		virtual void allocateOutputs(std::vector< TypeTools::Any >& output, bool shrink = true)
		{
			unsigned int size = this->outputsInfo.size();

			if (size <= output.size() && !shrink)
				return;

			output.resize(size);
		}

		virtual void allocateInputs(std::vector< TypeTools::Any >& input, bool shrink = true)
		{
			unsigned int size = this->inputsInfo.size();

			if (size <= input.size() && !shrink)
				return;

			input.resize(size);
		}

		virtual const std::vector< std::type_index >& getOutputsInfo() const
		{
			return this->outputsInfo;
		}

		virtual const std::vector< std::type_index >& getInputsInfo() const
		{
			return this->inputsInfo;
		}
	};

	template<typename ... Args>
	class FilterInterface : public AbstractFilter
	{
	public:

		virtual void filter(Args&... args) = 0;
	};

	template<typename FilterPolicy>
	class Filter;

	template<typename Filter, typename LoopPolicy, bool flag>
	class LoopFilter;

	template<typename Filter, unsigned int ... index>
	class OutputFilter;

	namespace Private
	{
		template<typename Tuple, typename VPack>
		struct CropTuple
		{
		private:

			template<typename T, typename U>
			struct InternalCrop;

			template<typename T, unsigned int ... index>
			struct InternalCrop<T, TypeTools::ValuePack<index...>>
			{
				typedef std::tuple< typename std::tuple_element<index, T>::type... > Result;
			};

		public:

			typedef typename InternalCrop<Tuple, VPack>::Result Result;
		};

		template<
			typename FilterPolicy,
			typename FilterCommand = decltype(&FilterPolicy::filter),
			unsigned int nbOutputs = FilterPolicy::OutputsNumber
		>
		class Filter;

		template<typename FilterPolicy, unsigned int nbOutputs, typename ... Args>
		class Filter<FilterPolicy, void(FilterPolicy::*)(Args&...), nbOutputs> : public FilterPolicy,
																				 public FilterInterface<Args...>
		{
		protected:

			typedef TypeTools::TypePack<Args...> ArgsPack;

			typedef typename TypeTools::SliceTypePack<0, nbOutputs - 1, ArgsPack>::Result OutputPack;
			typedef typename TypeTools::SliceTypePack<nbOutputs, sizeof...(Args)-1, ArgsPack>::Result InputPack;

			typedef typename TypeTools::FillValuePack< nbOutputs >::Result OutputIndex;
			typedef typename TypeTools::FillValuePack< sizeof...(Args)-nbOutputs >::Result InputIndex;
			typedef typename TypeTools::FillValuePack< sizeof...(Args) >::Result AllIndex;

			template<typename VPackO, typename VPackI>
			class FilterHelper;

			template<unsigned int ... oValues, unsigned int ... iValues>
			class FilterHelper< TypeTools::ValuePack< oValues... >, TypeTools::ValuePack< iValues... > >
			{
			public:

				template<typename OutTuple, typename InTuple>
				static void filter(Filter* filter, OutTuple& out, InTuple& in)
				{
					filter->filter(std::get<oValues>(out)..., std::get<iValues>(in)...);
				}
			};

		public:

			Filter()
			{
				DataHelper::fillInfo(this->outputsInfo, OutputPack());
				DataHelper::fillInfo(this->inputsInfo, InputPack());
			}

			typedef typename TypeTools::TypePackToTuple< OutputPack >::Result Output;
			typedef typename TypeTools::TypePackToTuple< InputPack >::Result Input;

		protected:

			void filter(Args& ... args)
			{
				FilterPolicy::filter(args...);
			}

			void filter(Output& output, Input& input)
			{
				FilterHelper<OutputIndex, InputIndex>::filter(this, output, input);
			}

			void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
			{
				Output actualOutput;
				Input actualInput;

				DataHelper::fillTuple(input, actualInput, InputIndex());

				FilterHelper<OutputIndex, InputIndex>::filter(this, actualOutput, actualInput);

				DataHelper::fillVector(output, actualOutput, OutputIndex());
			}
		};

		template<
			typename LoopPolicy,
			typename LoopCommand = decltype(&LoopPolicy::hasEnded)
		>
		class LoopChecker;

		template<typename LoopPolicy, typename ... Args>
		class LoopChecker< LoopPolicy, bool(LoopPolicy::*)(Args&...) > : public LoopPolicy,
																		 public LoopCheckerInterface< Args... >
		{
		private:

			typedef typename TypeTools::FillValuePack< sizeof...(Args) >::Result ArgsIndex;

			template<unsigned int ... index>
			bool internalHasEnded(std::tuple< Args... >& tuple, TypeTools::ValuePack<index...>)
			{
				return LoopPolicy::hasEnded(std::get<index>(tuple)...);
			}

		protected:

			bool hasEnded(Args& ... args)
			{
				return LoopPolicy::hasEnded(args...);
			}

			bool hasEnded(std::tuple<Args...>& args)
			{
				return this->internalHasEnded(args, ArgsIndex());
			}

			bool hasEnded(std::vector<TypeTools::Any>& args)
			{
				std::tuple<Args...> actualArgs;

				DataHelper::fillTuple(args, actualArgs, ArgsIndex());

				return this->internalHasEnded(actualArgs, ArgsIndex());
			}
		};

		template<
			typename Filter,
			typename LoopChecker,
			typename Output = typename Filter::Output,
			typename Input = typename Filter::Input
		>
		class LoopFilter;

		template<typename Filter, typename LoopPolicy, typename ... Outputs, typename ... Inputs>
		class LoopFilter<Filter, LoopPolicy, std::tuple<Outputs...>, std::tuple<Inputs...> > : public LoopChecker<LoopPolicy>,
																							   public FilterInterface<Outputs..., Inputs...>
		{
		private:

			Filter f;

		protected:

			using FilterInterface<Outputs..., Inputs...>::outputsInfo;
			using FilterInterface<Outputs..., Inputs...>::inputsInfo;

			void filter(Outputs& ... outputs, Inputs&... inputs)
			{
				this->f.filter(outputs..., inputs...);
				while (!LoopChecker<LoopPolicy>::hasEnded(outputs...))
					this->f.filter(outputs..., inputs...);
			}

			void filter(std::tuple<Outputs...>& output, std::tuple<Inputs...>& input)
			{
				this->f.filter(output, input);
				while (!LoopChecker<LoopPolicy>::hasEnded(output))
					this->f.filter(output, input);
			}

			void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
			{
				this->f.filter(output, input);
				while (!LoopChecker<LoopPolicy>::hasEnded(output))
				{
					this->f.filter(output, input);
				}
			}

		public:

			LoopFilter()
			{
				DataHelper::fillInfo(this->outputsInfo, TypeTools::TypePack< Outputs... >());
				DataHelper::fillInfo(this->inputsInfo, TypeTools::TypePack< Inputs... >());
			}

			typedef std::tuple<Outputs...> Output;
			typedef std::tuple<Inputs...> Input;
		};

		template<
			typename InputTuple,
			typename InputIndexPack,
			typename OutputTuple = typename CropTuple< InputTuple, InputIndexPack >::Result,
			typename OutputIndexPack = typename TypeTools::FillValuePack< std::tuple_size< OutputTuple >::value >::Result
		>
		class OutputFilter;

		template<typename ... Inputs, unsigned int ... inputIndex, typename ... Outputs, unsigned int ... outputIndex >
		class OutputFilter< std::tuple<Inputs...>, TypeTools::ValuePack<inputIndex...>, std::tuple<Outputs...>, TypeTools::ValuePack<outputIndex...> > : public FilterInterface< Outputs..., Inputs... >
		{
		private:

			template<typename ... Index>
			void fillArray(unsigned int* a, unsigned int nextPos, unsigned int value, Index ... values)
			{
				a[nexPos] = value;
				this->fillArray(a, nextPos + 1, values...)
			}

			template<>
			void fillArray(unsigned int* a, unsigned int nextPos, unsigned int value)
			{
				a[nextPos] = value;
			}

		public:

			OutputFilter()
			{
				DataHelper::fillInfo(this->outputsInfo, TypeTools::TypePack< Outputs... >());
				DataHelper::fillInfo(this->inputsInfo, TypeTools::TypePack< Inputs... >());
			}

			typedef std::tuple<Outputs...> Output;
			typedef std::tuple<Inputs...> input;

		protected:

			void filter(Outputs& ... outputs, Inputs& ... inputs)
			{
				auto inputTuple = std::tie(inputs...);
				auto actualInputTuple = std::tie(std::get<inputIndex>(inputTuple)...);

				auto actualOutputTuple = std::tie(outputs...);
				actualOutputTuple = actualInputTuple;
			}

			void filter(std::tuple<Outputs...>& output, std::tuple<Inputs...>& input)
			{
				auto actualInputTuple = std::tie(std::get<inputIndex>(input)...);

				auto actualOutputTuple = std::tie(std::get<outputIndex>(output)...);
				actualOutputTuple = actualInputTuple;
			}

			void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
			{
				unsigned int* a = new unsigned int[sizeof...(inputIndex)];

				this->fillArray(a, 0, inputIndex...);

				for (unsigned int i = 0; i < sizeof...(inputIndex); i++)
				{
					output[i] = input[a[i]];
				}

				delete[] a;
			}
		};

		template<typename T, bool flag = std::is_base_of<AbstractFilter, T>::value>
		struct FilterPolicyModifier
		{
			typedef ArchPatterns::Filter<T> Result;
		};

		template<typename T>
		struct FilterPolicyModifier<T, true>
		{
			typedef T Result;
		};
	}

	template<
		typename FilterCommandType,
		FilterCommandType,
		typename FilterPolicyType = typename TypeTools::Decompose< FilterCommandType >::ObjectType,
		typename FilterCommandArgs = typename TypeTools::Decompose< FilterCommandType >::ArgsPack
	>
	class FilterPolicy;

	template<
		typename FilterCommandType,
		FilterCommandType command,
		typename FilterPolicyType,
		typename ... Args
	>
	class FilterPolicy<FilterCommandType, command, FilterPolicyType, TypeTools::TypePack< Args... >> : public FilterPolicyType
	{
	public:

		void filter(Args& ... args)
		{
			(this->*command)(args...);
		}
	};

	template<typename FilterPolicy>
	class Filter : public Private::Filter<FilterPolicy>
	{
	public:

		template<typename ... Args>
		void filter(Args& ... args)
		{
			Private::Filter<FilterPolicy>::filter(args...);
		}

		const std::vector< std::type_index >& getOutputsInfo() const
		{
			return Private::Filter<FilterPolicy>::outputsInfo;
		}

		const std::vector< std::type_index >& getInputsInfo() const
		{
			return Private::Filter<FilterPolicy>::inputsInfo;
		}
	};

	template<typename Filter = TypeTools::NullType,
		typename LoopPolicy = TypeTools::NullType,
		bool flag = std::is_same< LoopPolicy, TypeTools::NullType >::value >
	class LoopFilter;

	template<typename Filter, typename LoopPolicy>
	class LoopFilter<
		Filter,
		LoopPolicy,
		false
	> : public Private::LoopFilter< typename Private::FilterPolicyModifier< Filter >::Result,
	LoopPolicy
	>
	{
	private:

		typedef Private::LoopFilter< typename Private::FilterPolicyModifier< Filter >::Result,
			LoopPolicy
		> Parent;

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
	class LoopFilter<TypeTools::NullType, TypeTools::NullType, true> : public AbstractFilter
	{
	private:

		AbstractLoopChecker* checker;
		AbstractFilter* f;

	public:

		LoopFilter() : checker(0), f(0)
		{
		}

		void setFilter(AbstractFilter* filter)
		{
			this->f = filter;
			this->outputsInfo = this->f->getOutputsInfo();
			this->inputsInfo = this->f->getInputsInfo();
		}

		template<typename LoopPolicy>
		void setLoopChecker()
		{
			delete this->checker;
			this->checker = new Private::LoopChecker<LoopPolicy>();
		}

		void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
		{
			this->f->filter(output, input);
			while (!this->checker->hasEnded(output))
			{
				this->f->filter(output, input);
			}
		}
	};

	template<typename Filter = TypeTools::NullType, unsigned int ... index>
	class OutputFilter : public Private::OutputFilter< typename Private::FilterPolicyModifier< Filter >::Result::Output, TypeTools::ValuePack<index...> >
	{
	private:

		typedef Private::OutputFilter< typename Private::FilterPolicyModifier< Filter >::Result::Output, TypeTools::ValuePack<index...> > Parent;

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
	class OutputFilter<> : public AbstractFilter
	{
	private:

		unsigned int nbInputs;

		std::vector<unsigned int> index;

	public:

		void setIndex(unsigned int nbInputs, const std::vector<unsigned int>& outputIndex)
		{
			this->index = outputIndex;
			this->nbInputs = nbInputs;

			this->inputsInfo.resize(nbInputs, std::type_index(typeid(TypeTools::Any)));
			this->outputsInfo.resize(outputIndex.size(), std::type_index(typeid(TypeTools::Any)));
		}

		void filter(std::vector<TypeTools::Any>& output, std::vector<TypeTools::Any>& input)
		{
			for (unsigned int i = 0; i < this->index.size(); i++)
			{
				output[i] = input[this->index[i]];
			}
		}
	};
}

#define PILLAR_FILTER(FilterCommand) ArchPatterns::FilterPolicy< decltype(FilterCommand), FilterCommand > 

#endif