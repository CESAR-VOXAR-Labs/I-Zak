#ifndef __Resource_H__
#define __Resource_H__

namespace Resources
{
	template<typename Data>
	class ResourceManager;

	enum LoadState
	{
		NOT_LOADED,
		PARTIALY_LOADED,
		FULLY_LOADED
	};

	enum UpdateState
	{
		NOT_UPDATED,
		PARTIALY_UPDATED,
		FULLY_UPDATED
	};

	template<typename Data>
	class Resource
	{
	private:

		friend class ResourceManager<Data>;

		template<bool flag>
		void destroyData()
		{
			delete this->data;
		}

		template<>
		void destroyData<false>()
		{
		}

	protected:

		Data data;

		UpdateState updateState;
		LoadState loadState;

		std::string name;
		std::string group;
		std::string type;

		Resource(const std::string& name, const std::string& group, const std::string& type, const Data& data) :
			name(name), group(group), type(type), data(data), loadState(FULLY_LOADED), updateState(FULLY_UPDATED)
		{
		}

		Resource(const std::string& name, const std::string& group = "", const std::string& type = "") :
			name(name), group(group), type(type), loadState(NOT_LOADED), updateState(NOT_UPDATED)
		{
		}

		virtual ~Resource()
		{
			this->destroyData< TypeTools::TypeTraits<Data>::isPointer >();
		}

	public:		

		const std::string& getName() const
		{
			return this->name;
		}

		const std::string& getGroup() const
		{
			return this->name;
		}

		const std::string& getType() const
		{
			return this->name;
		}

		const Data& getData() const
		{
			if (this->loadState)
				return this->data;
			else
				throw LibTools::Exception<Resource*>(this, "The resource is not loaded yet.");
		}

		void setData(const Data& data)
		{
			this->updateState = NOT_UPDATED;
			this->loadState = this->loadState ? this->loadState : FULLY_LOADED;
			this->data = data;
		}
	};
}

#endif