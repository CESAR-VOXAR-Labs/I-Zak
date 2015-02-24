#ifndef __ResourceManager_H__
#define __ResourceManager_H__

namespace Resources
{
	template<typename Data>
	class ResourceManager : public DesignPatterns::Storehouse<>
	{
	private:
	protected:
	public:

		void read(Resource<Data>* resource)
		{			
		}

		void read(Resource<Data>* resource, unsigned long long begin, unsigned long long end)
		{
		}

		void write(Resource<Data>* resource)
		{
		}

		void write(Resource<Data>* resource, unsigned long long begin, unsigned long long end)
		{
		}

		void getResource();
	};
}

#endif