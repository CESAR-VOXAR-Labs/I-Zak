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

#ifndef __Pool_H__
#define __Pool_H__

namespace DesignPatterns
{
	/** \cond PRIVATE */
	namespace Private
	{
		template<typename T, T keyPtr, bool unique_>
		class Key
		{
		private:

			template<typename T_>
			struct KeyInfo;

			template<typename ObjectType_, typename KeyType_>
			struct KeyInfo<KeyType_(ObjectType_::*)() const>
			{
				typedef typename std::decay<KeyType_>::type KeyType;
				typedef ObjectType_ ObjectType;
			};

			template<typename ObjectType_, typename KeyType_>
			struct KeyInfo<KeyType_(ObjectType_::*)() volatile>
			{
				typedef typename std::decay<KeyType_>::type KeyType;
				typedef ObjectType_ ObjectType;
			};

			template<typename ObjectType_, typename KeyType_>
			struct KeyInfo<KeyType_(ObjectType_::*)() const volatile>
			{
				typedef typename std::decay<KeyType_>::type KeyType;
				typedef ObjectType_ ObjectType;
			};

			template<typename ObjectType_, typename KeyType_>
			struct KeyInfo<KeyType_(ObjectType_::*)()>
			{
				typedef typename std::decay<KeyType_>::type KeyType;
				typedef ObjectType_ ObjectType;
			};

		public:

			typedef typename KeyInfo<T>::KeyType KeyType;
			typedef typename KeyInfo<T>::ObjectType ObjectType;
			static const bool unique = unique_;

			static KeyType execute(ObjectType* obj)
			{
				return (obj->*keyPtr)();
			}

			static KeyType execute(ObjectType obj)
			{
				return (obj.*keyPtr)();
			}
		};

		template<typename ObjectType_, typename Key_, unsigned int index_>
		class RepositoryImpl
		{
		private:

			std::multimap< typename Key_::KeyType, unsigned int > pool;

		public:

			bool exist(typename Key_::KeyType key) const
			{
				return this->pool.count(key) > 0;
			}

			unsigned int count(typename Key_::KeyType key) const
			{
				return this->pool.count(key);
			}

			bool canInsert(const ObjectType_& obj) const
			{
				return !Key_::unique || !this->exist(Key_::execute(obj));
			}

			void insert(const ObjectType_& obj, const unsigned int id)
			{
				typename Key_::KeyType key = Key_::execute(obj);
				this->pool.insert(std::pair<typename Key_::KeyType, unsigned int>(key, id));
			}

			void removeById(const unsigned int id)
			{
				typedef std::multimap<typename Key_::KeyType, unsigned int>::iterator it_type;
				for (it_type iterator = this->pool.begin(); iterator != this->pool.end(); iterator++)
				{
					if (iterator->second == id)
					{
						this->pool.erase(iterator);
						return;
					}
				}
			}

			void remove(typename Key_::KeyType key)
			{
				std::multimap<typename Key_::KeyType, unsigned int>::iterator it = this->pool.find(key);
				this->pool.erase(it);
			}

			void removeAt(typename Key_::KeyType key, unsigned int index)
			{
				std::multimap<typename Key_::KeyType, unsigned int>::const_iterator it = this->pool.find(key);

				while (index)
				{
					it++;
					index--;
				}

				this->pool.erase(it);
			}

			void removeAll(typename Key_::KeyType key)
			{
				this->pool.erase(key);
			}

			void clear()
			{
				this->pool.clear();
			}

			void update(const ObjectType_& obj, unsigned int id)
			{
				this->removeById(id);
				this->insert(obj, id);
			}

			unsigned int get(typename Key_::KeyType key) const
			{
				return this->pool.find(key)->second;
			}

			unsigned int getAt(typename Key_::KeyType key, unsigned int index) const
			{
				std::multimap<typename Key_::KeyType, unsigned int>::const_iterator it = this->pool.find(key);

				while (index)
				{
					it++;
					index--;
				}

				return it->second;
			}

			auto getAll(typename Key_::KeyType key) const -> decltype(pool.find(key))
			{
				return this->pool.find(key);
			}
		};

		template<typename RepositoryType>
		class Helper
		{
		private:

			typedef typename RepositoryType::Type T;
			typedef typename RepositoryType::TList TList;

		public:
			template<unsigned int repoIndex>
			bool canInsert(const T& obj, RepositoryType* repo) const
			{
				if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) repo)->canInsert(obj))
					return this->canInsert<repoIndex - 1>(obj, repo);
				else
					return false;
			}

			template<>
			bool canInsert<0>(const T& obj, RepositoryType* repo) const
			{
				return ((const typename TypeTools::TypeAt< TList, 0 >::Result*) repo)->canInsert(obj);
			}

			template<unsigned int repoIndex>
			void insert_(const T& obj, RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, repoIndex >::Result*) repo)->insert(obj, this->nextId);
				this->insert_<repoIndex - 1>(obj, repo);
			}

			template<>
			void insert_<0>(const T& obj, RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, 0 >::Result*) repo)->insert(obj, repo->nextId);

				repo->repo.insert(std::pair< unsigned int, typename std::remove_reference<T>::type >(repo->nextId, obj));

				repo->nextId++;
			}

			template<unsigned int repoIndex>
			void remove_(const unsigned int id, unsigned int actualRepoIndex, RepositoryType* repo)
			{
				if (repoIndex != actualRepoIndex)
					((typename TypeTools::TypeAt< TList, repoIndex >::Result*) repo)->removeById(id);

				this->remove_<repoIndex - 1>(id, actualRepoIndex, repo);
			}

			template<>
			void remove_<0>(const unsigned int id, const unsigned int actualRepoIndex, RepositoryType* repo)
			{
				if (actualRepoIndex != 0)
					((typename TypeTools::TypeAt< TList, 0 >::Result*) repo)->removeById(id);
			}

			template<unsigned int repoIndex>
			void clear_(RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, repoIndex >::Result*) repo)->clear();

				this->clear_< repoIndex - 1 >(repo);
			}

			template<>
			void clear_<0>(RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, 0 >::Result*) repo)->clear();
			}

			template<unsigned int repoIndex>
			void update_(const T& obj, unsigned int id, RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, repoIndex >::Result*) repo)->update(obj, id);

				this->update_< repoIndex - 1 >(obj, id, repo);
			}

			template<>
			void update_<0>(const T& obj, unsigned int id, RepositoryType* repo)
			{
				((typename TypeTools::TypeAt< TList, 0 >::Result*) repo)->update(obj, id);
			}
		};

		template<typename T, typename KeyPack, typename IndexPack>
		class Repository;

		template<typename T, typename ... Keys, unsigned int ... Index>
		class Repository<T, TypeTools::TypePack<Keys...>, TypeTools::ValuePack<Index...>> : private RepositoryImpl<T, Keys, Index>...
		{
		private:

			friend class Helper<Repository<T, TypeTools::TypePack<Keys...>, TypeTools::ValuePack<Index...>>>;

			typedef TypeTools::TypePack< RepositoryImpl<T, Keys, Index>... > TList;
			typedef T Type;

			Helper<Repository<T, TypeTools::TypePack<Keys...>, TypeTools::ValuePack<Index...>>> helper;

			unsigned int nextId;
			std::map< unsigned int, typename std::remove_reference<T>::type > repo;

		protected:

			typedef typename std::conditional<
				std::is_reference<T>::value,
				std::vector< std::reference_wrapper<typename std::remove_reference<T>::type> >,
				std::vector< T >
			>::type AllReturn;

		public:

			Repository() : nextId(0){}

			unsigned int size() const
			{
				return this->repo.size();
			}

			template<unsigned int repoIndex, typename KeyType>
			bool exist(const KeyType& key) const
			{
				return ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key);
			}

			template<unsigned int repoIndex, typename KeyType>
			unsigned int count(const KeyType& key) const
			{
				return ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->count(key);
			}

			unsigned int insert(const T& obj)
			{
				if (this->helper.canInsert< TypeTools::Length< TList >::value - 1 >(obj, this))
				{
					this->helper.insert_< TypeTools::Length< TList >::value - 1 >(obj, this);
					return this->nextId - 1;
				}
				else
					throw LibTools::Exception< const typename std::remove_reference<T>::type >(obj, "There is already an object with the same key");
			}

			template<unsigned int repoIndex, typename KeyType>
			T get(const KeyType& key) const
			{
				if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
					return (T)this->repo.at(((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->get(key));
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			template<unsigned int repoIndex, typename KeyType>
			T getAt(const KeyType& key, unsigned int index) const
			{
				if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
				{
					if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->count(key) > index)
						return (T)this->repo.at(((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->getAt(key, index));
					else
						throw LibTools::Exception< const KeyType >(key, std::string("Index '").append(DataTools::StringTools::toString(index)).append("' is out of range"));
				}
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			template<unsigned int repoIndex, typename KeyType>
			AllReturn getAll(const KeyType& key) const
			{
				typedef
					typename std::conditional<
					std::is_reference<T>::value,
					std::vector< std::reference_wrapper<typename std::remove_reference<T>::type> >,
					std::vector< T >
					>::type ReturnType;

				typedef
					typename std::conditional<
					std::is_reference<T>::value,
					std::reference_wrapper<typename std::remove_reference<T>::type>,
					T
					>::type ObjectType;

				if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
				{
					ReturnType resp;

					auto allObj = ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->getAll(key);

					for (unsigned int i = 0; i < ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->count(key); i++)
					{
						ObjectType obj = (T)this->repo.at(allObj->second);
						resp.push_back(obj);
						allObj++;
					}

					return resp;
				}
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			T getById(unsigned int id) const
			{
				if (this->repo.count(id) > 0)
					return (T)this->repo.at(id);
				else
					throw LibTools::Exception< unsigned int >(id, "There is no object with this id");
			}

			T getAt(unsigned int index) const
			{
				if (this->repo.size() > index)
				{
					auto it = this->repo.begin();

					while (index)
					{
						it++;
						index--;
					}

					return (T)it->second;
				}
				else
					throw LibTools::Exception< unsigned int >(index, std::string("Index '").append(StringTools::toString(index)).append("' is out of range"));
			}

			AllReturn getAll() const
			{
				if (this->repo.size() > 0)
				{
					typedef
						typename std::conditional<
						std::is_reference<T>::value,
						std::vector< std::reference_wrapper<typename std::remove_reference<T>::type> >,
						std::vector< T >
						>::type ReturnType;

					typedef
						typename std::conditional<
						std::is_reference<T>::value,
						std::reference_wrapper<typename std::remove_reference<T>::type>,
						T
						>::type ObjectType;

					ReturnType resp;

					auto it = this->repo.begin();

					for (unsigned int i = 0; i < this->repo.size(); i++)
					{
						ObjectType obj = (T)this->repo.at(it->first);
						resp.push_back(obj);
						it++;
					}

					return resp;
				}
				else
					throw LibTools::Exception< const unsigned int >(0, "There is no object in the repository");
			}

			template<unsigned int repoIndex, typename KeyType>
			void remove(const KeyType& key)
			{
				if (((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
				{
					unsigned int id = ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->get(key);

					this->helper.remove_< TypeTools::Length< TList >::value - 1 >(id, repoIndex, this);
					((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->remove(key);

					this->repo.erase(id);
				}
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			template<unsigned int repoIndex, typename KeyType>
			void removeAt(const KeyType& key, unsigned int index)
			{
				if (((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
				{
					if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->count(key) > index)
					{
						unsigned int id = ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->getAt(key, index);

						this->helper.remove_< TypeTools::Length< TList >::value - 1 >(id, repoIndex, this);
						((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->removeAt(key, index);

						this->repo.erase(id);
					}
					else
						throw LibTools::Exception< const KeyType >(key, std::string("Index '").append(DataTools::StringTools::toString(index)).append("' is out of range"));
				}
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			template<unsigned int repoIndex, typename KeyType>
			void removeAll(const KeyType& key)
			{
				if (((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->exist(key))
				{
					auto allObj = ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->getAll(key);

					for (unsigned int i = 0; i < ((const typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->count(key); i++)
					{
						unsigned int id = allObj->second;

						this->helper.remove_< TypeTools::Length< TList >::value - 1 >(id, repoIndex, this);

						this->repo.erase(id);

						allObj++;
					}

					((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->removeAll(key);
				}
				else
					throw LibTools::Exception< const KeyType >(key, "There is no object with this key");
			}

			void removeAt(unsigned int index)
			{
				if (this->repo.size() > index)
				{
					auto it = this->repo.begin();

					while (index)
					{
						it++;
						index--;
					}

					this->helper.remove_< TypeTools::Length< TList >::value - 1 >(it->first, -1, this);

					this->repo.erase(it);
				}
				else
					throw LibTools::Exception< unsigned int >(index, std::string("Index '").append(DataTools::StringTools::toString(index)).append("' is out of range"));
			}

			void clear()
			{
				this->helper.clear_< TypeTools::Length< TList >::value - 1 >(this);
				this->repo.clear();

				this->nextId = 0;
			}

			template<unsigned int repoIndex>
			void update(const T& obj)
			{
				for (auto pair : this->repo)
				{
					if (pair.second == obj)
					{
						((typename TypeTools::TypeAt< TList, repoIndex >::Result*) this)->update(obj, pair.first);
						return;
					}
				}

				throw LibTools::Exception< const typename std::remove_reference<T>::type >(obj, "This object does not exist in the repository");
			}

			void update(const T& obj)
			{
				for (auto pair : this->repo)
				{
					if (pair.second == obj)
					{
						this->helper.update_< TypeTools::Length< TList >::value - 1 >(obj, pair.first, this);
						return;
					}
				}

				throw LibTools::Exception< const typename std::remove_reference<T>::type >(obj, "This object does not exist in the repository");
			}
		};
	}
	/** \endcond */

	/**
	* Class responsible for implemente the Pool Design Pattern
	* @tparam T Type that must be stored.
	* @tparam Keys... Variadic set of keys (Private::Key) used to index the stored objects.
	*/
	template<typename T, typename ... Keys>
	class Repository : public Private::Repository<T, 
												  TypeTools::TypePack<Keys...>,
												  typename TypeTools::FillValuePack< sizeof...(Keys) >::Result>
	{
	private:

		typedef Private::Repository<T, TypeTools::TypePack<Keys...>, typename TypeTools::FillValuePack< sizeof...(Keys) >::Result> Parent;

	public:

		/**
		* Returns the current number of objects strored.
		* @return The number of objects stored.
		*/
		unsigned int size() const
		{
			return Parent::size();
		}

		/**
		* Are there any objects stored with this key?
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @return True if there is any. 
		*/
		template<unsigned int repoIndex, typename KeyType>
		bool exist(const KeyType& key) const
		{
			return Parent::exist<repoIndex>(key);
		}

		/**
		* Counts the number of objects indexed by the key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @return The number of objects with this key.
		*/
		template<unsigned int repoIndex, typename KeyType>
		unsigned int count(const KeyType& key) const
		{
			return Parent::count<repoIndex>(key);
		}

		/**
		* Inserts an object into the pool.
		* @param obj The object to store.
		* @return The unique id generated for the stored object.
		*/
		unsigned int insert(const T& obj)
		{
			return Parent::insert(obj);
		}

		/**
		* Returns the first object indexed by the key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @return The object instance.
		*/
		template<unsigned int repoIndex, typename KeyType>
		T get(const KeyType& key) const
		{
			return Parent::get<repoIndex>(key);
		}

		/**
		* Returns the nth object indexed by the key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @param index The object position.
		* @return The object instance.
		*/
		template<unsigned int repoIndex, typename KeyType>
		T getAt(const KeyType& key, unsigned int index) const
		{
			return Parent::getAt<repoIndex>(key, index);
		}

		/**
		* Returns all the objects indexed by the key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @return A std::vector containing the objects (if T is a reference, the vector will contain reference_wrapper<T> instances).
		*/
		template<unsigned int repoIndex, typename KeyType>
		auto getAll(const KeyType& key) const -> typename Parent::AllReturn
		{
			return Parent::getAll<repoIndex>(key);
		}

		/**
		* Returns the object identified by the id.
		* @param id The object id.
		* @return The object instance.
		*/
		T getById(unsigned int id) const
		{
			return Parent::getById(id);
		}

		/**
		* Returns the nth object.
		* @param index The object position.
		* @return The object instance.
		*/
		T getAt(unsigned int index) const
		{
			return Parent::getAt(index);
		}

		/**
		* Returns all the objects in the pool.
		* @return A std::vector containing the objects (if T is a reference, the vector will contain reference_wrapper<T> instances instead).
		*/
		auto getAll() const -> typename Parent::AllReturn
		{
			return Parent::getAll();
		}

		/**
		* Removes the first object identified by the given key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		*/
		template<unsigned int repoIndex, typename KeyType>
		void remove(const KeyType& key)
		{
			Parent::remove<repoIndex>(key);
		}

		/**
		* Removes the object identified by the given key at the nth index.
		* @tparam repoIndex Key index.
		* @param key The object key.
		* @param index The object index.
		*/
		template<unsigned int repoIndex, typename KeyType>
		void removeAt(const KeyType& key, unsigned int index)
		{
			Parent::removeAt<repoIndex>(key, index);
		}

		/**
		* Removes all the objects identified by the given key.
		* @tparam repoIndex Key index.
		* @param key The object key.
		*/
		template<unsigned int repoIndex, typename KeyType>
		void removeAll(const KeyType& key)
		{
			Parent::removeAll<repoIndex>(key);
		}

		/**
		* Removes the nth object.
		* @param index The object position.		
		*/
		void removeAt(unsigned int index)
		{
			Parent::removeAt(index);
		}

		/**
		* Removes all the objects and resets the pool status.
		*/
		void clear()
		{
			Parent::clear();
		}

		/**
		* Updates the nth key of the given object if it exists in the pool.
		* @tparam repoIndex Key index.
		* @param obj The object to be updated.
		*/
		template<unsigned int repoIndex>
		void update(const T& obj)
		{
			return Parent::update<repoIndex>(obj);
		}

		/**
		* Updates all the keys of the given object if it exists in the pool.		
		* @param obj The object to be updated.
		*/
		void update(const T& obj)
		{
			return Parent::update(obj);
		}
	};
}

/** 
* Macro used to define a Private::Key object.
* @param key An object pointer-to-member.
* @param isUnique A flag indicating if the key must be unique.
*/
#define PILLAR_KEY(key, isUnique) DesignPatterns::Private::Key< decltype(key), key, isUnique>

#endif