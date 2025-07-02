


#ifndef PATH_HASH_TABLE
#define PATH_HASH_TABLE

//#define PATH_HASH_TABLE_TEST

#include <vector>
#include <map>
#include "primes.h"
#include <cstddef>
#include <stdint.h>
#include <typeinfo>
#include <unordered_set>

template <class T>
class PathHashTableData
{
	public:
	unsigned long long key1;
	uint64_t key2;
	//uint32_t key2;
	T data;
	#ifdef PATH_HASH_TABLE_TEST
	std::vector<int> path;
	#endif
};

template <class T>
class PathHashTableCase
{
	public:
	
	std::vector< PathHashTableData<T> > datas;
};

template <class T>
class PathHashTable 
{
	public:
		
		PathHashTable() : _len(0), _cases(0),_count(0){}
		PathHashTable(size_t len) : _len(len), _cases(len),_count(0){
			printf("PathHashTable constructor size of cases:%zu type:%s\n",len,typeid(T).name());
		}
	
		~PathHashTable()
		{
			#ifdef PATH_HASH_TABLE_TEST
			CheckCollision();
			#endif
			for(size_t i=0;i<_cases.size();i++)
				if(_cases[i] != NULL)
					delete _cases[i];
		}
	
		void Assign(std::vector<int> & path, T data)
		{
			if(path.size() == 0) return;
			
			unsigned long long key1 = GetSetId(path);
			//uint32_t key2 = GetPathId(path);
			uint64_t key2 = GetPathId(path);
			size_t pos = (size_t)(key1 % (unsigned long long)_len);
			
			if(_cases[pos] == NULL)
			{
				_cases[pos] = new PathHashTableCase<T>();
				PathHashTableData<T> dt;
				dt.key1 = key1;
				dt.key2 = key2;
				dt.data = data;
				
				//printf("Adding new entry key1:%lld key:%u pos:%zu\n", key1, key2, pos);
				
				#ifdef PATH_HASH_TABLE_TEST
				dt.path = path;
				#endif
				_cases[pos]->datas.push_back(dt);
				_count++;
			}
			else
			{
				//find in the case if there is a DataBlock for this path
				bool found = false;
				for(size_t i=0;i<_cases[pos]->datas.size();i++)
					if(_cases[pos]->datas[i].key1 == key1 && _cases[pos]->datas[i].key2 == key2)
					{
						_cases[pos]->datas[i].data = data;					
						#ifdef PATH_HASH_TABLE_TEST
							//check if they have both the same path, otherwise output an warning
							TestPaths(key1,key2, _cases[pos]->datas[i].path,path);
						#endif
						found=true;
						break;	
					}
				if(!found)
				{
					PathHashTableData<T> dt;
					dt.key1 = key1;
					dt.key2 = key2;
					dt.data = data;	
					#ifdef PATH_HASH_TABLE_TEST
					dt.path = path;
					#endif
					_cases[pos]->datas.push_back(dt);
					_count++;
				}
			}
		}
		
		T GetData(std::vector<int> & path)
		{
			if(path.size() == 0) return T();
			
			unsigned long long key1 = GetSetId(path);
			//unsigned int key2 = GetPathId(path);
			uint64_t key2 = GetPathId(path);
			size_t pos = (size_t)(key1 % (unsigned long long)_len);
			
			//printf("Key1: %llu, Key2: %u, Pos: %zu TotalCases: %zu\n", key1, key2, pos,_cases.size());
			
			if(_cases[pos] == NULL) return T();
			
			for(size_t i=0;i<_cases[pos]->datas.size();i++)
				if(_cases[pos]->datas[i].key1 == key1 && _cases[pos]->datas[i].key2 == key2)
				{
					#ifdef PATH_HASH_TABLE_TEST
						//check if they have both the same path, otherwise output an warning
						TestPaths(key1,key2, _cases[pos]->datas[i].path,path);
					#endif
					return _cases[pos]->datas[i].data;
				}
			return T();
		}
		
		bool GetData(std::vector<int> & list, T ** data)
		{
			if(list.size() == 0) return false;
			
			unsigned long long key1 = GetSetId(list);
			//unsigned int key2 = GetPathId(list);
			uint64_t key2 = GetPathId(list);
			size_t pos = (size_t)(key1 % (unsigned long long)_len);
			if(_cases[pos] == NULL) return false;
			
			//printf("BooleanGetData Key1: %llu, Key2: %u, Pos: %zu TotalCases: %zu\n", key1, key2, pos,_cases.size());
			
			for(size_t i=0;i<_cases[pos]->datas.size();i++)
				if(_cases[pos]->datas[i].key1 == key1 && _cases[pos]->datas[i].key2 == key2)
				{
					*data = &_cases[pos]->datas[i].data;
					return true;
				}
			return false;	
		}
		
		void Remove(std::vector<int> & path)
		{
			if(path.size() == 0) return;
			
			unsigned long long key1 = GetSetId(path);
			unsigned int key2 = GetPathId(path);
			size_t pos = (size_t)(key1 % (unsigned long long)_len);
			if(_cases[pos] == NULL) return ;
			
			for(size_t i=0;i<_cases[pos]->datas.size();i++)
				if(_cases[pos]->datas[i].key1 == key1 && _cases[pos]->datas[i].key2 == key2)
				{
					_cases[pos]->datas.erase( _cases[pos]->datas.begin() + i );
					_count--;
					break;
				}
		}
		
		uint64_t GetSetId(const std::vector<int>& path) 
		{
			std::unordered_set<int> seen(path.begin(), path.end());
			uint64_t hash = 14695981039346656037ull;  // FNV offset
			for (int x : seen) {
				hash ^= static_cast<uint64_t>(x);
				hash *= 1099511628211ull;  // FNV prime
			}
			return hash;
		}	

		/* Prime multiplications can exceed integer capacity
		unsigned long long GetSetId(std::vector<int> & path)
		{
			unsigned long long key = 1;
			//printf("path size:%d numbers:\n",(int)path.size());
			for(size_t i=0;i<path.size();i++)
			{
				//printf("i:%d path_i:%d\n",(int)i,path[i]);
				key *= prime_get_ith(path[i]);
			}
				
			return key;
		}*/
		
		uint64_t GetPathId64(const std::vector<int>& path) 
		{
			uint64_t hash = 14695981039346656037ull; // FNV offset basis
			for (int x : path) {
				hash ^= (uint64_t)(x);           // XOR the current path element
				hash *= 1099511628211ull;                   // Multiply by the FNV prime
			}

			// Final mixing (borrowed from MurmurHash3 to improve avalanche) : small changes in bit string produce large changes in output
			hash ^= hash >> 33;
			hash *= 0xff51afd7ed558ccdULL;
			hash ^= hash >> 33;
			hash *= 0xc4ceb9fe1a85ec53ULL;
			hash ^= hash >> 33;

			return hash;
		}

		
		uint32_t GetPathId(std::vector<int> & path)
		{
		    //Used in 2SL-CVRP
			/*if (path.size() == 0) return 0;
			if(path[0] > path[ path.size() - 1 ])
			{
				int half = (int)(path.size() / 2);
				for(int i=0;i<half;i++)
				{
					int t = path[i];
					path[i] = path[	path.size() - i - 1 ];
					path[	path.size() - i - 1 ] = t;
				}
			}
				
			uint32_t len = (uint32_t)path.size();
			uint32_t hash, i;
		    for(hash = i = 0; i < len; ++i)
		    {
		        hash += (path[i]+1); 	//added by JFC to avoid the case where path[i] = 0
		        hash += (hash << 10);
		        hash ^= (hash >> 6);
		    }
		    hash += (hash << 3);
		    hash ^= (hash >> 11);
		    hash += (hash << 15);
		    return hash;*/
			
			//Murmur2 hash
		    int32_t len = (int32_t)path.size();
		    uint32_t hash = (uint32_t)len;
		    int32_t remainingBytes = len & 3; // mod 4
		    int32_t numberOfLoops = len >> 2; // div 4
		    int32_t currentIndex = 0;
		    while (numberOfLoops > 0)
		    {
		        hash += (uint16_t)(path[currentIndex++] | path[currentIndex++] << 8);
		        uint32_t tmp = (uint32_t)((uint32_t)(path[currentIndex++] | path[currentIndex++] << 8) << 11) ^ hash;
		        hash = (hash << 16) ^ tmp;
		        hash += hash >> 11;
		        numberOfLoops--;
		    }
		
		    switch (remainingBytes)
		    {
		        case 3:
		            hash += (uint16_t)(path[currentIndex++] | path[currentIndex++] << 8);
		            hash ^= hash << 16;
		            hash ^= ((uint32_t)path[currentIndex]) << 18;
		            hash += hash >> 11;
		            break;
		        case 2:
		            hash += (uint16_t)(path[currentIndex++] | path[currentIndex] << 8);
		            hash ^= hash << 11;
		            hash += hash >> 17;
		            break;
		        case 1: 
		            hash += path[currentIndex];
		            hash ^= hash << 10;
		            hash += hash >> 1;
		            break;
		        default:
		            break;
		    }
		
		    // Force "avalanching" of final 127 bits
		    hash ^= hash << 3;
		    hash += hash >> 5;
		    hash ^= hash << 4;
		    hash += hash >> 17;
		    hash ^= hash << 25;
		    hash += hash >> 6;
		
		    return hash;
		    
		}
	
		//check if they have both the same path, otherwise output an warning
		void TestPaths(unsigned long long key1,uint32_t key2,std::vector<int> & path1, std::vector<int> & path2)
		{
			if(path1.size() != path2.size())
			{
				printf("Warning different paths have the same keys key1:%llu key2:u\n",key1,key2);
				printf("path1:"); 
				for(size_t j=0;j<path1.size();j++)
					printf("%d ", path1[j]);
				printf("\npath2:");
				for(size_t j=0;j<path2.size();j++)
					printf("%d ", path2[j]);
				printf("\n");
				exit(1);
			}
			else	
				for(size_t j=0;j<path1.size();j++)
					if(path1[j] != path2[j])
					{
						printf("Warning different paths have the same keys key1:%llu key2:%u\n",key1,key2);
						printf("path1:"); 
						for(size_t j=0;j<path1.size();j++)
							printf("%d ", path1[j]);
						printf("\npath2:");
						for(size_t j=0;j<path2.size();j++)
							printf("%d ", path2[j]);
						printf("\n");
						break;
						exit(1);
					}
		}
	
		void CheckCollision()
		{
			int nbelements = 0;
			int nbnonnull = 0;
			int max_nb_diff = 0;
			for(size_t i=0;i<_cases.size();i++)
				if(_cases[i] != NULL)
				{
					nbnonnull++;
					nbelements += (int)_cases[i]->datas.size();
					
					int nbdiff = 0;
					std::map<unsigned long long, int> keys;
					for(size_t j=0;j<_cases[i]->datas.size();j++)
					{
						int v = keys[ _cases[i]->datas[j].key1 ];
						if(v != 0) nbdiff++;
						keys[ _cases[i]->datas[j].key1 ] = 1;	
					}
					max_nb_diff = std::max(nbdiff, max_nb_diff);
				}
			
			
			printf("CheckCollision\n");
			printf("nbelements:%d\n", nbelements);
			printf("nbnonnull:%d\n", nbnonnull);
			printf("maxdiff:%d\n", max_nb_diff);
			
		}
		
		int GetCount(){ return _count;}
		
	private:
	size_t _len;
	std::vector< PathHashTableCase<T>* > _cases;
	int _count;
};



#endif


