#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>


class Util
{
public:
    // string utilities
	static void ResolveFileName(const std::string& filename, std::string& file_path, std::string& file_title, std::string& file_ext);
	static void MakeLower(std::string& str);
	static void MakeUpper(std::string& str);

    // Free various vectors
    template <class T>
    static  void FreeVector(T& arr){
        T tmp;
        arr.clear();
        arr.swap(tmp);
    }

    // Flag utilities
    template <class T>
        static void SetFlag(T& flag_ele, T flag) { flag_ele |= flag; }

    template <class T>
        static void ClearFlag(T& flag_ele, T flag) { flag_ele &= ~flag; }

    template <class T>
        static bool IsSetFlag(const T& flag_ele, T flag) { return ((flag_ele&flag) == flag); }

    template <class T>
        static void ToggleFlag(T& flag_ele, T flag){
            if(IsSetFlag(flag_ele, flag)) ClearFlag(flag_ele, flag);
            else SetFlag(flag_ele, flag);
        }
};
