#include "utility.h"
#include <cassert>

void Util::ResolveFileName(const std::string& filename, std::string& file_path, std::string& file_title, std::string& file_ext)
{
    //! TODO: add linux . .. tracing
    
    char separator;
    #ifdef __WIN32
    separator = '\\';
    #else
    separator = '/';
    #endif
    
	size_t i;
    size_t lash_index = filename.rfind(separator);
	size_t dot_index = filename.rfind('.');
	size_t length = filename.length();

	for(i = 0; i <= lash_index; ++ i)	
		file_path += filename[i];
	for(i = lash_index+1; i < dot_index; ++ i)
		file_title += filename[i];
	for(i = dot_index; i < length; ++ i)
		file_ext += filename[i];
}

void Util::MakeLower(std::string& str)
{
    size_t length = str.length();
    for(size_t i = 0; i < length; ++ i){
        char& ch = str[i];
        if(ch >= 'A' && ch <= 'Z') ch = ch - 'A' + 'a';
    }
}

void Util::MakeUpper(std::string& str)
{
    size_t length = str.length();
    for(size_t i = 0; i < length; ++ i){
        char& ch = str[i];
        if(ch >= 'a' && ch <= 'a') ch = ch - 'a' + 'A';
    }
}
