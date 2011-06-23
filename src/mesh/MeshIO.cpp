#include "MeshIO.h"
#include "Mesh.h"
#include "MeshKernel.h"
#include "../util/Utility.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

namespace MeshLib{

    MeshIO::MeshIO(Mesh& mesh) : m_mesh(mesh){}
    MeshIO::~MeshIO(){}


    bool MeshIO::LoadModel(const std::string& filename)
    {
        // Resolve file name
        std::string file_path, file_title, file_ext;
        Utility::ResolveFileName(filename, file_path, file_title, file_ext);

        printf("Load Model %s... ", (file_title+file_ext).c_str());

        // Set model informaion
        ModelInfo& mInfo = *m_mesh.p_ModelInfo;
        mInfo.m_FileName = filename;

        // Load model
        Utility::MakeLower(file_ext);

        bool bOpenFlag;
        if(file_ext == ".tm"){
            bOpenFlag = OpenTmFile(filename);
        }else if(file_ext == ".ply2"){
            bOpenFlag = OpenPly2File(filename);
        }else if(file_ext == ".off"){
            bOpenFlag = OpenOffFile(filename);
        }else if(file_ext == ".obj"){
            bOpenFlag = OpenObjFile(filename);
        }else{
            bOpenFlag = false;
        }

        printf("%s\n", (bOpenFlag == true) ? "Success" : "Fail");
        if(bOpenFlag){
            size_t nVertex = mInfo.m_nVertices;
            size_t nFace   = mInfo.m_nFaces;
            printf("#Vertex = %d, #Face = %d\n\n", nVertex, nFace);
        }

        return bOpenFlag;
    }

    bool MeshIO::StoreModel(const std::string& filename) const
    {
        // Resolve file name
        std::string file_path, file_title, file_ext;
        Utility::ResolveFileName(filename, file_path, file_title, file_ext);

        printf("Store Model %s... ", (file_title+file_ext).c_str());

        // Store model
        Utility::MakeLower(file_ext);

        bool bSaveFlag;
        if(file_ext == ".tm")
            {
                bSaveFlag = SaveTmFile(filename);
            }
        else if(file_ext == ".ply2")
            {
                bSaveFlag = SavePly2File(filename);
            }
        else if(file_ext == ".off")
            {
                bSaveFlag = SaveOffFile(filename);
            }
        else if(file_ext == ".obj")
            {
                bSaveFlag = SaveObjFile(filename);
            }
        else
            {
                bSaveFlag = false;
            }

        printf("%s\n", (bSaveFlag == true) ? "Success" : "Fail");

        return bSaveFlag;
    }

    // .tm file I/O functions
    bool MeshIO::OpenTmFile(const std::string& filename)
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    bool MeshIO::SaveTmFile(const std::string& filename) const
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    // .ply2 file I/O functions
    bool MeshIO::OpenPly2File(const std::string& filename)
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    bool MeshIO::SavePly2File(const std::string& filename) const
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    // .off file I/O functions
    bool MeshIO::OpenOffFile(const std::string& filename)
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    // .off file I/O functions
    bool MeshIO::SaveOffFile(const std::string& filename) const
    {
        //! TODO:
        std::cerr<<"@@ERROR@@:code havn't write ." << __FILE__ << __LINE__ << std::endl;
        return false;
    }

    bool MeshIO::OpenObjFile(const std::string& filename)
    {
        int nVertex = 0, nFace = 0, nVertTex = 0,nVertNorm = 0;

        std::string str;
        std::ifstream ifs(filename.c_str());

        if(ifs.fail()) return false;

        // find the vertex and face number here.
        while(!ifs.eof()) {
            getline(ifs, str, '\n');
            if(ifs.fail()) break;
            if(str.empty()) continue;
        
            if(str[0] == '#')	continue;
            if(str[0] == 'g')	continue;
            if(str[0] == 'v' && str[1] == ' ') 
                {
                    ++nVertex; continue;
                }
            if(str[0] == 'f' && str[1] == ' ') 
                {
                    ++nFace; continue;
                }
            if(str[0] == 'v' && str[1] == 't')
                {
                    ++nVertTex; continue;
                }
            if(str[0] == 'v' && str[1] == 'n')
                {
                    ++nVertNorm; continue;
                }
        }

        ifs.clear();
        ifs.seekg(0, std::ifstream::beg);        

        std::vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        std::vector<Face>& face_vec = m_mesh.p_Kernel->GetFaceArray();
        
        size_t vn = 0, fn = 0, vt_num = 0, vn_num = 0;

        std::string s, ss, line;
        std::stringstream stream;

        int len, mark, c, v, n, t;

        while(std::getline(ifs,line)){
            switch(line[0]){
			case '#':	break;
			case 'v':	/* v, vn , vt*/
				stream << line;
				stream >> s;
				if(s.length() == 1)	{                    
                    stream >> vert_vec[vn].coord[0] >> vert_vec[vn].coord[1] >> vert_vec[vn].coord[2];
					stream.str(""); stream.clear(); 
					++vn;
				}else if(s[1] == 'n') {
                    stream >> vert_vec[vn].normal[0] >> vert_vec[vn].normal[1]>> vert_vec[vn].normal[2];
					stream.str(""); stream.clear();
					++vt_num;
				}else if(s[1] == 't') {
                    //TODO: add vertex texture coordinate
					++vn_num;
				}
				break;

			case 'f':	/* f */
				len=(int)line.length();
				while(line[len-1]=='\\') {
                    getline(ifs,ss);
                    line[len-1]=' '; line +=ss;
                    len=(int)line.length();
                }
                //                Face& face = face_vec[fn];
				v=0, t=0, n=0;
				for(int i=1;i<len;i++){
                    c=line[i]-'0';
                    if(line[i-1]==' '&&isdigit(line[i])){ // v begin
                        mark=1;	v= c;
                    }else if(isdigit(line[i-1]) && isdigit(line[i])){ // di form
                        if(mark==1) v = v*10 + c; 
                        else if(mark==2) t = t*10 + c; 
                        else if(mark==3) n = n*10 + c; 
                    }else if(line[i-1]=='/' && isdigit(line[i])){
                        if(mark==1){ // t begin;
                            mark = 2;	t = c; 
                        }else if(mark==2){ // n begin
                            mark = 3;	n = c; 
                        }
                    }else if(line[i-1]=='/' && line[i]=='/'){
                        mark=2;
                    }
                    if((line[i]==' '&&isdigit(line[i-1]))|| i == (int) line.length()-1){
                        face_vec[fn].vert_handle_vec.push_back(v-1);
                        if(t >=1){
                            // TODO: add texture index for this face
                        }
                        v = t = n  = 0;
                    }
                }
				++fn;
				break;
			default:	break;
            }
        }

        ifs.close();

        return true;
    }
    bool MeshIO::SaveObjFile(const std::string& filename) const
    {
        std::ofstream file(filename.c_str());
        if(!file)
            return false;

        file << "# " << std::endl;
        file << "# Wavefront OBJ file" << std::endl;
        file << "# object ..." + filename << std::endl;
        
        const std::vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        const std::vector<Face>& face_vec = m_mesh.p_Kernel->GetFaceArray();
        // Store vertex information
        for(size_t i=0; i<vert_vec.size(); ++i){
            const Vert& vert = vert_vec[i];
            file << "v ";
            for(size_t j=0; j<3; ++j)
                file << vert.coord[j] << ((j<2) ? ' ' : '\n');
        }
        // TODO: Store vertex texture
    
        bool with_tex = false;
        // Store face information
        for(size_t i = 0; i < face_vec.size(); ++ i){
            const Face& face = face_vec[i];
            const std::vector<VertHandle>& vert_handle_vec = face.vert_handle_vec;
            file << "f ";
            if(!with_tex){
                for(size_t j = 0; j < vert_handle_vec.size(); ++ j){
                    file << vert_handle_vec[j] + 1 << ((j<vert_handle_vec.size()-1) ? ' ' : '\n');
                }
            }else{}
        }

        file.close();
	
        return true;
    }
}
