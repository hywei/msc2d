#ifndef MESHMODELIO_H_
#define MESHMODELIO_H_

#include <string>

namespace meshlib{
    
    class Mesh;

    class MeshIO
    {
    private:
        Mesh& m_mesh;
    public:
        MeshIO(Mesh& mesh);
        ~MeshIO();

        // General I/O functions
        bool LoadModel(const std::string& filename);
        bool StoreModel(const std::string& filename) const;

    private:
        // .tm file I/O functions
        bool OpenTmFile(const std::string& filename);
        bool SaveTmFile(const std::string& filename) const;

        // .ply2 file I/O functions
        bool OpenPly2File(const std::string& filename);
        bool SavePly2File(const std::string& filename) const;

        // .off file I/O functions
        bool OpenOffFile(const std::string& filename);
        bool SaveOffFile(const std::string& filename) const;
	
        // .obj file I/O functions
        bool OpenObjFile(const std::string& filename);
        bool SaveObjFile(const std::string& filename) const;
    };
} // 
#endif
