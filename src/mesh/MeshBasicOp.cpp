#include "MeshBasicOp.h"
#include "Mesh.h"
#include "MeshKernel.h"
#include "MeshInfo.h"
#include <queue>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <fstream>
using namespace std;

namespace MeshLib{

    MeshBasicOP::MeshBasicOP(Mesh& mesh) : m_mesh(mesh){}
    MeshBasicOP::~MeshBasicOP() {}

    void MeshBasicOP::InitModel()
    {
        CalFaceNormal();
        CalVertNormal();
        
        ModelInfo& m_info = *m_mesh.p_ModelInfo;
        m_info.m_nComponents = CountComponentNum();
        m_info.m_AvgEdgeLength = CalAvgEdgeLength();
        CalBoundingBox(m_info.m_BoxMin, m_info.m_BoxMax, m_info.m_BoxDim);
        CalBoundingSphere(m_info.m_SphereCenter, m_info.m_SphereRadius);
        
    }
    
    vector<VertHandle> MeshBasicOP::GetAdjVertArray(const VertHandle& vh) const
    {
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        const vector<HalfEdge>& he_vec = m_mesh.p_Kernel->GetHEArray();
        const Vert& vert = vert_vec[vh];
        vector<VertHandle> vh_vec;
        HalfEdgeHandle he_handle = vert.he_handle;
        if(he_handle == -1) return vh_vec;
        HalfEdgeHandle _he_handle = he_handle;
        do{
            HalfEdgeHandle op_he_hl = he_vec[_he_handle].oppo_he_handle;
            _he_handle = he_vec[op_he_hl].next_he_handle;
            const HalfEdge& he = he_vec[op_he_hl];
            vh_vec.push_back(he.vert_handle);
        }while(_he_handle != he_handle);

        return vh_vec;
    }

    vector<FaceHandle> MeshBasicOP::GetAdjFaceArray(const FaceHandle& vh) const
    {
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        const vector<HalfEdge>& he_vec = m_mesh.p_Kernel->GetHEArray();
        const Vert& vert = vert_vec[vh];
        vector<FaceHandle> fh_vec;
        HalfEdgeHandle he_handle = vert.he_handle;
        if(he_handle == -1) return fh_vec;
        HalfEdgeHandle _he_handle = he_handle;
        do{
            HalfEdgeHandle op_he_hl = he_vec[_he_handle].oppo_he_handle;
            _he_handle = he_vec[op_he_hl].next_he_handle;
            const HalfEdge& he = he_vec[op_he_hl];
            fh_vec.push_back(he.face_handle);
        }while(_he_handle != he_handle);

        return fh_vec;
    }
    
    void MeshBasicOP::CalBoundingBox(Coord3D& box_min, Coord3D& box_max, Coord3D& box_dim) const
    {
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        if(vert_vec.size() == 0) return;
        box_min = box_max = vert_vec[0].coord; 
        for(VertHandle vh = 0; vh < vert_vec.size(); ++vh){
            const Coord3D& coord = vert_vec[vh].coord;
            for(size_t k=0; k<3; ++k){
                box_min[k] = min(box_min[k], coord[k]);
                box_max[k] = max(box_max[k], coord[k]);
            }
        }
        box_dim = box_max - box_min;
    }   

    void MeshBasicOP::CalBoundingSphere(Coord3D& sphere_center, double radius) const
    {
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        if(vert_vec.size() == 0) return;
        sphere_center.setVec3Ds(0, 0, 0); 
        for(VertHandle vh = 0; vh < vert_vec.size(); ++vh){
            sphere_center += vert_vec[vh].coord;
        }
        sphere_center /= vert_vec.size();
        radius = (vert_vec[0].coord - sphere_center).abs();
        for(VertHandle vh = 0; vh < vert_vec.size(); ++vh){
            radius = max(radius, (vert_vec[vh].coord - sphere_center).abs());
        }
    }


    void MeshBasicOP::CalFaceNormal()
    {
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        vector<Face>& face_vec = m_mesh.p_Kernel->GetFaceArray();

        for(FaceHandle fh = 0; fh < face_vec.size(); ++fh){
            Face& face = face_vec[fh];
            const vector<VertHandle>& vh_vec = face.vert_handle_vec;
            const Vert& v0 = vert_vec[vh_vec[0]];
            const Vert& v1 = vert_vec[vh_vec[1]];
            const Vert& v2 = vert_vec[vh_vec[2]];
            face.normal = cross(v1.coord - v0.coord, v2.coord - v0.coord);
            if(!face.normal.normalize()) face.normal = COORD_AXIS_Z;
        }
    }

    void MeshBasicOP::CalVertNormal()
    {
        vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        const vector<Face>& face_vec = m_mesh.p_Kernel->GetFaceArray();

        for(VertHandle vh = 0; vh < vert_vec.size(); ++vh){
            Vert& vert = vert_vec[vh];
            vert.normal.setVec3Ds(0, 0, 0);
            const vector<FaceHandle>& adj_faces = GetAdjFaceArray(vh);
            for(size_t k=0; k<adj_faces.size(); ++k){
                const Face& face = face_vec[adj_faces[k]];
                vert.normal += face.normal;
            }
            if(!vert.normal.normalize()) vert.normal = COORD_AXIS_Z;
        }
    }

    size_t MeshBasicOP::CountComponentNum() const
    {
        size_t vert_num = m_mesh.p_ModelInfo->GetVertNum();

        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        size_t component_num = 0;
        std::vector<bool> visited_flag(vert_num, false);

        for(VertHandle vh = 0; vh < vert_vec.size(); ++vh){
            if(!visited_flag[vh]){
                component_num ++;
                queue<VertHandle> q;
                q.push(vh); visited_flag[vh] = true;
                while(!q.empty()){
                    VertHandle _vh = q.front(); q.pop();
                    const vector<VertHandle>& adj_vert = GetAdjVertArray(_vh);
                    for(size_t k=0; k<adj_vert.size(); ++k){
                        if(!visited_flag[adj_vert[k]] ){
                            q.push(adj_vert[k]); visited_flag[adj_vert[k]] = true;
                        }
                    }
                }
            }
        }
        return component_num;
    }


    double MeshBasicOP::CalAvgEdgeLength() const
    {
        const vector<Edge>& edge_vec = m_mesh.p_Kernel->GetEdgeArray();
        const vector<Vert>& vert_vec = m_mesh.p_Kernel->GetVertArray();
        const vector<HalfEdge>& he_vec = m_mesh.p_Kernel->GetHEArray();

        double sum_len = 0;
        size_t edge_num = edge_vec.size();
        for(size_t k=0; k<edge_num; ++k){
            const Edge& edge = edge_vec[k];
            const Vert& vert1 = vert_vec[he_vec[edge.he_handle_1].vert_handle];
            const Vert& vert2 = vert_vec[he_vec[edge.he_handle_2].vert_handle];

            double edge_len = (vert1.coord - vert2.coord).abs();
            sum_len += edge_len;
        }
        return sum_len / edge_num*1.0;
    }

}
