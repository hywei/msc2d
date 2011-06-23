#include "MeshKernel.h"
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <cassert>
using namespace std;

namespace MeshLib{
    MeshKernel::MeshKernel(Mesh& _mesh) : mesh(_mesh) {}
    MeshKernel::~MeshKernel() {}
    
    void MeshKernel::AnalysisModel()
    {
        bool tri_mesh(true), quad_mesh(false), poly_mesh(false), manifold(true);
        typedef pair<VertHandle, VertHandle> Mesh_Edge;
        map< Mesh_Edge, vector<FaceHandle> > edge_map;
        map< VertHandle, set<VertHandle> > vert_adjacent;
        
        for(size_t k=0; k<face_vec.size(); ++k){
            const Face& face = face_vec[k];
            size_t vert_num = face.vert_handle_vec.size();
            if(vert_num < 3) {
                //TODO: set MESHFLAG bad mesh
            }            
            if(vert_num !=3 && vert_num != 4){
                //TODO: set MESHFLAG general ploy
                poly_mesh = true; tri_mesh = quad_mesh = false;
            }
            if(vert_num == 4){
                //TODO: set MESHFLAG QUAD mesh
                if(!poly_mesh) { quad_mesh = true; tri_mesh = false;}
            }
            for(size_t i=0; i<face.vert_handle_vec.size(); ++i){
                VertHandle cur_handle = face.vert_handle_vec[i];
                VertHandle nxt_handle = face.vert_handle_vec[(i+1)%vert_num];
                VertHandle pre_handle = face.vert_handle_vec[(i+vert_num-1)%vert_num];

                if(cur_handle == nxt_handle){
                    //TODO: set FACEFLAG non-manifold
                    manifold = false;
                    continue;
                }

                Mesh_Edge me = (cur_handle < nxt_handle) ? make_pair(cur_handle, nxt_handle) : make_pair(nxt_handle, cur_handle);

                edge_map[me].push_back(k);

                set<VertHandle>& adj_verts = vert_adjacent[cur_handle];
                adj_verts.insert(nxt_handle);
                adj_verts.insert(pre_handle);
                //                if(find(adj_verts.begin(), adj_verts.end(), nxt_handle) == adj_verts.end()) adj_verts.push_back(nxt_handle);
                //                if(find(adj_verts.begin(), adj_verts.end(), pre_handle) == adj_verts.end()) adj_verts.push_back(pre_handle);
                
            }
        }

        for(map<Mesh_Edge, vector<FaceHandle> >::const_iterator it = edge_map.begin(); it!=edge_map.end(); ++it){
            if(it->second.size() == 1){
                //TODO: set EDGEFLAG boundary                
            }else if(it->second.size() == 2){
            }else{
                //TODO: set EDGEFLAG non-manifold
                manifold = false;
            }
        }
        
        for(map<VertHandle, set<VertHandle> >::const_iterator it = vert_adjacent.begin(); it!=vert_adjacent.end(); ++it){
            const set<VertHandle>& adj_verts = it->second;
            if(adj_verts.size() == 0){
                //TODO: set VERTFLAG isolated                
            }else{
                int bdy_num = 0;
                for(set<VertHandle>::const_iterator is = adj_verts.begin(); is!= adj_verts.end(); ++is){
                    VertHandle vh1 = it->first, vh2 = *is;
                    Mesh_Edge me = (vh1 < vh2) ? make_pair(vh1, vh2) : make_pair(vh2, vh1);
                    if(edge_map[me].size() ==1) bdy_num ++;
                }
                if(bdy_num > 2) { manifold = false; }
            }
        }
        
        // set mesh flag
        mesh_info.tri_mesh = tri_mesh;
        mesh_info.quad_mesh = quad_mesh;
        mesh_info.poly_mesh = poly_mesh;
        mesh_info.manifold = manifold;

        mesh_info.vert_num = vert_vec.size();
        mesh_info.face_num = face_vec.size();
        mesh_info.edge_num = edge_map.size();
    }

    bool MeshKernel::CreateHalfEdgeDS()
    {
        if(mesh_info.IsManifold() == false) return false;

        typedef pair<VertHandle, VertHandle> MeshEdge;
        map <MeshEdge, HalfEdgeHandle> edge_map; 
        
        for(size_t fid=0; fid<face_vec.size(); ++fid){
            const Face& face = face_vec[fid];
            const vector<VertHandle>& vh_vec = face.vert_handle_vec;
            HalfEdgeHandle origin_he_handle = he_vec.size();
            size_t vh_num = vh_vec.size();
            for(size_t k=0; k<vh_vec.size(); ++k){
                const VertHandle& vh1 = vh_vec[k];
                const VertHandle& vh2 = vh_vec[(k+1)%vh_vec.size()];
                MeshEdge me = make_pair(vh1, vh2);
                MeshEdge oppo_me = make_pair(vh2, vh1);

                /// create a new halfedge
                HalfEdgeHandle curr_he_handle = origin_he_handle + k;
                HalfEdgeHandle next_he_handle = origin_he_handle + (k+1)%vh_num;
                HalfEdgeHandle prev_he_handle = origin_he_handle + (k+vh_num-1)%vh_num;
                HalfEdgeHandle oppo_he_handle = -1;                
                
                if(edge_map.find(oppo_me) != edge_map.end()){
                    oppo_he_handle = edge_map[oppo_me];
                    HalfEdge& oppo_he = he_vec[oppo_he_handle];
                    oppo_he.oppo_he_handle = curr_he_handle;
                }

                HalfEdge he;
                he.vert_handle = vh1;
                he.face_handle = fid;
                he.prev_he_handle = prev_he_handle;
                he.next_he_handle = next_he_handle;
                he.oppo_he_handle = oppo_he_handle;

                he_vec.push_back(he);

                edge_map[me] = curr_he_handle;
            }
        }
        /// form boundary halfedge
        vector<HalfEdgeHandle> bdy_he_vec;
        for(size_t k=0; k<he_vec.size(); ++k){
            if(he_vec[k].oppo_he_handle == -1) bdy_he_vec.push_back(k);
        }
        for(size_t k=0; k<bdy_he_vec.size(); ++k){
            HalfEdge& he = he_vec[bdy_he_vec[k]];
            assert(he.oppo_he_handle == -1);
            const VertHandle& vh1 = he.vert_handle;
            const VertHandle& vh2 = he_vec[he.next_he_handle].vert_handle;

            HalfEdge bdy_he;
            bdy_he.vert_handle = vh2;
            bdy_he.face_handle = -1; /// no face

            he_vec.push_back(bdy_he);
            MeshEdge bdy_edge = make_pair(vh2, vh1);
            edge_map[bdy_edge] = he_vec.size()-1;            
        }
        for(size_t k=0; k<bdy_he_vec.size(); ++k){
            HalfEdge& inner_he = he_vec[bdy_he_vec[k]];
            HalfEdgeHandle curr_he_handle = bdy_he_vec[k];
            HalfEdgeHandle prev_he_handle = he_vec[bdy_he_vec[k]].prev_he_handle;
            HalfEdgeHandle next_he_handle = he_vec[bdy_he_vec[k]].next_he_handle;

            while(he_vec[prev_he_handle].oppo_he_handle != -1){
                curr_he_handle = he_vec[prev_he_handle].oppo_he_handle;
                prev_he_handle = he_vec[curr_he_handle].prev_he_handle;
            }

            while(he_vec[next_he_handle].oppo_he_handle != -1){
                curr_he_handle = he_vec[next_he_handle].oppo_he_handle;
                next_he_handle = he_vec[curr_he_handle].next_he_handle;
            }

            const VertHandle& vh1 = inner_he.vert_handle;
            const VertHandle& vh2 = he_vec[inner_he.next_he_handle].vert_handle;
            const VertHandle& vh3 = he_vec[curr_he_handle].vert_handle;

            MeshEdge outer_edge = make_pair(vh2, vh1);
            HalfEdgeHandle outer_he_handle = edge_map[outer_edge];
            HalfEdge& outer_he = he_vec[outer_he_handle];
            inner_he.oppo_he_handle = outer_he_handle;
            outer_he.oppo_he_handle = bdy_he_vec[k];
            outer_he.prev_he_handle = prev_he_handle;
            outer_he.next_he_handle = next_he_handle;
            
        }
        return true;
    }
}
