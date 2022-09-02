/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2016                                           \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#pragma once
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import_ply.h>
#include <wrap/io_trimesh/export_ply.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/geodesic.h>
#include <vcg/complex/algorithms/update/color.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace vcg;
using namespace std;

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<	Use<MyVertex>   ::AsVertexType,
    Use<MyEdge>     ::AsEdgeType,
    Use<MyFace>     ::AsFaceType> {};

class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::Mark, vertex::VFAdj, vertex::Color4b, vertex::Qualityf, vertex::BitFlags  > {};
class MyFace : public Face< MyUsedTypes, face::VFAdj, face::VertexRef, face::Normal3f, face::BitFlags > {};
class MyEdge : public Edge<MyUsedTypes> {};
class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>  > {};


class TriGeodesic_Mymesh {

public:
    
    vector<vector<int>> face_Temp;
    vector<Point3f> point_Temp; 
    vector<int> point_Border_Index;
    int point_center;
    int eyemiddle = -1;

private:

    Point3f eyemiddle_p;

    vector<float> vertexGeoDis;
    MyMesh m;
    float thed_Temp;
    float max_g;
    float min_g;
    vector<Point3f> point_new;//insert new point to smooth border
    vector<vector<int>> point_new_face;
    vector<float> point_Temp_dis;//geodesic distance
    vector<int> point_Temp_index;
    vector<vector<int>> rgb_Temp;//extract point color
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

public:

    void TriGeodesic_Mymesh_init(char* fileName) {
               
        //  if(tri::io::ImporterPLY<MyMesh>::Open(m,"../../meshes/disk_irregular_1k.ply")!=0)
        if (tri::io::ImporterPLY<MyMesh>::Open(m, fileName) != 0)
        {
            printf("Error reading file %s!\n", fileName);
            exit(0);
        }

        //start point
        Point3f c = m.bbox.Center();
        TriGeodesic_Mymesh_Geodesic(c);       

    }

    void TriGeodesic_Mymesh_init(char* fileName, Point3f c) {

        //MyMesh m;

        //  if(tri::io::ImporterPLY<MyMesh>::Open(m,"../../meshes/disk_irregular_1k.ply")!=0)
        if (tri::io::ImporterPLY<MyMesh>::Open(m, fileName) != 0)
        {
            printf("Error reading file %s!\n", fileName);
            exit(0);
        }

        TriGeodesic_Mymesh_Geodesic(c);       
        

    }

    void TriGeodesic_Mymesh_init(char* fileName, int pointIndex) {

        // MyMesh m;
        // if(tri::io::ImporterPLY<MyMesh>::Open(m,"../../meshes/disk_irregular_1k.ply")!=0)

        if (tri::io::ImporterPLY<MyMesh>::Open(m, fileName) != 0)
        {
            printf("Error reading file %s!\n", fileName);
            exit(0);
        }

        auto vi = m.vert.begin() + pointIndex;
        Point3f c;
        c[0] = (*vi).P()[0];
        c[1] = (*vi).P()[1];
        c[2] = (*vi).P()[2];

        TriGeodesic_Mymesh_Geodesic(c);

    }

    void TriGeodesic_Mymesh_init(char* fileName, int pointIndex, int eyemiddle_input) {

        // MyMesh m;
        // if(tri::io::ImporterPLY<MyMesh>::Open(m,"../../meshes/disk_irregular_1k.ply")!=0)

        if (tri::io::ImporterPLY<MyMesh>::Open(m, fileName) != 0)
        {
            printf("Error reading file %s!\n", fileName);
            exit(0);
        }
        eyemiddle = eyemiddle_input;
        auto vi = m.vert.begin() + pointIndex;
       

        Point3f c;
        c[0] = (*vi).P()[0];
        c[1] = (*vi).P()[1];
        c[2] = (*vi).P()[2];

        if (eyemiddle_input >= 0) {
            auto vi_2 = m.vert.begin() + eyemiddle_input;
            eyemiddle_p[0] = (*vi_2).P()[0];
            eyemiddle_p[1] = (*vi_2).P()[1];
            eyemiddle_p[2] = (*vi_2).P()[2];        
        }

        TriGeodesic_Mymesh_Geodesic(c);

    }

    void TriGeodesic_Mymesh_Extract(float thed) {

        if (face_Temp.size() > 0|| point_Temp.size()>0) {
            face_Temp.clear(); 
            point_Temp.clear();
        }  

        if (vertexGeoDis.size() <= 0) {
            cout << "error, the geodesic field is empty!" << endl;
            return;           
        }

        if (thed > max_g) {
            cout << "error, the geodesic radius is larger than max distance!" << endl;
            return;        
        }

        thed_Temp = thed;
        
        vector<int> pointIndex(m.vert.size(), -1);
        vector<Point3f> point_Geo;
        vector<int> point_Geo_int;
        vector<vector<int>> face_Geo;
        
        int index = 0;
        int p_index = 0;
        for (auto vi = m.vert.begin(); vi != m.vert.end(); ++vi) if (!(*vi).IsD())
        {  
            if (vertexGeoDis[index] <= thed) {
                Point3f p_i;
                p_i[0] = (*vi).P()[0];
                p_i[1] = (*vi).P()[1];
                p_i[2] = (*vi).P()[2];
                point_Geo.push_back(p_i);
                point_Geo_int.push_back(index);
                pointIndex[index] = p_index;
                p_index++;            
            }             
            index++;
        }

        cout << "Exact point finished!" << endl;

        for (auto fi = m.face.begin(); fi != m.face.end(); ++fi) if (!(*fi).IsD()) {

            int b1 = tri::Index(m, (*fi).V(0));
            int b2 = tri::Index(m, (*fi).V(1));
            int b3 = tri::Index(m, (*fi).V(2));

            //cout << b1 << " " << b2 << " " << b3 << endl;
            //if (b1 == 9178 && b2 == 9176) {
                //cout << b1 << " " << b2 << " " << b3 << endl;            
            //}

            if (vertexGeoDis[b1] > thed || vertexGeoDis[b2] > thed || vertexGeoDis[b3] > thed) {  
                if (vertexGeoDis[b1] > thed && vertexGeoDis[b2] > thed && vertexGeoDis[b3] > thed) {
                    continue;                
                }
                else {
                    vector<int> b_large;
                    vector<int> b_small;
                    if (vertexGeoDis[b1] > thed) {
                        b_large.push_back(b1);
                    }
                    else {
                        b_small.push_back(b1);
                    }
                    if (vertexGeoDis[b2] > thed) {
                        b_large.push_back(b2);
                    }
                    else {
                        b_small.push_back(b2);
                    }
                    if (vertexGeoDis[b3] > thed) {
                        b_large.push_back(b3);
                    }
                    else {
                        b_small.push_back(b3);
                    }

                    if (b_large.size() == 2) {
                        //move point with small value
                        int targetPoint = b_small[0];
                        Point3f center;

                        Point3f p_large1;
                        auto vi1 = m.vert.begin() + b_large[0];
                        p_large1[0] = (*vi1).P()[0];
                        p_large1[1] = (*vi1).P()[1];
                        p_large1[2] = (*vi1).P()[2];

                        Point3f p_large2;
                        auto vi2 = m.vert.begin() + b_large[1];
                        p_large2[0] = (*vi2).P()[0];
                        p_large2[1] = (*vi2).P()[1];
                        p_large2[2] = (*vi2).P()[2];

                        center[0] = (p_large1[0] + p_large2[0]) / 2;
                        center[1] = (p_large1[1] + p_large2[1]) / 2;
                        center[2] = (p_large1[2] + p_large2[2]) / 2;

                        float weight_dis = (vertexGeoDis[b_large[0]] + vertexGeoDis[b_large[1]]) / 2;
                        float weight = (thed - vertexGeoDis[b_small[0]]) / (weight_dis - vertexGeoDis[b_small[0]]);

                        int index_update = pointIndex[b_small[0]];

                        point_Geo[index_update][0] = point_Geo[index_update][0] * (1 - weight) + center[0] * weight;
                        point_Geo[index_update][1] = point_Geo[index_update][1] * (1 - weight) + center[1] * weight;
                        point_Geo[index_update][2] = point_Geo[index_update][2] * (1 - weight) + center[2] * weight;

                        vertexGeoDis[b_small[0]] = thed;

                    }
                    else {
                        //move point with large value
                        int targetPoint = b_large[0];
                        Point3f center;
                        Point3f center_new;
                        center[0] = (point_Geo[pointIndex[b_small[0]]][0] + point_Geo[pointIndex[b_small[1]]][0]) / 2;
                        center[1] = (point_Geo[pointIndex[b_small[0]]][1] + point_Geo[pointIndex[b_small[1]]][1]) / 2;
                        center[2] = (point_Geo[pointIndex[b_small[0]]][2] + point_Geo[pointIndex[b_small[1]]][2]) / 2;
                        float weight_dis = (vertexGeoDis[b_small[0]] + vertexGeoDis[b_small[1]]) / 2;
                        float weight = (thed - vertexGeoDis[b_large[0]]) / (weight_dis - vertexGeoDis[b_large[0]]);

                        Point3f p_large;
                        auto vi = m.vert.begin() + b_large[0];                       
                        p_large[0] = (*vi).P()[0];
                        p_large[1] = (*vi).P()[1];
                        p_large[2] = (*vi).P()[2];

                        center_new[0] = p_large[0] * (1 - weight) + center[0] * weight;
                        center_new[1] = p_large[1] * (1 - weight) + center[1] * weight;
                        center_new[2] = p_large[2] * (1 - weight) + center[2] * weight;

                        point_new.push_back(center_new);
                        vector<int> face_i(3);
                        face_i[0] = pointIndex[b1];//index of vertex per face   
                        face_i[1] = pointIndex[b2];//index of vertex per face  
                        face_i[2] = pointIndex[b3];//index of vertex per face  
                        point_new_face.push_back(face_i);

                    }                
                }                                                        
            }
            else {
                vector<int> face_i(3);
                face_i[0] = pointIndex[b1];//index of vertex per face   
                face_i[1] = pointIndex[b2];//index of vertex per face  
                face_i[2] = pointIndex[b3];//index of vertex per face  
                face_Geo.push_back(face_i);           
            } 
        }

        face_Temp = face_Geo;
        point_Temp = point_Geo;
        point_Temp_index = point_Geo_int;//global index

        for (int i = 0; i < point_Temp_index.size(); i++) {
            int global_index = point_Temp_index[i];
            if (vertexGeoDis[global_index] == thed) {
                point_Border_Index.push_back(i);            
            } 
            if (vertexGeoDis[global_index] == 0) {
                point_center = i;            
            }
        }
        for (int i = 0; i < point_new.size(); i++) {
            point_Border_Index.push_back(i + point_Temp.size());        
        }

        TriGeodesic_Mymesh_FixMesh();
        TriGeodesic_Mymesh_FixMesh_RemoveErrorPoint();        

        if (eyemiddle >= 0) {

            float dis_min = TriGeodesic_Mymesh_Dis(point_Border_Index[0]);
            int index_min = 0;

            //extract nearest border point to be the first one 
            for (int i = 1; i < point_Border_Index.size(); i++) {
                float dis_i = TriGeodesic_Mymesh_Dis(point_Border_Index[i]);
                if (dis_i < dis_min) {
                    dis_min = dis_i;
                    index_min = i;                
                }            
            } 

            int t_temp = point_Border_Index[index_min];
            point_Border_Index[index_min] = point_Border_Index[0];
            point_Border_Index[0] = t_temp;

        }       

        cout << "Exact face finished!" << endl;    
    
    }

    void TriGeodesic_Mymesh_Save_PLY(string fileStore) {        

        ofstream f1(fileStore);
        f1 << "ply" << endl;
        f1 << "format ascii 1.0" << endl;
        f1 << "comment VCGLIB generated" << endl;
        f1 << "element vertex " << point_Temp.size()<< endl;
        f1 << "property float x" << endl;
        f1 << "property float y" << endl;
        f1 << "property float z" << endl;
        f1 << "property uchar red " << endl;
        f1 << "property uchar green" << endl;
        f1 << "property uchar blue" << endl;
        f1 << "element face "<< face_Temp.size()<< endl;
        f1 << "property list uchar int vertex_indices" << endl;
        f1 << "end_header" << endl;       

        for (int i = 0; i < point_Temp.size(); i++) {
            
            f1 << point_Temp[i][0] << " " << point_Temp[i][1] << " " << point_Temp[i][2] << " " <<
                rgb_Temp[i][0] << " " << rgb_Temp[i][1] << " " << rgb_Temp[i][2] << endl;

        }        

        for (int i = 0; i < face_Temp.size(); i++) {
            f1 << face_Temp[i].size() << " " << face_Temp[i][0] << " " << face_Temp[i][1] << " " <<
                face_Temp[i][2] << endl;        
        }       
        
        f1.close();

    }

private:

    void TriGeodesic_Mymesh_FixMesh_RemoveErrorPoint() {

        //update edge

        vector<int> point_Temp_Mapping(point_Temp.size());
        for (int i = 0; i < point_Temp_Mapping.size(); i++) {
            point_Temp_Mapping[i] = i;        
        }

        vector<bool> point_Border_Index_global(point_Temp.size(), false);
        for (int i = 0; i < point_Border_Index.size(); i++) {
            int index_i = point_Border_Index[i];
            point_Border_Index_global[index_i] = true;        
        }

        vector<vector<int>> pointNeighbor(point_Temp.size());
        for (int i = 0; i < face_Temp.size(); i++) {
            int b1 = face_Temp[i][0];
            int b2 = face_Temp[i][1];
            int b3 = face_Temp[i][2];
            pointNeighbor[b1].push_back(b2);
            pointNeighbor[b1].push_back(b3);
            pointNeighbor[b2].push_back(b3);
            pointNeighbor[b2].push_back(b1);
            pointNeighbor[b3].push_back(b1);
            pointNeighbor[b3].push_back(b2);
        }

        for (int i = 0; i < point_Border_Index_global.size(); i++) {

            if (pointNeighbor[i].size() == 0) {

                point_Temp_Mapping[i] = -1;
            
            }

            if (point_Border_Index_global[i] && pointNeighbor[i].size() == 2) {

                int b1 = pointNeighbor[i][0];
                int b2 = pointNeighbor[i][1];

                if (point_Border_Index_global[b1] && point_Border_Index_global[b2]) {

                    point_Temp_Mapping[i] = -1;  

                }

            }
        
        }   

        //update
        vector<Point3f> point_Temp_Update;
        vector<vector<int>> rgb_Temp_Update;//extract point color
        vector<vector<int>> face_Temp_Update;
        vector<int> point_Border_Index_Update;
        int point_center_Update;

        int diff = 0;
        for (int i = 0; i < point_Temp_Mapping.size(); i++) {

            if (point_Temp_Mapping[i] == -1) {
                diff++;            
            }
            else {
                point_Temp_Mapping[i] = point_Temp_Mapping[i] - diff;    
                point_Temp_Update.push_back(point_Temp[i]);
                rgb_Temp_Update.push_back(rgb_Temp[i]);
            }        
        
        }

        for (int i = 0; i < face_Temp.size(); i++) {

            int b1 = face_Temp[i][0];
            int b2 = face_Temp[i][1];
            int b3 = face_Temp[i][2];
            if (point_Temp_Mapping[b1] == -1 || point_Temp_Mapping[b2] == -1 || point_Temp_Mapping[b3] == -1) {
                continue;            
            }
            else {
                vector<int> face_Temp_i(3);
                face_Temp_i[0] = point_Temp_Mapping[b1];
                face_Temp_i[1] = point_Temp_Mapping[b2];
                face_Temp_i[2] = point_Temp_Mapping[b3];
                face_Temp_Update.push_back(face_Temp_i);
            }
        
        }

        for (int i = 0; i < point_Border_Index.size(); i++) {

            int index_i = point_Temp_Mapping[point_Border_Index[i]];
            if (index_i == -1) {
                continue;            
            }
            else {
                point_Border_Index_Update.push_back(index_i);            
            }
        
        }

        point_center_Update = point_Temp_Mapping[point_center];

        point_Temp =point_Temp_Update;
        rgb_Temp = rgb_Temp_Update;//extract point color
        face_Temp = face_Temp_Update;
        point_Border_Index = point_Border_Index_Update;
        point_center = point_center_Update;

    
    }

    void TriGeodesic_Mymesh_FixMesh() {

        //return color
        vector<vector<int>> rgb(point_Temp.size()+ point_new.size());

        for (int i = 0; i < point_Temp.size(); i++) {
            int r_i = 0;
            int g_i = 0;
            int b_i = 0;
            float dis_i = vertexGeoDis[point_Temp_index[i]];
            point_Temp_dis.push_back(dis_i);
            vector<int> rgb_vector = TriGeodesic_Mymesh_Color(thed_Temp, 0, dis_i);
            rgb[i] = rgb_vector;
        }   

        vector<int> rgb_i(3);
        rgb_i[0] = 0;
        rgb_i[1] = 0;
        rgb_i[2] = 255;

        for (int i = 0; i < point_new.size(); i++) {
            point_Temp_dis.push_back(thed_Temp);
            rgb[i + point_Temp.size()] = rgb_i;
            
        }

        int point_Temp_Number = point_Temp.size();
        point_Temp.insert(point_Temp.end(), point_new.begin(), point_new.end());

        for (int i = 0; i < point_new_face.size(); i++) {

            int b1 = point_new_face[i][0];
            int b2 = point_new_face[i][1];
            int b3 = point_new_face[i][2];

            if (b1 == -1) {
                b1 = i + point_Temp_Number;
            }
            if (b2 == -1) {
                b2 = i + point_Temp_Number;
            }
            if (b3 == -1) {
                b3 = i + point_Temp_Number;
            }

            vector<int> face_i;
            face_i.push_back(b1);
            face_i.push_back(b2);
            face_i.push_back(b3);
            face_Temp.push_back(face_i);
            
        }  

        //update edge
        vector<vector<int>> pointNeighbor(point_Temp.size());
        for (int i = 0; i < face_Temp.size(); i++) {
            int b1 = face_Temp[i][0];
            int b2 = face_Temp[i][1];
            int b3 = face_Temp[i][2];
            pointNeighbor[b1].push_back(b2);
            pointNeighbor[b1].push_back(b3);
            pointNeighbor[b2].push_back(b3);
            pointNeighbor[b2].push_back(b1);
            pointNeighbor[b3].push_back(b1);
            pointNeighbor[b3].push_back(b2);       
        }

        //add point (wait for fix) into list
        vector<int> pointFix;
        vector<int> pointFix_2;
        vector<bool> pointFix_judge;
        for (int i = 0; i < pointNeighbor.size(); i++) {
            if (point_Temp_dis[i] == thed_Temp) {
                //check two edge points   
                vector<int> pointNeighbor_i = pointNeighbor[i];
                vector<int> t;
                for (int j = 0; j < pointNeighbor_i.size(); j++) {
                    int pointNeighbor_ij = pointNeighbor_i[j];
                    if (point_Temp_dis[pointNeighbor_ij] == thed_Temp) {
                        t.push_back(pointNeighbor_ij);                    
                    }                
                }
                if (t.size() == 1) {
                    pointFix.push_back(i);
                }
                else if(t.size() == 0) {
                    pointFix_2.push_back(i);
                }
            }        
        }

        //construct kd-tree
        NeighborSearch_Kd_Tree(point_Temp);
        pointFix_judge.resize(pointFix.size(), false);
        int k_n = 8;

        for (int i = 0; i < pointFix.size(); i++) {            
            //cout << i << endl;
            if (pointFix_judge[i]) {
                continue;            
            }
            else {
                int index_i = pointFix[i];
                int index_in = -1;//connect point index in point_Temp (global)
                int index_index = -1;//connect point index in pointFix (local)
                std::vector<int> pointIdxNKNSearch(k_n);
                std::vector<float> pointNKNSquaredDistance(k_n);               
                pcl::PointXYZ searchPoint;
                searchPoint.x = point_Temp[index_i][0];
                searchPoint.y = point_Temp[index_i][1];
                searchPoint.z = point_Temp[index_i][2];
                kdtree.nearestKSearch(searchPoint, k_n, pointIdxNKNSearch, pointNKNSquaredDistance);                
                for (int j = 1; j < pointIdxNKNSearch.size(); j++) {
                    int index_j = pointIdxNKNSearch[j];
                    int judge_j = TriGeodesic_Mymesh_VectorSearching(index_j, pointFix);
                    if (judge_j>0) {
                        index_in = index_j; 
                        index_index = judge_j;
                        break;
                    }                
                }
                if (index_in == -1) {
                    continue;                
                }
                else {
                    vector<int> newFace = TriGeodesic_Mymesh_CommonNeighbor(index_i, pointNeighbor[index_i],
                        index_in, pointNeighbor[index_in]);

                    if (newFace.size() == 3) {
                        face_Temp.push_back(newFace);
                    }
                    pointFix_judge[index_index] = true;                
                }                
                pointFix_judge[i] = true;            
            }          
        }

        for (int i = 0; i < pointFix_2.size(); i++) {
            
            int index_i = pointFix_2[i];
            int index_in1 = -1;
            int index_in2 = -1;
            std::vector<int> pointIdxNKNSearch(k_n);
            std::vector<float> pointNKNSquaredDistance(k_n);
            pcl::PointXYZ searchPoint;
            searchPoint.x = point_Temp[index_i][0];
            searchPoint.y = point_Temp[index_i][1];
            searchPoint.z = point_Temp[index_i][2];
            kdtree.nearestKSearch(searchPoint, k_n, pointIdxNKNSearch, pointNKNSquaredDistance);
            vector<int> index_n_list;
            for (int j = 1; j < pointIdxNKNSearch.size(); j++) {
                int index_j = pointIdxNKNSearch[j];
                int judge_j = TriGeodesic_Mymesh_VectorSearching(index_j, pointFix);
                if (judge_j >= 0) {
                    index_n_list.push_back(index_j);
                    if (index_n_list.size() == 2) {
                        break;                    
                    }
                }
            }
            if (index_n_list.size() <= 1) {
                continue;            
            }
            else {
                index_in1 = index_n_list[0];
                index_in2 = index_n_list[1];
                vector<int> newFace1 = TriGeodesic_Mymesh_CommonNeighbor(index_i, pointNeighbor[index_i],
                    index_in1, pointNeighbor[index_in1]);
                vector<int> newFace2 = TriGeodesic_Mymesh_CommonNeighbor(index_i, pointNeighbor[index_i],
                    index_in2, pointNeighbor[index_in2]);
                if (newFace1.size() == 3) {
                    face_Temp.push_back(newFace1);
                }
                if (newFace2.size() == 3) {
                    face_Temp.push_back(newFace2);
                }            
            }  
        }

        rgb_Temp = rgb;
        //return rgb;
    
    }

    vector<int> TriGeodesic_Mymesh_CommonNeighbor(int b1, vector<int> b1n, int b2, vector<int> b2n) {

        vector<int> tri_b3;
        vector<int> tri_b3_R;
        int b3 = -1;
        for (int i = 0; i < b1n.size(); i=i+2) {
            int b1_i1 = b1n[i];
            int b1_i2 = b1n[i + 1];
            for (int j = 0; j < b2n.size(); j = j + 2) {
                int b2_i1 = b2n[j];
                int b2_i2 = b2n[j + 1];

                if (b1_i1 == b2_i1|| b1_i1 == b2_i2) {
                    b3 = b1_i1; 
                    break;
                }
                else if(b1_i2 == b2_i1|| b1_i2 == b2_i2) {
                    b3 = b1_i2;
                    break;
                }                
                else {
                    continue;                
                } 
            }
            if (b3 >= 0) {
                tri_b3_R.push_back(b1);
                tri_b3_R.push_back(b1_i1);
                tri_b3_R.push_back(b1_i2);
                break;            
            }        
        }            
        if (b3 < 0 || tri_b3_R.size() != 3) {
            tri_b3.clear();
            return tri_b3;        
        }
        else {
            if (tri_b3_R[1] == b3) {
                tri_b3.push_back(b1);
                tri_b3.push_back(b2);
                tri_b3.push_back(b3);            
            }
            else {
                tri_b3.push_back(b1);
                tri_b3.push_back(b3);
                tri_b3.push_back(b2);           
            }
            return tri_b3;        
        }
    }

    int TriGeodesic_Mymesh_VectorSearching(int b, vector<int> p) {

        for (int i = 0; i < p.size(); i++) {

            if (b == p[i]) {

                return i;
            
            }
        
        }   

        return -1;
    
    }

    void NeighborSearch_Kd_Tree(vector<Point3f> pointSet) {

        std::cout << "Init kdtree" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = pointSet.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
        // fills a PointCloud with random data
        for (int i = 0; i < pointSet.size(); i++)
        {
            pcl::PointXYZ pxyz;
            cloud->points[i].x = pointSet[i][0];
            cloud->points[i].y = pointSet[i][1];
            cloud->points[i].z = pointSet[i][2];

        }
        kdtree.setInputCloud(cloud);

    }

    void TriGeodesic_Mymesh_Geodesic(Point3f c) {

        //Point3f c = m.bbox.Center();
        MyVertex* closest = &*m.vert.begin();
        float minDist = Distance(closest->P(), c);
        for (MyMesh::VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        {
            if (Distance(vi->P(), c) < minDist)
            {
                minDist = Distance(vi->P(), c);
                closest = &*vi;
            }
        }
        vector<MyVertex*> seedVec;
        seedVec.push_back(closest);
        tri::EuclideanDistance<MyMesh> ed;
        tri::Clean<MyMesh>::RemoveUnreferencedVertex(m);
        tri::Allocator<MyMesh>::CompactEveryVector(m);
        tri::UpdateTopology<MyMesh>::VertexFace(m);
        tri::Geodesic<MyMesh>::Compute(m, seedVec, ed);
        pair<float, float> minmax = tri::Stat<MyMesh>::ComputePerVertexQualityMinMax(m);
        tri::UpdateColor<MyMesh>::PerVertexQualityRamp(m);
        printf("min %f max %f\n", minmax.first, minmax.second);
        tri::io::ExporterPLY<MyMesh>::Save(m, "Data/base.ply", tri::io::Mask::IOM_VERTCOLOR | tri::io::Mask::IOM_VERTQUALITY);
        vertexGeoDis.resize(m.vert.size(), 0);
        float minDis = 9999;
        float maxDis = -9999;
        for (size_t i = 0; i < m.vert.size(); ++i)
        {
            MyVertex* nextV = &*(m.vert.begin() + i);
            vertexGeoDis[i] = nextV->Q();
            if (vertexGeoDis[i] > maxDis) {
                maxDis = vertexGeoDis[i];
            }
            if (vertexGeoDis[i] < minDis) {
                minDis = vertexGeoDis[i];
            } 
        }

        max_g = maxDis;
        min_g = minDis;

        cout << "geodesic computation finished" << endl;     
    }

    vector<int> TriGeodesic_Mymesh_Color(float maxf, float minf, float v) {

        vector<int> rgb3(3, 0);
        if (minf > maxf) {
            float t = minf;
            minf = maxf;
            maxf = t;        
        }
        float step = (maxf - minf) / 4;
        if (v < minf) { 
            rgb3[0] = 255;
            rgb3[1] = 0;
            rgb3[2] = 0;
            return rgb3;            
        }
        v -= minf;
        if (v < step) {
            float weight_i = v / step;
            rgb3[0] = 255;
            rgb3[1] = 255 * weight_i;
            rgb3[2] = 0;
            return rgb3;        
        }
        v -= step;
        if (v < step) {
            float weight_i = v / step;
            rgb3[0] = 255 * (1 - weight_i);
            rgb3[1] = 255;
            rgb3[2] = 0;
            return rgb3;
        }        
        v -= step;
        if (v < step) {
            float weight_i = v / step;
            rgb3[0] = 0;
            rgb3[1] = 255;
            rgb3[2] = 255 * weight_i;
            return rgb3;
        }        
        v -= step;
        if (v < step) { 
            float weight_i = v / step;
            rgb3[0] = 0;
            rgb3[1] = 255 * (1- weight_i);
            rgb3[2] = 255;
            return rgb3;        
        }
        rgb3[0] = 0;
        rgb3[1] = 0;
        rgb3[2] = 255;

        return rgb3;
    }
       
    float TriGeodesic_Mymesh_Dis(int a) {

        float dis_ab = sqrt((point_Temp[a][0]- eyemiddle_p[0])* (point_Temp[a][0] - eyemiddle_p[0])+
            (point_Temp[a][1] - eyemiddle_p[1]) * (point_Temp[a][1] - eyemiddle_p[1])+
            (point_Temp[a][2] - eyemiddle_p[2]) * (point_Temp[a][2] - eyemiddle_p[2]));
        return  dis_ab;
    
    }
};