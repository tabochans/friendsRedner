#include"Scene.h"
#include"tiny_obj_loader.h"
#include"stb_image.hpp"
#include"stb_image_write.h"
#include"ScopedTimer.h"
#include"OBVH_Normal.h"
#include"OBVH_SIMDFacet.h"
#include<iostream>
#include<fstream>
using namespace PTUtility;
using namespace Primitive;

bool Scene::CreateScene(const char* OBJ_FileName, const PTUtility::SceneMaterialManager* sceneMaterial){
	//Load wavefrontobj file
	LoadOBJ(OBJ_FileName, sceneMaterial);

	return true;
}

bool Scene::LoadOBJ(const char* FileName, const PTUtility::SceneMaterialManager* sceneMaterial){

	const PTUtility::SceneMaterialManager* smat = sceneMaterial;
	SceneMaterialManager defmat;
	if (smat == nullptr){
		smat = &defmat;
	}


	std::cout << "Create Scene START" << std::endl;
	{
		ScopedTimer _prof("Create Scene");
		time_t stt = time(NULL);

		std::vector<Primitive::inoutFacet> Facets;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;
		std::string err;
		bool ret = tinyobj::LoadObj(shapes, materials, err, FileName, "OBJ//");

		if (!err.empty()) {
			std::cerr << err << std::endl;
		}
		if (!ret) {
			std::cerr << "Failed to load/parse .obj.\n" << std::endl;
			return false;
		}

		//Load all textures
		for (int m = 0; m < materials.size(); m++){
			if (materials[m].diffuse_texname.size() > 2){
				m_Attributes.LoadTexture(materials[m].diffuse_texname.c_str());
			}
			if (materials[m].specular_texname.size() > 2){
				m_Attributes.LoadTexture(materials[m].specular_texname.c_str());
			}
		}

		//Create Attributes
		for (int m = 0; m < materials.size(); m++){
			Attribute* At = new Attribute;
			At->m_Index = 1.0f;
			At->m_Albedo = Vec3(materials[m].diffuse[0], materials[m].diffuse[1], materials[m].diffuse[2]);
			At->m_Emission = Vec3(materials[m].emission[0], materials[m].emission[1], materials[m].emission[2]);
			smat->SetParamtoAttribute(At, materials[m].name);
			std::vector<std::string> tfn; 
			if (m_Attributes.FindTexture(materials[m].diffuse_texname)){ tfn.push_back(materials[m].diffuse_texname); }
			if (m_Attributes.FindTexture(materials[m].specular_texname)){ tfn.push_back(materials[m].specular_texname); }
			At->TextureFileNames = tfn;
			m_Attributes.Add_Attribute(At, m);
		}

		std::vector<Primitive::inoutFacet> m_Data;
		int NumFacets = 0;
		for (int sps = 0; sps < shapes.size(); sps++){
			NumFacets += shapes[sps].mesh.indices.size() / 3;
		}
		m_Data.resize(NumFacets);

		int Fcount = 0;
		for (int sps = 0; sps < shapes.size(); sps++){
			for (int idx = 0, face = 0; idx + 2 < shapes[sps].mesh.indices.size(); idx += 3){

				const int mtid = shapes[sps].mesh.material_ids[face++];
				if (materials.size() > 0){
					std::array<PTUtility::Vec3, 3> pos;
					pos[0] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
					pos[1] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
					pos[2] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);

					std::array<PTUtility::Vec3, 3> normal;
					if (shapes[sps].mesh.normals.size() > 0){
						normal[0] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
						normal[1] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
						normal[2] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);
					}
					else{
						Vec3 fNormal = (pos[1] - pos[2]).normalized().cross((pos[2] - pos[0]).normalized());
						normal[0] = fNormal;
						normal[1] = fNormal;
						normal[2] = fNormal;
					}

					std::array<PTUtility::Vec3, 3> UV;
					if (shapes[sps].mesh.texcoords.size() > 0){
						UV[0] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 0]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 0]], 0);
						UV[1] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 1]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 1]], 0);
						UV[2] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 2]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 2]], 0);
					}
					else{
						UV[0] = Vec3(0, 0, 0);
						UV[1] = Vec3(0, 0, 0);
						UV[2] = Vec3(0, 0, 0);
					}
					m_Data[Fcount++] = inoutFacet(pos, normal, UV, mtid);

					//Add Light Data to LightManager
					if (mtid >= 0){
						if (materials[mtid].emission[0] + materials[mtid].emission[1] + materials[mtid].emission[2] > 0.00001){
							bool bilight = materials[mtid].dissolve < 0.5;
							m_Lmana.AddAreaLight(LightManager::AreaLight(m_Data[Fcount - 1], PTUtility::Vec3(materials[mtid].emission[0], materials[mtid].emission[1], materials[mtid].emission[2]), bilight));
						}
					}
				}
				else{
					std::array<PTUtility::Vec3, 3> pos;
					pos[0] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
					pos[1] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
					pos[2] = Vec3(
						shapes[sps].mesh.positions[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
						shapes[sps].mesh.positions[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);

					std::array<PTUtility::Vec3, 3> normal;
					if (shapes[sps].mesh.normals.size() > 0){
						normal[0] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 0]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 0]]);
						normal[1] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 1]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 1]]);
						normal[2] = Vec3(
							shapes[sps].mesh.normals[0 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[1 + 3 * shapes[sps].mesh.indices[idx + 2]],
							shapes[sps].mesh.normals[2 + 3 * shapes[sps].mesh.indices[idx + 2]]);
					}
					else{
						Vec3 fNormal = (pos[1] - pos[2]).normalized().cross((pos[2] - pos[0]).normalized());
						normal[0] = fNormal;
						normal[1] = fNormal;
						normal[2] = fNormal;
					}

					std::array<PTUtility::Vec3, 3> UV;
					if (shapes[sps].mesh.texcoords.size() > 0){
						UV[0] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 0]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 0]], 0);
						UV[1] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 1]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 1]], 0);
						UV[2] = Vec3(shapes[sps].mesh.texcoords[0 + 2 * shapes[sps].mesh.indices[idx + 2]], shapes[sps].mesh.texcoords[1 + 2 * shapes[sps].mesh.indices[idx + 2]], 0);
					}
					else{
						
					}
					m_Data[Fcount++] = inoutFacet(pos, normal, UV, mtid);
				}
			}
		}
		//m_OBVH.reset(new OBVH_SIMDFacet);
		m_OBVH.reset(new OBVH_Normal);
		m_OBVH->CreateGraph(m_Data);

		AddExtraLight();
		m_Lmana.FinishAddingLight();
	}

	return true;
}

void Scene::AddExtraLight(){

	LightManager::SphereLight SL1(PTUtility::Vec3(-0.8, 0.5, 0.6), PTUtility::Vec3(24, 120, 24) * (4.0 * 0.1 * 0.1 * 3.14), 0.1);
	LightManager::SphereLight SL2(PTUtility::Vec3(0, 2.5, 0), PTUtility::Vec3(1, 0, 4) * (4.0 * 0.4 * 0.4 * 3.14), 0.4);
	LightManager::SphereLight SL3(PTUtility::Vec3(0.2, 0.88, 1.8), PTUtility::Vec3(24, 12, 10) * (4.0 * 1.2 * 1.2 * 3.14), 1.2);

	//m_Lmana.AddSphereLight(SL1);
	//m_Lmana.AddSphereLight(SL2);
	//m_Lmana.AddSphereLight(SL3);

}

int Scene::NextEventEstimation_SphereLight(const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const{
	return m_Lmana.NextEventEstimation_SphereLight(*this, currentRay, MT, result, pdf, ToLight);
}
int Scene::NextEventEstimation_AreaLight(const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const{
	return m_Lmana.NextEventEstimation_AreaLight(*this, currentRay, MT, result, pdf, ToLight);
}

void Scene::ClearScene(){
	m_OBVH.release();
}

int Scene::GetFacetsFromRay(const Ray& ray, std::vector<AbsG::RF>& Facets)const{
	return m_OBVH->GetElements(ray, Facets);
}

int Scene::GetOneFacetFromRay(const Ray& ray, AbsG::RF& Facet)const{
	return m_OBVH->GetOneElement(ray, Facet);
}

bool Scene::IfOccluded(const Ray& ray, float MinDist, float MaxDist) const{
	return m_OBVH->IfOccluded(ray, MinDist, MaxDist);
}