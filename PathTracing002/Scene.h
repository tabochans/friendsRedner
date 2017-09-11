#pragma once
#include"Texture.h"
#include"LightManager.h"
#include"Vec3.h"
#include"AbsBVH.h"
#include"Intersect.h"
#include"Sampling.h"
#include"material.h"
#include"SceneMaterial.h"
#include<utility>
#include<vector>
#include<map>

class iScene{
private:

public:
	virtual bool CreateScene(const char* OBJ_FileName) = 0;
	virtual void ClearScene() = 0;
	virtual int GetFacetsFromRay(const std::vector<AbsG::RF>& Facets)const = 0;
	virtual int GetOneFacetFromRay(const AbsG::RF& Facet)const = 0;
	virtual PTUtility::Vec3 GetSceneWidth()const = 0;
	virtual PTUtility::Vec3 GetSceneCenter()const = 0;

	iScene(){ ; }
	virtual ~iScene(){ ; }
};

class Scene{
private:
	bool LoadOBJ(const char* FileName, const PTUtility::SceneMaterialManager* sceneMaterial = nullptr);
	PTUtility::AttributeManager m_Attributes;
	std::unique_ptr<AbsG> m_OBVH;
	LightManager m_Lmana;
	void AddExtraLight();

public:

	const LightManager* GetLightManager()const{ return (&m_Lmana) ; }
	int NextEventEstimation_SphereLight(const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const;
	int NextEventEstimation_AreaLight(const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const;

	virtual bool CreateScene(const char* OBJ_FileName, const PTUtility::SceneMaterialManager* sceneMaterial);
	virtual void ClearScene();
	virtual int GetFacetsFromRay(const Primitive::Ray& ray, std::vector<AbsG::RF>& Facets)const;
	virtual int GetOneFacetFromRay(const Primitive::Ray& ray, AbsG::RF& Facet)const;
	virtual bool IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist)const;
	virtual PTUtility::Vec3 GetSceneWidth()const{
		return m_OBVH->GetSceneCenter();
	}
	virtual PTUtility::Vec3 GetSceneCenter()const{
		return PTUtility::Vec3::Ones() * m_OBVH->GetSceneWidth();
	}
	const PTUtility::Attribute* GetAttribute(short ID)const{
		return m_Attributes.GetAttribute(ID);
	}
	const Texture2D* GetTexture(const std::string& TextureName)const{
		if (TextureName.size() < 2){
			return nullptr;
		}
		return m_Attributes.GetTexture(TextureName);
	}

	Scene(){}
	virtual ~Scene(){}
};
