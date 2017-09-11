#pragma once

#include"Vec3.h"
#include"Intersect.h"
#include"Normal_Facet.h"
#include"IBL_Texture.h"
#include"MT.h"
#include<vector>

class Scene;

class LightManager{
public:
	struct AreaLight{
	private:

	public:
		Primitive::inoutFacet _Facet;
		PTUtility::Vec3 _Radiance;
		bool _BiDirectional;
		explicit AreaLight(const Primitive::inoutFacet& facet, const PTUtility::Vec3& Radiance, bool BiDirectional = false) : _BiDirectional(BiDirectional), _Facet(facet), _Radiance(Radiance){
		}
		virtual ~AreaLight(){}
	};

	struct SphereLight{
	private:
	public:
		PTUtility::Vec3 _Position;
		PTUtility::Vec3 _Flux;
		float _Radius;
		float _Area;

		explicit SphereLight(const PTUtility::Vec3& Position, const PTUtility::Vec3& Flux, float Radius) : _Position(Position), _Flux(Flux), _Radius(Radius), _Area(4.0f * PTUtility::PI * Radius*Radius){
		}
		virtual ~SphereLight(){}
	};


private:
	std::vector<SphereLight> m_SphereLightArray;
	std::vector<float> m_SLsuf;
	float _Total_Area_SL;

	std::vector<AreaLight> m_AreaLightArray;
	std::vector<float> m_ARsuf;
	float _Total_Area_AR;
	float _Total_WeightAR;

	IBLTexture* m_pIBL;

	int SelectLight(const std::vector<float>& AreaArray, float& pdf, RandomMT& MT)const;

public:

	int AddSphereLight(const SphereLight& spLight);
	int AddAreaLight(const AreaLight& ArLight);
	void SetIBL(const IBLTexture& IBL);
	void FinishAddingLight();
	float GetLightWeightTotal()const { return _Total_WeightAR; }

	// result  is ------- cos(neeVec, LightNormal) * radiance / (distance*distance) )
	int NextEventEstimation_SphereLight(const Scene& scene, const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const;
	int NextEventEstimation_AreaLight(const Scene& scene, const Primitive::Ray& currentRay, RandomMT& MT, PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const;

	const SphereLight* SamplePosition_SphereLight(float& pdf, PTUtility::Vec3& Position, RandomMT& MT)const;
	const AreaLight* SamplePosition_AreaLight(float& pdf, PTUtility::Vec3& Position, RandomMT& MT)const;

	LightManager() : m_pIBL(nullptr), _Total_Area_AR(0.0f), _Total_Area_SL(0.0f), _Total_WeightAR(0.0f){
	}
	virtual ~LightManager(){
		delete m_pIBL;
	}
};