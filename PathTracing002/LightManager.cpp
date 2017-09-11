#include"LightManager.h"
#include"Sampling.h"
#include"Scene.h"

int LightManager::AddSphereLight(const SphereLight& spLight){
	m_SphereLightArray.push_back(spLight);
	m_SLsuf.push_back(spLight._Area);
	_Total_Area_SL += spLight._Area;

	return m_SphereLightArray.size();
}
int LightManager::AddAreaLight(const AreaLight& ArLight){
	m_AreaLightArray.push_back(ArLight);

	float Weight = ArLight._Facet.CalculateArea() * (ArLight._Radiance.x() + ArLight._Radiance.y() + ArLight._Radiance.z());
	m_ARsuf.push_back(Weight);
	_Total_Area_AR += ArLight._Facet.CalculateArea();
	_Total_WeightAR += Weight;

	return m_AreaLightArray.size();
}
void LightManager::SetIBL(const IBLTexture& IBL){
	delete m_pIBL;
	m_pIBL = new IBLTexture(IBL);
}

void LightManager::FinishAddingLight(){
	for (auto& s : m_SLsuf){
		s /= _Total_Area_SL;
	}
	for (auto& s : m_ARsuf){
		s /= _Total_WeightAR;
	}
}

int LightManager::SelectLight(const std::vector<float>& AreaArray, float& selectpdf, RandomMT& MT)const{
	if (AreaArray.size() == 0){
		return -1;
	}
	float rnum = MT.genrand64_real1();
	float total = AreaArray[0];
	int result = 0;
	while (result < AreaArray.size()){
		if (total > rnum){
			selectpdf = AreaArray[result];
			return result;
		}
		result++;
		total += AreaArray[result];
	}
	selectpdf = AreaArray[AreaArray.size() - 1];
	return AreaArray.size() - 1;
}

int LightManager::NextEventEstimation_SphereLight(
	const Scene& scene, const Primitive::Ray& currentRay, RandomMT& MT, 
	PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const{
	result = PTUtility::Vec3::Zero();
	if (m_SphereLightArray.size() > 0){
		float selectpdf = 1.0;
		const auto pSL = (m_SphereLightArray.begin() + SelectLight(m_SLsuf, selectpdf, MT));
		
		PTUtility::Vec3 toLightCenter = pSL->_Position - currentRay.m_Org;
		PTUtility::Vec3 PointOnLight = pSL->_Position + (1.0001 * pSL->_Radius) * Sampling::HemSphere_Sampling(MT, -toLightCenter.normalized(), nullptr);
		PTUtility::Vec3 Ldir = (-PointOnLight + currentRay.m_Org).normalized();
		float Distance2 = (currentRay.m_Org - PointOnLight).norm2();

		AbsG::RF res;
		scene.GetOneFacetFromRay(Primitive::Ray(PointOnLight, Ldir), res);
		if (res._t * res._t + 0.0001 > Distance2){
			pdf = selectpdf / (pSL->_Area);
			PTUtility::Vec3 LightSurfaceNormal = (PointOnLight - pSL->_Position) / pSL->_Radius;
			float cosLV = LightSurfaceNormal.dot(Ldir);
			result = 0.5f * pSL->_Flux * cosLV / Distance2 / pdf;
			ToLight = -Ldir;
			return 1;
		}
	}
	return 0;
}

int LightManager::NextEventEstimation_AreaLight(
	const Scene& scene, const Primitive::Ray& currentRay, RandomMT& MT, 
	PTUtility::Vec3& result, float& pdf, PTUtility::Vec3& ToLight)const{

	result = PTUtility::Vec3::Zero();
	if (m_AreaLightArray.size() > 0){
		float selectpdf = 1.0;
		const auto pAR = (m_AreaLightArray.begin() + SelectLight(m_ARsuf, selectpdf, MT));

		float u = MT.genrand64_real1();
		float v = MT.genrand64_real1();
		if (u + v > 1.000){
			u = 1.0 - u;
			v = 1.0 - v;
		}
		PTUtility::Vec3 LightSurfaceNormal = (pAR->_Facet._Normal[0] + pAR->_Facet._Normal[1] + pAR->_Facet._Normal[2]).normalized();
		PTUtility::Vec3 PointOnLight = (u * pAR->_Facet._Pos[1]) + (v * pAR->_Facet._Pos[2]) + (pAR->_Facet._Pos[0] * (1.0 - u - v)) + LightSurfaceNormal * 0.0001;
		float Distance2 = (currentRay.m_Org - PointOnLight).norm2();
		PTUtility::Vec3 Ldir = (-PointOnLight + currentRay.m_Org).normalized();

		AbsG::RF res;
		scene.GetOneFacetFromRay(Primitive::Ray(PointOnLight + Ldir * 0.001, Ldir), res);
		if (res._t * res._t + 0.01 > Distance2 && Distance2){
			pdf = (selectpdf) / pAR->_Facet.CalculateArea();
			float cosLV = LightSurfaceNormal.dot(Ldir);
			if (pAR->_BiDirectional) {
				cosLV = std::abs(cosLV);
			}
			result = pAR->_Radiance * (cosLV / Distance2 / pdf);
			ToLight = -Ldir;
			return 1;
		}
	}
	return 0;
}


const LightManager::SphereLight* LightManager::SamplePosition_SphereLight(float& pdf, PTUtility::Vec3& Position, RandomMT& MT)const{
	if (m_SphereLightArray.size() > 0){
		float selectpdf = 1.0;
		const auto pSL = (m_SphereLightArray.begin() + SelectLight(m_SLsuf, selectpdf, MT));
		Position = pSL->_Position + (1.0001 * pSL->_Radius) * Sampling::Sphere_Sampling(MT, nullptr);
		pdf = selectpdf / pSL->_Area;
		return pSL._Ptr;
	}
	else{
		return nullptr;
	}
}
const LightManager::AreaLight* LightManager::SamplePosition_AreaLight(float& pdf, PTUtility::Vec3& Position, RandomMT& MT)const{
	if (m_AreaLightArray.size() > 0){
		float selectpdf = 1.0;
		const auto pAR = (m_AreaLightArray.begin() + SelectLight(m_ARsuf, selectpdf, MT));

		float u = MT.genrand64_real1();
		float v = MT.genrand64_real1();
		if (u + v > 1.000){
			u = 1.0 - u;
			v = 1.0 - v;
		}
		PTUtility::Vec3 LightSurfaceNormal = (pAR->_Facet._Normal[0] + pAR->_Facet._Normal[1] + pAR->_Facet._Normal[2]).normalized();
		Position = (u * pAR->_Facet._Pos[1]) + (v * pAR->_Facet._Pos[2]) + (pAR->_Facet._Pos[0] * (1.0 - u - v)) + LightSurfaceNormal * 0.00001;
		pdf = (selectpdf) / pAR->_Facet.CalculateArea();
		return pAR._Ptr;
	}
	else{
		return nullptr;
	}
}