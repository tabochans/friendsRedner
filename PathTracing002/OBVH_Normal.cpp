#include"OBVH_Normal.h"

using namespace PTUtility;

int OBVH_Normal::GetElements(const Primitive::Ray& ray, std::vector<RF>& Facets) const{
	Primitive::Ray sray(ray.m_Org, ray.m_Dir);
	std::vector<mSec::OBVH<Primitive::inoutFacet>::IscData> result;
	m_OBVH.GetElementFromRay(sray, result);
	for (const auto& r : result){
		RF rf;
		rf._u = r._u; rf._v = r._v; rf._t = r._d; rf._Facet = r._pElement;
		Facets.push_back(rf);
	}
	return result.size();
}

int OBVH_Normal::GetOneElement(const Primitive::Ray& ray, RF& Facet) const{
	Primitive::Ray sray(ray.m_Org, ray.m_Dir);
	std::vector<mSec::OBVH<Primitive::inoutFacet>::IscData> result;
	m_OBVH.GetElementFromRay(sray, result);

	if (result.size() > 0){
		RF rf;
		rf._u = result[0]._u; rf._v = result[0]._v; rf._t = result[0]._d; rf._Facet = result[0]._pElement;
		Facet = rf;
	}
	return result.size();
}

bool OBVH_Normal::IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const {
	return m_OBVH.IfOccluded(ray, MinDist, MaxDist);
}

void OBVH_Normal::CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray){
	m_OBVH.CreateGraph(FacetArray);
}
void OBVH_Normal::ClearGraph(){
	m_OBVH.ClearGraph();
}