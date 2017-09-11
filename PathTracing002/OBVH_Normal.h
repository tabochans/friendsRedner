#pragma once
#include"Normal_Facet.h"
#include"AbsBVH.h"
#include"Intersect.h"
#include"OBVH.h"


class OBVH_Normal : public AbsG{
private:
	mSec::OBVH<Primitive::inoutFacet> m_OBVH;

public:
	virtual void CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray);
	virtual void ClearGraph();

	//Ray‚ÆŒğ·‚µ‚½OŠpŒ`‚ğ‚·‚×‚Ä•Ô‚µ‚Ä‚­‚ê‚é‚Ì
	virtual int GetElements(const Primitive::Ray& ray, std::vector<RF>& Facets) const;
	virtual int GetOneElement(const Primitive::Ray& ray, RF& Facets) const;
	bool IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const;

	virtual float GetSceneWidth(){ return m_OBVH.GetSceneWidth(); }
	virtual PTUtility::Vec3 GetSceneCenter(){ return m_OBVH.GetSceneCenter(); }
	virtual void WriteObj(std::ostream& ost) const{
		m_OBVH.WriteObj(ost);
	}

public:
	OBVH_Normal(){

	}
	virtual ~OBVH_Normal(){

	}
};