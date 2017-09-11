#pragma once
#include"Vec3.h"
#include"Intersect.h"
#include"Intersect.h"
#include"Normal_Facet.h"
#include<vector>

class AbsG{
private:
protected:
public:

	//れいとれデータの受け渡し用の構造体なの
	struct RF{
	public:
		float _u, _v, _t;
		const Primitive::inoutFacet* _Facet;

		bool operator<(const RF& ref){
			return this->_t < ref._t;
		}
		bool operator>(const RF& ref){
			return this->_t > ref._t;
		}
		bool operator<=(const RF& ref){
			return this->_t <= ref._t;
		}
		bool operator>=(const RF& ref){
			return this->_t >= ref._t;
		}
		RF& operator=(const RF& ref){
			_Facet = ref._Facet;
			_u = ref._u;
			_v = ref._v;
			_t = ref._t;
			return *this;
		}
	};

	virtual void CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray) = 0;
	virtual void ClearGraph() = 0;
	virtual void PrintGraph(std::ostream& ost) const{ ; }
	virtual void WriteObj(std::ostream& ost) const{ ; }
	virtual const std::string Print() const{
		return std::string("iGraph");
	}

	//Rayと交差した三角形をすべて返してくれるの
	virtual int GetElements(const Primitive::Ray& ray, std::vector<RF>& Facets) const = 0;
	virtual int GetOneElement(const Primitive::Ray& ray, RF& Facets) const = 0;
	virtual bool IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const = 0;
	
	//球と交差した三角形をすべて返してくれるの
	virtual int GetElements(const PTUtility::Vec3& Center, float R, std::vector<RF>& Facets){ return 0; }

	virtual float GetSceneWidth(){ return 1.0f; }
	virtual PTUtility::Vec3 GetSceneCenter(){ return PTUtility::Vec3::Zero(); }

	AbsG(){

	}
	virtual ~AbsG(){

	}

};