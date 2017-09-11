#include"OBVH.h"
#include <algorithm>

namespace mSec{

	template<>
	int OBVH<Primitive::Polygon>::PGetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result, int PID)const{

		float u, v, t;
		u = v = t = 0.0f;
		if (m_NArray[PID]._NumChildren == 0){
			int nc = m_NArray[PID]._Elements.size();

			for (int i = 0; i < nc; i++){
				if (Isc::Intersect(ray, *m_NArray[PID]._Elements[i], t, u, v)){
					if (t > FLT_MIN){
						result.push_back(IscData(m_NArray[PID]._Elements[i], t, u, v));
					}
				}
			}
			return result.size();
		}

		__m256 org[3];
		__m256 idir[3];
		int sign[3];
		int mask = 0;

		for (int i = 0; i < 3; i++){

			sign[i] = ray.m_Dir[i] < 0;
			org[i] = _mm256_set1_ps(ray.m_Org[i]);

			if (abs(ray.m_Dir[i]) < 1.0f / 100000.0f){
				idir[i] = _mm256_set1_ps(100000.0f - 200000.0f * sign[i]);
			}
			else{
				idir[i] = _mm256_set1_ps(1.0f / ray.m_Dir[i]);
			}

		}


		if (Isc::AABB_RAY_SIMD(m_NArray[PID].CAABB, org, idir, sign, mask)){
			for (int i = 0; i < 8; i++){
				int fg = mask & (1 << (i));
				if ((fg) > 0){
					int iid = m_NArray[PID]._Children[i];
					if (iid > 0){ PGetElementFromRay(ray, result, iid); }
				}
			}
		}

		return result.size();
	}


	template<>
	int OBVH<Primitive::Polygon>::GetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result)const{
		result.clear();

		int n = PGetElementFromRay(ray, result, 1);
		::std::sort(result.begin(), result.end());
		return n;
	}


	template<>
	void OBVH<Primitive::inoutFacet>::CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray) {

		m_NArray.clear();

		Node parent;
		Primitive::AABB pAB;

		parent._AABBs.push_back(pAB);
		m_NArray.push_back(parent);
		parent._AABBs.clear();

		m_data = FacetArray;

		for (int i = 0; i < m_data.size(); i++) {
			pAB.Expand(m_data[i]);
		}
		m_Max = pAB.m_MaxPos;
		m_Min = pAB.m_MinPos;
		parent._AABBs.push_back(pAB);
		parent._Elements.resize(m_data.size());
		for (int i = 0; i < m_data.size(); i++) {
			parent._Elements[i] = &m_data[i];
		}
		m_NArray.push_back(parent);
		CreateChildNode(m_NArray[1], 0);
		RefineGraph();

		return;
	}

	template<>
	bool OBVH<Primitive::inoutFacet>::IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const {

		__m256 far = _mm256_set1_ps(MaxDist);
		__m256 near = _mm256_set1_ps(MinDist);

		std::vector<IscData> Facets;
		//Create SIMD Ray
		__m256 org[3];
		__m256 idir[3];
		__m256 dir[3];
		int sign[3];
		int mask = 0;

		for (int i = 0; i < 3; i++) {
			sign[i] = ray.m_Dir[i] < 0;
			org[i] = _mm256_set1_ps(ray.m_Org[i]);
			dir[i] = _mm256_set1_ps(ray.m_Dir[i]);

			if (abs(ray.m_Dir[i]) < 1.0f / 100000.0f) {
				idir[i] = _mm256_set1_ps(100000.0f - 200000.0f * sign[i]);
			}
			else {
				idir[i] = _mm256_set1_ps(1.0f / ray.m_Dir[i]);
			}
		}

		std::vector<int> _Stack;
		_Stack.reserve(10);
		_Stack.push_back(1);

		while (_Stack.size() > 0) {
			int CurrentNodeID = _Stack[_Stack.size() - 1];
			_Stack.pop_back();
			const CmpNode& temp(m_CmpNodeArray[CurrentNodeID]);

			if (temp._NumChildren == 0) {
				//Leaf Node

				int nc = m_CmpNodeArray[CurrentNodeID]._NumElements;
				float t, u, v = 0.0f;
				for (int i = 0; i < nc; i++) {
					if (Isc::Intersect(ray, *(m_data.cbegin()._Ptr + m_CmpNodeArray[CurrentNodeID]._ElementHeadIDX + i), t, u, v)) {
						Facets.push_back(IscData((m_data.cbegin()._Ptr + m_CmpNodeArray[CurrentNodeID]._ElementHeadIDX + i), t, u, v));
					}
				}
			}
			else {
				//Not Leaf Node

				if (Isc::AABB_RAY_SIMD_MASK(m_CmpNodeArray[CurrentNodeID].CAABB, org, idir, far, near, sign, mask)) {
					for (int i = 0; i < 8; i++) {
						int fg = mask & (1 << (i));
						if ((fg) > 0) {
							int iid = m_CmpNodeArray[CurrentNodeID]._Children[i];
							if (iid > 0) { _Stack.push_back(iid); }
						}
					}
				}
			}
		}

		if (Facets.size() == 0) {
			return false;
		}
		if (std::min_element(Facets.begin(), Facets.end())->_d < MaxDist - 0.005) {
			return true;
		}
		return false;
	}

	template<>
	int OBVH<Primitive::inoutFacet>::PGetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result, int PID)const{
		float u, v, t;
		u = v = t = 0.0f;
		if (m_CmpNodeArray[PID]._NumChildren == 0){
			for (int i = 0; i < m_CmpNodeArray[PID]._NumElements; i++){
				if (Isc::Intersect(ray, *(m_data.cbegin()._Ptr + m_CmpNodeArray[PID]._ElementHeadIDX + i), t, u, v)){
					if (t > FLT_MIN){
						result.push_back(IscData((m_data.cbegin()._Ptr + m_CmpNodeArray[PID]._ElementHeadIDX + i), t, u, v));
					}
				}
			}
			return result.size();
		}

		__m256 org[3];
		__m256 idir[3];
		int sign[3];
		int mask = 0;

		for (int i = 0; i < 3; i++){

			sign[i] = ray.m_Dir[i] < 0;
			org[i] = _mm256_set1_ps(ray.m_Org[i]);

			if (abs(ray.m_Dir[i]) < 1.0f / 100000.0f){
				idir[i] = _mm256_set1_ps(100000.0f - 200000.0f * sign[i]);
			}
			else{
				idir[i] = _mm256_set1_ps(1.0f / ray.m_Dir[i]);
			}

		}
		if (Isc::AABB_RAY_SIMD(m_CmpNodeArray[PID].CAABB, org, idir, sign, mask)){
			for (int i = 0; i < 8; i++){
				int fg = mask & (1 << (i));
				if ((fg) > 0){
					int iid = m_CmpNodeArray[PID]._Children[i];
					if (iid > 0){ PGetElementFromRay(ray, result, iid); }
				}
			}
		}
		return result.size();
	}


	template<>
	int OBVH<Primitive::inoutFacet>::GetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result)const{

		std::vector<IscData> Facets;
		//Create SIMD Ray
		__m256 org[3];
		__m256 idir[3];
		__m256 dir[3];
		int sign[3];
		int mask = 0;

		for (int i = 0; i < 3; i++){
			sign[i] = ray.m_Dir[i] < 0;
			org[i] = _mm256_set1_ps(ray.m_Org[i]);
			dir[i] = _mm256_set1_ps(ray.m_Dir[i]);

			if (abs(ray.m_Dir[i]) < 1.0f / 100000.0f){
				idir[i] = _mm256_set1_ps(100000.0f - 200000.0f * sign[i]);
			}
			else{
				idir[i] = _mm256_set1_ps(1.0f / ray.m_Dir[i]);
			}
		}

		std::vector<int> _Stack;
		_Stack.reserve(10);
		_Stack.push_back(1);

		while (_Stack.size() > 0){
			int CurrentNodeID = _Stack[_Stack.size() - 1];
			_Stack.pop_back();
			const CmpNode& temp(m_CmpNodeArray[CurrentNodeID]);

			if (temp._NumChildren == 0){
				//Leaf Node

				int nc = m_CmpNodeArray[CurrentNodeID]._NumElements;
				float t, u, v = 0.0f;
				for (int i = 0; i < nc; i++){
					if (Isc::Intersect(ray, *(m_data.cbegin()._Ptr + m_CmpNodeArray[CurrentNodeID]._ElementHeadIDX + i), t, u, v)){
						Facets.push_back(IscData((m_data.cbegin()._Ptr + m_CmpNodeArray[CurrentNodeID]._ElementHeadIDX + i), t, u, v));
					}
				}
			}
			else{
				//Not Leaf Node

				if (Isc::AABB_RAY_SIMD(m_CmpNodeArray[CurrentNodeID].CAABB, org, idir, sign, mask)){
					for (int i = 0; i < 8; i++){
						int fg = mask & (1 << (i));
						if ((fg) > 0){
							int iid = m_CmpNodeArray[CurrentNodeID]._Children[i];
							if (iid > 0){ _Stack.push_back(iid); }
						}
					}
				}
			}
		}

		if (Facets.size() == 1) {
			result.push_back(Facets[0]);
		}
		else if (Facets.size() > 1) {
			result.push_back(*std::min_element(Facets.begin(), Facets.end()));
		}
		return Facets.size();
	}
}