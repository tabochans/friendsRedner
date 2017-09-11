#include"OBVH_SIMDFacet.h"
using namespace PTUtility;

std::vector<Primitive::inoutFacet> const* OBVH_SIMDFacet::m_Temp_inoutFacets = nullptr;


bool OBVH_SIMDFacet::Ray_Triangle8_Intersect(
	const PTUtility::SIMD_Facet8& facet8, const __m256 rayOrg[3], const __m256 rayDir[3], std::vector<RF>& result, std::vector<int>& IDs) const{
	//m_pos[3 : TriangleID][3 : xyz]
	//rayOrg[3 : xyz]
	//rayDir[3 : xyz]

	//ひたすらにカオス 

	__m256 E1[3];
	__m256 E2[3];
	__m256 T[3];
	for (int i = 0; i < 3; ++i){
		E1[i] = _mm256_sub_ps(facet8.m_Triangle8.m_pos[1][i], facet8.m_Triangle8.m_pos[0][i]);
		E2[i] = _mm256_sub_ps(facet8.m_Triangle8.m_pos[2][i], facet8.m_Triangle8.m_pos[0][i]);
		T[i] = _mm256_sub_ps(rayOrg[i], facet8.m_Triangle8.m_pos[0][i]);
	}

	__m256 D_X_E2[3];
	__m256 T_X_E1[3];
	D_X_E2[0] = _mm256_sub_ps(_mm256_mul_ps(rayDir[1], E2[2]), _mm256_mul_ps(rayDir[2], E2[1]));
	T_X_E1[0] = _mm256_sub_ps(_mm256_mul_ps(T[1], E1[2]), _mm256_mul_ps(T[2], E1[1]));
	D_X_E2[1] = _mm256_sub_ps(_mm256_mul_ps(rayDir[2], E2[0]), _mm256_mul_ps(rayDir[0], E2[2]));
	T_X_E1[1] = _mm256_sub_ps(_mm256_mul_ps(T[2], E1[0]), _mm256_mul_ps(T[0], E1[2]));
	D_X_E2[2] = _mm256_sub_ps(_mm256_mul_ps(rayDir[0], E2[1]), _mm256_mul_ps(rayDir[1], E2[0]));
	T_X_E1[2] = _mm256_sub_ps(_mm256_mul_ps(T[0], E1[1]), _mm256_mul_ps(T[1], E1[0]));

	__m256 D_X_E2oE1 = _mm256_add_ps(_mm256_add_ps(
		_mm256_mul_ps(D_X_E2[0], E1[0]),
		_mm256_mul_ps(D_X_E2[1], E1[1])), 
		_mm256_mul_ps(D_X_E2[2], E1[2]));

	__m256 t = _mm256_div_ps(_mm256_add_ps(_mm256_add_ps(
		_mm256_mul_ps(T_X_E1[0], E2[0]),
		_mm256_mul_ps(T_X_E1[1], E2[1])),
		_mm256_mul_ps(T_X_E1[2], E2[2])), D_X_E2oE1);
	__m256 u = _mm256_div_ps(_mm256_add_ps(_mm256_add_ps(
		_mm256_mul_ps(D_X_E2[0], T[0]),
		_mm256_mul_ps(D_X_E2[1], T[1])),
		_mm256_mul_ps(D_X_E2[2], T[2])), D_X_E2oE1);
	__m256 v = _mm256_div_ps(_mm256_add_ps(_mm256_add_ps(
		_mm256_mul_ps(T_X_E1[0], rayDir[0]),
		_mm256_mul_ps(T_X_E1[1], rayDir[1])),
		_mm256_mul_ps(T_X_E1[2], rayDir[2])), D_X_E2oE1);

	__m256 zero = _mm256_set1_ps(0.000f);
	__m256 one = _mm256_set1_ps(1.000f);
	__m256 near = _mm256_set1_ps(0.00000001f);
	__m256 far = _mm256_set1_ps(1000.0f);

	int mask[8];
	mask[0] = _mm256_movemask_ps(_mm256_cmp_ps(u, zero, _CMP_GE_OS));
	mask[1] = _mm256_movemask_ps(_mm256_cmp_ps(one, u, _CMP_GE_OS));
	mask[2] = _mm256_movemask_ps(_mm256_cmp_ps(v, zero, _CMP_GE_OS));
	mask[3] = _mm256_movemask_ps(_mm256_cmp_ps(one, v, _CMP_GE_OS));
	mask[4] = _mm256_movemask_ps(_mm256_cmp_ps(one, _mm256_add_ps(u, v), _CMP_GE_OS));
	mask[5] = _mm256_movemask_ps(_mm256_cmp_ps(t, near, _CMP_GE_OS));
	//mask[6] = _mm256_movemask_ps(_mm256_cmp_ps(far, t, _CMP_GE_OS));
	mask[6] = 255;
	mask[7] = mask[0] & mask[1] & mask[2] & mask[3] & mask[4] & mask[5] & mask[6];

	const U256f res_t = { t };
	const U256f res_u = { u };
	const U256f res_v = { v };
	for (int i = 0; i < 8; ++i){
		if ((mask[7] & (1 << (i))) > 0){
			RF rf;
			rf._u = res_u._a[i];
			rf._v = res_v._a[i];
			rf._t = res_t._a[i];
			IDs.push_back(i);
			result.push_back(rf);
		}
	}
	return result.size();
}


bool OBVH_SIMDFacet::IsHitAABB_SIMD(
	const __m256 value[2][3],
	const __m256 org[3],
	const __m256 idir[3],
	const int* sign,
	int& mask) const
{
	__m256 tmin = _mm256_set1_ps(-100000.0f);
	__m256 tmax = _mm256_set1_ps(100000.0f);

	int idx0, idx1;

	// Y軸.
	idx0 = sign[1];
	idx1 = 1 - idx0;
	tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
	tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));

	// Z軸.
	idx0 = sign[2];
	idx1 = 1 - idx0;
	tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
	tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));

	// X軸.
	idx0 = sign[0];
	idx1 = 1 - idx0;
	tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
	tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));

	mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GT_OS));
	return (mask > 0);
}

void  OBVH_SIMDFacet::Divide(Node& node, int GID, int Mode){

	int NElm = node._facetID.size();

	//分割軸
	int Axis = -1;
	//分割場所leftが含むポリゴン数
	int Dpos = -1;
	//分割コスト
	float Cost = T_TRI * NElm;

	std::vector<int> FA[3];
	FA[0] = node._facetID;
	FA[1] = node._facetID;
	FA[2] = node._facetID;
	std::sort(FA[0].begin(), FA[0].end(), &compFacetXP);
	std::sort(FA[1].begin(), FA[1].end(), &compFacetYP);
	std::sort(FA[2].begin(), FA[2].end(), &compFacetZP);

	//三軸について
	for (int ax = 0; ax < 3; ax++){
		//表面積の控え
		//SufArray[i] には、i個のポリゴンが含まれた場合の表面積ね
		std::vector<float> SufArray(NElm + 2);
		AABB Bright, Bleft;

		//rightの表面積を計算しとく
		SufArray[0] = 1000000;
		Bleft.Reset();
		for (int i = NElm - 1; i >= 0; i--){
			Bright.InTriangle((*m_Temp_inoutFacets)[FA[ax][i]]._Pos[0], (*m_Temp_inoutFacets)[FA[ax][i]]._Pos[1], (*m_Temp_inoutFacets)[FA[ax][i]]._Pos[2]);
			SufArray[NElm - i] = Bright.GetArea();
		}
		Bright.Reset();
		Bleft.Reset();

		//SAHの計算
		for (int i = 0; i < NElm; i++){
			Bleft.InTriangle((*m_Temp_inoutFacets)[FA[ax][i]]._Pos[0], (*m_Temp_inoutFacets)[FA[ax][i]]._Pos[1], (*m_Temp_inoutFacets)[FA[ax][i]]._Pos[2]);
			float tCost = CostAABB(node._AABB[0], Bleft.GetArea(), i + 1, SufArray[NElm - i - 1], NElm - i - 1);
			if (Cost > tCost){
				Axis = ax;
				Dpos = i + 1;
				Cost = tCost;
			}
		}
	}

	if (Axis != -1 && Dpos > 0 && Dpos < NElm && NElm > MIN_ELEMENT){
		//分割先が見つかった

		Node Nright, Nleft;
		Nleft._facetID.resize(Dpos);
		Nright._facetID.resize(NElm - Dpos);
		AABB Bright, Bleft;
		for (int i = 0; i < Dpos; i++){
			Bleft.InTriangle((*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[0], (*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[1], (*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[2]);
			Nleft._facetID[i] = FA[Axis][i];
		}
		for (int i = Dpos; i < NElm; i++){
			Bright.InTriangle((*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[0], (*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[1], (*m_Temp_inoutFacets)[FA[Axis][i]]._Pos[2]);
			Nright._facetID[i - Dpos] = FA[Axis][i];
		}
		Nleft._AABB.push_back(Bleft);
		Nright._AABB.push_back(Bright);


		//OBVHだから、さらに二回分割をする
		if (Mode == 0){
			Divide(Nleft, GID, 1);
			Divide(Nright, GID, 1);
			std::vector<int>().swap(node._facetID);
		}
		if (Mode == 1){
			Divide(Nleft, GID, 2);
			Divide(Nright, GID, 2);
			std::vector<int>().swap(node._facetID);
			return;
		}
		if (Mode == 2){
			int cid = m_NArray.size();
			m_NArray.push_back(Nleft);
			m_NArray.push_back(Nright);

			m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid;
			m_NArray[GID]._NumChildren++;

			m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid + 1;
			m_NArray[GID]._NumChildren++;

			std::vector<int>().swap(node._facetID);

			return;
		}

		//そして時は動き出す
		for (int i = 0; i < 8; i++){
			int cd = m_NArray[GID]._Children[i];
			if (cd > 0){ Divide(m_NArray[cd], cd, 0); }
		}
	}
	else{
		//分割しない
		if (Mode == 1 || Mode == 2){
			int cid = m_NArray.size();
			m_NArray.push_back(node);
			m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid;
			m_NArray[GID]._NumChildren++;
		}
	}
}


float OBVH_SIMDFacet::CostAABB(
	const PTUtility::AABB& Parent,
	float sA, int NumTriangleA,
	float sB, int NumTriangleB) const{
	if (NumTriangleA + NumTriangleB < 16){
		return 
			2.0f * T_AABB + NumTriangleA * T_TRI * (sA) / (Parent.GetArea()) + NumTriangleB * T_TRI * (sB) / (Parent.GetArea()) + 
			exp(-(NumTriangleA - 8)*5.0)*0.01 + exp(-(NumTriangleB - 8) * 5.0) * 0.01;
	}
	else{
		return 2.0f * T_AABB + NumTriangleA * T_TRI * (sA) / (Parent.GetArea()) + NumTriangleB * T_TRI * (sB) / (Parent.GetArea());
	}
}

float OBVH_SIMDFacet::ZeroCost(const PTUtility::AABB& Parent, int NumElements) const{
	return CostAABB(Parent, Parent.GetArea(), NumElements, 10000000000000, 0);
}

int OBVH_SIMDFacet::CreateAABBTree(int ID){
	//親ノードが子のAABB８個を一括で持つよーにする

	if (m_NArray[ID]._NumChildren == 0){
		//leaf node
		if (m_NArray[ID]._facetID.size() == 0){

		}
		else{
			int num8Facet;
			if (m_NArray[ID]._facetID.size() % 8 == 0){
				num8Facet = m_NArray[ID]._facetID.size() / 8;
			}
			else{
				num8Facet = 1 + (m_NArray[ID]._facetID.size() / 8);
			}

			m_NArray[ID]._8Facet.clear();
			for (int i = 0; i < num8Facet; ++i){
				std::array<Primitive::inoutFacet, 8> tri8;
				for (int t = 0; t < 8; ++t){
					int iid = 8 * i + t;
					if (iid >= m_NArray[ID]._facetID.size()){
						tri8[t] = Primitive::inoutFacet::CreateDummyFacet();
					}
					else{
						m_NumFacets++;
						tri8[t] = (*m_Temp_inoutFacets)[m_NArray[ID]._facetID[iid]];
					}
				}
				m_NArray[ID]._8Facet.push_back(SIMD_Facet8(tri8));
			}

			std::vector<int>().swap(m_NArray[ID]._facetID);
			return 1;
		}
	}
	else{
		float AABBMax[3][8];
		float AABBMin[3][8];

		for (int i = 0; i < 8; i++){
			if (m_NArray[ID]._Children[i] > 0){
				const Node& cn = m_NArray[m_NArray[ID]._Children[i]];
				for (int j = 0; j < 3; j++){
					AABBMax[j][i] = cn._AABB[0].GetMaxPos()[j];
					AABBMin[j][i] = cn._AABB[0].GetMinPos()[j];
				}
			}
			else{
				for (int j = 0; j < 3; j++){
					AABBMax[j][i] = -1000000;
					AABBMin[j][i] = 1000000;
				}
			}
		}

		for (int i = 0; i < 3; i++){
			m_NArray[ID].CAABB[0][i] = _mm256_loadu_ps(AABBMin[i]);
			m_NArray[ID].CAABB[1][i] = _mm256_loadu_ps(AABBMax[i]);
		}
		m_NArray[ID]._AABB.clear();

		for (int i = 0; i < 8; i++){
			if (m_NArray[ID]._Children[i] > 0){
				CreateAABBTree(m_NArray[ID]._Children[i]);
			}
		}
	}
	return m_NumFacets;
}

void OBVH_SIMDFacet::CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray){
	m_Temp_inoutFacets = &FacetArray;
	m_NumFacets = 0;

	m_NArray.clear();

	Node parent;
	AABB pAB;

	parent._AABB.push_back(pAB);
	m_NArray.push_back(parent);
	parent._AABB.clear();

	for (const auto& f : FacetArray){
		pAB.InTriangle(f._Pos[0], f._Pos[1], f._Pos[2]);
	}
	m_Max = pAB.GetMaxPos();
	m_Min = pAB.GetMinPos();
	parent._AABB.push_back(pAB);
	parent._facetID.resize(FacetArray.size());
	for (int i = 0; i < FacetArray.size(); i++){
		parent._facetID[i] = i;
	}
	m_NArray.push_back(parent);
	Divide(m_NArray[1], 1, 0);
	CreateAABBTree(1);
	return;
}

void OBVH_SIMDFacet::ClearGraph(){
}


void OBVH_SIMDFacet::PWriteObj(std::ostream& ost, int PID) const{

	if (m_NArray[PID]._8Facet.size() > 0){

		AABB A = m_NArray[PID]._AABB[0];
		Vec3 c = A.GetCenter();
		float dx, dy, dz;
		dx = A.GetWidth()[0] * 0.5f;
		dy = A.GetWidth()[1] * 0.5f;
		dz = A.GetWidth()[2] * 0.5f;

		_VList.push_back(c + Vec3(-dx, -dy, dz));
		_VList.push_back(c + Vec3(-dx, dy, dz));
		_VList.push_back(c + Vec3(dx, dy, dz));
		_VList.push_back(c + Vec3(dx, -dy, dz));
		_VList.push_back(c + Vec3(-dx, -dy, -dz));
		_VList.push_back(c + Vec3(-dx, dy, -dz));
		_VList.push_back(c + Vec3(dx, dy, -dz));
		_VList.push_back(c + Vec3(dx, -dy, -dz));

		std::vector<int> tt;

		tt.push_back(1 + _NF * 8); tt.push_back(4 + _NF * 8); tt.push_back(3 + _NF * 8); tt.push_back(2 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		tt.push_back(1 + _NF * 8); tt.push_back(5 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(4 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		tt.push_back(2 + _NF * 8); tt.push_back(6 + _NF * 8); tt.push_back(5 + _NF * 8); tt.push_back(1 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		tt.push_back(3 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(6 + _NF * 8); tt.push_back(2 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		tt.push_back(4 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(3 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		tt.push_back(6 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(5 + _NF * 8);
		_FList.push_back(tt);
		tt.clear();

		_NF++;
		_SSS += m_NArray[PID]._8Facet.size();
	}


	for (int i = 0; i < 8; i++){
		if (m_NArray[PID]._Children[i] > 0){ PWriteObj(ost, m_NArray[PID]._Children[i]); }
	}
}

void OBVH_SIMDFacet::PrintGraph(std::ostream& ost) const{
}


void OBVH_SIMDFacet::WriteObj(std::ostream& ost) const{

	_VList.clear();
	_FList.clear();
	_NF = 0;
	_SSS = 0;

	ost << " o BVH" << std::endl;

	PWriteObj(ost, 1);

	for (int v = 0; v < _VList.size(); v++){
		ost << "v " << _VList[v][0] << " " << _VList[v][1] << " " << _VList[v][2] << std::endl;
	}
	for (int f = 0; f < _FList.size(); f++){
		ost << "f ";
		for (int ff = 0; ff < _FList[0].size(); ff++){
			ost << _FList[f][ff] << " ";
		}
		ost << std::endl;
	}
}

int OBVH_SIMDFacet::GetOneElement(const Primitive::Ray& ray, RF& Facet) const{

	std::vector<RF> Facets;
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
	_Stack.push_back(1);

	while (_Stack.size() > 0){
		int CurrentNodeID = _Stack[_Stack.size() - 1];
		_Stack.pop_back();
		const Node& temp(m_NArray[CurrentNodeID]);

		if (temp._NumChildren == 0){
			//Leaf Node

			for (const auto& f8 : temp._8Facet){
				std::vector<RF> resres;
				std::vector<int> IDs;
				if (Ray_Triangle8_Intersect(f8, org, dir, resres, IDs)){
					for (int s = 0; s < IDs.size(); ++s){
						resres[s]._Facet = &f8.m_Facet8[IDs[s]];
					}
				}
				for (const auto& re : resres){
					Facets.push_back(re);
				}
			}
		}
		else{
			//Not Leaf Node

			if (IsHitAABB_SIMD(temp.CAABB, org, idir, sign, mask)){
				for (int i = 0; i < 8; i++){
					int fg = mask & (1 << (i));
					if ((fg) > 0){
						int iid = temp._Children[i];
						if (iid > 0){ _Stack.push_back(iid); }
					}
				}
			}
		}
	}
	if (Facets.size() > 0){
		std::sort(Facets.begin(), Facets.end());
		Facet = Facets[0];
	}
	return Facets.size();
}

int OBVH_SIMDFacet::GetElements(const Primitive::Ray& ray, std::vector<RF>& Facets) const{
	Facets.clear();

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
	_Stack.push_back(1);

	while (_Stack.size() > 0){
		int CurrentNodeID = _Stack[_Stack.size() - 1];
		_Stack.pop_back();
		const Node& temp(m_NArray[CurrentNodeID]);

		if (temp._NumChildren == 0){
			//Leaf Node

			for (const auto& f8 : temp._8Facet){
				std::vector<RF> resres;
				std::vector<int> IDs;
				if (Ray_Triangle8_Intersect(f8, org, dir, resres, IDs)){
					for (int s = 0; s < IDs.size(); ++s){
						resres[s]._Facet = &f8.m_Facet8[IDs[s]];
					}
				}
				for (const auto& re : resres){
					Facets.push_back(re);
				}
			}
		}
		else{
			//Not Leaf Node

			if (IsHitAABB_SIMD(temp.CAABB, org, idir, sign, mask)){
				for (int i = 0; i < 8; i++){
					int fg = mask & (1 << (i));
					if ((fg) > 0){
						int iid = temp._Children[i];
						if (iid > 0){ _Stack.push_back(iid); }
					}
				}
			}
		}
	}
	std::sort(Facets.begin(), Facets.end());

	return Facets.size();
}

int OBVH_SIMDFacet::PGetElements(const Vec3& Center, float R, std::vector<RF>& Facets, int ID) const{
	return 0;
}

int OBVH_SIMDFacet::GetElements(const Vec3& Center, float R, std::vector<RF>& Facets){
	return 0;
}

float OBVH_SIMDFacet::GetSceneWidth(){
	Vec3 w = m_Max - m_Min;
	return std::max(w[0], std::max(w[1], w[2]));
}

Vec3 OBVH_SIMDFacet::GetSceneCenter(){
	return 0.5 * (m_Max + m_Min);
}


OBVH_SIMDFacet::OBVH_SIMDFacet() : T_AABB(1.0f), T_TRI(1.0), MAX_DEPTH(10), MIN_ELEMENT(8), ZEROP(FLT_MIN){
}

OBVH_SIMDFacet::~OBVH_SIMDFacet(){
}