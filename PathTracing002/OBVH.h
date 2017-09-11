#pragma once

#include<deque>
#include <immintrin.h>
#include"AbsBVH.h"
#include"Intersect.h"
#include<vector>


namespace mSec{

	template<typename E>
	class OBVH{

	public:
		struct IscData{
			float _d;
			float _u;
			float _v;
			const E* _pElement;

			bool operator<(const IscData& ref){
				return this->_d < ref._d;
			}
			bool operator>(const IscData& ref){
				return this->_d > ref._d;
			}
			bool operator<=(const IscData& ref){
				return this->_d <= ref._d;
			}
			bool operator>=(const IscData& ref){
				return this->_d >= ref._d;
			}
			IscData& operator=(const IscData& ref){
				_pElement = ref._pElement;
				_u = ref._u;
				_v = ref._v;
				_d = ref._d;
				return *this;
			}

			IscData(const E* e, float d, float u, float v) : _u(u), _v(v), _d(d), _pElement(e){
			}

		};

	private:

		bool m_IfCreated;

		const float COST_ELEMTNT;
		const float COST_AABB;
		const int MAX_DEPTH;
		const int MIN_ELEMENT;
		const float ZEROP;

		PTUtility::Vec3 m_Max;
		PTUtility::Vec3 m_Min;
		std::vector<E> m_data;

		struct Node{
			std::vector<const E*> _Elements;
			std::vector<Primitive::AABB> _AABBs;
			int _Children[8];
			char _NumChildren;
			__m256 CAABB[2][3];

			Node(){
				_Children[0] = 0;
				_Children[1] = 0;
				_Children[2] = 0;
				_Children[3] = 0;
				_Children[4] = 0;
				_Children[5] = 0;
				_Children[6] = 0;
				_Children[7] = 0;
				_NumChildren = 0;
			}
			~Node(){
			}
		};


		struct CmpNode {
			int _ElementHeadIDX;
			short _NumElements;
			short _NumChildren;
			int _Children[8];
			
			__m256 CAABB[2][3];

			CmpNode() {
				_ElementHeadIDX = 0;
				_NumElements = 0;
				_Children[0] = 0;
				_Children[1] = 0;
				_Children[2] = 0;
				_Children[3] = 0;
				_Children[4] = 0;
				_Children[5] = 0;
				_Children[6] = 0;
				_Children[7] = 0;
				_NumChildren = 0;
			}
			~CmpNode() {
			}
		};
		std::vector<CmpNode> m_CmpNodeArray;
		std::deque<Node> m_NArray;

		void Divide(Node& node, int GID, int mode, int Depth);

		bool DivideNode(const Node& Pnode, Node& leftNode, Node& rightNode);
		void CreateChildNode(Node& node, int Depth);
		int DivideNode8(const Node& ParentNode, std::vector<Node>& result, int Mode);

		float CostAABB(
			const Primitive::AABB& Parent,
			float sA, int NumElementsA,
			float SB, int NumElementsB)const;
		float ZeroCost(const Primitive::AABB& Parent, int NumElements)const;
		
		int PGetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result, int PID)const;

		template<typename K>
		int PGetElement(const K& key, ::std::vector<E>& result, int PID)const;

		int CreateAABBTree(int ID);
		void OBVH::PWriteObj(std::ostream& ost, int PID, std::vector<PTUtility::Vec3>& _VList, std::vector<std::vector<int>>& _FList, int& _NF, int& _SSS)const;

		int RefineGraph();

	protected:

	public:

		int GetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result)const;

		template<typename K>
		int GetElement(const K& key, std::vector<E>& result)const;

		const std::string Print() const{
			return ::std::string("OBVH");
		}
		float GetSceneWidth() const ;
		PTUtility::Vec3 GetSceneCenter() const;
		void CreateGraph(const ::std::vector<E>& Elements);
		void ClearGraph();
		void WriteObj(std::ostream& ost) const;

		bool IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const;

		OBVH();
		OBVH(
			float cost_AABB,
			float cost_Element,
			float Max_Depth,
			float MIN_Element);

		~OBVH();

		//return true if graph is created
		bool IfCreated()const{
			return m_IfCreated;
		}
	};

	template<typename E>
	int OBVH<E>::RefineGraph() {

		std::vector<E> NewData;

		for (const auto& n : m_NArray) {
			CmpNode nd;
			for (int i = 0; i < 8; ++i) {
				nd._Children[i] = n._Children[i];
			}
			nd._NumChildren = n._NumChildren;


			if (nd._NumChildren > 0) {
				float AABBMax[3][8];
				float AABBMin[3][8];
				for (int i = 0; i < 8; i++) {

					if (n._Children[i] > 0) {
						const Node& cn = m_NArray[n._Children[i]];
						for (int j = 0; j < 3; j++) {
							AABBMax[j][i] = cn._AABBs[0].m_MaxPos[j];
							AABBMin[j][i] = cn._AABBs[0].m_MinPos[j];
						}
					}
					else {
						for (int j = 0; j < 3; j++) {
							AABBMax[j][i] = -1000000;
							AABBMin[j][i] = 1000000;
						}
					}
				}
				for (int i = 0; i < 3; i++) {
					nd.CAABB[0][i] = _mm256_loadu_ps(AABBMin[i]);
					nd.CAABB[1][i] = _mm256_loadu_ps(AABBMax[i]);
				}
			}

			nd._ElementHeadIDX = -1;
			nd._NumElements = 0;
			for (int i = 0; i < n._Elements.size(); ++i) {
				NewData.push_back(*n._Elements[i]);
				if (i == 0) {
					nd._ElementHeadIDX = NewData.size() - 1;
				}
				++nd._NumElements;
			}

			m_CmpNodeArray.push_back(nd);
		}

		m_data = NewData;
		std::deque<Node>().swap(m_NArray);

		return 1;
	}

	template<typename E>
	int  OBVH<E>::DivideNode8(const Node& ParentNode, std::vector<Node>& result, int Mode) {
		
		if (Mode <= 0) {
			result.push_back(ParentNode);
			return result.size();
		}

		Node rightNode, leftNode;
		if (DivideNode(ParentNode, leftNode, rightNode)) {
			DivideNode8(leftNode, result, Mode - 1);
			DivideNode8(rightNode, result, Mode - 1);
		}
		else {
			if (Mode != 3) {
				result.push_back(ParentNode);
			}
		}

		return result.size();
	}


	template<typename E>
	void  OBVH<E>::CreateChildNode(Node& node, int Depth) {
		std::vector<Node> result;
		int NumChild = DivideNode8(node, result, 3);

		for (int i = 0; i < NumChild; ++i) {
			int cid = m_NArray.size();
			m_NArray.push_back(result[i]);
			node._Children[node._NumChildren] = cid;
			node._NumChildren++;

			CreateChildNode(m_NArray[cid], Depth + 1);
		}
		return;
	}

	template<typename E>
	bool  OBVH<E>::DivideNode(const Node& Pnode, Node& leftNode, Node& rightNode) {
		const int NElm = Pnode._Elements.size();

		//分割軸
		int Axis = -1;

		//分割場所leftが含むポリゴン数
		int Dpos = -1;

		//分割コスト
		float Cost = COST_ELEMTNT * NElm;

		std::vector<const E*> FA[3];
		FA[0] = Pnode._Elements;
		FA[1] = Pnode._Elements;
		FA[2] = Pnode._Elements;

		std::sort(FA[0].begin(), FA[0].end(), &E::compPX);
		std::sort(FA[1].begin(), FA[1].end(), &E::compPY);
		std::sort(FA[2].begin(), FA[2].end(), &E::compPZ);

		//三軸について
		for (int ax = 0; ax < 3; ax++) {

			//表面積の控え
			//SufArray[i] には、i個のポリゴンが含まれた場合の表面積
			std::vector<float> SufArray(NElm + 2);

			Primitive::AABB Bright, Bleft;

			//rightの表面積を計算しとく
			SufArray[0] = 1000000;
			Bleft.Reset();
			for (int i = NElm - 1; i >= 0; i--) {
				Bright.Expand(*FA[ax][i]);
				SufArray[NElm - i] = Bright.GetArea();
			}
			Bright.Reset();
			Bleft.Reset();

			//SAHの計算
			for (int i = 0; i < NElm; i++) {
				Bleft.Expand(*FA[ax][i]);
				float tCost = CostAABB(Pnode._AABBs[0], Bleft.GetArea(), i + 1, SufArray[NElm - i - 1], NElm - i - 1);
				if (Cost > tCost) {
					Axis = ax;
					Dpos = i + 1;
					Cost = tCost;
				}
			}
		}

		if (Axis != -1 && Dpos > 0 && Dpos < NElm && NElm > MIN_ELEMENT) {
			//分割先が見つかった

			leftNode._Elements.resize(Dpos);
			rightNode._Elements.resize(NElm - Dpos);
			Primitive::AABB Bright, Bleft;
			for (int i = 0; i < Dpos; i++) {
				Bleft.Expand(*FA[Axis][i]);
				leftNode._Elements[i] = FA[Axis][i];
			}
			for (int i = Dpos; i < NElm; i++) {
				Bright.Expand(*FA[Axis][i]);
				rightNode._Elements[i - Dpos] = FA[Axis][i];
			}
			leftNode._AABBs.push_back(Bleft);
			rightNode._AABBs.push_back(Bright);

			return true;
		}
		else {
			//分割しない

			return false;
		}
	}

	template<typename E>
	void  OBVH<E>::Divide(Node& node, int GID, int Mode, int Depth){

		int NElm = node._Elements.size();

		//分割軸
		int Axis = -1;
		//分割場所leftが含むポリゴン数
		int Dpos = -1;
		//分割コスト
		float Cost = COST_ELEMTNT * NElm;

		std::vector<const E*> FA[3];
		FA[0] = node._Elements;
		FA[1] = node._Elements;
		FA[2] = node._Elements;

		std::sort(FA[0].begin(), FA[0].end(), &E::compPX);
		std::sort(FA[1].begin(), FA[1].end(), &E::compPY);
		std::sort(FA[2].begin(), FA[2].end(), &E::compPZ);

		//三軸について
		for (int ax = 0; ax < 3; ax++){

			//表面積の控え
			//SufArray[i] には、i個のポリゴンが含まれた場合の表面積
			std::vector<float> SufArray(NElm + 2);

			Primitive::AABB Bright, Bleft;

			//rightの表面積を計算しとく
			SufArray[0] = 1000000;
			Bleft.Reset();
			for (int i = NElm - 1; i >= 0; i--){
				Bright.Expand(*FA[ax][i]);
				SufArray[NElm - i] = Bright.GetArea();
			}
			Bright.Reset();
			Bleft.Reset();

			//SAHの計算
			for (int i = 0; i < NElm; i++){
				Bleft.Expand(*FA[ax][i]);
				float tCost = CostAABB(node._AABBs[0], Bleft.GetArea(), i + 1, SufArray[NElm - i - 1], NElm - i - 1);
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
			Nleft._Elements.resize(Dpos);
			Nright._Elements.resize(NElm - Dpos);
			Primitive::AABB Bright, Bleft;
			for (int i = 0; i < Dpos; i++){
				Bleft.Expand(*FA[Axis][i]);
				Nleft._Elements[i] = FA[Axis][i];
			}
			for (int i = Dpos; i < NElm; i++){
				Bright.Expand(*FA[Axis][i]);
				Nright._Elements[i - Dpos] = FA[Axis][i];
			}
			Nleft._AABBs.push_back(Bleft);
			Nright._AABBs.push_back(Bright);


			//OBVHだから、さらに二回分割をする
			if (Mode == 0){
				Divide(Nleft, GID, 1, Depth++);
				Divide(Nright, GID, 1, Depth++);
				std::vector<const E*>().swap(node._Elements);
			}
			else if (Mode == 1){
				Divide(Nleft, GID, 2, Depth++);
				Divide(Nright, GID, 2, Depth++);
				std::vector<const E*>().swap(node._Elements);
				return;
			}
			else if (Mode == 2){
				int cid = m_NArray.size();
				m_NArray.push_back(Nleft);
				m_NArray.push_back(Nright);

				m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid;
				m_NArray[GID]._NumChildren++;

				m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid + 1;
				m_NArray[GID]._NumChildren++;

				std::vector<const E*>().swap(node._Elements);

				return;
			}

			for (int i = 0; i < 8; i++){
				int cd = m_NArray[GID]._Children[i];
				if (cd > 0){ Divide(m_NArray[cd], cd, 0, Depth++); }
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

	template<typename E>
	float OBVH<E>::CostAABB(
		const Primitive::AABB& Parent,
		float sA, int NumTriangleA,
		float sB, int NumTriangleB) const{

		return 2.0f * COST_AABB +
			NumTriangleA * COST_ELEMTNT * (sA) / (Parent.GetArea()) +
			NumTriangleB * COST_ELEMTNT * (sB) / (Parent.GetArea());
	}
	template<typename E>
	float OBVH<E>::ZeroCost(const Primitive::AABB& Parent, int NumElements) const{
		return CostAABB(Parent, Parent.GetArea(), NumElements, 10000000000000, 0);
	}
	template<typename E>
	int OBVH<E>::CreateAABBTree(int ID){
		//親ノードが子のAABB８個を一括で持つよーにする

		if (m_NArray[ID]._NumChildren == 0){
			return 1;
		}
		else{
			float AABBMax[3][8];
			float AABBMin[3][8];

			for (int i = 0; i < 8; i++){

				if (m_NArray[ID]._Children[i] > 0){
					const Node& cn = m_NArray[m_NArray[ID]._Children[i]];
					for (int j = 0; j < 3; j++){
						AABBMax[j][i] = cn._AABBs[0].m_MaxPos[j];
						AABBMin[j][i] = cn._AABBs[0].m_MinPos[j];
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

			m_NArray[ID]._AABBs.resize(8);
			for (int i = 0; i < 8; i++){
				m_NArray[ID]._AABBs[i] = Primitive::AABB(
					PTUtility::Vec3(
					AABBMax[0][i], AABBMax[1][i], AABBMax[2][i]
					),
					PTUtility::Vec3(
					AABBMin[0][i], AABBMin[1][i], AABBMin[2][i]
					)
					);
			}

			for (int i = 0; i < 8; i++){
				if (m_NArray[ID]._Children[i] > 0){
					CreateAABBTree(m_NArray[ID]._Children[i]);
				}
			}
		}
		return 1;
	}

	template<typename E>
	void OBVH<E>::CreateGraph(const std::vector<E>& FacetArray){

		m_NArray.clear();

		Node parent;
		Primitive::AABB pAB;

		parent._AABBs.push_back(pAB);
		m_NArray.push_back(parent);
		parent._AABBs.clear();

		m_data = FacetArray;

		for (int i = 0; i < m_data.size(); i++){
			pAB.Expand(m_data[i]);
		}
		m_Max = pAB.m_MaxPos;
		m_Min = pAB.m_MinPos;
		parent._AABBs.push_back(pAB);
		parent._Elements.resize(m_data.size());
		for (int i = 0; i < m_data.size(); i++){
			parent._Elements[i] = &m_data[i];
		}
		m_NArray.push_back(parent);
		CreateChildNode(m_NArray[1], 0);

		CreateAABBTree(1);

		return;
	}

	template<typename E>
	void OBVH<E>::ClearGraph(){
	}

	template<typename E>
	void OBVH<E>::PWriteObj(std::ostream& ost, int PID, std::vector<PTUtility::Vec3>& _VList, std::vector<std::vector<int>>& _FList, int& _NF, int& _SSS) const{

		if (m_NArray[PID]._Elements.size() > 0){

			Primitive::AABB A = m_NArray[PID]._AABBs[0];
			PTUtility::Vec3 c = A.GetCenter();
			float dx, dy, dz;
			dx = A.GetWidth()[0] * 0.5f;
			dy = A.GetWidth()[1] * 0.5f;
			dz = A.GetWidth()[2] * 0.5f;

			_VList.push_back(c + PTUtility::Vec3(-dx, -dy, dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, dy, dz));
			_VList.push_back(c + PTUtility::Vec3(dx, dy, dz));
			_VList.push_back(c + PTUtility::Vec3(dx, -dy, dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, -dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(dx, dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(dx, -dy, -dz));

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
			_SSS += m_NArray[PID]._Elements.size();
		}
		for (int i = 0; i < 8; i++){
			if (m_NArray[PID]._Children[i] > 0){ PWriteObj(ost, m_NArray[PID]._Children[i], _VList, _FList, _NF, _SSS); }
		}
	}

	template<typename E>
	void OBVH<E>::WriteObj(std::ostream& ost) const{

		if (m_NArray.size() <= 0) {
			return;
		}

		std::vector<PTUtility::Vec3> _VList;
		std::vector<std::vector<int>> _FList;
		int _NF = 0;
		int _SSS = 0;

		ost << " o BVH" << ::std::endl;

		PWriteObj(ost, 1, _VList, _FList, _NF, _SSS);

		for (int v = 0; v < _VList.size(); v++){
			ost << "v " << _VList[v][0] << " " << _VList[v][1] << " " << _VList[v][2] << ::std::endl;
		}
		for (int f = 0; f < _FList.size(); f++){
			ost << "f ";
			for (int ff = 0; ff < _FList[0].size(); ff++){
				ost << _FList[f][ff] << " ";
			}
			ost << ::std::endl;
		}
	}


	template<typename E>
	float OBVH<E>::GetSceneWidth() const{
		PTUtility::Vec3 w = m_Max - m_Min;
		return ::std::max(w[0], std::max(w[1], w[2]));
	}

	template<typename E>
	PTUtility::Vec3 OBVH<E>::GetSceneCenter() const{
		return 0.5 * (m_Max + m_Min);
	}

	template<typename E>
	OBVH<E>::OBVH(
		float cost_AABB,
		float cost_Element,
		float Max_Depth,
		float MIN_Element
		) : COST_AABB(cost_AABB), COST_ELEMTNT(cost_Element), MAX_DEPTH(Max_Depth), MIN_ELEMENT(MIN_Element), ZEROP(FLT_MIN){
	}

	template<typename E>
	OBVH<E>::OBVH(
		) : COST_AABB(0.8f), COST_ELEMTNT(1.0f), MAX_DEPTH(10), MIN_ELEMENT(1), ZEROP(FLT_MIN){
	}

	template<typename E>
	OBVH<E>::~OBVH(){
	}

	template<typename E>template<typename K>
	int OBVH<E>::GetElement(const K& key, std::vector<E>& result)const{
		result.clear();
		int n = PGetElement(key, result, 1);
		return n;
	}

	template<typename E>template<typename K>
	int OBVH<E>::PGetElement(const K& key, std::vector<E>& result, int PID)const{

		if (m_NArray[PID]._NumChildren == 0){

			for (int i = 0; i < m_NArray[PID]._Elements.size(); i++){

				E* vv = m_NArray[PID]._Elements[i];
				if (Isc::Intersect(key, *(vv))){
					result.push_back(*vv);
				}
			}
			return result.size();
		}
		else{

			const AABB* tab;
			for (int i = 0; i < m_NArray[PID]._NumChildren; i++){

				if (Isc::Intersect(m_NArray[PID]._AABBs[i], key)){
					PGetElement(key, result, m_NArray[PID]._Children[i]);
				}
			}
			return result.size();
		}

		return result.size();
	}

	template<typename E>
	int OBVH<E>::PGetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result, int PID)const{

		if (m_NArray[PID]._NumChildren == 0){

			for (int i = 0; i < m_NArray[PID]._Elements.size(); i++){

				E* vv = m_NArray[PID]._Elements[i];
				float d = 0.0f;
				if (Isc::Intersect(ray, *(vv), d)){
					IscData isc(vv, d, 0.0f, 0.0f);
					result.push_back(isc);
				}
			}
			return result.size();
		}
		else{

			const Primitive::AABB* tab = nullptr;
			for (int i = 0; i < m_NArray[PID]._NumChildren; i++){

				if (Isc::Intersect(m_NArray[PID]._AABBs[i], ray)){
					PGetElementFromRay(ray, result, m_NArray[PID]._Children[i]);
				}
			}
			return result.size();
		}

		return result.size();
	}

	
	template<typename E>
	int OBVH<E>::GetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result)const{
		result.clear();

		int n = PGetElementFromRay(ray, result, 1);
		std::sort(result.begin(), result.end());
		return n;
	}

	template<typename E>
	bool OBVH<E>::IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const {
		std::vector<IscData> result;
		int n = PGetElementFromRay(ray, result, 1);
		if (n > 0) {
			std::sort(result.begin(), result.end());

			if (result[0]._d < MaxDist && result[result.size() - 1]._d > MinDist) {
				return true;
			}		
		}
		return false;
	}

	template<>
	int OBVH<Primitive::Polygon>::PGetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result, int PID)const;

	template<>
	int OBVH<Primitive::Polygon>::GetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result)const;

	template<>
	int OBVH<Primitive::inoutFacet>::PGetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result, int PID)const;

	template<>
	int OBVH<Primitive::inoutFacet>::GetElementFromRay(const Primitive::Ray& ray, std::vector<IscData>& result)const;

	template<>
	void OBVH<Primitive::inoutFacet>::CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray);

	template<>
	bool OBVH<Primitive::inoutFacet>::IfOccluded(const Primitive::Ray& ray, float MinDist, float MaxDist) const;
}