#pragma once
#include"SIMD_Facet.h"
#include"AbsBVH.h"
#include"AABB.h"
#include"Vec3.h"
#include<algorithm>
#include<memory>
#include<ostream>
#include<deque>
#include <immintrin.h>


namespace PTUtility{

	union U256f {
		__m256 _v;
		float _a[8];
	};

	//OBVH ‚æ‚­‚í‚©‚ñ‚È‚¢‚¯‚ÇSIMD‰»‚³‚ê‚Ä‚¢‚é‚ç‚µ‚¢
	class OBVH_SIMDFacet : public AbsG{

	private:

		int m_NumFacets;
		bool Ray_Triangle8_Intersect(const PTUtility::SIMD_Facet8& facet8, const __m256 rayOrg[3], const __m256 rayDir[3], std::vector<RF>& result, std::vector<int>& IDs)const;

		bool IsHitAABB_SIMD(
			const __m256 value[2][3],
			const __m256 org[3],
			const __m256 idir[3],
			const int* sign,
			int& mask)const;

		bool IsHitTriangle_SIMD(
			const __m256 value[2][3],
			const __m256 org[3],
			const __m256 idir[3],
			const int* sign,
			int& mask)const;


		static std::vector<Primitive::inoutFacet> const* m_Temp_inoutFacets;

		const float T_AABB;
		const float T_TRI;
		const int MAX_DEPTH;
		const int MIN_ELEMENT;
		const float ZEROP;

		//std::vector<PTUtility::SIMD_Facet8> m_data;
		Vec3 m_Max;
		Vec3 m_Min;

		enum DIVMODE{
			FIRST = 0,
			SECOND = 1,
			THIRD = 3
		};

		struct Node{
			//only leaf nodes have this value 
			std::vector<PTUtility::SIMD_Facet8> _8Facet;

			struct Facet__ID{
				int _ID;
			};
			std::vector<int> _facetID;
			std::vector<PTUtility::AABB> _AABB;
			int _Children[8];
			int _NumChildren;

			__m256 CAABB[2][3];

			Node()  : _8Facet(){
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
		std::deque<Node> m_NArray;

		static bool compFacetXP(const int left, const int right){
			return ((*m_Temp_inoutFacets)[left]._Center.x() - (*m_Temp_inoutFacets)[right]._Center.x()) < 0;
		}
		static bool compFacetYP(const int left, const int right){
			return ((*m_Temp_inoutFacets)[left]._Center.y() - (*m_Temp_inoutFacets)[right]._Center.y()) < 0;
		}
		static bool compFacetZP(const int left, const int right){
			return ((*m_Temp_inoutFacets)[left]._Center.z() - (*m_Temp_inoutFacets)[right]._Center.z()) < 0;
		}

		void Divide(Node& node, int GID, int mode);
		float CostAABB(
			const PTUtility::AABB& Parent,
			float sA, int NumTriangleA,
			float sB, int NumTriangleB) const;

		float ZeroCost(const PTUtility::AABB& Parent, int NumElements) const;

		void PWriteObj(std::ostream& ost, int PID) const;


		mutable std::vector<std::vector<int>> _FList;
		mutable std::vector<Vec3> _VList;
		mutable int _NF;
		mutable int _SS;
		mutable int _SSS;

		int PGetElements(const Primitive::Ray& ray, std::vector<RF>& Facets, int ID) const;
		int PGetElements(const Vec3& Center, float R, std::vector<RF>& Facets, int ID) const;
		int CreateAABBTree(int ID);

	protected:

	public:

		virtual const std::string Print() const{
			return std::string("OBVH");
		}
		virtual float GetSceneWidth();
		virtual Vec3 GetSceneCenter();
		virtual void CreateGraph(const std::vector<Primitive::inoutFacet>& FacetArray);
		virtual void ClearGraph();
		virtual void PrintGraph(std::ostream& ost) const;
		virtual void WriteObj(std::ostream& ost) const;
		virtual int GetElements(const Primitive::Ray& ray, std::vector<RF>& Facets) const;
		virtual int GetOneElement(const Primitive::Ray& ray, RF& Facet) const;
		virtual int GetElements(const Vec3& Center, float R, std::vector<RF>& Facets);

		OBVH_SIMDFacet();
		virtual ~OBVH_SIMDFacet();
	};



}