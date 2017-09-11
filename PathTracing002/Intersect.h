#pragma once

#include"Vec3.h"
#include"Vec2.h"
#include<immintrin.h>
#include<vector>
#include<algorithm>

namespace Primitive{
	struct Sphere{

		PTUtility::Vec3 m_Pos;
		float m_Radius;

		static bool compPX(const Sphere* left, const Sphere* right){
			return left->m_Pos.x() - right->m_Pos.x() < 0;
		}
		static bool compPY(const Sphere* left, const Sphere* right){
			return left->m_Pos.y() - right->m_Pos.y() < 0;
		}
		static bool compPZ(const Sphere* left, const Sphere* right){
			return left->m_Pos.z() - right->m_Pos.z() < 0;
		}

		Sphere(const PTUtility::Vec3& Center, float R) : m_Pos(Center), m_Radius(R){
		}
		Sphere() : m_Pos(PTUtility::Vec3::Zero()), m_Radius(0.5f){
		}
		virtual ~Sphere(){
		}
	};

	struct Ray{

	public:

		PTUtility::Vec3 m_Org;
		PTUtility::Vec3 m_Dir;
		float m_Index;

		Ray operator-()const{
			return Ray(this->m_Org - 0.0001 * this->m_Dir, -this->m_Dir, this->m_Index);
		}

		static PTUtility::Vec3 ReflectRay(const Ray& in, const PTUtility::Vec3& normal){
			return (in.m_Dir - 2.0f * (normal.dot(in.m_Dir)) * normal).normalized();
		}

		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction) : m_Org(Origin), m_Dir(Direction.normalized()), m_Index(1.000){

		}
		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction, float Index) : m_Org(Origin), m_Dir(Direction.normalized()), m_Index(Index){

		}
		~Ray(){

		}

	};

	struct Vertex{
		PTUtility::Vec3 m_Pos;
		PTUtility::Vec3 m_Normal;
		PTUtility::Vec2 m_UV;


		static bool compPX(const Vertex* left, const Vertex* right){
			return (left->m_Pos.x() - right->m_Pos.x()) < 0;
		}
		static bool compPY(const Vertex* left, const Vertex* right){
			return (left->m_Pos.y() - right->m_Pos.y()) < 0;
		}
		static bool compPZ(const Vertex* left, const Vertex* right){
			return (left->m_Pos.z() - right->m_Pos.z()) < 0;
		}

		
		explicit Vertex(){

		}
		explicit Vertex(const PTUtility::Vec3& Pos) : m_Pos(Pos), m_Normal(0,1,0),m_UV(0,0){

		}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(0, 0){

		}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal, const PTUtility::Vec2& UV) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(UV){

		}

	};


	struct Polygon{
		std::vector<Vertex> m_V;

		PTUtility::Vec3 GetCenter()const{
			return 0.33333333 * (m_V[0].m_Pos + m_V[1].m_Pos + m_V[2].m_Pos);
		}

		static bool compPX(const Polygon* left, const Polygon* right){
			return (left->GetCenter().x() - right->GetCenter().x()) < 0;
		}
		static bool compPY(const Polygon* left, const Polygon* right){
			return (left->GetCenter().y() - right->GetCenter().y()) < 0;
		}
		static bool compPZ(const Polygon* left, const Polygon* right){
			return (left->GetCenter().z() - right->GetCenter().z()) < 0;
		}

		Polygon(const Vertex& v1, const Vertex& v2, const Vertex& v3) : m_V(3){
			m_V[0] = v1;
			m_V[1] = v2;
			m_V[2] = v3;
		}
		void CalNormal(){
			PTUtility::Vec3 N(0, 1, 0);
			PTUtility::Vec3 u = (m_V[2].m_Pos - m_V[0].m_Pos).normalized();
			PTUtility::Vec3 s = (m_V[1].m_Pos - m_V[0].m_Pos).normalized();

			N = u.cross(s).normalized();
			for (auto v : m_V){
				v.m_Normal = N;
			}

		}
		~Polygon(){

		}

	};

	struct AABB{

		PTUtility::Vec3 m_MinPos;
		PTUtility::Vec3 m_MaxPos;

		const PTUtility::Vec3 GetWidth(){
			return m_MaxPos - m_MinPos;
		}
		const PTUtility::Vec3 GetCenter(){
			return 0.500 * (m_MaxPos + m_MinPos);
		}

		//無効なAABBをつくる
		AABB() : m_MinPos(1000.001, 1000.001, 1000.001), m_MaxPos(-1000.001, -1000.001, -1000.001){}

		//最大と最小位置を指定
		AABB(const PTUtility::Vec3& max, const PTUtility::Vec3& min) : m_MinPos(min), m_MaxPos(max){}

		~AABB(){}



		template<typename T>
		void Expand(const T& obj);
		
		template<>
		void Expand<Polygon>(const Polygon& obj){
			
			float max[3], min[3];

			PTUtility::Vec3 p1(obj.m_V[0].m_Pos);
			PTUtility::Vec3 p2(obj.m_V[1].m_Pos);
			PTUtility::Vec3 p3(obj.m_V[2].m_Pos);

			//よくわからんがたぶん？あってる
			for (int i = 0; i < 3; i++){
				max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = (max[i] + 0.001f > m_MaxPos[i] ? max[i] + 0.001f : m_MaxPos[i]);
				m_MinPos(i) = (min[i] - 0.001f < m_MinPos[i] ? min[i] - 0.001f : m_MinPos[i]);
			}
		}

		template<>
		void Expand<PTUtility::Vec3>(const PTUtility::Vec3& p){
			PTUtility::Vec3 max = m_MaxPos;
			PTUtility::Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > p(i) ? max(i) : p(i);
				m_MinPos(i) = min(i) < p(i) ? min(i) : p(i);
			}
		}

		template<>
		void Expand<Primitive::Vertex>(const Primitive::Vertex& p){
			Expand<PTUtility::Vec3>(p.m_Pos);
		}

		template<>
		void Expand<AABB>(const AABB& ref){
			PTUtility::Vec3 max = m_MaxPos;
			PTUtility::Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > ref.m_MaxPos(i) ? max(i) : ref.m_MaxPos(i);
				m_MinPos(i) = min(i) < ref.m_MinPos(i) ? min(i) : ref.m_MinPos(i);
			}
		}
		template<>
		void Expand<Sphere>(const Sphere& ref){
			PTUtility::Vec3 max = ref.m_Pos + PTUtility::Vec3::Ones() * ref.m_Radius;
			PTUtility::Vec3 min = ref.m_Pos - PTUtility::Vec3::Ones() * ref.m_Radius;


			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = m_MaxPos(i) > max(i) ? m_MaxPos(i) : max(i);
				m_MinPos(i) = m_MinPos(i) < min(i) ? m_MinPos(i) : min(i);
			}
		}



		//無効化する
		void Reset(){
			m_MaxPos = PTUtility::Vec3(-1000.001, -1000.001, -1000.001);
			m_MinPos = PTUtility::Vec3(1000.001, 1000.001, 1000.001);
		}

		int MaxDimension()const{
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			if (x(0) > x(1)){
				if (x(0) > x(2)){ return 0; }
				else{ return 2; }
			}
			else{
				if (x(1) > x(2)){ return 1; }
				else{ return 2; }
			}
		}

		//表面積を得ます
		float GetArea()const{
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			return abs(2.0f * (x(0)*x(1) + x(1)*x(2) + x(2)*x(0)));
		}


	};



}




namespace Isc{


	template<typename T, typename S>
	bool Intersect(const T&, const S&);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d, float& u, float& v);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d, PTUtility::Vec3& Normal);

	bool AABB_RAY_SIMD(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const int* sign,
		int& mask);

	bool AABB_RAY_SIMD_MASK(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const __m256 farMask,
		const __m256 nearMask,
		const int* sign,
		int& mask);
}