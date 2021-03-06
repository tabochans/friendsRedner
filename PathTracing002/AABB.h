#pragma once

#include"Vec3.h"

namespace PTUtility{

	class AABB{

	private:
		Vec3 m_MinPos;
		Vec3 m_MaxPos;

	protected:


	public:

		const Vec3 GetWidth(){
			return m_MaxPos - m_MinPos;
		}
		const Vec3 GetCenter(){
			return 0.500 * (m_MaxPos + m_MinPos);
		}

		//無効なAABBをつくる
		AABB() : m_MinPos(1000.001, 1000.001, 1000.001), m_MaxPos(-1000.001, -1000.001, -1000.001){}

		//最大と最小位置を指定
		AABB(const Vec3& max, Vec3& min) : m_MinPos(min), m_MaxPos(max){}

		virtual ~AABB(){}


		//ある三角形をふくむよーに拡張
		void InTriangle(
			const Vec3& p1,
			const Vec3& p2,
			const Vec3& p3){

			float max[3], min[3];
			

			//よくわからんがたぶん？あってる
			for (int i = 0; i < 3; i++){
				max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = (max[i] + 0.01 > m_MaxPos[i] ? max[i] + 0.01 : m_MaxPos[i]);
				m_MinPos(i) = (min[i] - 0.01 < m_MinPos[i] ? min[i] - 0.01 : m_MinPos[i]);
			}

		}

		//球を含むように拡張　といってもたいしたことは無い
		void InSphere(
			const Vec3& Center, 
			const float R){

			float max[3], min[3];

			max[0] = Center[0] + R;
			max[1] = Center[1] + R;
			max[2] = Center[2] + R;

			min[0] = Center[0] - R;
			min[1] = Center[1] - R;
			min[2] = Center[2] - R;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = (max[i] + 0.01 > m_MaxPos[i] ? max[i] + 0.01 : m_MaxPos[i]);
				m_MinPos(i) = (min[i] - 0.01 < m_MinPos[i] ? min[i] - 0.01 : m_MinPos[i]);
			}

		}

		

		//融合してでかいＡＡＢＢをつくる
		void Combine(const AABB& ref){
			Vec3 max = m_MaxPos;
			Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > ref.m_MaxPos(i) ? max(i) : ref.m_MaxPos(i);
				m_MinPos(i) = min(i) < ref.m_MinPos(i) ? min(i) : ref.m_MinPos(i);
			}
		}

		//ある点を含むように拡張
		void Fit(const Vec3& p){
			Vec3 max = m_MaxPos;
			Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > p(i) ? max(i) : p(i);
				m_MinPos(i) = min(i) < p(i) ? min(i) : p(i);
			}
		}

		//無効化する
		void Reset(){
			m_MaxPos = Vec3(-1000.001, -1000.001, -1000.001);
			m_MinPos = Vec3(1000.001, 1000.001, 1000.001);
		}

		int MaxDimension()const{
			Vec3 x = m_MaxPos - m_MinPos;
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
			Vec3 x = m_MaxPos - m_MinPos;
			return abs(2.0f * (x(0)*x(1) + x(1)*x(2) + x(2)*x(0)));
		}

		const Vec3& GetMaxPos()const{ return m_MaxPos; }
		const Vec3& GetMinPos()const{ return m_MinPos; }

	private:


	};

}