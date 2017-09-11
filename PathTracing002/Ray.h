#pragma once
#include"Vec3.h"
namespace PTUtility{

	//Discribe Ray object 
	class Ray{
	public:
		PTUtility::Vec3 m_Org;
		PTUtility::Vec3 m_Dir;
		float m_Index;

		Ray operator-()const{
			return Ray(this->m_Org - 0.0001 * this->m_Dir, -this->m_Dir, this->m_Index);
		}

		Ray(const PTUtility::Vec3& Orgin, const PTUtility::Vec3& Direction) : m_Org(Orgin), m_Dir(Direction.normalized()), m_Index(1.0f){
		}
		Ray(const PTUtility::Vec3& Orgin, const PTUtility::Vec3& Direction, float Index) : m_Org(Orgin), m_Dir(Direction.normalized()), m_Index(Index){
		}
	};
}