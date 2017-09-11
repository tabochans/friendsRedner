#pragma once
#include"Vec3.h"
#include"Texture.h"
#include"Normal_Facet.h"
#include <immintrin.h>
#include<array>
#include<vector>
#include<map>
#include<string>
#include<memory>

namespace PTUtility{

	//SIMD friendly Polygon ??
	//This class pack eight polygons
	struct SIMD_Triangle8{
		//m_Pos[vertexID][Dimention]
		__m256 m_pos[3][3];

		SIMD_Triangle8(){ ; }

		//assert the size of Triangle8 is 8 
		SIMD_Triangle8(const std::array<Primitive::Triangle, 8>& Triangle8){
			for (int i = 0; i < 3; ++i){
				for (int j = 0; j < 3; ++j){
					float ar[8] = { 
						Triangle8[0].m_Pos[j](i),
						Triangle8[1].m_Pos[j](i),
						Triangle8[2].m_Pos[j](i),
						Triangle8[3].m_Pos[j](i),
						Triangle8[4].m_Pos[j](i),
						Triangle8[5].m_Pos[j](i),
						Triangle8[6].m_Pos[j](i),
						Triangle8[7].m_Pos[j](i)};
					m_pos[j][i] = _mm256_loadu_ps(ar);
				}
			}
		}
	};

	struct SIMD_Facet8{
	public:
		SIMD_Triangle8 m_Triangle8;
		std::array<Primitive::inoutFacet, 8> m_Facet8;

		SIMD_Facet8(const std::array<Primitive::inoutFacet, 8>& facet8) : m_Facet8(facet8), m_Triangle8(){
			std::array<Primitive::Triangle, 8> tri8;
			for (int i = 0; i < 8; ++i){
				std::array<Vec3, 3> tri = { facet8[i]._Pos[0], facet8[i]._Pos[1], facet8[i]._Pos[2] };
				tri8[i] = Primitive::Triangle(tri);
			}
			m_Triangle8 = SIMD_Triangle8(tri8);
		}
		~SIMD_Facet8(){
		}
	};
};