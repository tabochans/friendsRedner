#pragma once
#include"Vec3.h"
#include"Intersect.h"
#include"Texture.h"
#include"MT.h"
#include"material.h"
#include<array>
#include<vector>
#include<map>
#include<string>
#include<memory>
#include<functional>

namespace Primitive{

	struct Triangle{
	public:
		std::array<PTUtility::Vec3, 3> m_Pos;
		explicit Triangle(const std::array<PTUtility::Vec3, 3>& v) : m_Pos(v){
		}
		explicit Triangle() : m_Pos(std::array<PTUtility::Vec3, 3>()){
		}
	};

	struct inoutFacet{

		std::array<PTUtility::Vec3, 3> _Pos;
		std::array<PTUtility::Vec3, 3> _Normal;
		std::array<PTUtility::Vec3, 3> _UV;

		PTUtility::Vec3 _Vec_1_0;
		PTUtility::Vec3 _Vec_2_0;

		PTUtility::Vec3 _Center;
		short _AttributeID;

		inoutFacet(const std::array<PTUtility::Vec3, 3>& Pos, const std::array<PTUtility::Vec3, 3>& Normal, const std::array<PTUtility::Vec3, 3>& UV, short ID)
			: _Pos(Pos), _Normal(Normal), _UV(UV), _AttributeID(ID), _Center(0.33333333 * (Pos[0] + Pos[1] + Pos[2])){
			_Vec_1_0 = _Pos[1] - _Pos[0];
			_Vec_2_0 = _Pos[2] - _Pos[0];
		}
		inoutFacet()
			: _Pos(std::array<PTUtility::Vec3, 3>()), _Normal(std::array<PTUtility::Vec3, 3>()), _UV(std::array<PTUtility::Vec3, 3>()), _AttributeID(-1234), _Center(){
		}

		static bool compPX(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.x() < right->_Center.x();
		}
		static bool compPY(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.y() < right->_Center.y();
		}
		static bool compPZ(const inoutFacet* left, const inoutFacet* right){
			return left->_Center.z() < right->_Center.z();
		}

		static inoutFacet CreateDummyFacet(){
			std::array<PTUtility::Vec3, 3> Pos = { PTUtility::Vec3(10000.00001, 10000, 10000), PTUtility::Vec3(10000.0, 10000.00001, 10000), PTUtility::Vec3(10000.0, 10000, 10000.00001) };
			std::array<PTUtility::Vec3, 3> Normal = { PTUtility::Vec3(1, 0, 0), PTUtility::Vec3(1, 0, 0), PTUtility::Vec3(1, 0, 0) };
			std::array<PTUtility::Vec3, 3> UV = { PTUtility::Vec3::Zero(), PTUtility::Vec3::Zero(), PTUtility::Vec3::Zero() };
			return inoutFacet(Pos, Normal, UV, -1234);
		}

		float CalculateArea()const{
			return 0.5f * ((_Vec_1_0).cross(_Vec_2_0).norm());
		}
	};

}

namespace PTUtility{

	class Vec3;

	class Attribute{
	private:
		Attribute(const Attribute& ref);
		Attribute& operator=(const Attribute& ref);
	public:
		float m_Index;
		bool operator<(const Attribute& ref){
			return m_Albedo.norm2() * m_Index < ref.m_Albedo.norm2() * ref.m_Index;
		}
		bool operator>(const Attribute& ref){
			return m_Albedo.norm2() * m_Index > ref.m_Albedo.norm2() * ref.m_Index;
		}
		bool operator<=(const Attribute& ref){
			return m_Albedo.norm2() * m_Index <= ref.m_Albedo.norm2() * ref.m_Index;
		}
		bool operator>=(const Attribute& ref){
			return m_Albedo.norm2() * m_Index >= ref.m_Albedo.norm2() * ref.m_Index;
		}
		bool operator==(const Attribute& ref){
			return m_Albedo.norm2() * m_Index == ref.m_Albedo.norm2() * ref.m_Index;
		}
		typedef Vec3(*RF_PDF)(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float* pdf, RandomMT& MT);

		// if material is opaque, InDir.dot(OutDir) < 0
		typedef float(*RF)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir);
		
		PTUtility::SurfaceMaterial::BRDF_Function m_BRDF;
		PTUtility::SurfaceMaterial::BRDF_Function m_BTDF;
		PTUtility::SurfaceMaterial::Sampling_Function m_BRDF_Sample;
		PTUtility::SurfaceMaterial::Sampling_Function m_BTDF_Sample;
		PTUtility::SurfaceMaterial::PDF_Function m_BRDF_PDF;
		PTUtility::SurfaceMaterial::PDF_Function m_BTDF_PDF;
		PTUtility::Vec3 m_Albedo;
		PTUtility::Vec3 m_Emission;
		std::vector<std::string> TextureFileNames;
		std::string _AttributeName;
		bool m_DeltaSurface;
		float m_Param1;
		float m_Param2;

		Attribute() : m_Param1(0.00001f), m_Param2(0.00001), m_DeltaSurface(false), m_BRDF(nullptr), m_BTDF(nullptr), m_BRDF_PDF(nullptr), m_BTDF_PDF(nullptr), m_BRDF_Sample(nullptr), m_BTDF_Sample(nullptr), m_Albedo(0, 0, 0), m_Index(1.0), _AttributeName("Default"){
		}
	};

	class AttributeManager{
	private:
		std::map<std::string, std::unique_ptr<Texture2D>> m_TextureMap;
		std::map<short, std::unique_ptr<Attribute>>m_AttributeTable;
	public:

		bool LoadTexture(const char* TextureFileName){
			if (std::string(TextureFileName).size() < 2){
				return false;
			}
			if (m_TextureMap.find(TextureFileName) == m_TextureMap.end()){
				Texture2D* tex = new Texture2D();
				bool load = tex->LoadTextureFromFile(TextureFileName);
				if (load){
					m_TextureMap[std::string(TextureFileName)] = std::unique_ptr<Texture2D>(tex);
					return true;
				}
				else{
					return false;
				}
			}
			else{
				return true;
			}
		}

		void Add_Attribute(Attribute* attribute, short AttributeID){
			m_AttributeTable[AttributeID] = std::unique_ptr<Attribute>(attribute);
		}
		const Attribute* GetAttribute(short AttributeID)const{
			return m_AttributeTable.at(AttributeID).get();
		}
		const Texture2D* GetTexture(const std::string& TextureName)const{
			return m_TextureMap.at(TextureName).get();
		}
		bool FindTexture(const std::string& TextureName)const{
			return (m_TextureMap.find(TextureName) != m_TextureMap.end());
		}
	};

}

namespace Isc{

	template<>
	bool Intersect(const Primitive::inoutFacet& p, const Primitive::Ray& ray, float& d, float& u, float& v);

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::inoutFacet& p, float& d, float& u, float& v);
}

template<>
void Primitive::AABB::Expand<Primitive::inoutFacet>(const Primitive::inoutFacet&);