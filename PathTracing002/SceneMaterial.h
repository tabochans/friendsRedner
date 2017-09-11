#pragma once
#include"material.h"
#include"Normal_Facet.h"
#include<string>
#include<fstream>

namespace PTUtility{
	class SceneMaterialManager{
	public:
		virtual void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const{
			at->m_BRDF = DiffuseSurface::Diffuse;
			at->m_BRDF_PDF = DiffuseSurface::PDF;
			at->m_BRDF_Sample = DiffuseSurface::Diffuse_Sampling;
			at->m_BTDF = nullptr;
			at->m_BTDF_PDF = nullptr;
			at->m_BTDF_Sample = nullptr;
		}
		virtual void LoadMaterialTable(const char* FileName){
		}
	};
}