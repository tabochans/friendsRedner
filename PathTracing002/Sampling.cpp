#include"Sampling.h"


	PTUtility::Vec3 Sampling::Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf){
		PTUtility::Vec3 result;
		PTUtility::Vec3 e1, e2, e3;
		e1 = Normal;

		if (abs(e1.x()) > 0.000001){
			e2 = PTUtility::Vec3(0, 1, 0).cross(e1).normalized();
		}
		else{
			e2 = PTUtility::Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1();
		float th2 = sqrt(th);

		result = e1*sqrt(1.000000001 - th) + e2*cos(ph)*th2 + e3*sin(ph)*th2;
		result.normalize();
		if (pdf) *pdf = result.dot(Normal) / PTUtility::PI;
		return result;
	}

	PTUtility::Vec3 Sampling::Bi_Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float Reflectance, float* pdf) {
		PTUtility::Vec3 result;
		if (MT.genrand64_real1() < Reflectance) {
			//Reflect

			result = Sampling::Cos_Sampling(MT, Normal, pdf);
			if (pdf) {
				*pdf *= Reflectance;
			}
		}
		else {
			//Transmit

			result = Sampling::Cos_Sampling(MT, -Normal, pdf);
			if (pdf) {
				*pdf *= (1.0 - Reflectance);
			}
		}
		return result;
	}

	PTUtility::Vec3 Sampling::Sphere_Sampling(RandomMT& MT, float* pdf){
		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1() * PTUtility::PI;
		th = acos(MT.genrand64_real1() * 2.0f - 1.0f);

		PTUtility::Vec3 result(sinf(th)*cosf(ph), sinf(th)*sinf(ph), cosf(th));
		result.normalize();
		if (pdf)*pdf = 1.0 / (4.0 * PTUtility::PI);
		return result;
	}
	PTUtility::Vec3 Sampling::HemSphere_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf){
		PTUtility::Vec3 e1, e2, e3;
		e1 = Normal;
		if (abs(e1.x()) > 0.00001){
			e2 = PTUtility::Vec3(0, 1, 0).cross(e1).normalized();
		}
		else{
			e2 = PTUtility::Vec3(1, 0, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1() * PTUtility::PI * 0.5f;
		th = acos(MT.genrand64_real1());

		PTUtility::Vec3 result(e1*cosf(th) + e2*cos(ph)*sinf(th) + e3*sin(ph)*sinf(th));
		result.normalize();
		if (pdf)*pdf = 1.0 / (2.0 * PTUtility::PI);

		return result;
	}
	PTUtility::Vec3 Sampling::Phong_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Power, float* pdf){
		PTUtility::Vec3 e1, e2, e3;
		PTUtility::Vec3 result;

		e1 = Sampling::Reflect_Sampling(Normal, InDir);

		if (abs(e1.x()) < 0.00001){
			e2 = PTUtility::Vec3(1, 0, 0).cross(e1).normalized();
		}
		else{
			e2 = PTUtility::Vec3(0, 1, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float ph = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float th = MT.genrand64_real1();
		float cth2 = pow(1 - th, 1.0f / (Power + 1.0f));

		float cph = cos(ph);
		float sph = sin(ph);
		float sth2 = sqrt(1.0001f - cth2*cth2);

		result = PTUtility::Vec3(e3* sth2 * cph + e2*sth2 * sph + e1*cth2).normalized();
		if (result.dot(Normal) < 0) {
			if (pdf) *pdf = 0.00001f;
		}
		else {
			if (pdf) *pdf = std::pow(std::max(0.0f, std::min(1.0f, result.dot(e1))), Power) * ((Power + 1.0f) / (2.0f * PTUtility::PI));
		}
		return result;
	}

	PTUtility::Vec3 Sampling::GGX_Sampling(RandomMT & MT, const PTUtility::Vec3 & Normal, const PTUtility::Vec3 & InDir, float Alpha, float * pdf)
	{
		PTUtility::Vec3 e1 = Normal;
		float phi = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float e = MT.genrand64_real1();
		float cth = std::sqrt((1.000001 - e) / (e * (Alpha*Alpha - 0.99999) + 1.0f));
		
		PTUtility::Vec3 e2, e3;
		if (abs(e1.x()) < 0.00001) {
			e2 = PTUtility::Vec3(1, 0, 0).cross(e1).normalized();
		}
		else {
			e2 = PTUtility::Vec3(0, 1, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();
		
		float cph = cos(phi);
		float sph = sin(phi);
		float sth = std::sqrt(1.000001f - cth*cth);
		PTUtility::Vec3 mN = PTUtility::Vec3(e3*sth*cph + e2*sth*sph + e1*cth).normalized();

		PTUtility::Vec3 result = Sampling::Reflect_Sampling(mN, InDir);

		if (result.dot(Normal) < 0.0001) {
			return GGX_Sampling(MT, Normal, InDir, Alpha, pdf);
		}
		if (pdf) {
			float cos = std::max(Sampling::sMin, std::min(1.0f, mN.dot(Normal)));
			float sin = std::sqrt(1.0 - cos*cos);
			*pdf = (Alpha * Alpha * cos) / (PTUtility::PI * std::pow((Alpha*Alpha - 1.0f)*cos*cos + 1.0f, 2.0f));
		}
		return result;
	}

	PTUtility::Vec3 Sampling::GGXGlass_Sampling(RandomMT & MT, const PTUtility::Vec3 & Normal, const PTUtility::Vec3 & InDir, float Alpha, float In_Index, float Out_Index, float * pdf)
	{
		PTUtility::Vec3 e1 = Normal;
		float phi = MT.genrand64_real1() * 2.0f * PTUtility::PI;
		float e = MT.genrand64_real1();
		float cth = std::sqrt((1.000001 - e) / (e * (Alpha*Alpha - 0.99999) + 1.0f));

		PTUtility::Vec3 e2, e3;
		if (abs(e1.x()) < 0.00001) {
			e2 = PTUtility::Vec3(1, 0, 0).cross(e1).normalized();
		}
		else {
			e2 = PTUtility::Vec3(0, 1, 0).cross(e1).normalized();
		}
		e3 = e1.cross(e2).normalized();

		float cph = cos(phi);
		float sph = sin(phi);
		float sth = std::sqrt(1.000001f - cth*cth);
		PTUtility::Vec3 mN = PTUtility::Vec3(e3*sth*cph + e2*sth*sph + e1*cth).normalized();
		float RefPDF = Sampling::Fresnel_Reflect_PDF(mN, InDir, In_Index, Out_Index);
		float reflectance = Sampling::Fresnel_Reflectance(mN, InDir, In_Index, Out_Index);

		if (MT.genrand64_real1() < RefPDF || reflectance > 0.99f) {
			PTUtility::Vec3 result = Sampling::Reflect_Sampling(mN, InDir);

			if (result.dot(Normal) < 0.01) {
				if (pdf) {
					*pdf = Sampling::DeltaPDF * Sampling::DeltaPDF;
				}
				return Normal;
			}

			if (pdf) {
				float cos = std::max(Sampling::sMin, std::min(1.0f, mN.dot(Normal)));
				float sin = std::sqrt(1.0 - cos*cos);
				*pdf = RefPDF * (Alpha * Alpha * cos) / (PTUtility::PI * std::pow((Alpha*Alpha - 1.0f)*cos*cos + 1.0f, 2.0f));
			}
			return result;
		}
		else {
			PTUtility::Vec3 result = Sampling::Refract_Sampling(mN, InDir, In_Index, Out_Index);

			if (result.dot(Normal) > -0.01) {
				if (pdf) {
					*pdf = Sampling::DeltaPDF * Sampling::DeltaPDF;
				}
				return -Normal;
			}

			if (pdf) {
				float cos = std::max(Sampling::sMin, std::min(1.0f, mN.dot(Normal)));
				float sin = std::sqrt(1.0 - cos*cos);
				*pdf = (1.0f - RefPDF) * (Alpha * Alpha * cos) / (PTUtility::PI * std::pow((Alpha*Alpha - 1.0f)*cos*cos + 1.0f, 2.0f));
			}
			return result;
		}
	}

	PTUtility::Vec3 Sampling::Reflect_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir){
		return (InDir - 2.0f * (Normal.dot(InDir)) * Normal).normalized();
	}

	float Sampling::Fresnel_Reflect_PDF(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index) {
		return 0.3f;
	}

	float Sampling::Fresnel_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index){
		float n1_div_n2 = (In_Index / Out_Index);
		float cosA = Normal.dot(-InDir);
		float sinA = std::sqrt(1.0000001f - cosA*cosA);
		float sinB = n1_div_n2 * sinA;
		if (sinB > 0.999999){
			return 1.0f;
		}
		float cosB = std::sqrt(1.0000001f - sinB*sinB);

		float tp, rp, ts, rs;
		tp = (2.0f * In_Index * cosA) / (Out_Index*cosA + In_Index*cosB);
		rp = (Out_Index * cosA - In_Index*cosB) / (Out_Index*cosA + In_Index * cosB);
		ts = (2.0f * In_Index * cosA) / (In_Index*cosA + Out_Index*cosB);
		rs = (In_Index*cosA - Out_Index*cosB) / (In_Index*cosA + Out_Index*cosB);
		return (0.5f * rs * rs) + (0.5f * rp *rp);
	}

	float Sampling::FresnelConductor_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Index, float absorption) {
		const float cos = -InDir.dot(Normal);
		const float idk2 = (Index * Index + absorption*absorption);
		const float icos2 = 2.0f * Index * cos;

		float rp_u = (idk2 * cos * cos + 1.0001);
		float rp = (rp_u - icos2) / (rp_u + icos2);

		float rs_u = (idk2 + cos * cos);
		float rs = (rs_u - icos2) / (rs_u + icos2);

		return 0.5f * (rs*rs + rp*rp);
	}

	PTUtility::Vec3 Sampling::Fresnel_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index, float* pdf){
		PTUtility::Vec3 result;
		PTUtility::Vec3 NN = (InDir.dot(Normal) > 0) ? -Normal : Normal;

		float RefPDF = Fresnel_Reflect_PDF(NN, InDir, In_Index, Out_Index);
		if (MT.genrand64_real1() > RefPDF) {
			//Refract

			result = Refract_Sampling(Normal, InDir, In_Index, Out_Index);
			if (pdf) *pdf = DeltaPDF * (1.0000001 - RefPDF);
		}
		else {
			//Reflect
			result = Reflect_Sampling(NN, InDir);
			if (pdf) *pdf = DeltaPDF * (RefPDF);
		}
		return result;
	}

	PTUtility::Vec3 Sampling::Refract_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index) {
		float eta = In_Index / Out_Index;
		PTUtility::Vec3 NN = (InDir.dot(Normal) > 0) ? -Normal : Normal;
		float LN = -InDir.dot(NN);

		float sq = 1.0f - eta*eta*(1.0f - LN*LN);
		if (sq < 0.0000000001) {
			return InDir;
		}
		return (-NN * std::sqrt(sq) - eta*(-InDir - LN * NN)).normalized();
	}
