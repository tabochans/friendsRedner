#include"Intersect.h"


namespace Isc{


	template<>
	bool Intersect(const Primitive::Sphere& A, const Primitive::Sphere& B){
		if ((A.m_Pos - B.m_Pos).norm2() < (A.m_Radius *B.m_Radius)*(A.m_Radius*B.m_Radius)){
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Sphere& s, float& d){
		PTUtility::Vec3 ss = ray.m_Org - s.m_Pos;

		double A = 1.0f;
		double B = 2.0 * ss.dot(ray.m_Dir);
		double C = ss.norm2() - s.m_Radius*s.m_Radius;

		double D = B*B - 4.0 * A*C;

		if (D > 0.00001){

			if ((s.m_Pos - ray.m_Org).norm2() < s.m_Radius*s.m_Radius){
				d = 0.5 * (-B + sqrt(D));
			}
			else{
				d = 0.5 * (-B - sqrt(D));
			}
			return true;
		}


		return false;
	}

	template<>
	bool Intersect<Primitive::Sphere, Primitive::Ray>(const Primitive::Sphere& s, const Primitive::Ray& ray, float& d){
		return Intersect(ray, s, d);
	}


	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Polygon& p, float& d, float& u, float& v){
		const PTUtility::Vec3& dd = ray.m_Dir;
		const PTUtility::Vec3& e1 = (p.m_V[1].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& e2 = (p.m_V[2].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& ps = dd.cross(e2);

		float a = e1.dot(ps);
		if (a > -FLT_MIN && a < FLT_MIN) { u = v = d = 0.0; return false; }
		float f = 1.0f / a;
		PTUtility::Vec3 s = ray.m_Org - p.m_V[0].m_Pos;
		u = f * s.dot(ps);
		if (u < -0.01 || u > 1.0 + 0.01) { u = v = d = 0.0; return false; }

		PTUtility::Vec3 q = s.cross(e1);
		v = f * dd.dot(q);
		if (v < -0.01 || u + v > 1.0 + 0.01) { u = v = d = 0.0; return false; }

		d = f * e2.dot(q);

		return true;
	}

	template<>
	bool Intersect(const Primitive::Polygon& p, const Primitive::Ray& ray, float& d, float& u, float& v){
		return Intersect(ray, p, d, u, v);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Ray& ray){

		float tmax, tmin;
		const PTUtility::Vec3& max = aabb.m_MaxPos;
		const PTUtility::Vec3& min = aabb.m_MinPos;

		if (abs(ray.m_Dir.x()) < 0.0001){
			tmax = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (max.x() - ray.m_Org.x());
			tmin = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (min.x() - ray.m_Org.x());
		}
		else{
			tmax = (max.x() - ray.m_Org.x()) / ray.m_Dir.x();
			tmin = (min.x() - ray.m_Org.x()) / ray.m_Dir.x();
		}
		if (tmax < tmin){
			std::swap(tmax, tmin);
		}

		float tymin, tymax;
		if (abs(ray.m_Dir.y()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (max.y() - ray.m_Org.y());
			tymin = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (min.y() - ray.m_Org.y());
		}
		else{
			tymax = (max.y() - ray.m_Org.y()) / ray.m_Dir.y();
			tymin = (min.y() - ray.m_Org.y()) / ray.m_Dir.y();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		if (abs(ray.m_Dir.z()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (max.z() - ray.m_Org.z());
			tymin = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (min.z() - ray.m_Org.z());
		}
		else{
			tymax = (max.z() - ray.m_Org.z()) / ray.m_Dir.z();
			tymin = (min.z() - ray.m_Org.z()) / ray.m_Dir.z();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		//if (tmin < FLT_MIN){ return false; }

		return true;
	}
	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::AABB& aabb){
		return Intersect(aabb, ray);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Ray& ray, float& t, PTUtility::Vec3& Normal){

		float mm[3];
		float MM[3];


		float tmax, tmin;
		const PTUtility::Vec3& max = aabb.m_MaxPos;
		const PTUtility::Vec3& min = aabb.m_MinPos;

		if (abs(ray.m_Dir.x()) < 0.0001){
			tmax = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (max.x() - ray.m_Org.x());
			tmin = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (min.x() - ray.m_Org.x());
		}
		else{
			tmax = (max.x() - ray.m_Org.x()) / ray.m_Dir.x();
			tmin = (min.x() - ray.m_Org.x()) / ray.m_Dir.x();
		}
		if (tmax < tmin){
			std::swap(tmax, tmin);
		}
		MM[0] = tmax;
		mm[0] = tmin;



		float tymin, tymax;
		if (abs(ray.m_Dir.y()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (max.y() - ray.m_Org.y());
			tymin = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (min.y() - ray.m_Org.y());
		}
		else{
			tymax = (max.y() - ray.m_Org.y()) / ray.m_Dir.y();
			tymin = (min.y() - ray.m_Org.y()) / ray.m_Dir.y();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[1] = tmax;
		mm[1] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		if (abs(ray.m_Dir.z()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (max.z() - ray.m_Org.z());
			tymin = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (min.z() - ray.m_Org.z());
		}
		else{
			tymax = (max.z() - ray.m_Org.z()) / ray.m_Dir.z();
			tymin = (min.z() - ray.m_Org.z()) / ray.m_Dir.z();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[2] = tmax;
		mm[2] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;


		if (tmin < 0){
			t = tmax;
		}
		else{
			t = tmin;
		}


		PTUtility::Vec3 ppp = ray.m_Org + t*ray.m_Dir;

		int ii = 0;
		int jj = 0;
		float dd = 10000;
		for (int i = 0; i < 3; i++){
			if (abs(ppp[i] - max[i]) < dd){
				ii = i;
				jj = 1;
				dd = abs(ppp[ii] - max[ii]);
			}
			if (abs(ppp[i] - min[i]) < dd){
				ii = i;
				jj = -1;
				dd = abs(ppp[ii] - min[ii]);
			}
			Normal = PTUtility::Vec3(0, 0, 0);
			Normal[ii] = jj;
		}

		return true;
	}
	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::AABB& aabb, float& t, PTUtility::Vec3& Normal){
		return Intersect(aabb, ray, t, Normal);
	}

	template<>
	bool Intersect(const Primitive::AABB& A, const Primitive::AABB& B){

		for (int i = 0; i < 3; i++){
			if (A.m_MinPos[i] > B.m_MaxPos[i] || A.m_MaxPos[i] < B.m_MinPos[i]){
				return false;
			}

		}

		return true;
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::AABB& aabb){
		Primitive::AABB aa(s.m_Pos + s.m_Radius * PTUtility::Vec3::Ones(), s.m_Pos - s.m_Radius * PTUtility::Vec3::Ones());
		return Intersect(aa, aabb);
	}
	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Sphere& s){
		return Intersect(s, aabb);
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Vertex& v){
		if ((s.m_Pos - v.m_Pos).norm2() < s.m_Radius*s.m_Radius){
			return true;
		}
		else{
			return false;
		}
	}
	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::Sphere& s){
		return Intersect(s, v);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Vertex& v){
		for (int i = 0; i < 3; i++){
			if (aabb.m_MaxPos[i] < v.m_Pos[i] || aabb.m_MinPos[i] > v.m_Pos[i]){
				return false;
			}
		}
		return true;
	}
	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::AABB& aabb){
		return Intersect(aabb, v);
	}




	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Polygon& PL){

		PTUtility::Vec3 N = (0.333333 * (PL.m_V[0].m_Normal + PL.m_V[1].m_Normal + PL.m_V[2].m_Normal)).normalized();
		PTUtility::Vec3 p = PL.m_V[1].m_Pos - PL.m_V[0].m_Pos;
		PTUtility::Vec3 q = PL.m_V[2].m_Pos - PL.m_V[0].m_Pos;
		PTUtility::Vec3 pq = s.m_Pos - PL.m_V[0].m_Pos;

		float dd = 0;

		float mat[3][3] = {
			{ -N.x(), p.x(), q.x() },
			{ -N.y(), p.y(), q.y() },
			{ -N.z(), p.z(), q.z() }
		};

		float det =
			mat[0][0] * mat[1][1] * mat[2][2] + mat[1][0] * mat[2][1] * mat[0][2] + mat[2][0] * mat[0][1] * mat[1][2] -
			mat[0][0] * mat[2][1] * mat[1][2] - mat[2][0] * mat[1][1] * mat[0][2] - mat[1][0] * mat[0][1] * mat[2][2];

		if (abs(det) < 0.000001){
			return false;
		}


		float invmat[3][3] = {
			{ mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1], mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2], mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1] },
			{ mat[1][2] * mat[2][0] - mat[0][0] * mat[2][2], mat[0][0] * mat[2][2] - mat[0][2] * mat[2][1], mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2] },
			{ mat[1][0] * mat[2][1] - mat[1][1] * mat[2][1], mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1], mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0] }
		};

		PTUtility::Vec3 x(0, 0, 0);
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				x[i] += invmat[i][j] * pq[j] / det;
			}
		}

		if (x[0] <= s.m_Radius && x[1] >= 0.0f && x[2] >= 0.0f && x[1] + x[2] <= 1.0f){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[2].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[1].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if ((s.m_Pos - PL.m_V[0].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] < 0.0f){
			if (Intersect(Primitive::Ray(PL.m_V[0].m_Pos, p), Primitive::Sphere(s.m_Pos, s.m_Radius), dd)){
				return true;
			}
		}
		else if (x[2] < 0.0f){
			if (Intersect(Primitive::Ray(PL.m_V[0].m_Pos, q), Primitive::Sphere(s.m_Pos, s.m_Radius), dd)){
				return true;
			}
		}
		else if (x[1] + x[2] < 1.0f){
			if (Intersect(Primitive::Ray(PL.m_V[1].m_Pos, PL.m_V[2].m_Pos - PL.m_V[1].m_Pos), Primitive::Sphere(s.m_Pos, s.m_Radius), dd)){
				return true;
			}
		}

		else if (x[1] <= 0.0f && x[2] >= 1.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - PL.m_V[2].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] >= 1.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (s.m_Pos - PL.m_V[1].m_Pos).norm() <= s.m_Radius){
			return true;
		}
		else if (x[1] <= 0.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) <= 1.0f && (s.m_Pos - PL.m_V[0].m_Pos).norm() <= s.m_Radius){
			return true;
		}

		return false;
	}
	template<>
	bool Intersect(const Primitive::Polygon& PL, const Primitive::Sphere& s){
		return Intersect(s, PL);
	}


	bool AABB_RAY_SIMD_MASK(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const __m256 farMask,
		const __m256 nearMask,
		const int* sign,
		int& mask
	){

		__m256 tmin = _mm256_set1_ps(-100000.0f);
		__m256 tmax = _mm256_set1_ps(100000.0f);

		int idx0, idx1;

		idx0 = sign[1];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));

		idx0 = sign[2];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));

		idx0 = sign[0];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));

		mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GT_OS));
		mask = mask & (_mm256_movemask_ps(_mm256_cmp_ps(tmax, nearMask, _CMP_GT_OS)));
		mask = mask & (_mm256_movemask_ps(_mm256_cmp_ps(farMask, tmin, _CMP_GT_OS)));

		return (mask > 0);
	}

	bool AABB_RAY_SIMD(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const int* sign, 
		int& mask
		)
	{
		__m256 tmin = _mm256_set1_ps(-100000.0f);
		__m256 tmax = _mm256_set1_ps(100000.0f);
		__m256 near = _mm256_set1_ps(0.00000001f);

		int idx0, idx1;

		idx0 = sign[1];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));

		idx0 = sign[2];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));

		idx0 = sign[0];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));

		mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GT_OS));
		mask = mask & _mm256_movemask_ps(_mm256_cmp_ps(tmax, near, _CMP_GE_OS));

		return (mask > 0);

	}
}
