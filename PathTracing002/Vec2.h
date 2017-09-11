#pragma once
#include<cmath>

namespace PTUtility{
	class Vec2{
	private:
		float a[3];

	public:
		static Vec2 Ones(){
			return Vec2(1, 1);
		}
		static Vec2 Zero(){
			return Vec2(0, 0);
		}

		inline const Vec2 operator+(const Vec2& ref)const{
			return Vec2(a[0] + ref.x(), a[1] + ref.y());
		}
		inline Vec2& operator+=(const Vec2& ref){
			a[0] += ref.x();
			a[1] += ref.y();
			return *this;
		}

		inline const Vec2 operator-(const Vec2& ref)const{
			return Vec2(a[0] - ref.x(), a[1] - ref.y());
		}
		inline Vec2& operator-=(const Vec2& ref){
			a[0] -= ref.x();
			a[1] -= ref.y();
			return *this;
		}

		inline Vec2& operator*=(const Vec2& ref){
			a[0] *= ref.x();
			a[1] *= ref.y();
			return *this;
		}
		inline Vec2 operator*(const Vec2& ref) const{
			return PTUtility::Vec2(a[0] * ref.x(), a[1] * ref.y());
		}

		inline Vec2& operator/=(const Vec2& ref){
			a[0] /= ref.x();
			a[1] /= ref.y();
			return *this;
		}
		inline Vec2 operator/(const Vec2& ref) const{
			return PTUtility::Vec2(a[0] / ref.x(), a[1] / ref.y());
		}

		inline const Vec2 operator-()const{
			return Vec2(-a[0], -a[1]);
		}
		inline const Vec2 operator+()const{
			return *this;
		}

		Vec2& operator=(const Vec2& ref){
			a[0] = ref.x();
			a[1] = ref.y();
			return *this;
		}

		//Other operator
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		float dot(const Vec2& ref)const{
			return a[0] * ref.x() + a[1] * ref.y();
		}
		float norm()const{
			return sqrt(a[0] * a[0] + a[1] * a[1]);
		}
		float norm2()const{
			return a[0] * a[0] + a[1] * a[1];
		}

		void normalize(){
			float d = norm();
			a[0] /= d;
			a[1] /= d;
		}
		Vec2 normalized()const{
			float d = norm();
			return Vec2(a[0] / d, a[1] / d);
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Geter 
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline float x()const{
			return a[0];
		}
		inline float y()const{
			return a[1];
		}
		inline float operator[](size_t id)const{
			return a[id];
		}
		inline float operator()(size_t id)const{
			return a[id];
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Seter
		inline float& x(){
			return a[0];
		}
		inline float& y(){
			return a[1];
		}
		inline float& operator[](size_t id){
			return a[id];
		}
		inline float& operator()(size_t id){
			return a[id];
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Copy Constructor
		Vec2(const Vec2& ref){
			a[0] = ref.x();
			a[1] = ref.y();
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//Constructor
		explicit Vec2(){
			a[0] = a[1] = 0.0f;
		}
		explicit Vec2(float x, float y){
			a[0] = x;
			a[1] = y;
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Destructor
		~Vec2(){}
	};

	inline const Vec2 operator*(float left, const Vec2& right){
		return Vec2(left*right.x(), left*right.y());
	}
	inline const Vec2 operator*(const Vec2& left, float right){
		return Vec2(right*left.x(), right*left.y());
	}
	inline const Vec2 operator*=(Vec2& left, float right){
		left.x() *= right;
		left.y() *= right;
		return left;
	}
	inline const Vec2 operator/=(Vec2& left, float right){
		left.x() /= right;
		left.y() /= right;
		return left;
	}
	inline const Vec2 operator/(const Vec2& left, float right){
		return Vec2(left.x() / right, left.y() / right);
	}
}