#pragma once
#include<random>
#include<time.h>
#include<algorithm>

class RandomMT{

private:
	RandomMT& operator=(const RandomMT&);
	RandomMT(const RandomMT&);
	std::uniform_real_distribution<double> m_distribution;
	std::mt19937 m_MT;

public:
	//乱数シードと生成値の上限・下限を指定
	explicit RandomMT(
		int Seed, float min, float max) : m_MT(Seed), m_distribution(min, max){
	}

	//乱数シードを指定 生成値の範囲は0から1
	explicit RandomMT(int Seed) : m_MT(Seed), m_distribution(0.0, 1.0){
	}

	//ものぐさなあなたにおすすめ
	explicit RandomMT() : m_MT(11101 + 1973 * time(nullptr)), m_distribution(0.0, 1.0){
	}

	// [0,1]
	double genrand64_real1(){
		return m_distribution(m_MT);
	}

	// [0,1)
	double genrand64_real2(){
		return std::min(m_distribution(m_MT), 1.0 - DBL_MIN);
	}

	// (0,1)
	double genrand64_real3(){
		return std::max(DBL_MIN, std::min(m_distribution(m_MT), 1.0 - DBL_MIN));
	}

};