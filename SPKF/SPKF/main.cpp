#include <iostream>
#include "motionModel.hpp"
#include "positionMeasureModel.hpp"
#include "orientationMeasureModel.hpp"
#include "spkf.hpp"

using namespace SPKF_namespace;

int main()
{
	State<SPKF_namespace::T> x;
	x.set_mat(0, 0, 0);
	std::cout << x.get_mat() << std::endl;

	MotionModel m;
	m.init();
	
	Control<SPKF_namespace::T> u;
	
	PositionMeasureModel<SPKF_namespace::T, Eigen::Matrix<T, 2, 1>> p(0, 0, 10, 10);
	p.init();
	std::cout << p.h(x) << std::endl;
	OrientationMeasureModel<SPKF_namespace::T, Eigen::Matrix<T, 1, 1>> o;
	o.init();
	std::cout << o.h(x) << std::endl;
	
	SPKF<SPKF_namespace::T> spkf;
	spkf.init();

    for(int i = 0; i < 100; i++)
    {
		// control simulation
		u.set_mat(1. + std::sin(2*i), std::cos(2*i));
		
		x = m.f(x, u);
		spkf.computeSigmaPoints(x);
		
		auto x_new = spkf.predict(m, u);

		std::cout << "predict " << i << std::endl;
		std::cout << x_new.get_mat() << std::endl;
		
		spkf.computeSigmaPoints(x_new);

		// orientation measurement
		auto om = o.h(x_new);
		x_new = spkf.update(o, om);
		
		spkf.computeSigmaPoints(x_new);
		
		// position measurement
		auto pm = p.h(x_new);
		x_new = spkf.update(p, pm);
		
		std::cout << "update " << i << std::endl;
		std::cout << x_new.get_mat() << std::endl;
	}
}