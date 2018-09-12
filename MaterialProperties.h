
#ifndef MATERIAL_PROPERTIES_H
#define MATERIAL_PROPERTIES_H

struct Material
{
	float viscosity = 100;
	float youngs_modulus = 1000;
	float poisson_ratio = 0.25;
	float density = 1;
	float dt = 0.1;
};

class Wilson
{
public:
	Wilson(float dt)
	{
		time_step = dt;
		theta = 1.6;
		tau = theta*time_step;
		a0 = 6.0 / (tau*tau);
		a1 = 6.0 / tau;
		a2 = 2;
		a3 = a0 / theta;
		a4 = -a1 / theta;
		a5 = 1 - 3.0 / theta;
		a6 = time_step / 2.0;
		a7 = time_step*time_step / 6.0;
	}
	float time_step;
	float theta;
	float tau;

	float a0;
	float a1;
	float a2;

	float a3;
	float a4;

	float a5;
	float a6;
	float a7;

};

#endif
