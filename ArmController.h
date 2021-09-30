#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <queue>

#include "Kinematics.h"

using namespace std;

class ArmController
{
private:
	Kinematics Kin;
	
public:
	
	ArmController(array<double, 6> init_position);
	~ArmController(void);
};