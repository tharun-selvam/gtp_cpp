#include <iostream>
#include <Eigen/Dense>
#include "spline.h"

using namespace std;
using namespace Eigen;

int main(){
    int n_steps = 5;
    ArrayX3d Ai(n_steps, 3);
    // Ai = ArrayX3d::Zero();
    // ArrayX3d Bi(n_steps, 3) = ArrayX3d::Zero();

    cout << Ai << endl;
}