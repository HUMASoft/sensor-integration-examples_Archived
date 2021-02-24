
#include "imu3dmgx510.h"



int main()
{

    IMU3DMGX510 imu("/dev/ttyUSB0",100);

    imu.set_IDLEmode();
    imu.set_devicetogetgyroacc();
    imu.set_streamon();
    vector<double> a1(2);
    double* a2 = imu.EulerAngles();

    cout << a2[0] << ", " << a2[1] ;

    return 0;
}



