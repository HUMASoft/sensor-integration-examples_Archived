
#include "imu3dmgx510.h"
#include "IPlot.h"


int main()
{


    uint freq=100;
    IMU3DMGX510 imu("/dev/ttyUSB1",freq);

//    imu.set_IDLEmode();
//    imu.set_devicetogetgyroacc();
//    imu.set_streamon();
//    vector<double> a1(2);
    double* angles;
    double pitch,roll;

    double dts=1.0/freq;
    IPlot plPitch(dts);
    IPlot plRoll(dts);



    for (int i=0; i<6*freq; i++)
    {
        imu.GetPitchRoll(pitch,roll);
//        cout << pitch << ", " << roll << endl;

        plPitch.pushBack(pitch);
        plRoll.pushBack(roll);

    }

//    imu.set_streamon();
//    for (int i=0; i<6*freq; i++)
//    {
//            angles = imu.EulerAngles();
//            cout << angles[0] << ", " << angles[1] << endl;
//            cout << i << endl;
//            usleep(1000*1000*dts);
//        plPitch.pushBack(angles[0]);
//        plRoll.pushBack(angles[1]);


//    }

    plPitch.Plot();
    plRoll.Plot();
    return 0;
}



