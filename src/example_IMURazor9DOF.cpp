
#include "IMURazor9DOF.h"
#include "IPlot.h"


int main()
{


    IMURazor9DOF imu("/dev/ttyUSB0");


    double dts=0.01;
    double pitch, roll, yaw;
    IPlot plPitch(dts,"Pitch");
    IPlot plRoll(dts,"Roll");

    imu.GetPitchRollYaw(dts, pitch, roll, yaw );

    for (double t=0; t<10; t+=dts)
    {
        imu.GetPitchRollYaw(dts, pitch, roll, yaw );
        cout << "-> pitch: " << pitch << ", roll: " << roll<< ", yaw: " << yaw<< endl;


    }



//    plPitch.Plot();
//    plRoll.Plot();
    return 0;
}



