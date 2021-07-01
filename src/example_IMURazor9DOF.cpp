
#include "IMURazor9DOF.h"
#include "IPlot.h"
#include "math.h"


int main()
{


    IMURazor9DOF imu("/dev/ttyUSB0");


    double dts=0.01;
    double pitch, roll, yaw;
    IPlot plPitch(dts,"Pitch");
    IPlot plRoll(dts,"Roll");

    imu.GetPitchRollYaw(dts, pitch, roll, yaw );

    for (double t=0; t<100; t+=dts)
    {
        imu.GetPitchRollYaw(dts, pitch, roll, yaw );
        cout << "-> pitch: " << pitch*180/M_PI << ", roll: " << roll*180/M_PI << ", yaw: " << yaw*180/M_PI << endl;
//        cout << imu.GetLine() << endl;
        usleep(dts*1000*1000);

    }



//    plPitch.Plot();
//    plRoll.Plot();
    return 0;
}



