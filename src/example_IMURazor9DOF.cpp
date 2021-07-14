
#include "IMURazor9DOF.h"
#include "IPlot.h"
#include "math.h"


int main()
{


    IMURazor9DOF imu("/dev/ttyUSB1");


    double dts=0.01;
    double pitch, roll, yaw;
    IPlot plPitch(dts,"Pitch");
    IPlot plRoll(dts,"Roll");

    imu.GetYawPitchRoll(dts, yaw ,pitch, roll );

    for (double t=0; t<100; t+=dts)
    {
        imu.GetYawPitchRoll(dts, yaw ,pitch, roll );
        cout << "-> pitch: " << pitch << ", roll: " << roll << ", yaw: " << yaw << endl;
//        cout << imu.GetLine() << endl;
        usleep(dts*1000*1000);

    }



//    plPitch.Plot();
//    plRoll.Plot();
    return 0;
}



