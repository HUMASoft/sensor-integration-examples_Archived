
#include "imu3dmgx510.h"



int main()
{


    SerialComm port;
    string ret;

//    for(int i=0;i<2;i++)
//    {
//        port.WriteLine("\x75\x65\x01\x02\x02\x02\xe1\xc7");
//        cout << port.ReadUntil("l") <<endl;
//        cout << "\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"<<endl;
//    }


    IMU3DMGX510 imu("/dev/ttyUSB0",100);

    imu.set_IDLEmode();
    imu.set_devicetogetgyroacc();
    imu.set_streamon();
    vector<double> a1(2);
    double* a2 = imu.EulerAngles();
    for (int i=0; i<10; i++)
    {
        cout << a2[0] << ", " << a2[1] << endl;
    }

    return 0;
}



