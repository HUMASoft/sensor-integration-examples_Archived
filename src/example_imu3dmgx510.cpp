
#include "imu3dmgx510.h"
#include "IPlot.h"


int main()
{


//    SerialComm port;
//    string ret;
//    string expect("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C");

//    for(int i=0;i<2;i++)
//    {
//        port.WriteLine("\x75\x65\x01\x02\x02\x02\xe1\xc7");
//        ret= port.ReadUntil('l');
//        cout << ret <<endl;
//        cout << "\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"<<endl;
//    }

//    for(uint i=0;i<ret.size();i++)
//    {
//        cout << int(ret[i]) << ",";
//    }
//    cout <<endl;
//    for(uint i=0;i<expect.size();i++)
//    {
//        cout << int(expect[i]) << ",";
//    }
//    cout <<endl;
//    ret.resize(expect.size());
//    cout << ret.compare(expect) << endl;

    uint freq=100;
    IMU3DMGX510 imu("/dev/ttyUSB0",freq);

//    imu.set_IDLEmode();
//    imu.set_devicetogetgyroacc();
//    imu.set_streamon();
//    vector<double> a1(2);
    double* a2;
    double pitch,roll;

    double dts=1.0/freq;
    IPlot plPitch(dts);
    IPlot plRoll(dts);


    for (int i=0; i<600*freq; i++)
    {
        imu.GetPitchRoll(pitch,roll);
//        cout << pitch << ", " << roll << endl;

        plPitch.pushBack(pitch);
        plRoll.pushBack(roll);

//        a = imu.EulerAngles();
//        a2 = imu.get_euleranglesPolling();
//        cout << a2[0] << ", " << a2[1] << endl;
//        cout << i << endl;
//        usleep(1000*1000*dts);
    }

    plPitch.Plot();
    plRoll.Plot();
    return 0;
}



