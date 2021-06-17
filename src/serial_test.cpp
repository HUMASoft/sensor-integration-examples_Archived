
#include "SerialComm.h"

SerialComm p1("/dev/ttyUSB0");
string response;
int main()
{
    p1.SetBaudRate(57600);
    for (int i=0;i<30;i++)
    {
        response = p1.GetLine();
        cout << i <<response << endl;
    }

    return 0;
}
