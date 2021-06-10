
#include "SerialComm.h"

SerialComm p1("/dev/ttyUSB0",57600);
string response;

using namespace std;

int main()
{

    response = p1.GetChars(1000);
    cout  << response << endl ;
    return 0;
}
