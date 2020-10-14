#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <string>


#include "SerialComm.h"

using namespace boost::asio;
using std::cin;

// These are the values our port needs to connect
#ifdef _WIN64
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORT = "COM4";
#endif

std::tuple <std::string,int> ReadFromDataString (int valorinicial, int valor, std::string cadena) {

    valorinicial = valor;
    int stop=0;
    do{
        if (cadena[valor]==','){
            stop=1;
            valor++;
            int diferencia = valor - valorinicial;
            std::string output = cadena.substr(valorinicial,diferencia-1);
            return std::make_tuple(output,valor);
        }else{
            valor++;
        }
    }while(stop==0);

}


int main()
{
//    boost::asio::io_context io; //Active I/0 Functions
//    boost::asio::serial_port port(io); //Creation of the object
//    port.open("COM4");
//    port.set_option(boost::asio::serial_port_base::baud_rate(9600));
//    boost::system::error_code error;
//    boost::asio::streambuf buffer;


    SerialComm p1;


    for (int y=0;y<=5;y++){

//        boost::asio::read_until( port, buffer, "\n", error );
//        std::istream str(&buffer); //Transform our info buffer into a string
//        std::string s;
//        std::getline(str, s); //Copy of the info from our buffer to our new string

//        sleep(2);
        std::string str1="hola";

        p1.ReadLine(str1);

        cout << str1 << endl;
/*
        //Hay que analizar los datos que se leen para extraer los que necesito

        std::string DeviceName = str1.substr(0,14);
        std::cout << "Device Name: " << DeviceName << '\n';
        int x=14, xinicial=14;

        std::string Acceleration_X;
        std::tie(Acceleration_X,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Acceleration_X: " << Acceleration_X<<"\n";

        std::string Acceleration_Y;
        std::tie(Acceleration_Y,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Acceleration_Y: " << Acceleration_Y << "\n";

        std::string Acceleration_Z;
        std::tie(Acceleration_Z,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Acceleration_Z: " << Acceleration_Z << "\n";

        std::string AngularVelocity_X;
        std::tie(AngularVelocity_X,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "AngularVelocity_X: " << AngularVelocity_X << "\n";

        std::string AngularVelocity_Y;
        std::tie(AngularVelocity_Y,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "AngularVelocity_Y: " << AngularVelocity_Y << "\n";

        std::string AngularVelocity_Z;
        std::tie(AngularVelocity_Z,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "AngularVelocity_Z: " << AngularVelocity_Z << "\n";

        std::string Angle_X;
        std::tie(Angle_X,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Angle_X: " << Angle_X << "\n";

        std::string Angle_Y;
        std::tie(Angle_Y,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Angle_Y: " << Angle_Y << "\n";

        std::string Angle_Z;
        std::tie(Angle_Z,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Angle_Z: " << Angle_Z << "\n";

        std::string MagneticField_X;
        std::tie(MagneticField_X,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "MagneticField_X " << MagneticField_X << "\n";

        std::string MagneticField_Y;
        std::tie(MagneticField_Y,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "MagneticField_Y: " << MagneticField_Y << "\n";

        std::string MagneticField_Z;
        std::tie(MagneticField_Z,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "MagneticField_Z: " << MagneticField_Z << "\n";

        std::string ModuleTemperature;
        std::tie(ModuleTemperature,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "TemperatureModule: " << ModuleTemperature << "\n";

        std::string ModuleDepth;
        std::tie(ModuleDepth,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "DepthModule:  " << ModuleDepth << "\n";

        std::string AirPressure;
        std::tie(AirPressure,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "AirePressure:  " << AirPressure << "\n";

        std::string ModuleHeight;
        std::tie(ModuleHeight,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "HeightModule:  " << ModuleHeight << "\n";

        std::string q0;
        std::tie(q0,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Q0:  " << q0 << "\n";

        std::string q1;
        std::tie(q1,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Q1:  " << q1 << "\n";

        std::string q2;
        std::tie(q2,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Q2:  " << q2 << "\n";

        std::string q3;
        std::tie(q3,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Q3:  " << q3 << "\n";

        std::string PortD1;
        std::tie(PortD1,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Port1:  " << PortD1 << "\n";

        std::string PortD2;
        std::tie(PortD2,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Port2:  " << PortD2 << "\n";

        std::string PortD3;
        std::tie(PortD3,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Port3:  " << PortD3 << "\n";

        std::string PortD4;
        std::tie(PortD4,x)=ReadFromDataString(xinicial,x,str1);
        std::cout << "Port4:  " << PortD4 << "\n";
*/
    }

    return 0;
}



