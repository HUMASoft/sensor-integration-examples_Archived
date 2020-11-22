//Used libs in the project

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <string.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/hex.hpp>
#include "attitude_estimator.h"


using namespace boost::asio;
using namespace boost::algorithm;
using namespace std::string_literals;
using namespace stateestimation;
using std::cin;
using std::cout;

// Port is defined depending on the OS used by the user
// These are the values our port needs to connect
#ifdef _WIN32
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORT = "COM7";
#else
//default port usb
const char *PORT = "/dev/ttyUSB0";
#endif

//This func is used to calculate a float value from a string
union ulf
{
    unsigned long ul;
    float f;
};



int main()
{

    boost::asio::io_context io; //Active I/0 Functions
    boost::asio::serial_port port(io); //Creation of the object
    port.open(PORT);//Port need to be open to read/write data

    if (!port.is_open()){
        cout << "Port can't be opened \n";
    }else{
        cout << "Port has been sucesfully opened \n";
    }

    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    boost::system::error_code error;
    boost::asio::streambuf buffer;

    //Definition of the estimator used to calculate Euler Angles (Roll&Pitch) from gyrox,gyroy,gyroz,accx,accy,accz
    AttitudeEstimator estimator;
    //Our device has no magnotemeter
    estimator.setMagCalib(0.0, 0.0, 0.0);
    //Setting of GyroBias
    double bx = -0.002786;
    double by = -0.001833;
    double bz = -0.001066;
    estimator.setGyroBias(bx,by,bz);

    //Defaults gains used
    estimator.setPIGains(2.2, 2.65, 10, 1.25);

    //Preguntamos al usuario que quiere hacer con el sensor correctamente conectado
    int end;
    int end1;
    int end2;
    int comp;
    int fin;

    double roll[1899];
    double pitch[1899];
    double yaw[1899];


    double absrollaverage=0.0;
    double abspitchaverage=0.0;
    double absyawaverage=0.0;

    //Declaration of the packets which will be sent to the device
    std::string idle = "\x75\x65\x01\x02\x02\x02\xe1\xc7";
    std::string imudata1 = "\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x03\xe8\xee\x04";
    std::string imudata100("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x0a\x0d\x20"s);
    std::string imudata1000 ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x01\x04\x17"s);
    std::string reset = "\x75\x65\x01\x02\x02\x7e\x5d\x43";
    std::string baudratenew ("\x75\x65\x0c\x07\x07\x40\x01\x00\x03\x84\x00\xbc\x64"s);
    std::string gyracc("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x03\xe8\x05\x03\xe8\xe4\x0b"s);
    std::string gyracc100("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x00\x0a\x05\x00\x0a\x22\xa0"s);
    std::string streamon = "\x75\x65\x0c\x05\x05\x11\x01\x01\x01\x04\x1a";
    std::string streamoff ("\x75\x65\x0c\x05\x05\x11\x01\x01\x00\x03\x19"s);


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
                    boost::asio::write(
                                port,
                                boost::asio::buffer(idle.c_str(), idle.size()),
                                boost::asio::transfer_at_least(idle.size()),
                                error
                                );

                    //Leemos la respuesta concreta que queremos leer
                    //Que confirme que la orden se ha ejecutado correctamente
                    //75 65 01 04 04 F1 02 00 D6 6C

                    char a;
                    std::string feedbackdevice;

                    do{
                        //Leemos el ue inicial del paquete de respuesta
                        do{
                            boost::asio::read(port, boost::asio::buffer(&a,1));
                            switch (a) {
                            case 'u':{

                                comp=1;
                                feedbackdevice+=a;
                                break;}
                            case 'e':{

                                if (comp==1){
                                    fin=1;
                                    feedbackdevice+=a;

                                }else{
                                    comp=0;
                                    feedbackdevice+=a;
                                }
                                break;}

                            default:{
                                feedbackdevice.clear();
                                break;}
                            }
                        }while (fin==0);

                        fin=0;
                        comp=0;

                        //Y posteriormente, leemos el desc y la longitud
                        char feedback_desc;
                        char feedback_long;
                        boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
                        feedbackdevice +=feedback_desc;
                        boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
                        feedbackdevice +=feedback_long;

                        if (int(feedback_desc)==1 && int(feedback_long)==4){
                            //Leo hasta que se acabe la longitud

                            for (int i=0; i<= ((int)feedback_long+1); i++){
                                boost::asio::read(port, boost::asio::buffer(&a,1));
                                feedbackdevice +=a;
                            }
                            //cout << hex(feedbackdevice) << '\n';
                            end2=1;
                        }else{
                            end2=0;
                            feedbackdevice.clear();
                        }

                    }while(end2==0);

                    //Si el feedback es el correcto, seguimos
                    std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);

                    if (hex(feedbackdevice) == hex(respuestacorrecta)){

                    }else{
                        cout << "An error has ocurred.";
                    }
    //After it, user could be able to select the functionality of the sensor

    do{
        do{

//            cout << "1 - Set to IDLE \n";
//            cout << "2 - Solicitar xx a f=1Hz  \n";
            cout << "1 - Reset device  \n";
//            cout << "4 - Solicitar xx a f=1000Hz  \n";
            cout << "2 - Solicitar xx a f=100Hz  \n";
//            cout << "6 - Cambiar el BaudRate a 230400bps \n";
            cout << "3 - Get Euler Angles 1Hz \n";
            cout << "4 - Get Euler Angles 100Hz \n";


            int numero;
            cin >> numero;

            switch(numero) {

//            case 1: {

//                boost::asio::write(
//                            port,
//                            boost::asio::buffer(idle.c_str(), idle.size()),
//                            boost::asio::transfer_at_least(idle.size()),
//                            error
//                            );

//                //Leemos la respuesta concreta que queremos leer
//                //Que confirme que la orden se ha ejecutado correctamente
//                //75 65 01 04 04 F1 02 00 D6 6C

//                char a;
//                std::string feedbackdevice;

//                do{
//                    //Leemos el ue inicial del paquete de respuesta
//                    do{
//                        boost::asio::read(port, boost::asio::buffer(&a,1));
//                        switch (a) {
//                        case 'u':{

//                            comp=1;
//                            feedbackdevice+=a;
//                            break;}
//                        case 'e':{

//                            if (comp==1){
//                                fin=1;
//                                feedbackdevice+=a;

//                            }else{
//                                comp=0;
//                                feedbackdevice+=a;
//                            }
//                            break;}

//                        default:{
//                            feedbackdevice.clear();
//                            break;}
//                        }
//                    }while (fin==0);

//                    fin=0;
//                    comp=0;

//                    //Y posteriormente, leemos el desc y la longitud
//                    char feedback_desc;
//                    char feedback_long;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
//                    feedbackdevice +=feedback_desc;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
//                    feedbackdevice +=feedback_long;

//                    if (int(feedback_desc)==1 && int(feedback_long)==4){
//                        //Leo hasta que se acabe la longitud

//                        for (int i=0; i<= ((int)feedback_long+1); i++){
//                            boost::asio::read(port, boost::asio::buffer(&a,1));
//                            feedbackdevice +=a;
//                        }
//                        cout << hex(feedbackdevice) << '\n';
//                        end2=1;
//                    }else{
//                        end2=0;
//                        feedbackdevice.clear();
//                    }

//                }while(end2==0);

//                //Si el feedback es el correcto, seguimos
//                std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);

//                if (hex(feedbackdevice) == hex(respuestacorrecta)){
//                    cout << "Nice done! \n";
//                }else{
//                    cout << "An error has ocurred.";
//                }

//                break;}

//            case 2: {
//                std::string imudata1 = "\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x03\xe8\xee\x04";
//                boost::asio::write(
//                            port,
//                            boost::asio::buffer(imudata1.c_str(), imudata1.size()),
//                            boost::asio::transfer_at_least(imudata1.size()),
//                            error
//                            );

//                //Leemos la respuesta concreta que queremos leer
//                //Que confirme que la orden se ha ejecutado correctamente
//                //75 65 0c 04 04 F1 08 00 e7 ba
//                char a;
//                std::string feedbackdevice;

//                do{
//                    //Leemos el ue inicial del paquete de respuesta
//                    do{
//                        boost::asio::read(port, boost::asio::buffer(&a,1));
//                        switch (a) {
//                        case 'u':{

//                            comp=1;
//                            feedbackdevice+=a;
//                            break;}
//                        case 'e':{

//                            if (comp==1){
//                                fin=1;
//                                feedbackdevice+=a;

//                            }else{
//                                comp=0;
//                                feedbackdevice+=a;
//                            }
//                            break;}

//                        default:{
//                            feedbackdevice.clear();
//                            break;}
//                        }
//                    }while (fin==0);

//                    fin=0;
//                    comp=0;

//                    //Y posteriormente, leemos el desc y la longitud
//                    char feedback_desc;
//                    char feedback_long;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
//                    feedbackdevice +=feedback_desc;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
//                    feedbackdevice +=feedback_long;

//                    if (int(feedback_desc)==12 && int(feedback_long)==4){
//                        //Leo hasta que se acabe la longitud

//                        for (int i=0; i<= ((int)feedback_long+1); i++){
//                            boost::asio::read(port, boost::asio::buffer(&a,1));
//                            feedbackdevice +=a;
//                        }
//                        cout << hex(feedbackdevice) << '\n';
//                        end2=1;
//                    }else{
//                        end2=0;
//                        feedbackdevice.clear();
//                    }

//                }while(end2==0);

//                //Si el feedback es el correcto, seguimos
//                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

//                if (hex(feedbackdevice) == hex(respuestacorrecta)){
//                    cout << "Nice done! \n";
//                }else{
//                    cout << "An error has ocurred.";
//                }


//                break;}
            case 2: {

                boost::asio::write(
                            port,
                            boost::asio::buffer(imudata100.c_str(), imudata100.size()),
                            boost::asio::transfer_at_least(imudata100.size()),
                            error
                            );

                //Leemos la respuesta concreta que queremos leer
                //Que confirme que la orden se ha ejecutado correctamente
                //75 65 0c 04 04 F1 08 00 e7 ba
                char a;
                std::string feedbackdevice;

                do{
                    //Leemos el ue inicial del paquete de respuesta
                    do{
                        boost::asio::read(port, boost::asio::buffer(&a,1));
                        switch (a) {
                        case 'u':{

                            comp=1;
                            feedbackdevice+=a;
                            break;}
                        case 'e':{

                            if (comp==1){
                                fin=1;
                                feedbackdevice+=a;

                            }else{
                                comp=0;
                                feedbackdevice+=a;
                            }
                            break;}

                        default:{
                            feedbackdevice.clear();
                            break;}
                        }
                    }while (fin==0);

                    fin=0;
                    comp=0;

                    //Y posteriormente, leemos el desc y la longitud
                    char feedback_desc;
                    char feedback_long;
                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
                    feedbackdevice +=feedback_desc;
                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
                    feedbackdevice +=feedback_long;

                    if (int(feedback_desc)==12 && int(feedback_long)==4){
                        //Leo hasta que se acabe la longitud

                        for (int i=0; i<= ((int)feedback_long+1); i++){
                            boost::asio::read(port, boost::asio::buffer(&a,1));
                            feedbackdevice +=a;
                        }
                        //cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    //cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }

                break;}
//            case 4: {

//                std::string imudata1000 ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x01\x04\x17"s);
//                boost::asio::write(
//                            port,
//                            boost::asio::buffer(imudata1000.c_str(), imudata1000.size()),
//                            boost::asio::transfer_at_least(imudata1000.size()),
//                            error
//                            );
//                //Leemos la respuesta concreta que queremos leer
//                //Que confirme que la orden se ha ejecutado correctamente
//                //75 65 0c 04 04 F1 08 00 e7 ba
//                char a;
//                std::string feedbackdevice;

//                do{
//                    //Leemos el ue inicial del paquete de respuesta
//                    do{
//                        boost::asio::read(port, boost::asio::buffer(&a,1));
//                        switch (a) {
//                        case 'u':{

//                            comp=1;
//                            feedbackdevice+=a;
//                            break;}
//                        case 'e':{

//                            if (comp==1){
//                                fin=1;
//                                feedbackdevice+=a;

//                            }else{
//                                comp=0;
//                                feedbackdevice+=a;
//                            }
//                            break;}

//                        default:{
//                            feedbackdevice.clear();
//                            break;}
//                        }
//                    }while (fin==0);

//                    fin=0;
//                    comp=0;

//                    //Y posteriormente, leemos el desc y la longitud
//                    char feedback_desc;
//                    char feedback_long;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
//                    feedbackdevice +=feedback_desc;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
//                    feedbackdevice +=feedback_long;

//                    if (int(feedback_desc)==12 && int(feedback_long)==4){
//                        //Leo hasta que se acabe la longitud

//                        for (int i=0; i<= ((int)feedback_long+1); i++){
//                            boost::asio::read(port, boost::asio::buffer(&a,1));
//                            feedbackdevice +=a;
//                        }
//                        cout << hex(feedbackdevice) << '\n';
//                        end2=1;
//                    }else{
//                        end2=0;
//                        feedbackdevice.clear();
//                    }

//                }while(end2==0);

//                //Si el feedback es el correcto, seguimos
//                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

//                if (hex(feedbackdevice) == hex(respuestacorrecta)){
//                    cout << "Nice done! \n";
//                }else{
//                    cout << "An error has ocurred.";
//                }

//                break;}

            case 1:{

                boost::asio::write(
                            port,
                            boost::asio::buffer(reset.c_str(), reset.size()),
                            boost::asio::transfer_at_least(reset.size()),
                            error
                            );
                //Leemos la respuesta concreta que queremos leer
                //Que confirme que la orden se ha ejecutado correctamente
                //75 65 0c 04 04 F1 08 00 e7 ba
                char a;
                std::string feedbackdevice;

                do{
                    //Leemos el ue inicial del paquete de respuesta
                    do{
                        boost::asio::read(port, boost::asio::buffer(&a,1));
                        switch (a) {
                        case 'u':{

                            comp=1;
                            feedbackdevice+=a;
                            break;}
                        case 'e':{

                            if (comp==1){
                                fin=1;
                                feedbackdevice+=a;

                            }else{
                                comp=0;
                                feedbackdevice+=a;
                            }
                            break;}

                        default:{
                            feedbackdevice.clear();
                            break;}
                        }
                    }while (fin==0);

                    fin=0;
                    comp=0;

                    //Y posteriormente, leemos el desc y la longitud
                    char feedback_desc;
                    char feedback_long;
                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
                    feedbackdevice +=feedback_desc;
                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
                    feedbackdevice +=feedback_long;

                    if (int(feedback_desc)==1 && int(feedback_long)==4){
                        //Leo hasta que se acabe la longitud

                        for (int i=0; i<= ((int)feedback_long+1); i++){
                            boost::asio::read(port, boost::asio::buffer(&a,1));
                            feedbackdevice +=a;
                        }
                        //cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x7e\x00\x52\x64"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    //cout << "The device has been reset! \n";
                }else{
                    cout << "An error has ocurred.";
                }


                break;}
//            case 6:{
//                //port.set_option(boost::asio::serial_port_base::baud_rate(921600));
//                std::string baudratenew ("\x75\x65\x0c\x07\x07\x40\x01\x00\x03\x84\x00\xbc\x64"s);
//                boost::asio::write(
//                            port,
//                            boost::asio::buffer(baudratenew.c_str(), baudratenew.size()),
//                            boost::asio::transfer_at_least(baudratenew.size()),
//                            error
//                            );

//                char a;
//                std::string feedbackdevice;
//                do{
//                    //Leemos el ue inicial del paquete de respuesta
//                    do{
//                        boost::asio::read(port, boost::asio::buffer(&a,1));
//                        switch (a) {
//                        case 'u':{

//                            comp=1;
//                            feedbackdevice+=a;
//                            break;}
//                        case 'e':{

//                            if (comp==1){
//                                fin=1;
//                                feedbackdevice+=a;

//                            }else{
//                                comp=0;
//                                feedbackdevice+=a;
//                            }
//                            break;}

//                        default:{
//                            feedbackdevice.clear();
//                            break;}
//                        }
//                    }while (fin==0);

//                    fin=0;
//                    comp=0;

//                    //Y posteriormente, leemos el desc y la longitud
//                    char feedback_desc;
//                    char feedback_long;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
//                    feedbackdevice +=feedback_desc;
//                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
//                    feedbackdevice +=feedback_long;

//                    if (int(feedback_desc)==12 && int(feedback_long)==4){
//                        //Leo hasta que se acabe la longitud

//                        for (int i=0; i<= ((int)feedback_long+1); i++){
//                            boost::asio::read(port, boost::asio::buffer(&a,1));
//                            feedbackdevice +=a;
//                        }
//                        cout << hex(feedbackdevice) << '\n';
//                        end2=1;
//                    }else{
//                        end2=0;
//                        feedbackdevice.clear();
//                    }

//                }while(end2==0);

//                //Si el feedback es el correcto, seguimos
//                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x40\x00\x1f\x2a"s);

//                if (hex(feedbackdevice) == hex(respuestacorrecta)){
//                    cout << "Baudrate has been chenged to 230400! \n";
//                    port.set_option(boost::asio::serial_port_base::baud_rate(230400));
//                }else{
//                    cout << "An error has ocurred.";
//                }



//                break;}

            case 3:{

                boost::asio::write(
                            port,
                            boost::asio::buffer(gyracc.c_str(), gyracc.size()),
                            boost::asio::transfer_at_least(gyracc.size()),
                            error
                            );

                //Leemos la respuesta concreta que queremos leer
                //Que confirme que la orden se ha ejecutado correctamente
                //75 65 0c 04 04 F1 08 00 e7 ba
                char a;
                std::string feedbackdevice;

                do{
                    //Leemos el ue inicial del paquete de respuesta
                    do{
                        boost::asio::read(port, boost::asio::buffer(&a,1));
                        switch (a) {
                        case 'u':{

                            comp=1;
                            feedbackdevice+=a;
                            break;}
                        case 'e':{

                            if (comp==1){
                                fin=1;
                                feedbackdevice+=a;

                            }else{
                                comp=0;
                                feedbackdevice+=a;
                            }
                            break;}

                        default:{
                            feedbackdevice.clear();
                            break;}
                        }
                    }while (fin==0);

                    fin=0;
                    comp=0;

                    //Y posteriormente, leemos el desc y la longitud
                    char feedback_desc;
                    char feedback_long;
                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
                    feedbackdevice +=feedback_desc;
                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
                    feedbackdevice +=feedback_long;

                    if (int(feedback_desc)==12 && int(feedback_long)==4){
                        //Leo hasta que se acabe la longitud

                        for (int i=0; i<= ((int)feedback_long+1); i++){
                            boost::asio::read(port, boost::asio::buffer(&a,1));
                            feedbackdevice +=a;
                        }
                        //cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    //cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }


                break;}

            case 4:{

                boost::asio::write(
                            port,
                            boost::asio::buffer(gyracc100.c_str(), gyracc100.size()),
                            boost::asio::transfer_at_least(gyracc100.size()),
                            error
                            );

                //Leemos la respuesta concreta que queremos leer
                //Que confirme que la orden se ha ejecutado correctamente
                //75 65 0c 04 04 F1 08 00 e7 ba
                char a;
                std::string feedbackdevice;

                do{
                    //Leemos el ue inicial del paquete de respuesta
                    do{
                        boost::asio::read(port, boost::asio::buffer(&a,1));
                        switch (a) {
                        case 'u':{

                            comp=1;
                            feedbackdevice+=a;
                            break;}
                        case 'e':{

                            if (comp==1){
                                fin=1;
                                feedbackdevice+=a;

                            }else{
                                comp=0;
                                feedbackdevice+=a;
                            }
                            break;}

                        default:{
                            feedbackdevice.clear();
                            break;}
                        }
                    }while (fin==0);

                    fin=0;
                    comp=0;

                    //Y posteriormente, leemos el desc y la longitud
                    char feedback_desc;
                    char feedback_long;
                    boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
                    feedbackdevice +=feedback_desc;
                    boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
                    feedbackdevice +=feedback_long;

                    if (int(feedback_desc)==12 && int(feedback_long)==4){
                        //Leo hasta que se acabe la longitud

                        for (int i=0; i<= ((int)feedback_long+1); i++){
                            boost::asio::read(port, boost::asio::buffer(&a,1));
                            feedbackdevice +=a;
                        }
                        //cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    //cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }


                break;}

            default: {
                cout << "Not defined";
                break;}

            }

            cout << "Have you finished configurating the device? Insert ----> y/n      \n";
            char answer1;
            cin >> answer1;
            if (answer1=='y'){
                end=1;

            }else{
                end =0;
            }


        }while(end==0);

        cout << "Start stream? Insert ----> y/n     \n";



        char answer;
        cin >> answer;
        if (answer=='y'){


            boost::asio::write(
                        port,
                        boost::asio::buffer(streamon.c_str(), streamon.size()),
                        boost::asio::transfer_at_least(streamon.size()),
                        error
                        );
        }

        //We will read 2000 samples aprox, it can ba changed anytime
        for (int h=0; h<=1999;h++){


            int comp=0;
            int fin=0;
            char c;
            std::string result;

            //Data is read till we find the start of a new data packet, 'ue'
            do{

                boost::asio::read(port, boost::asio::buffer(&c,1));
                switch (c) {
                case 'u':{

                    comp=1;
                    result+=c;
                    break;}
                case 'e':{

                    if (comp==1){
                        fin=1;
                        result+=c;

                    }else{
                        comp=0;
                        result+=c;
                    }
                    break;}

                default:{
                    result+=c;
                    if (comp==1){
                        //fin++;
                    }
                    break;}

                }
            }while(fin==0);


            //Descriptor
            char descriptor;
            boost::asio::read(port, boost::asio::buffer(&descriptor,1));
            result+=descriptor;

            //Length
            char longitud;
            boost::asio::read(port, boost::asio::buffer(&longitud,1));
            result+=longitud;

            //Once we have descriptor and lenght of our packet, we know the exact moment to stop reading it
            //Ojo a LSB,MSB ---> long+1

            cout << "Packet length is:  "<< (int)longitud << '\n';

            for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
                boost::asio::read(port, boost::asio::buffer(&c,1));
                result+=c;
            }

            //Mostramos el paquete en hexadecimal
            cout << "Data packet is: " << hex(result) << '\n';

            //Extraigo los tres vectores de gyro
            //X

            if (int(longitud) == 14){
                cout << hex(result.substr(6,4)) << '\n';

                ulf x;
                std::string str =hex(result.substr(6,4));
                std::stringstream ss(str);
                ss >> std::hex >> x.ul;
                float f = x.f;
                cout << "Gyro x:  " << f << "rad/s" << '\n';

                //y
                cout << hex(result.substr(10,4)) << "rad/s" << '\n';

                ulf y;
                std::string str1 =hex(result.substr(10,4));
                std::stringstream ss1(str1);
                ss1 >> std::hex >> y.ul;
                float f1 = y.f;
                cout << "Gyro y:  " << f1 << "rad/s" << '\n';

                //z
                cout << hex(result.substr(14,4)) << '\n';

                ulf z;
                std::string str2 =hex(result.substr(14,4));
                std::stringstream ss2(str2);
                ss2 >> std::hex >> z.ul;
                float f2 = z.f;
                cout << "Gyro z:  " << f2 << "rad/s" << '\n';
            } else if(int(longitud) == 28){
                ulf accx;
                std::string str =hex(result.substr(6,4));
                std::stringstream ss(str);
                ss >> std::hex >> accx.ul;
                double f = accx.f;
                cout << "Acc x:  " << f << "rad/s2" << '\n';

                //y
                //cout << hex(result.substr(10,4)) << "rad/s" << '\n';

                ulf accy;
                std::string str1 =hex(result.substr(10,4));
                std::stringstream ss1(str1);
                ss1 >> std::hex >> accy.ul;
                double f1 = accy.f;
                cout << "Acc y:  " << f1 << "rad/s2" << '\n';

                //z
                //cout << hex(result.substr(14,4)) << '\n';

                ulf accz;
                std::string str2 =hex(result.substr(14,4));
                std::stringstream ss2(str2);
                ss2 >> std::hex >> accz.ul;
                double f2 = accz.f;
                cout << "Acc z:  " << f2 << "rad/s2" << '\n';

                ulf gyrox;
                std::string str3 =hex(result.substr(20,4));
                std::stringstream ss3(str3);
                ss3 >> std::hex >> gyrox.ul;
                double f3 = gyrox.f;
                cout << "Gyro x:  " << f3 << "rad/s" << '\n';

                ulf gyroy;
                std::string str4 =hex(result.substr(24,4));
                std::stringstream ss4(str4);
                ss4 >> std::hex >> gyroy.ul;
                double f4 = gyroy.f;
                cout << "Gyro x:  " << f4 << "rad/s" << '\n';

                ulf gyroz;
                std::string str5 =hex(result.substr(28,4));
                std::stringstream ss5(str5);
                ss5>> std::hex >> gyroz.ul;
                double f5 = gyroz.f;
                cout << "Gyro x:  " << f5 << "rad/s" << '\n';

                //We start updating our euler angles vector once data is stable. For example, we start 1s after device started reading
                if (h>=100 && h<=1999){

                    estimator.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
                    roll[h-100]=estimator.eulerRoll();
                    pitch[h-100]=estimator.eulerPitch();
                    yaw[h-100]=estimator.eulerYaw();
                    cout << "My attitude is (ZYX Euler): (" << estimator.eulerYaw() << "," << estimator.eulerPitch() << "," << estimator.eulerRoll() << ")" << '\n';

                    //This way we can correct initial offset

                    if(h>=225 && h<=350){

                        absrollaverage= absrollaverage + estimator.eulerRoll();
                        abspitchaverage= abspitchaverage + estimator.eulerPitch();
                        absyawaverage=absyawaverage + estimator.eulerYaw();

                        if (h==350){

                            absrollaverage = absrollaverage/125;
                            abspitchaverage = abspitchaverage/125;
                            absyawaverage = absyawaverage/125;
                        }


                    }

                  }

            }

            //Data packet reset to prepare a new reading
            result.clear();

        }
        //End of stream
        boost::asio::write(
                    port,
                    boost::asio::buffer(streamoff.c_str(), streamoff.size()),
                    boost::asio::transfer_at_least(streamoff.size()),
                    error
                    );

        boost::asio::write(
                    port,
                    boost::asio::buffer(idle.c_str(), idle.size()),
                    boost::asio::transfer_at_least(idle.size()),
                    error
                    );

        //Leemos la respuesta concreta que queremos leer
        //Que confirme que la orden se ha ejecutado correctamente
        //75 65 01 04 04 F1 02 00 D6 6C

        char a;
        std::string feedbackdevice;

        do{
            //Leemos el ue inicial del paquete de respuesta
            do{
                boost::asio::read(port, boost::asio::buffer(&a,1));
                switch (a) {
                case 'u':{

                    comp=1;
                    feedbackdevice+=a;
                    break;}
                case 'e':{

                    if (comp==1){
                        fin=1;
                        feedbackdevice+=a;

                    }else{
                        comp=0;
                        feedbackdevice+=a;
                    }
                    break;}

                default:{
                    feedbackdevice.clear();
                    break;}
                }
            }while (fin==0);

            fin=0;
            comp=0;

            //Y posteriormente, leemos el desc y la longitud
            char feedback_desc;
            char feedback_long;
            boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
            feedbackdevice +=feedback_desc;
            boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
            feedbackdevice +=feedback_long;

            if (int(feedback_desc)==1 && int(feedback_long)==4){
                //Leo hasta que se acabe la longitud

                for (int i=0; i<= ((int)feedback_long+1); i++){
                    boost::asio::read(port, boost::asio::buffer(&a,1));
                    feedbackdevice +=a;
                }
                //cout << hex(feedbackdevice) << '\n';
                end2=1;
            }else{
                end2=0;
                feedbackdevice.clear();
            }

        }while(end2==0);

        //Si el feedback es el correcto, seguimos
        std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);

        if (hex(feedbackdevice) == hex(respuestacorrecta)){

        }else{
            cout << "An error has ocurred.";
        }

        cout << "New scan? ---> y/n" << '\n';
        char answer2;
        cin >> answer2;
        if (answer2 == 'n'){
            end1=1;
        }else{
            end1=0;
        }
    }while (end1==0);


    cout << "Initial offset in pitch is: "<< abspitchaverage << '\n';
    cout << "Initial offset in roll is: "<< absrollaverage << '\n';


    for (int z=0; z<=1899;z++){

        if (z==0){
        cout << "pitch = [" << pitch[z]-abspitchaverage << " ";
        }
        if (z==1899){
        cout << pitch[z]-abspitchaverage << ']'<< '\n';
        }else{
        cout << pitch[z]-abspitchaverage << " ";
        }
    }

    for (int p=0; p<=1899;p++){

//        if (p>200 && abs(roll[p]-roll[p-1])>5){
//            roll[p]=roll[p]+6;
//        }


//        if (roll[p]<=-3.2){
//            roll[p]=roll[p]+6;
//            }

        if (p==0){
        cout << "roll = [" << roll[p]-absrollaverage<< " ";
        }
        if (p==1899){
        cout << roll[p]-absrollaverage << ']'<< '\n';
        }else{
        cout << roll[p]-absrollaverage<< " ";
        }
    }

    return 0;
}
//||

//La cadena en bits que recibo es 111010101100101100000000000111000001110000001011011101101011001110001100100110110111011000111011000100000101001101110101100011001101011001001110011110110100101
//Alrededor de 160bits
//Si quiero irme a 1000Hz, necesitaria 160000, es decir, 115200 no nos vale. Habría que subir el baudrate a 230400(el inmediato superior)
//Para este baudrate, el paquete es 7565 0c07 0740 0100 0384 00bc 64


