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
#include <SerialComm.h>
#include <tuple>

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




//Object declaration (boost library)
boost::asio::io_context io; //Active I/0 Functions
boost::asio::serial_port port(io); //Creation of the object

//Definition of the estimator used to calculate Euler Angles (Roll&Pitch) from gyrox,gyroy,gyroz,accx,accy,accz
AttitudeEstimator estimator;





//This func is used to calculate a float value from a string
union ulf
{
    unsigned long ul;
    float f;
};

//This func will be used to send a data packet to the device, and to read the answer it gives back
void WriteDataPacket (boost::system::error_code error, std::string datapacket, std::string datapacketanswer){



    //We want to read a specific answer depending on the sent data packet

    char a;
    std::string feedbackdevice;
    int comp=0;
    int end2=0;
    int fin=0;

    do{

        //Send the data packet
        boost::asio::write(
                    port,
                    boost::asio::buffer(datapacket.c_str(), datapacket.size()),
                    boost::asio::transfer_at_least(datapacket.size()),
                    error
                    );
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

        //Once we read 'ue', we can read the following bites (descriptor and lenght of the packet)
        char feedback_desc;
        char feedback_long;
        boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
        feedbackdevice +=feedback_desc;
        boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
        feedbackdevice +=feedback_long;

        //We'll read till the end of the packet now we know its lenght

        for (int i=0; i<= ((int)feedback_long+1); i++){
            boost::asio::read(port, boost::asio::buffer(&a,1));
            feedbackdevice +=a;
        }

        //Checking if the read answer is the correct one
        //Testing
//        cout << "We read: " << hex(feedbackdevice) << '\n';
//        cout << "We have: " << hex(datapacketanswer) << '\n';


        if (feedbackdevice==datapacketanswer){
            end2=1;
            //cout << "No errors. \n";


        }else{
            //cout << "Error, go again";
            feedbackdevice.clear();
        }


    }while(end2==0);

    feedbackdevice.clear();

}

//This func will be used to read Gyro Data from the device
//Func is reading xx samples. The number can be changed anytime
int muestras=1200;
void ReadDataPacketGyro (std::string answer, int samples){

    char c;

    for (int h=0; h<=samples;h++){

        int comp=0;
        int fin=0;

        //Data is read till we find the start of a new data packet, 'ue'
        do{
            boost::asio::read(port, boost::asio::buffer(&c,1));
            switch (c) {
            case 'u':{
                comp=1;
                answer+=c;
                break;}
            case 'e':{
                if (comp==1){
                    fin=1;
                    answer+=c;
                }else{
                    comp=0;
                    answer+=c;
                }
                break;}

            default:{
                answer+=c;
                if (comp==1){
                }
                break;}

            }
        }while(fin==0);


        //Descriptor
        char descriptor;
        boost::asio::read(port, boost::asio::buffer(&descriptor,1));
        answer+=descriptor;

        //Length
        char longitud;
        boost::asio::read(port, boost::asio::buffer(&longitud,1));
        answer+=longitud;

        //Once we have descriptor and lenght of our packet, we know the exact moment to stop reading it

        //cout << "Packet length is:  "<< (int)longitud << '\n';

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            boost::asio::read(port, boost::asio::buffer(&c,1));
            answer+=c;
        }

        //Our read data packet is the next
        cout << "Data packet is: " << hex(answer) << '\n';
        //His length is:
        //std::string readinglength = answer.substr(3,1);

        //I get gyro values from the packet

        if (int(longitud) == 14){
            //X
            cout << hex(answer.substr(6,4)) << '\n';

            ulf x;
            std::string str =hex(answer.substr(6,4));
            std::stringstream ss(str);
            ss >> std::hex >> x.ul;
            float f = x.f;
            cout << "Gyro x:  " << f << "rad/s" << '\n';

            //y
            cout << hex(answer.substr(10,4)) << "rad/s" << '\n';

            ulf y;
            std::string str1 =hex(answer.substr(10,4));
            std::stringstream ss1(str1);
            ss1 >> std::hex >> y.ul;
            float f1 = y.f;
            cout << "Gyro y:  " << f1 << "rad/s" << '\n';

            //z
            cout << hex(answer.substr(14,4)) << '\n';

            ulf z;
            std::string str2 =hex(answer.substr(14,4));
            std::stringstream ss2(str2);
            ss2 >> std::hex >> z.ul;
            float f2 = z.f;
            cout << "Gyro z:  " << f2 << "rad/s" << '\n';
            }

        //Data packet reset to prepare a new reading
        answer.clear();

        }

}

//This func will be used to read Euler Angles from the device
//Func is reading xx samples. The number can be changed anytime
std::tuple <double*,double*, double, double> ReadDataPacketEuler (std::string answer, int samples){

    char c;
    static double rollvector[10000];
    static double pitchvector[10000];
    //double yaw[1899];
    double rollaverage=0.0;
    double pitchaverage=0.0;
    //double yawaverage=0.0;

    for (int h=0; h<=samples;h++){

         answer.clear();
         int comp=0;
         int fin=0;

        //Data is read till we find the start of a new data packet, 'ue'
        do{
            boost::asio::read(port, boost::asio::buffer(&c,1));
            switch (c) {
            case 'u':{
                comp=1;
                answer+=c;
                break;}
            case 'e':{
                if (comp==1){
                    fin=1;
                    answer+=c;
                }else{
                    comp=0;
                    answer+=c;
                }
                break;}

            default:{
                answer+=c;
                break;}

            }
        }while(fin==0);

        //Descriptor
        char descriptor;
        boost::asio::read(port, boost::asio::buffer(&descriptor,1));
        answer+=descriptor;

        //Length
        char longitud;
        boost::asio::read(port, boost::asio::buffer(&longitud,1));
        answer+=longitud;

        //Once we have descriptor and lenght of our packet, we know the exact moment to stop reading it

        //cout << "Packet length is:  "<< (int)longitud << '\n';

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            boost::asio::read(port, boost::asio::buffer(&c,1));
            answer+=c;
        }

        //Mostramos el paquete en hexadecimal
        cout << "Data packet is: " << hex(answer) << '\n';

        //}

        //if(int(longitud) == 28){

            ulf accx;
            std::string str =hex(answer.substr(6,4));
            std::stringstream ss(str);
            ss >> std::hex >> accx.ul;
            double f = accx.f;
            cout << "Acc x:  " << f << "rad/s2" << '\n';

            //y
            //cout << hex(result.substr(10,4)) << "rad/s" << '\n';

            ulf accy;
            std::string str1 =hex(answer.substr(10,4));
            std::stringstream ss1(str1);
            ss1 >> std::hex >> accy.ul;
            double f1 = accy.f;
            cout << "Acc y:  " << f1 << "rad/s2" << '\n';

            //z
            //cout << hex(result.substr(14,4)) << '\n';

            ulf accz;
            std::string str2 =hex(answer.substr(14,4));
            std::stringstream ss2(str2);
            ss2 >> std::hex >> accz.ul;
            double f2 = accz.f;
            cout << "Acc z:  " << f2 << "rad/s2" << '\n';

            ulf gyrox;
            std::string str3 =hex(answer.substr(20,4));
            std::stringstream ss3(str3);
            ss3 >> std::hex >> gyrox.ul;
            double f3 = gyrox.f;
            cout << "Gyro x:  " << f3 << "rad/s" << '\n';

            ulf gyroy;
            std::string str4 =hex(answer.substr(24,4));
            std::stringstream ss4(str4);
            ss4 >> std::hex >> gyroy.ul;
            double f4 = gyroy.f;
            cout << "Gyro x:  " << f4 << "rad/s" << '\n';

            ulf gyroz;
            std::string str5 =hex(answer.substr(28,4));
            std::stringstream ss5(str5);
            ss5>> std::hex >> gyroz.ul;
            double f5 = gyroz.f;
            cout << "Gyro x:  " << f5 << "rad/s" << '\n';

            //We start updating our euler angles vector once data is stable. For example, we start 1s after device started reading
            if (h>=100 && h<=samples){

                estimator.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
                rollvector[h-100]=estimator.eulerRoll();
                pitchvector[h-100]=estimator.eulerPitch();
                //yawvector[h-100]=estimator.eulerYaw();
                cout << "My attitude is (YX Euler): (" << estimator.eulerPitch() << "," << estimator.eulerRoll() << ")" << '\n';

                //This way we can correct initial offset

                if(h>=225 && h<=350){

                    rollaverage= rollaverage + estimator.eulerRoll();
                    pitchaverage= pitchaverage + estimator.eulerPitch();
                    //absyawaverage=absyawaverage + estimator.eulerYaw();

                    if (h==350){

                        rollaverage = rollaverage/125;
                        pitchaverage = pitchaverage/125;
                        //absyawaverage = absyawaverage/125;
                    }
                }
            }
        //}
    }

    return std::make_tuple(rollvector,pitchvector, rollaverage, pitchaverage);
}

int main()
{


    //    boost::asio::io_context io; //Active I/0 Functions
    //    boost::asio::serial_port port(io); //Creation of the object
    port.open(PORT);//Port needs to be opened to read/write data

    if (!port.is_open()){
        cout << "Port can't be opened \n";
    }else{
        cout << "Port has been sucesfully opened \n";
    }

    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    boost::system::error_code error;
    //boost::asio::streambuf buffer;


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


    int end;
    //int muestras=1200;
    //    int end2;
    //    int comp;
    //    int fin;


    double *roll;
    double *pitch;

    //    double roll[1899];
    //    double pitch[1899];
    //    double yaw[1899];


    double absrollaverage=0.0;
    double abspitchaverage=0.0;
    //double absyawaverage=0.0;

    //Declaration of the packets which will be sent to the device
    std::string idle = "\x75\x65\x01\x02\x02\x02\xe1\xc7";
    std::string respuestacorrectaidle ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);
    std::string imudata1 = "\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x03\xe8\xee\x04";
    std::string imudata100("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x0a\x0d\x20"s);
    std::string imudata1000 ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x01\x04\x17"s);
    std::string reset = "\x75\x65\x01\x02\x02\x7e\x5d\x43";
    std::string respuestacorrectareset ("\x75\x65\x01\x04\x04\xF1\x7e\x00\x52\x64"s);
    std::string baudratenew ("\x75\x65\x0c\x07\x07\x40\x01\x00\x03\x84\x00\xbc\x64"s);
    std::string gyracc("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x03\xe8\x05\x03\xe8\xe4\x0b"s);
    std::string gyracc100("\x75\x65\x0c\x0a\x0a\x08\x01\x02\x04\x00\x0a\x05\x00\x0a\x22\xa0"s);
    std::string respuestacorrectaajustes ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);
    std::string streamon = "\x75\x65\x0c\x05\x05\x11\x01\x01\x01\x04\x1a";
    std::string streamoff ("\x75\x65\x0c\x05\x05\x11\x01\x01\x00\x03\x19"s);
    std::string respuestacorrectastreamonoff ("\x75\x65\x0c\x04\x04\xF1\x11\x00\xf0\xcc"s);
    std::string reading;



    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    WriteDataPacket(error,idle, respuestacorrectaidle);

    //After it, user could be able to select the functionality of the sensor

    do{

        //Here we can define as much use options as we wanted
        cout << "1 - Reset device  \n";
        cout << "2 - Get gyro 100Hz  \n";
        cout << "3 - Get Euler Angles 1Hz \n";
        cout << "4 - Get Euler Angles 100Hz \n";

        int numero;
        cin >> numero;

        switch(numero) {

        case 1:{
            WriteDataPacket(error,reset,respuestacorrectareset);
            break;}

        case 2: {
            WriteDataPacket(error,imudata100,respuestacorrectaajustes);
            WriteDataPacket(error,streamon,respuestacorrectastreamonoff);
            ReadDataPacketGyro(reading,muestras);
            WriteDataPacket(error,streamoff,respuestacorrectastreamonoff);
            break;}

        case 3:{
            WriteDataPacket(error,gyracc,respuestacorrectaajustes);
            WriteDataPacket(error,streamon,respuestacorrectastreamonoff);
            std::tie(roll, pitch, absrollaverage, abspitchaverage)=ReadDataPacketEuler(reading,muestras);
            //Vectors to plot'em in Matlab
            for (int c=0;c<=muestras-100;c++){

                //Jump corrections in case device is face up. Uncomment it if device is face up.
                //                        if (c>200 && abs(*(roll+c)-*(roll+c-1))>5){
                //                            *(roll+c)=*(roll+c)+6;
                //                        }

                //                        if (*(roll+c)<=-3.2){
                //                            *(roll+c)=*(roll+c)+6;
                //                            }

                if (c==0){
                    cout << "roll = [" << *(roll+c) << " ";
                }
                if (c==(muestras-100)){
                    cout << *(roll+c) << ']'<< '\n';
                }else{
                    cout << *(roll+c) << " ";
                }

            }
            for (int c=0;c<=muestras-100;c++){
                if (c==0){
                    cout << "pitch = [" << *(pitch+c) << " ";
                }
                if (c==(muestras-100)){
                    cout << *(pitch+c) << ']'<< '\n';
                }else{
                    cout << *(pitch+c) << " ";
                }
            }
            WriteDataPacket(error,streamoff,respuestacorrectastreamonoff);
            break;}

        case 4:{
            WriteDataPacket(error,gyracc100,respuestacorrectaajustes);
            WriteDataPacket(error,streamon,respuestacorrectastreamonoff);
            std::tie(roll, pitch, absrollaverage, abspitchaverage)=ReadDataPacketEuler(reading,muestras);
            cout << "Initial offset in pitch is: "<< abspitchaverage << '\n';
            cout << "Initial offset in roll is: "<< absrollaverage << '\n';
            //Vectors to plot data in Matlab
            for (int c=0;c<=muestras-100;c++){

                //Jump corrections in case device is face up. Uncomment it if device is face up.
                //                        if (c>200 && abs(*(roll+c)-*(roll+c-1))>5){
                //                            *(roll+c)=*(roll+c)+6;
                //                        }

                //                        if (*(roll+c)<=-3.2){
                //                            *(roll+c)=*(roll+c)+6;
                //                            }


                if (c==0){
                    cout << "roll = [" << *(roll+c)-absrollaverage << " ";
                }
                if (c==(muestras-100)){
                    cout << *(roll+c)-absrollaverage << ']'<< '\n';
                }else{
                    cout << *(roll+c)-absrollaverage << " ";
                }

            }
            for (int c=0;c<=muestras-100;c++){
                if (c==0){
                    cout << "pitch = [" << *(pitch+c)-abspitchaverage << " ";
                }
                if (c==(muestras-100)){
                    cout << *(pitch+c)-abspitchaverage << ']'<< '\n';
                }else{
                    cout << *(pitch+c)-abspitchaverage << " ";
                }
            }
            WriteDataPacket(error,streamoff,respuestacorrectastreamonoff);
            break;}

        default: {
            cout << "The required use option is not defined.";
            break;}
        }

        cout << "New scan? Insert ----> y/n      \n";
        char answer1;
        cin >> answer1;
        if (answer1=='y'){
            end=0;
        }else{
            end =1;
        }

    }while(end==0);

    return 0;
}


