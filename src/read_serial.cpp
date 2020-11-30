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

//#include <yarp/os/RFModule.h>


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


    //Decl. of the variables
    char a;
    std::string feedbackdevice;
    int comp=0;
    int fin=0;
    int end2=0;
    int fin4=0;
    char feedback_desc;
    char feedback_long;

    //We send the specified data packet
    boost::asio::write(
                port,
                boost::asio::buffer(datapacket.c_str(), datapacket.size()),
                boost::asio::transfer_at_least(datapacket.size()),
                error
                );

    //The methodology will be the next:
    //   1) First, we will go through the 1st do-while loop untill we find 'ue' in our data packet. Then, varible "fin" will be set to 1.
    //   2) After reading 'ue', two next bites in the data packet are the descriptor and the lenght. Both will be read.
    //   3) We know the lenght of the correct answer of the device from "datapacketanswer", at his 4th bit. We will compare it with the recent lenght read.
    //   4) If  they match, we are in the correct way. If dont, we go back to the step 1.
    //   5) In case of matching, we will read the complete data packet answer.
    //   6) Finally, we compare "dtaapacketanswer" with our read data packet. If they match, process has been sucesfully done. If dont, we go back to step 1.

    do{

        do{
            //Reset of some variables to avoid infinite loops
            feedbackdevice.clear();
            comp=0;
            fin=0;
            end2=0;
            fin4=0;

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

        boost::asio::read(port, boost::asio::buffer(&feedback_desc,1));
        feedbackdevice +=feedback_desc;
        boost::asio::read(port, boost::asio::buffer(&feedback_long,1));
        feedbackdevice +=feedback_long;

        if (int(feedback_long) != int(datapacketanswer.at(4))){
            fin4=0;
        }else{
            fin4=1;
        }

        }while (fin4==0);

        for (int i=0; i<= ((int)feedback_long+1); i++){
            boost::asio::read(port, boost::asio::buffer(&a,1));
            feedbackdevice +=a;
        }

        if (feedbackdevice==datapacketanswer){
            end2=1;
        }else{
            feedbackdevice.clear();
            end2=0;
        }
    }while(end2==0);
}

//This func will be used to read Gyro Data from the device
//Func is reading xx samples. The number can be changed anytime by setting a specific value to the "muestras" variable.
int muestras=2000;
void ReadDataPacketGyro (std::string answer, int samples){

    //Decl. of the variables
    char c;
    char descriptor;
    char longitud;

    //The methodology will be the next:
    //   1) First, we will go through the 1st do-while loop untill we find 'ue' in our data packet. Then, varible "fin" will be set to 1.
    //   2) After reading 'ue', two next bites in the data packet are the descriptor and the lenght. Both will be read.
    //   3) We read the entire data packet with a for loop and its limit set by the lenght of the packet.
    //   4) We extract gyro values (gyrx,gyry,gyrz) from the recent read data packet.
    //   5) Repeat all steps "muestras" times.

    for (int h=0; h<=samples;h++){

        //Reset of some variables to avoid infinite loops
        int comp=0;
        int fin=0;

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
                break;}
            }
        }while(fin==0);

        boost::asio::read(port, boost::asio::buffer(&descriptor,1));
        answer+=descriptor;

        boost::asio::read(port, boost::asio::buffer(&longitud,1));
        answer+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            boost::asio::read(port, boost::asio::buffer(&c,1));
            answer+=c;
        }

        cout << "Data packet is: " << hex(answer) << '\n';

        if (int(longitud) == 14){ //It must be 14
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

        answer.clear();

        }

}

//This func will be used to read Euler Angles from the device
//Func is reading xx samples. The number can be changed anytime by setting a specific value to the "muestras" variable.
std::tuple <double*,double*, double, double> ReadDataPacketEuler (std::string answer, int samples){

    //Decl. of the variables
    char c;
    char longitud;
    char descriptor;
    static double rollvector[10000];
    static double pitchvector[10000];
    double rollaverage=0.0;
    double pitchaverage=0.0;

    //The methodology will be the next:
    //   1) First, we will go through the 1st do-while loop untill we find 'ue' in our data packet. Then, varible "fin" will be set to 1.
    //   2) After reading 'ue', two next bites in the data packet are the descriptor and the lenght. Both will be read.
    //   3) We read the entire data packet with a for loop and its limit set by the lenght of the packet.
    //   4) We extract float gyro values (gyrox,gyroy,gyroz) and float acc values(accx,accy,accz) from the recent read data packet.
    //   5) Now, we need to converts these values to Euler Angles(Pitch,Roll). "Attitude_estimator" lib is used to perform it.
    //   6) Once the receiving values are stable, we use the comment library to get Pitch,Roll. (If device is placed face down, values are stable since the very beginning).
    //   7) To correct the initial offset, we will get the average value of the first 125 values. This way, if our initial offset Yaw it 2'5, a correct value to the measurings of this angle will be: measuring - 2'5.
    //   8) Repeat all steps "muestras" times.

    for (int h=0; h<=samples;h++){

        //Reset of the variables to avoid infinite loops.
         answer.clear();
         int comp=0;
         int fin=0;

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

        boost::asio::read(port, boost::asio::buffer(&descriptor,1));
        answer+=descriptor;

        boost::asio::read(port, boost::asio::buffer(&longitud,1));
        answer+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            boost::asio::read(port, boost::asio::buffer(&c,1));
            answer+=c;
        }

        cout << "Data packet is: " << hex(answer) << '\n';

        ulf accx;
        std::string str =hex(answer.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> accx.ul;
        double f = accx.f;
        cout << "Acc x:  " << f << "rad/s2" << '\n';

        ulf accy;
        std::string str1 =hex(answer.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> accy.ul;
        double f1 = accy.f;
        cout << "Acc y:  " << f1 << "rad/s2" << '\n';

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

        //If sensor is placed face down, we can skip the if loop.
        if (h>=100 && h<=samples){

            estimator.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
            rollvector[h-100]=estimator.eulerRoll();
            pitchvector[h-100]=estimator.eulerPitch();
            cout << "My attitude is (YX Euler): (" << estimator.eulerPitch() << "," << estimator.eulerRoll() << ")" << '\n';


            if(h>=225 && h<=350){

                rollaverage= rollaverage + estimator.eulerRoll();
                pitchaverage= pitchaverage + estimator.eulerPitch();

                if (h==350){

                    rollaverage = rollaverage/125;
                    pitchaverage = pitchaverage/125;
                }
            }
        }
       //}
    }

    return std::make_tuple(rollvector,pitchvector, rollaverage, pitchaverage);
}

int main()
{

    string a_1;
    SerialComm puertoserie("COM1");

    puertoserie.ReadLine(a_1);

    port.open(PORT);//Port needs to be opened to read/write data

    if (!port.is_open()){
        cout << "Port can't be opened \n";
    }else{
        cout << "Port has been sucesfully opened \n";
    }

    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    boost::system::error_code error;
    //boost::asio::streambuf buffer;

    //Our device has no magnotemeter
    estimator.setMagCalib(0.0, 0.0, 0.0);
    //Setting of GyroBias
    double bx = -0.002786;
    double by = -0.001833;
    double bz = -0.001066;
    estimator.setGyroBias(bx,by,bz);

    //Defaults gains used
    estimator.setPIGains(2.2, 2.65, 10, 1.25);


    int end=0;
    double *roll;
    double *pitch;
    double absrollaverage=0.0;
    double abspitchaverage=0.0;

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


    do{

        //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
        WriteDataPacket(error,idle, respuestacorrectaidle);

        //After it, user could be able to select the functionality of the sensor

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
            WriteDataPacket(error,streamoff,respuestacorrectastreamonoff);
            break;}

        case 2: {
            WriteDataPacket(error,imudata100,respuestacorrectaajustes);
            WriteDataPacket(error,streamon,respuestacorrectastreamonoff);
            ReadDataPacketGyro(reading,muestras);
            cout << "Hola1 \n";
            WriteDataPacket(error,streamoff,respuestacorrectastreamonoff);
            cout << "Hola \n";
            break;}

        case 3:{
            WriteDataPacket(error,gyracc,respuestacorrectaajustes);
            WriteDataPacket(error,streamon,respuestacorrectastreamonoff);
            std::tie(roll, pitch, absrollaverage, abspitchaverage)=ReadDataPacketEuler(reading,muestras);
            //Vectors to plot'em in Matlab
            for (int c=0;c<=muestras-100;c++){

                //Jump corrections in case device is placed face up. Uncomment it if device is placed face up.
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

                //Jump corrections in case device is placed face up. Uncomment it if device is placed face up.
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
        cout << "Hola \n";

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



