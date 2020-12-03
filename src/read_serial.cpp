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
//#include <yarp/os/Bottle.h>



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

//Plotting functions are only used to copy-paste vector in Matlab (Should we rest initial offset or not?)

void PlotEulerAngles(double* rollangle,double* pitchangle, double rollaverage, double pitchaverage, int numero){

    cout << "Initial offset in pitch is: "<< pitchaverage << '\n';
    cout << "Initial offset in roll is: "<< rollaverage << '\n';

    for (int c=0;c<=numero-100;c++){

        //Jump corrections in case device is placed face up. Uncomment it if device is placed face up.
        //                        if (c>200 && abs(*(roll+c)-*(roll+c-1))>5){
        //                            *(roll+c)=*(roll+c)+6;
        //                        }

        //                        if (*(roll+c)<=-3.2){
        //                            *(roll+c)=*(roll+c)+6;
        //                            }


        if (c==0){
            cout << "roll = [" << *(rollangle+c) << " ";
        }
        if (c==(numero-100)){
            cout << *(rollangle+c) << ']'<< '\n';
        }else{
            cout << *(rollangle+c) << " ";
        }

    }
    for (int c=0;c<=numero-100;c++){
        if (c==0){
            cout << "pitch = [" << *(pitchangle+c) << " ";
        }
        if (c==(numero-100)){
            cout << *(pitchangle+c) << ']'<< '\n';
        }else{
            cout << *(pitchangle+c) << " ";
        }
    }
}
void PlotGyro(double* gyrosx,double* gyrosy, double* gyrosz, int numero){

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyrox = [" << *(gyrosx+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosx+c) << ']'<< '\n';
        }else{
            cout << *(gyrosx+c) << " ";
        }

    }

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyroy = [" << *(gyrosy+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosy+c) << ']'<< '\n';
        }else{
            cout << *(gyrosy+c) << " ";
        }

    }

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyroz = [" << *(gyrosz+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosz+c) << ']'<< '\n';
        }else{
            cout << *(gyrosz+c) << " ";
        }

    }


}



int main()
{


    LordIMU3DMGX10 misensor ("COM7");


    int end=0;
    double *roll;
    double *pitch;
    double absrollaverage=0.0;
    double abspitchaverage=0.0;
    double *gyrox;
    double *gyroy;
    double *gyroz;
    float gyroxvalue;
    float gyroyvalue;
    float gyrozvalue;
    double rollvalue, pitchvalue;


    do{

        //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
        misensor.set_IDLEmode();

        //After it, user could be able to select the functionality of the sensor

        //Here we can define as much using options as we wanted
        cout << "1 - Reset device" << endl;
        cout << "2 - Get gyro at 1/100/1000 Hz" << endl;
        cout << "3 - Get Euler Angles 1/100Hz" << endl;
        cout << "4 - Gyro Polling" << endl;
        cout << "5 - Euler Angles Polling" << endl;


        int numero;
        cin >> numero;

        switch(numero) {

        case 1:{
            misensor.set_reset();
            misensor.set_streamoff();
            break;}

        case 2: {

            int frecuenciagyro=0;
            int numeromuestras=0;
            cout << "Freq?" << endl;
            cin >> frecuenciagyro;
            cout << "Samples?" << endl;
            cin >> numeromuestras;


            misensor.set_devicetogetgyro(frecuenciagyro);
            misensor.set_streamon();
            std::tie(gyrox,gyroy,gyroz) = misensor.get_gyroContinuousStream(numeromuestras);
            PlotGyro(gyrox,gyroy,gyroz,numeromuestras);
            misensor.set_streamoff();
            break;}

        case 3:{

            int frecuenciaeuler=0;
            int numeromuestras=0;
            cout << "Freq?" << endl;
            cin >> frecuenciaeuler;
            cout << "Samples?" << endl;
            cin >> numeromuestras;
            misensor.set_devicetogetgyroacc(frecuenciaeuler);
            misensor.set_streamon();
            std::tie(roll, pitch, absrollaverage, abspitchaverage)= misensor.get_euleranglesContinuousStream(numeromuestras);

            //Vectors to plot data in Matlab
            PlotEulerAngles(roll, pitch, absrollaverage, abspitchaverage,numeromuestras);
            misensor.set_streamoff();

            break;}

        case 4:{
            misensor.set_devicetogetgyro(100);
            std::tie (gyroxvalue,gyroyvalue,gyrozvalue) = misensor.get_gyroPolling();
            misensor.set_streamoff();
            break;}

        case 5:{
            misensor.set_devicetogetgyroacc(100);
            std::tie (rollvalue, pitchvalue) = misensor.get_euleranglesPolling();
            misensor.set_streamoff();
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



