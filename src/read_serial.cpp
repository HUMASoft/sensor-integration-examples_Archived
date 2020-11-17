#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <string.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/hex.hpp>


using namespace boost::asio;
using namespace boost::algorithm;
using namespace std::string_literals;

using std::cin;
using std::cout;


// These are the values our port needs to connect
#ifdef _WIN32
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORT = "COM7";
#endif


union ulf
{
    unsigned long ul;
    float f;
};

int main()
{

    boost::asio::io_context io; //Active I/0 Functions
    boost::asio::serial_port port(io); //Creation of the object
    port.open("COM7");

    if (!port.is_open()){
        cout << "El puerto no ha podido abrirse \n";
    }else{
        cout << "El puerto se ha abierto correctamente \n";
    }


    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    boost::system::error_code error;
    boost::asio::streambuf buffer;

    //Preguntamos al usuario que quiere hacer con el sensor correctamente conectado
    int end;
    int end1;
    int end2;
    int comp;
    int fin;


    //Como al conectar el sensor, empieza a transmitir solo, paramos ese stream
    do{
        do{

            cout << "Que desea hacer? \n";
            cout << "1 - Set to IDLE \n";
            cout << "2 - Solicitar xx a f=1Hz  \n";
            cout << "3 - Solicitar xx a f=100Hz  \n";
            cout << "4 - Solicitar xx a f=1000Hz  \n";
            cout << "5 - Resetear el dispositivo  \n";
            cout << "6 - Cambiar el BaudRate a 230400bps \n";


            int numero;
            cin >> numero;

            switch(numero) {

            case 1: {
                std::string idle = "\x75\x65\x01\x02\x02\x02\xe1\xc7";
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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x02\x00\xD6\x6C"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }

                break;}

            case 2: {
                std::string imudata1 = "\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x03\xe8\xee\x04";
                boost::asio::write(
                            port,
                            boost::asio::buffer(imudata1.c_str(), imudata1.size()),
                            boost::asio::transfer_at_least(imudata1.size()),
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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }


                break;}
            case 3: {
                std::string imudata100("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x0a\x0d\x20"s);
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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }

                break;}
            case 4: {

                std::string imudata1000 ("\x75\x65\x0c\x07\x07\x08\x01\x01\x05\x00\x01\x04\x17"s);
                boost::asio::write(
                            port,
                            boost::asio::buffer(imudata1000.c_str(), imudata1000.size()),
                            boost::asio::transfer_at_least(imudata1000.size()),
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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x08\x00\xE7\xBA"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "Nice done! \n";
                }else{
                    cout << "An error has ocurred.";
                }

                break;}

            case 5:{
                std::string reset = "\x75\x65\x01\x02\x02\x7e\x5d\x43";
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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x01\x04\x04\xF1\x7e\x00\x52\x64"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "The device has been reset! \n";
                }else{
                    cout << "An error has ocurred.";
                }


                break;}
            case 6:{
                //port.set_option(boost::asio::serial_port_base::baud_rate(921600));
                std::string baudratenew ("\x75\x65\x0c\x07\x07\x40\x01\x00\x03\x84\x00\xbc\x64"s);
                boost::asio::write(
                            port,
                            boost::asio::buffer(baudratenew.c_str(), baudratenew.size()),
                            boost::asio::transfer_at_least(baudratenew.size()),
                            error
                            );

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
                        cout << hex(feedbackdevice) << '\n';
                        end2=1;
                    }else{
                        end2=0;
                        feedbackdevice.clear();
                    }

                }while(end2==0);

                //Si el feedback es el correcto, seguimos
                std::string respuestacorrecta ("\x75\x65\x0c\x04\x04\xF1\x40\x00\x1f\x2a"s);

                if (hex(feedbackdevice) == hex(respuestacorrecta)){
                    cout << "Baudrate has been chenged to 230400! \n";
                    port.set_option(boost::asio::serial_port_base::baud_rate(230400));
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

            std::string streamon = "\x75\x65\x0c\x05\x05\x11\x01\x01\x01\x04\x1a";
            boost::asio::write(
                        port,
                        boost::asio::buffer(streamon.c_str(), streamon.size()),
                        boost::asio::transfer_at_least(streamon.size()),
                        error
                        );
        }



        //Empezamos probando xx lecturas
        for (int h=0; h<=5000;h++){
            int comp=0;
            int fin=0;
            char c;
            std::string result;

            //Se leerá hasta que encontremos el inicio del paquete, 'ue'
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

            //Salimos a leer el descriptor, y la longitud
            //Descriptor
            char descriptor;
            boost::asio::read(port, boost::asio::buffer(&descriptor,1));
            result+=descriptor;

            //Longitud
            char longitud;
            boost::asio::read(port, boost::asio::buffer(&longitud,1));
            result+=longitud;

            //Una vez tenemos la longitud, ya sabemos de que tamaño es el paquete, y hasta donde hay que leer
            //Ojo a LSB,MSB ---> long+1

            cout << "La longitud del paquete es:  "<< (int)longitud << '\n';

            //Leemos hasta dicha longitud



            for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
                boost::asio::read(port, boost::asio::buffer(&c,1));
                result+=c;
            }

            //Mostramos el paquete en hexadecimal
            cout << "La cadena completa es: " << hex(result) << '\n';

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
            }

            //Reinicio de la cadena para empezar una nueva lectura
            result.clear();


        }
        //Finalizacion del stream 7565 0C05 0511 0101 0003 19
        std::string streamoff ("\x75\x65\x0c\x05\x05\x11\x01\x01\x00\x03\x19"s);
        boost::asio::write(
                    port,
                    boost::asio::buffer(streamoff.c_str(), streamoff.size()),
                    boost::asio::transfer_at_least(streamoff.size()),
                    error
                    );

        cout << "Nuevo escaneo? ---> y/n" << '\n';
        char answer2;
        cin >> answer2;
        if (answer2 == 'n'){
            end1=1;
        }else{
            end1=0;
        }
    }while (end1==0);



    return 0;
}
//||

//La cadena en bits que recibo es 111010101100101100000000000111000001110000001011011101101011001110001100100110110111011000111011000100000101001101110101100011001101011001001110011110110100101
//Alrededor de 160bits
//Si quiero irme a 1000Hz, necesitaria 160000, es decir, 115200 no nos vale. Habría que subir el baudrate a 230400(el inmediato superior)
//Para este baudrate, el paquete es 7565 0c07 0740 0100 0384 00bc 64


