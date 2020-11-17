#include <iostream>
#include <string>



#include "SerialComm.h"


//function definition at the end of the file
std::tuple <std::string,int> ReadFromDataString (int valorinicial, int valor, std::string cadena);

int main()
{


    SerialComm p1("COM7");


    for (int y=0;y<=5;y++){


//        sleep(2);
        std::string str1="hola";

        p1.ReadLine(str1);

        cout << str1 << endl;

    }

    return 0;
}





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

