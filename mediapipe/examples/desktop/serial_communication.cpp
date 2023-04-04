#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>
#include <chrono>
#include <unistd.h>

using namespace LibSerial;

int main()
{
    // Instantiate a Serial Port and a Serial Stream object.
    SerialPort serial_port;
    SerialStream serial_stream;

    // Open the hardware serial ports.
    serial_port.Open( "/dev/ttyACM2" );
    //serial_stream.Open( "/dev/ttyACM1" );

    // Set the baud rates.
    serial_port.SetBaudRate( BaudRate::BAUD_115200 );
    //serial_stream.SetBaudRate( BaudRate::BAUD_115200 );
    //Turn the led on and off
    std::string write_byte_1 = "i0l";
    std::string write_byte_2 = "i1l";
    std::string read_byte_1;

    for (int i = 0; i<10;i++){
        if (i%2 == 0){
            serial_port.Write(write_byte_1);
            serial_port.Read(read_byte_1,19);
	        std::cout<<read_byte_1;
        }
        else{
            serial_port.Write(write_byte_2);
        }
        //wait for 0.5 seconds
        usleep(500000);
    }
    serial_port.Write(write_byte_1);

    //char write_byte_2 = 'i1l';

    // Write a character.
    //serial_port.Write(write_byte_3);
    //serial_port.Write(&write_byte_1[1]);
    //serial_port.Write(&write_byte_1[2]);

    //serial_stream << write_byte_2;

    // Read a character.
    //serial_port.Read(read_byte_1, 1);
    //serial_stream >> read_byte_2;

    //std::cout << "serial_port read:   " << read_byte_1 << std::endl;
    //std::cout << "serial_stream read: " << read_byte_2 << std::endl;

    // Close the Serial Port and Serial Stream.
    serial_port.Close();
    //serial_stream.Close();
}