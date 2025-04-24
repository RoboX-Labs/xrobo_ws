#ifndef SERIALCOM_H
#define SERIALCOM_H

#include <libserial/SerialPort.h>
#include <string>
#include "Command.h"

class SerialCom
{
public:
    SerialCom(const std::string &port, LibSerial::BaudRate buadrate = LibSerial::BaudRate::BAUD_115200, int timeout = 100);
    ~SerialCom();

    void Send(const Command &cmd);
    bool Receive(Command &cmd);

private:
    LibSerial::SerialPort serialPort_;
    int timeout_;
};

#endif