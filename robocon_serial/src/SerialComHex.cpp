#include "robocon_serial/SerialComHex.h"
#include <stdexcept>
#include <iostream>

SerialCom::SerialCom(const std::string &port, LibSerial::BaudRate buadrate, int timeout)
    : timeout_(timeout)
{
    serialPort_.Open(port);
    serialPort_.SetVTime(timeout);
    serialPort_.SetBaudRate(buadrate);
    serialPort_.FlushIOBuffers();
}

SerialCom::~SerialCom()
{
    if (serialPort_.IsOpen())
    {
        serialPort_.Close();
    }
}

void SerialCom::Send(const Command &cmd)
{
    std::vector<uint8_t> dataBuffer;
    dataBuffer.push_back(cmd.head);
    dataBuffer.push_back(cmd.cmd);
    dataBuffer.push_back(cmd.id);
    dataBuffer.push_back(cmd.datalen);
    dataBuffer.push_back(cmd.checksum);
    dataBuffer.insert(dataBuffer.end(), cmd.data.begin(), cmd.data.end());
    dataBuffer.push_back(cmd.datasum);

    serialPort_.Write(dataBuffer);
    serialPort_.DrainWriteBuffer();
}

bool SerialCom::Receive(Command &cmd)
{
    try
    {
        if (!serialPort_.IsDataAvailable())
            return false;

        auto readByte = [this]() -> uint8_t
        {
            uint8_t byte;
            serialPort_.ReadByte(byte, timeout_);
            return byte;
        };

        // Read header
        while ((cmd.head = readByte()) != 0x3E)
            std::cerr << "Header not found! Waiting for 0x3E" << std::endl;

        // Read command, id, data length, and checksum
        cmd.cmd = readByte();
        cmd.id = readByte();
        cmd.datalen = readByte();
        cmd.CalculateChecksum();
        if (!cmd.ValidateChecksum(readByte()))
            throw std::runtime_error("Checksum error!");

        // Read data and validate data sum
        cmd.data.resize(cmd.datalen);
        serialPort_.Read(cmd.data, cmd.datalen, timeout_);
        cmd.CalculateChecksum();
        if (!cmd.ValidateDataSum(readByte()))
            throw std::runtime_error("DataSum error!");

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}