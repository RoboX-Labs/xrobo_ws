#ifndef COMMAND_H
#define COMMAND_H

#include <vector>
#include <iostream>

class Command
{
public:
    Command();
    Command(uint8_t head, uint8_t cmd, uint8_t id, uint8_t datalen, std::vector<uint8_t> data);

    void CalculateChecksum();
    bool ValidateChecksum(uint8_t receivedChecksum) const;
    bool ValidateDataSum(uint8_t receivedDataSum) const;

    friend std::ostream &operator<<(std::ostream &os, const Command &cmd);

    uint8_t head;
    uint8_t cmd;
    uint8_t id;
    uint8_t datalen;
    uint8_t checksum;
    std::vector<uint8_t> data;
    uint8_t datasum;
};

#endif