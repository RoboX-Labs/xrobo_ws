#include "robocon_serial/Command.h"
#include <numeric>
#include <iomanip>

Command::Command() : head(0x3E), cmd(0), id(0), datalen(0), checksum(0), datasum(0) {}

Command::Command(uint8_t head, uint8_t cmd, uint8_t id, uint8_t datalen, std::vector<uint8_t> data)
    : head(head), cmd(cmd), id(id), datalen(datalen), checksum(0), data(std::move(data)), datasum(0)
{
    CalculateChecksum();
}

void Command::CalculateChecksum()
{
    checksum = head + cmd + id + datalen;
    datasum = std::accumulate(data.begin(), data.end(), 0);
}

bool Command::ValidateChecksum(uint8_t receivedChecksum) const
{
    return checksum == receivedChecksum;
}

bool Command::ValidateDataSum(uint8_t receivedDataSum) const
{
    return datasum == receivedDataSum;
}

std::ostream &operator<<(std::ostream &os, const Command &cmd)
{
    os << "Command(head: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd.head)
       << ", cmd: 0x" << std::setw(2) << static_cast<int>(cmd.cmd)
       << ", id: 0x" << std::setw(2) << static_cast<int>(cmd.id)
       << ", datalen: " << std::dec << static_cast<int>(cmd.datalen)
       << ", checksum: 0x" << std::hex << std::setw(2) << static_cast<int>(cmd.checksum)
       << ", data: [";
    for (size_t i = 0; i < cmd.data.size(); ++i)
    {
        os << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd.data[i]);
        if (i != cmd.data.size() - 1)
            os << ", ";
    }
    os << "], datasum: 0x" << std::hex << std::setw(2) << static_cast<int>(cmd.datasum) << ")";
    return os;
}