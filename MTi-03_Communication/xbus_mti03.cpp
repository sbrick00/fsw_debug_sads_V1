#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <iostream>

constexpr uint8_t XBUS_PREAMBLE = 0xFA;
constexpr uint8_t XBUS_BID      = 0xFF;
constexpr uint8_t MID_MTDATA2   = 0x36;

static inline uint16_t read_u16_be(const uint8_t* p) {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static inline float read_f32_be(const uint8_t* p) {
    uint32_t u = (uint32_t(p[0]) << 24) |
                 (uint32_t(p[1]) << 16) |
                 (uint32_t(p[2]) <<  8) |
                 (uint32_t(p[3])      );
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

int openSerial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    termios tty{};
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

int main() {
    int fd = openSerial("/dev/ttyUSB0");
    if (fd < 0) return 1;

    std::cout << "Listening for MTData2 packets...\n";

    uint8_t rxbuf[2048];
    size_t rxlen = 0;

    while (true) {
    int n = read(fd, rxbuf + rxlen, sizeof(rxbuf) - rxlen);
    if (n <= 0) continue;
    rxlen += n;

    size_t idx = 0;

    while (rxlen - idx >= 6) { // minimum packet size
        if (rxbuf[idx] != XBUS_PREAMBLE ||
            rxbuf[idx+1] != XBUS_BID) {
            idx++;
            continue;
        }

        uint8_t mid = rxbuf[idx+2];
        uint8_t len = rxbuf[idx+3];

        size_t packet_len = 4 + len + 1; // header + payload + checksum

        if (rxlen - idx < packet_len)
            break; // wait for more data

        // Verify checksum
        uint8_t cs = 0;
        for (size_t i = idx+1; i < idx+4+len; i++)
            cs += rxbuf[i];
        cs = 0x100 - cs;

        if (cs != rxbuf[idx + 4 + len]) {
            idx++; // bad packet, resync
            continue;
        }

        // Valid packet
        if (mid == MID_MTDATA2) {
        const uint8_t* payload = &rxbuf[idx + 4];
        size_t k = 0;

        bool haveQuat = false;
        bool haveRate = false;

        float q[4] = {0};
        float w[3] = {0};

        while (k + 3 <= len) {
            uint16_t dataId = read_u16_be(&payload[k]);
            uint8_t  size   = payload[k+2];
            k += 3;

            if (k + size > len) break;

            switch (dataId) {

            case 0x2010: // Quaternion
                if (size == 16) {
                    q[0] = read_f32_be(&payload[k + 0]);
                    q[1] = read_f32_be(&payload[k + 4]);
                    q[2] = read_f32_be(&payload[k + 8]);
                    q[3] = read_f32_be(&payload[k + 12]);
                    haveQuat = true;
                }
                break;

            case 0x8020: // Angular rate
                if (size == 12) {
                    w[0] = read_f32_be(&payload[k + 0]);
                    w[1] = read_f32_be(&payload[k + 4]);
                    w[2] = read_f32_be(&payload[k + 8]);
                    haveRate = true;
                }
                break;

            default:
                break;
            }

            k += size;
        }

        if (haveQuat && haveRate) {
            std::cout << "Quat: "
                    << q[0] << " "
                    << q[1] << " "
                    << q[2] << " "
                    << q[3]
                    << " | Rates (rad/s): "
                    << w[0] << " "
                    << w[1] << " "
                    << w[2]
                    << "\n";
        }
    }

        idx += packet_len;
        }

        // Remove processed bytes
        if (idx > 0) {
            memmove(rxbuf, rxbuf + idx, rxlen - idx);
            rxlen -= idx;
        }
    }
}