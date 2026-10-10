#include "myarm_control/mycobot_serial.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <vector>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr std::uint8_t HEADER = 0xFE;
constexpr std::uint8_t FOOTER = 0xFA;

constexpr std::uint8_t STOP = 0x29;
constexpr std::uint8_t SET_ENCODERS = 0x3C;
constexpr std::uint8_t GET_ENCODERS = 0x3D;

bool setBaud(termios& tio, speed_t baud)
{
    return cfsetispeed(&tio, baud) == 0 && cfsetospeed(&tio, baud) == 0;
}
} // namespace

namespace myarm_control
{

MyCobotSerial::MyCobotSerial(const Config& config)
    : config_(config)
{
    for (int& s : config_.encoder_sign)
        s = (s >= 0) ? 1 : -1;

    config_.command_speed = std::clamp(config_.command_speed, 0, 100);
}

MyCobotSerial::~MyCobotSerial()
{
    close();
}

bool MyCobotSerial::open()
{
    std::lock_guard<std::mutex> lock(io_mutex_);

    if (fd_ >= 0)
        return true;

    fd_ = ::open(
        config_.device.c_str(),
        O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0)
        return false;

    if (!configurePort())
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void MyCobotSerial::close()
{
    std::lock_guard<std::mutex> lock(io_mutex_);

    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool MyCobotSerial::isOpen() const
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    return fd_ >= 0;
}

bool MyCobotSerial::configurePort()
{
    if (fd_ < 0)
        return false;

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0)
        return false;

    cfmakeraw(&tio);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

#ifdef B1000000
    speed_t baud = B1000000;
#else
    speed_t baud = B115200;
#endif

    if (config_.baudrate == 115200)
        baud = B115200;
    else if (config_.baudrate == 1000000)
    {
#ifdef B1000000
        baud = B1000000;
#else
        return false;
#endif
    }
    else
    {
        // Only the rates normally needed by this driver are accepted.
        return false;
    }

    if (!setBaud(tio, baud))
        return false;

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    return tcsetattr(fd_, TCSANOW, &tio) == 0;
}

bool MyCobotSerial::writeAll(
    const std::uint8_t* data,
    std::size_t size)
{
    std::size_t sent = 0;

    while (sent < size)
    {
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLOUT;

        const int rc = ::poll(&pfd, 1, config_.io_timeout_ms);
        if (rc <= 0)
            return false;

        const ssize_t n = ::write(
            fd_, data + sent, size - sent);

        if (n > 0)
            sent += static_cast<std::size_t>(n);
        else if (n < 0 && errno != EAGAIN && errno != EINTR)
            return false;
    }

    return true;
}

bool MyCobotSerial::readFrame(
    std::vector<std::uint8_t>& frame,
    int timeout_ms)
{
    frame.clear();

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    bool got_header_1 = false;
    bool got_header_2 = false;

    std::uint8_t byte = 0;
    std::size_t expected_size = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();

        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        const int rc = ::poll(
            &pfd, 1, static_cast<int>(std::max<long long>(1, remaining)));

        if (rc <= 0)
            continue;

        const ssize_t n = ::read(fd_, &byte, 1);
        if (n != 1)
            continue;

        if (!got_header_1)
        {
            if (byte == HEADER)
            {
                got_header_1 = true;
                frame = {HEADER};
            }
            continue;
        }

        if (!got_header_2)
        {
            if (byte == HEADER)
            {
                got_header_2 = true;
                frame.push_back(HEADER);
            }
            else
            {
                got_header_1 = false;
                frame.clear();
            }
            continue;
        }

        frame.push_back(byte);

        if (frame.size() == 3)
        {
            // Total frame bytes = data_length + 3.
            expected_size =
                static_cast<std::size_t>(frame[2]) + 3U;

            if (expected_size < 5U || expected_size > 64U)
            {
                frame.clear();
                got_header_1 = false;
                got_header_2 = false;
                expected_size = 0;
            }
        }

        if (expected_size != 0 && frame.size() >= expected_size)
        {
            if (frame.back() == FOOTER)
                return true;

            frame.clear();
            got_header_1 = false;
            got_header_2 = false;
            expected_size = 0;
        }
    }

    return false;
}

bool MyCobotSerial::transact(
    const std::vector<std::uint8_t>& tx,
    std::vector<std::uint8_t>& rx,
    int timeout_ms)
{
    std::lock_guard<std::mutex> lock(io_mutex_);

    if (fd_ < 0)
        return false;

    tcflush(fd_, TCIFLUSH);

    if (!writeAll(tx.data(), tx.size()))
        return false;

    return readFrame(rx, timeout_ms);
}

bool MyCobotSerial::readEncoders(EncoderSample& sample)
{
    sample.valid = false;

    const std::vector<std::uint8_t> tx{
        HEADER, HEADER, 0x02, GET_ENCODERS, FOOTER};

    std::vector<std::uint8_t> rx;
    if (!transact(tx, rx, config_.io_timeout_ms))
        return false;

    // Need: FE FE length 3D + >=12 bytes + FA.
    if (rx.size() < 17 ||
        rx[0] != HEADER ||
        rx[1] != HEADER ||
        rx[3] != GET_ENCODERS ||
        rx.back() != FOOTER)
        return false;

    for (std::size_t i = 0; i < kJoints; ++i)
    {
        const std::size_t idx = 4 + 2 * i;
        if (idx + 1 >= rx.size() - 1)
            return false;

        const std::uint16_t raw =
            (static_cast<std::uint16_t>(rx[idx]) << 8U) |
             static_cast<std::uint16_t>(rx[idx + 1]);

        sample.raw[i] = static_cast<int>(raw);
    }

    sample.valid = true;
    return true;
}

bool MyCobotSerial::setEncoders(
    const std::array<int, kJoints>& encoders,
    int speed)
{
    if (speed < 0)
        speed = config_.command_speed;

    speed = std::clamp(speed, 0, 100);

    // Data length = command byte + 12 encoder bytes + speed byte = 0x0E.
    // On the classic protocol documents the length byte is represented
    // together with the protocol framing as the equivalent "0x0F" frame
    // for the 6-servo set command; to avoid relying on a mistaken constant,
    // construct the documented frame exactly as:
    // FE FE 0F 3C [12 encoder bytes] [speed] FA.
    std::vector<std::uint8_t> tx;
    tx.reserve(18);

    tx.push_back(HEADER);
    tx.push_back(HEADER);
    tx.push_back(0x0F);
    tx.push_back(SET_ENCODERS);

    for (std::size_t i = 0; i < kJoints; ++i)
    {
        const int value =
            std::clamp(encoders[i], 0, 4095);

        tx.push_back(hiByte(static_cast<std::uint16_t>(value)));
        tx.push_back(loByte(static_cast<std::uint16_t>(value)));
    }

    tx.push_back(static_cast<std::uint8_t>(speed));
    tx.push_back(FOOTER);

    std::lock_guard<std::mutex> lock(io_mutex_);

    if (fd_ < 0)
        return false;

    tcflush(fd_, TCIFLUSH);
    return writeAll(tx.data(), tx.size());
}

bool MyCobotSerial::stopMotion()
{
    const std::vector<std::uint8_t> tx{
        HEADER, HEADER, 0x02, STOP, FOOTER};

    std::lock_guard<std::mutex> lock(io_mutex_);

    if (fd_ < 0)
        return false;

    tcflush(fd_, TCIFLUSH);
    return writeAll(tx.data(), tx.size());
}

std::uint8_t MyCobotSerial::hiByte(std::uint16_t x)
{
    return static_cast<std::uint8_t>((x >> 8U) & 0xFFU);
}

std::uint8_t MyCobotSerial::loByte(std::uint16_t x)
{
    return static_cast<std::uint8_t>(x & 0xFFU);
}

double MyCobotSerial::normalizePi(double x)
{
    while (x > M_PI)
        x -= 2.0 * M_PI;

    while (x < -M_PI)
        x += 2.0 * M_PI;

    return x;
}

double MyCobotSerial::encoderToRad(
    int encoder,
    std::size_t joint) const
{
    if (joint >= kJoints)
        throw std::out_of_range("joint index");

    const double raw =
        static_cast<double>(encoder);

    double diff = raw -
        static_cast<double>(config_.encoder_zero);

    // Encoder wraps around 0..4095.
    if (diff > config_.counts_per_rev / 2.0)
        diff -= config_.counts_per_rev;
    else if (diff < -config_.counts_per_rev / 2.0)
        diff += config_.counts_per_rev;

    const double rad =
        diff * (2.0 * M_PI / config_.counts_per_rev);

    return static_cast<double>(config_.encoder_sign[joint]) * rad;
}

int MyCobotSerial::radToEncoder(
    double rad,
    std::size_t joint) const
{
    if (joint >= kJoints)
        throw std::out_of_range("joint index");

    const double signed_rad =
        rad * static_cast<double>(config_.encoder_sign[joint]);

    const double counts =
        static_cast<double>(config_.encoder_zero) +
        signed_rad *
            config_.counts_per_rev /
            (2.0 * M_PI);

    int encoder =
        static_cast<int>(std::llround(counts));

    encoder %= 4096;
    if (encoder < 0)
        encoder += 4096;

    return std::clamp(encoder, 0, 4095);
}

} // namespace myarm_control
