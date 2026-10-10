#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace myarm_control
{

class MyCobotSerial
{
public:
    static constexpr std::size_t kJoints = 6;

    struct Config
    {
        std::string device = "/dev/ttyACM0";
        int baudrate = 1000000;

        // Elephant Robotics encoder convention.
        int encoder_zero = 2048;
        double counts_per_rev = 4096.0;

        // Hardware direction can differ between firmware/mechanical revisions.
        std::array<int, kJoints> encoder_sign{{1, 1, 1, 1, 1, 1}};

        int command_speed = 50;          // 0..100
        int io_timeout_ms = 15;
    };

    struct EncoderSample
    {
        std::array<int, kJoints> raw{};
        bool valid{false};
    };

    explicit MyCobotSerial(const Config& config);
    ~MyCobotSerial();

    MyCobotSerial(const MyCobotSerial&) = delete;
    MyCobotSerial& operator=(const MyCobotSerial&) = delete;

    bool open();
    void close();
    bool isOpen() const;

    // Reads the first 6 arm encoders. The protocol also exists on
    // MyArm variants exposing an additional actuator; extra values are ignored.
    bool readEncoders(EncoderSample& sample);

    // Sends six synchronous absolute encoder targets using command 0x3C.
    bool setEncoders(const std::array<int, kJoints>& encoders,
                     int speed = -1);

    // Convenience angle conversions.
    double encoderToRad(int encoder, std::size_t joint) const;
    int radToEncoder(double rad, std::size_t joint) const;

    // Protocol STOP (0x29). This is intentionally preferred over
    // releasing all servos because dropping a load can be less safe.
    bool stopMotion();

private:
    bool configurePort();
    bool writeAll(const std::uint8_t* data, std::size_t size);
    bool readFrame(std::vector<std::uint8_t>& frame, int timeout_ms);
    bool transact(const std::vector<std::uint8_t>& tx,
                  std::vector<std::uint8_t>& rx,
                  int timeout_ms);

    static std::uint8_t hiByte(std::uint16_t x);
    static std::uint8_t loByte(std::uint16_t x);

    static double normalizePi(double x);

    Config config_;
    int fd_{-1};
    mutable std::mutex io_mutex_;
};

} // namespace myarm_control
