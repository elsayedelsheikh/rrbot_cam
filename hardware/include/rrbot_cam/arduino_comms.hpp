#ifndef RRBOT_ARDUINO_COMMS_HPP
#define RRBOT_ARDUINO_COMMS_HPP

// #include <cstring>
// #include <cstdlib>
#include <sstream>
#include <libserial/SerialPort.h>

#include <iostream>
#include <cmath>

namespace arduino_comms {

LibSerial::BaudRate convert_baud_rate(int baud_rate) {
  // Just handle some common baud rates
  switch (baud_rate) {
    case 1200:
      return LibSerial::BaudRate::BAUD_1200;
    case 1800:
      return LibSerial::BaudRate::BAUD_1800;
    case 2400:
      return LibSerial::BaudRate::BAUD_2400;
    case 4800:
      return LibSerial::BaudRate::BAUD_4800;
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate
                << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms {
 public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate,
               int32_t timeout_ms) {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect() { serial_conn_.Close(); }

  bool connected() const { return serial_conn_.IsOpen(); }

  std::string send_msg(const std::string &msg_to_send,
                       bool print_output = false) {
    serial_conn_.FlushIOBuffers();  // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    } catch (const LibSerial::ReadTimeout &) {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (print_output) {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response
                << std::endl;
    }

    return response;
  }

  void send_empty_msg() { std::string response = send_msg("\r"); }

  void read_servos_position(double &val_1, double &val_2) {
    std::string response = send_msg("t\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    // Get the angles in degrees
    int angle_in_degrees_1, angle_in_degrees_2;
    angle_in_degrees_1 = std::atoi(token_1.c_str());
    angle_in_degrees_2 = std::atoi(token_2.c_str());

    // Convert to radians
    val_1 = angle_in_degrees_1 * M_PI / 180.0;
    val_2 = angle_in_degrees_2 * M_PI / 180.0;
  }

  void set_servos_position(double pan_angle_radians, double tilt_angle_radians) {
    // Convert to degrees
    int pan_angle = pan_angle_radians * 180.0 / M_PI;
    int tilt_angle = tilt_angle_radians * 180.0 / M_PI;

    // Send the command
    std::stringstream ss;
    ss << "s " << pan_angle << " " << tilt_angle << "\r";
    send_msg(ss.str());
  }

  void reset_servos_position() { send_msg("r\r"); }

 private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

}  // namespace arduino_comms

#endif  // RRBOT_ARDUINO_COMMS_HPP