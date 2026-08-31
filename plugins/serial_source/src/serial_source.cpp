/*
 * serial_source
 *
 * MADS source plugin that:
 *   1. Reads newline-delimited JSON from a serial port.
 *   2. Buffers valid JSON samples.
 *   3. Publishes one block after collecting 1000 samples.
 */

#include <source.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#ifndef PLUGIN_NAME
#define PLUGIN_NAME "serial_source"
#endif

using namespace std;
using json = nlohmann::json;


class Serial_sourcePlugin : public Source<json> {

public:
  using Source::Source;

  ~Serial_sourcePlugin() {
    close_serial();
  }

  string kind() override {
    return PLUGIN_NAME;
  }

  /*
   * MADS calls this method repeatedly.
   *
   * success:
   *   A complete block is ready and MADS publishes it.
   *
   * retry:
   *   Fewer than block_size samples are available, so nothing is published.
   *
   * critical:
   *   A fatal serial-port error occurred.
   */
  return_type get_output(
      json &out,
      vector<unsigned char> *blob = nullptr
  ) override {

    out.clear();

    try {
      // Read every byte currently available from the serial port.
      read_available_data();

      // Extract complete lines and convert valid lines to JSON.
      extract_json_lines();

    } catch (const exception &error) {
      cerr << "[" << PLUGIN_NAME << "] Serial error: "
           << error.what() << endl;

      return return_type::critical;
    }

    // A complete block is not ready yet.
    if (_samples.size() < _block_size) {
      return return_type::retry;
    }

    json block = json::array();

    // Move exactly block_size samples into the outgoing message.
    for (size_t i = 0; i < _block_size; ++i) {
      block.push_back(std::move(_samples.front()));
      _samples.pop_front();
    }

    out["block_id"] = _block_id++;
    out["sample_count"] = block.size();
    out["samples"] = std::move(block);

    // Useful metadata for checking sample continuity.
    if (!out["samples"].empty()) {
      const json &first_sample = out["samples"].front();
      const json &last_sample = out["samples"].back();

      if (first_sample.contains("n")) {
        out["first_n"] = first_sample["n"];
      }

      if (last_sample.contains("n")) {
        out["last_n"] = last_sample["n"];
      }

      if (first_sample.contains("t_us")) {
        out["first_t_us"] = first_sample["t_us"];
      }

      if (last_sample.contains("t_us")) {
        out["last_t_us"] = last_sample["t_us"];
      }
    }

    out["invalid_lines_total"] = _invalid_lines;
    out["buffered_samples"] = _samples.size();

    if (!_agent_id.empty()) {
      out["agent_id"] = _agent_id;
    }

    return return_type::success;
  }


  /*
   * Called once when MADS initializes the plugin.
   */
  void set_params(const json &params) override {

    Source::set_params(params);

    // Default settings.
    _params["port"] = "";
    _params["baud_rate"] = 1000000;
    _params["block_size"] = 1000;
    _params["max_line_length"] = 1024;

    // Replace defaults with values from mads.ini.
    _params.merge_patch(params);

    _port = _params.value("port", string{});
    _baud_rate = _params.value("baud_rate", 1000000);
    _block_size = _params.value(
        "block_size",
        static_cast<size_t>(1000)
    );
    _max_line_length = _params.value(
        "max_line_length",
        static_cast<size_t>(1024)
    );

    if (_port.empty()) {
      throw invalid_argument(
          "The serial port parameter 'port' is empty"
      );
    }

    if (_block_size == 0) {
      throw invalid_argument(
          "The parameter 'block_size' must be greater than zero"
      );
    }

    if (_max_line_length == 0) {
      throw invalid_argument(
          "The parameter 'max_line_length' must be greater than zero"
      );
    }

    open_serial();
  }


  map<string, string> info() override {
    return {
      {"description", "Buffered JSON serial source"},
      {"port", _port},
      {"baud_rate", to_string(_baud_rate)},
      {"block_size", to_string(_block_size)}
    };
  }


private:
  int _serial_fd = -1;

  string _port;
  string _rx_buffer;

  int _baud_rate = 1000000;

  size_t _block_size = 1000;
  size_t _max_line_length = 1024;

  deque<json> _samples;

  uint64_t _block_id = 0;
  uint64_t _invalid_lines = 0;


  /*
   * Convert an integer baud rate to its Linux termios constant.
   */
  speed_t baud_to_termios(int baud_rate) {

    switch (baud_rate) {
      case 9600:
        return B9600;

      case 19200:
        return B19200;

      case 38400:
        return B38400;

      case 57600:
        return B57600;

      case 115200:
        return B115200;

#ifdef B230400
      case 230400:
        return B230400;
#endif

#ifdef B460800
      case 460800:
        return B460800;
#endif

#ifdef B500000
      case 500000:
        return B500000;
#endif

#ifdef B921600
      case 921600:
        return B921600;
#endif

#ifdef B1000000
      case 1000000:
        return B1000000;
#endif

      default:
        throw invalid_argument(
            "Unsupported baud rate: " + to_string(baud_rate)
        );
    }
  }


  /*
   * Open and configure the serial port.
   */
  void open_serial() {

    close_serial();

    _serial_fd = open(
        _port.c_str(),
        O_RDONLY | O_NOCTTY | O_NONBLOCK
    );

    if (_serial_fd < 0) {
      throw runtime_error(
          "Cannot open " + _port + ": " + strerror(errno)
      );
    }

    termios tty{};

    if (tcgetattr(_serial_fd, &tty) != 0) {
      string error_message = strerror(errno);
      close_serial();

      throw runtime_error(
          "Cannot read serial settings: " + error_message
      );
    }

    // Disable terminal processing: receive bytes exactly as transmitted.
    cfmakeraw(&tty);

    speed_t speed = baud_to_termios(_baud_rate);

    if (cfsetispeed(&tty, speed) != 0 ||
        cfsetospeed(&tty, speed) != 0) {

      string error_message = strerror(errno);
      close_serial();

      throw runtime_error(
          "Cannot set serial baud rate: " + error_message
      );
    }

    // 8 data bits, no parity, one stop bit.
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;

#ifdef CRTSCTS
    // Disable hardware flow control.
    tty.c_cflag &= ~CRTSCTS;
#endif

    // Do not block while waiting for serial bytes.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(_serial_fd, TCSANOW, &tty) != 0) {
      string error_message = strerror(errno);
      close_serial();

      throw runtime_error(
          "Cannot apply serial settings: " + error_message
      );
    }

    // Remove incomplete bytes that existed before initialization.
    tcflush(_serial_fd, TCIFLUSH);

    _rx_buffer.clear();
    _samples.clear();

    cerr << "[" << PLUGIN_NAME << "] Opened "
         << _port
         << " at "
         << _baud_rate
         << " baud"
         << endl;
  }


  void close_serial() {

    if (_serial_fd >= 0) {
      close(_serial_fd);
      _serial_fd = -1;
    }
  }


  /*
   * Read all bytes currently available.
   *
   * Because the descriptor is nonblocking, this method returns immediately
   * when no additional bytes are available.
   */
  void read_available_data() {

    if (_serial_fd < 0) {
      throw runtime_error("Serial port is not open");
    }

    char temporary_buffer[8192];

    while (true) {

      ssize_t bytes_read = read(
          _serial_fd,
          temporary_buffer,
          sizeof(temporary_buffer)
      );

      if (bytes_read > 0) {
        _rx_buffer.append(
            temporary_buffer,
            static_cast<size_t>(bytes_read)
        );

        continue;
      }

      if (bytes_read == 0) {
        break;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      if (errno == EINTR) {
        continue;
      }

      throw runtime_error(
          "Serial read failed: " + string(strerror(errno))
      );
    }
  }


  /*
   * Separate the received byte stream using newline characters.
   */
  void extract_json_lines() {

    size_t newline_position;

    while (
        (newline_position = _rx_buffer.find('\n'))
        != string::npos
    ) {

      string line = _rx_buffer.substr(0, newline_position);

      _rx_buffer.erase(0, newline_position + 1);

      // Accept both "\n" and "\r\n".
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      if (line.empty()) {
        continue;
      }

      if (line.size() > _max_line_length) {
        ++_invalid_lines;
        continue;
      }

      // Parse without throwing an exception.
      json sample = json::parse(
          line,
          nullptr,
          false
      );

      if (sample.is_discarded() || !sample.is_object()) {
        ++_invalid_lines;
        continue;
      }

      _samples.push_back(std::move(sample));
    }

    /*
     * If no newline arrives and the partial line becomes abnormally large,
     * discard it to prevent unlimited memory growth.
     */
    if (_rx_buffer.size() > _max_line_length) {
      _rx_buffer.clear();
      ++_invalid_lines;
    }
  }
};


/*
 * Register the class as a MADS source plugin.
 */
INSTALL_SOURCE_DRIVER(Serial_sourcePlugin, json)


/*
 * Standalone test executable.
 *
 * Usage:
 *   ./serial_source PORT [BAUD_RATE] [BLOCK_SIZE]
 */
int main(int argc, char const *argv[]) {

  if (argc < 2) {
    cerr << "Usage: "
         << argv[0]
         << " PORT [BAUD_RATE] [BLOCK_SIZE]"
         << endl;

    return 1;
  }

  try {
    Serial_sourcePlugin plugin;

    json params;
    json output;

    params["port"] = argv[1];
    params["baud_rate"] =
        argc >= 3 ? stoi(argv[2]) : 1000000;

    params["block_size"] =
        argc >= 4 ? stoul(argv[3]) : 1000;

    plugin.set_params(params);

    cout << "Waiting for one complete serial block..."
         << endl;

    while (output.empty()) {
      plugin.get_output(output);

      // Wait 1 ms before checking again.
      usleep(1000);
    }

    cout << "Block received successfully" << endl;
    cout << "Block ID: " << output["block_id"] << endl;
    cout << "Samples: " << output["sample_count"] << endl;

    if (output.contains("first_n")) {
      cout << "First n: " << output["first_n"] << endl;
    }

    if (output.contains("last_n")) {
      cout << "Last n: " << output["last_n"] << endl;
    }

    cout << "Invalid lines: "
         << output["invalid_lines_total"]
         << endl;

  } catch (const exception &error) {
    cerr << "Error: " << error.what() << endl;
    return 1;
  }

  return 0;
}