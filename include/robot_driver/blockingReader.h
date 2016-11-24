#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class blockingReader
{
public:
  blockingReader(boost::asio::serial_port& port, const size_t timeout):
   port(port),
   timeout(timeout),
   timer(port.get_io_service()),
   read_error(true) {}

  const bool readNext(int8_t& val);
private:
  boost::asio::serial_port& port;
  size_t timeout;
  int8_t c;
  boost::asio::deadline_timer timer;
  bool read_error;

  void read_complete(const boost::system::error_code& error, size_t bytes_transferred);
  void time_out(const boost::system::error_code& error);
};
