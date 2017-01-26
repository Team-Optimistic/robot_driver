#include "robot_driver/blockingReader.h"

void blockingReader::read_complete(const boost::system::error_code& error, int bytes_transferred)
{
  read_error = (error || bytes_transferred == 0);
  timer.cancel();
}

void blockingReader::time_out(const boost::system::error_code& error)
{
  if (error)
    return;

  port.cancel();
}

const bool blockingReader::readNext(char& val)
{
  val = 0;
  c = 0;

  port.get_io_service().reset();
  boost::asio::async_read(port, boost::asio::buffer(&val, 1),
    boost::bind(&blockingReader::read_complete,
                this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));

  timer.expires_from_now(boost::posix_time::milliseconds(timeout));
  timer.async_wait(boost::bind(&blockingReader::time_out,
                               this,
                               boost::asio::placeholders::error));

  port.get_io_service().run();

  if (!read_error)
    val = c;

  return !read_error;
}
