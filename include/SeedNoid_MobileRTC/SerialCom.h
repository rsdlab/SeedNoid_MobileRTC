#ifndef _SERIAL_COM_H_
#define _SERIAL_COM_H_

//Boost include
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <vector>
#include <stdlib.h>


//using namespace std;
//using namespace boost::asio;

class SerialCom
{
 public:

  bool isCanceled;

  //std::vector<unsigned char> receiveData;


  //----------Constructor----------
  SerialCom()
    :ioService(), serialPort(ioService),deadlineTimer(ioService){}


  //----------Function----------
  //----------SerialOpen Function----------
  bool serialOpen(const std::string portName, const unsigned int baudRate)
  {
    serialPort.open(portName);
    if(!serialPort.is_open())
      {
	return false;
      }
    else
      {
	serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
      	return true;
      }
  }
  

  //----------SerialClose Function----------
  bool serialClose()
  {
    if(!serialPort.is_open())
      {
	return false;
      }
    else
      {
	serialPort.close();
      }
    return true;
  }

  //----------Write Handler----------
  void writeHandler(const boost::system::error_code& errorCode, std::size_t bytesTransferred)
  {
    if(errorCode)
      {
	std::cout << "Write failed : " << errorCode.message() << std::endl;
      }
  }


  //----------SerialWrite Function----------
  void serialWrite(std::vector<unsigned char>& sendData)
  {
    serialPort.write_some(boost::asio::buffer(sendData));
  }


  //----------Serial AsyncWrite Function---------
  void serialAsyncWrite(std::vector<unsigned char>& sendData)
  {
    serialPort.async_write_some(boost::asio::buffer(sendData), boost::bind(&SerialCom::writeHandler, this, _1, _2));

    ioService.reset();
    ioService.run();
  }


  //----------Read Handler----------
  void readHandler(const boost::system::error_code& errorCode, std::size_t bytesTransferred)
  {
    if(errorCode == boost::asio::error::operation_aborted)
      {
	std::cout << "Timeout"<< std::endl;
      }
    else
      {
	deadlineTimer.cancel();
	isCanceled = true;

	if(errorCode)
	  {
	    std::cout << "Another Error : " << errorCode.message() << std::endl;
	  }
      }  
  }
  
  
  //----------SerialRead Function----------
  void serialRead(std::vector<unsigned char>& receiveData,unsigned char size = 1)
  {  
    receiveData.resize(size);
    serialPort.read_some(boost::asio::buffer(receiveData,size));
  } 
  
  
   //----------Serial AsyncRead Function---------
  void serialAsyncRead(std::vector<unsigned char>& receiveData, unsigned char size = 1, unsigned char timeOut = 1)
  {
    receiveData.resize(size);
    boost::asio::async_read(serialPort,boost::asio::buffer(receiveData, size), boost::asio::transfer_all(), boost::bind(&SerialCom::readHandler, this, _1, _2));

    deadlineTimer.expires_from_now(boost::posix_time::seconds(timeOut));
    deadlineTimer.async_wait(boost::bind(&SerialCom::onTimer, this, _1));
    
    ioService.reset();
    ioService.run();
  }

  //----------On Timer Function----------
  void onTimer(const boost::system::error_code &errorCode)
  {
    if(!errorCode && !isCanceled)
      {
	serialPort.cancel();
      }
  }


  //----------Flush Serial Port----------
  void flushSerialPort()
  {
    tcflush(serialPort.lowest_layer().native_handle(),TCIOFLUSH);
  }
  
  
 private:

  boost::asio::io_service ioService;
  boost::asio::serial_port serialPort;
  boost::asio::deadline_timer deadlineTimer;

};

#endif     //_SERIAL_COM_H_
