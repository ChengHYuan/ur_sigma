// Generated by gencpp from file dmp/SetActiveDMP.msg
// DO NOT EDIT!


#ifndef DMP_MESSAGE_SETACTIVEDMP_H
#define DMP_MESSAGE_SETACTIVEDMP_H

#include <ros/service_traits.h>


#include <dmp/SetActiveDMPRequest.h>
#include <dmp/SetActiveDMPResponse.h>


namespace dmp
{

struct SetActiveDMP
{

typedef SetActiveDMPRequest Request;
typedef SetActiveDMPResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetActiveDMP
} // namespace dmp


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dmp::SetActiveDMP > {
  static const char* value()
  {
    return "10d64adb0a08cbb7afbd425801f828e5";
  }

  static const char* value(const ::dmp::SetActiveDMP&) { return value(); }
};

template<>
struct DataType< ::dmp::SetActiveDMP > {
  static const char* value()
  {
    return "dmp/SetActiveDMP";
  }

  static const char* value(const ::dmp::SetActiveDMP&) { return value(); }
};


// service_traits::MD5Sum< ::dmp::SetActiveDMPRequest> should match
// service_traits::MD5Sum< ::dmp::SetActiveDMP >
template<>
struct MD5Sum< ::dmp::SetActiveDMPRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dmp::SetActiveDMP >::value();
  }
  static const char* value(const ::dmp::SetActiveDMPRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dmp::SetActiveDMPRequest> should match
// service_traits::DataType< ::dmp::SetActiveDMP >
template<>
struct DataType< ::dmp::SetActiveDMPRequest>
{
  static const char* value()
  {
    return DataType< ::dmp::SetActiveDMP >::value();
  }
  static const char* value(const ::dmp::SetActiveDMPRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dmp::SetActiveDMPResponse> should match
// service_traits::MD5Sum< ::dmp::SetActiveDMP >
template<>
struct MD5Sum< ::dmp::SetActiveDMPResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dmp::SetActiveDMP >::value();
  }
  static const char* value(const ::dmp::SetActiveDMPResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dmp::SetActiveDMPResponse> should match
// service_traits::DataType< ::dmp::SetActiveDMP >
template<>
struct DataType< ::dmp::SetActiveDMPResponse>
{
  static const char* value()
  {
    return DataType< ::dmp::SetActiveDMP >::value();
  }
  static const char* value(const ::dmp::SetActiveDMPResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DMP_MESSAGE_SETACTIVEDMP_H
