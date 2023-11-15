// Generated by gencpp from file dmp/LearnDMPFromDemo.msg
// DO NOT EDIT!


#ifndef DMP_MESSAGE_LEARNDMPFROMDEMO_H
#define DMP_MESSAGE_LEARNDMPFROMDEMO_H

#include <ros/service_traits.h>


#include <dmp/LearnDMPFromDemoRequest.h>
#include <dmp/LearnDMPFromDemoResponse.h>


namespace dmp
{

struct LearnDMPFromDemo
{

typedef LearnDMPFromDemoRequest Request;
typedef LearnDMPFromDemoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct LearnDMPFromDemo
} // namespace dmp


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dmp::LearnDMPFromDemo > {
  static const char* value()
  {
    return "3ba13cfa47585560a2fd9cc202efdbff";
  }

  static const char* value(const ::dmp::LearnDMPFromDemo&) { return value(); }
};

template<>
struct DataType< ::dmp::LearnDMPFromDemo > {
  static const char* value()
  {
    return "dmp/LearnDMPFromDemo";
  }

  static const char* value(const ::dmp::LearnDMPFromDemo&) { return value(); }
};


// service_traits::MD5Sum< ::dmp::LearnDMPFromDemoRequest> should match
// service_traits::MD5Sum< ::dmp::LearnDMPFromDemo >
template<>
struct MD5Sum< ::dmp::LearnDMPFromDemoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dmp::LearnDMPFromDemo >::value();
  }
  static const char* value(const ::dmp::LearnDMPFromDemoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dmp::LearnDMPFromDemoRequest> should match
// service_traits::DataType< ::dmp::LearnDMPFromDemo >
template<>
struct DataType< ::dmp::LearnDMPFromDemoRequest>
{
  static const char* value()
  {
    return DataType< ::dmp::LearnDMPFromDemo >::value();
  }
  static const char* value(const ::dmp::LearnDMPFromDemoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dmp::LearnDMPFromDemoResponse> should match
// service_traits::MD5Sum< ::dmp::LearnDMPFromDemo >
template<>
struct MD5Sum< ::dmp::LearnDMPFromDemoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dmp::LearnDMPFromDemo >::value();
  }
  static const char* value(const ::dmp::LearnDMPFromDemoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dmp::LearnDMPFromDemoResponse> should match
// service_traits::DataType< ::dmp::LearnDMPFromDemo >
template<>
struct DataType< ::dmp::LearnDMPFromDemoResponse>
{
  static const char* value()
  {
    return DataType< ::dmp::LearnDMPFromDemo >::value();
  }
  static const char* value(const ::dmp::LearnDMPFromDemoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DMP_MESSAGE_LEARNDMPFROMDEMO_H
