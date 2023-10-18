#pragma once

#include <mutex>
#include <thread>

#include <ros/ros.h>

class SafeAndFastPublisher {
 private:
  ros::Publisher pub_;
  std::mutex m_;
  template <typename T>
  void publish(T msg) {
    std::unique_lock<std::mutex> lck(m_, std::defer_lock);
    if (lck.try_lock()) {
      pub_.publish(msg);
    }
  }
 public:
  SafeAndFastPublisher();
  SafeAndFastPublisher& operator=(const ros::Publisher& r);

  template <typename T>
  void try_publish(const T& message) {
    std::thread t(&SafeAndFastPublisher::publish<T>, this, message);
    t.detach();
  }
};
