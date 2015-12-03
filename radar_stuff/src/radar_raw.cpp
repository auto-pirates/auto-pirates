#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>
#include <exception>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

#define LINES_PER_ROTATION  (2048) // BR radars can generate up to 4096 lines per rotation, but use only 2048
#define RETURNS_PER_LINE     (512) // BR radars generate 512 separate values per range, at 8 bits each

using namespace boost::interprocess;

//from plugin
struct scan_line {
    int range;                        // range of this scan line in decimeters
    long age;                   // how old this scan line is. We keep old scans on-screen for a while
    uint8_t data[RETURNS_PER_LINE + 1]; // radar return strength, data[512] is an additional element, accessed in drawing the spokes
    uint8_t history[RETURNS_PER_LINE + 1]; // contains per bit the history of previous scans.
       //Each scan this byte is left shifted one bit. If the strength (=level) of a return is above the threshold
       // a 1 is added in the rightmost position, if below threshold, a 0.
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_raw");
  ros::NodeHandle n; 
  ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("raw_radar_image", 20);

  std::pair<scan_line*, managed_shared_memory::size_type> res;
  interprocess_mutex *mtx = NULL;
  managed_shared_memory managed_shm(open_only, "Boost");

  try {

    mtx = managed_shm.find_or_construct<interprocess_mutex>("mtx")(); 
    res = managed_shm.find<scan_line>("ScanLines");
  }
  catch (std::exception& e) {
    std::cout << e.what() << '\n';
  }
  

  ros::Rate loop_rate(2);

  int got_new_data = 0;

  scan_line local_scanlines[2][LINES_PER_ROTATION];

  //std::cout << sizeof(local_scanlines) << '\n';
  //std::cout << sizeof(((struct scan_line*)0)->data) << '\n';

  while (ros::ok())
  {

    std_msgs::UInt8MultiArray radar_image;
    
    if (mtx) {   
      
      
      scoped_lock<interprocess_mutex> lock(*mtx); //try to lock the mutex
      if (lock) {
        //read shared memory
        
        if(res.first) {
          memcpy(local_scanlines, res.first, sizeof(local_scanlines));
          got_new_data = 1;
        }     
        
      }
      
      
    } //unlock
    
    
    if (got_new_data) {
      //
      radar_image.data.clear();
      for (int i = 0; i < LINES_PER_ROTATION; i++) {
        radar_image.data.insert(radar_image.data.end(), &local_scanlines[0][i].data[0], &local_scanlines[0][i].data[RETURNS_PER_LINE]);
        //radar_image.data.push_back(local_scanlines[0][i]);
      }
      std::cout << radar_image.data.size() << '\n';
      pub.publish(radar_image);
      got_new_data = 0;
    }
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
