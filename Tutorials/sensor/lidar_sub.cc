/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

ignition::transport::Node node;
std::string topic_pub = "/stop";
ignition::msgs::StringMsg data;
//data.set_data("HELLO");
auto pub = node.Advertise<ignition::msgs::StringMsg>(topic_pub);

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::LaserScan &_msg)
{
  std::cout << "lidar_data: " << _msg.ranges_size() << std::endl;
  for (int i = 0; i < 10; i++)
  {
    std::cout << "r: " << _msg.ranges(i)<< std::endl;
    /*
    if (_msg.ranges(i) > 4.0)
    {
      break;
    }
    if (i == 9)
    {
      std::cout << "publish stop" << std::endl;
    }
    
    std::cout << "publish stop" << std::endl;
    data.set_data("HELLO");
    pub.Publish(data);
    */
  }
  data.set_data("HELLO");
  pub.Publish(data);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  
  std::string topic = "/lidar";

  if (!pub)
  {
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    return -1;
  }

  
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
