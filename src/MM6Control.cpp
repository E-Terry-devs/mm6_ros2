#include "zbCom/zbCom.hpp"
#include "mm6_ros2/mm6_control.hpp"


//defines for MiM6
#define MODE_FIXED_IP 0
#define MODE_DISCOVERY 1
#define IP_MM6 "10.42.0.70"

controller macs;

MM6Control::MM6Control() : Node("mm6control_node")
{
  topic_cmd_vel = std::string("/cmd_vel");

  initialiseMM6();

  RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_cmd_vel.c_str());
  sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
    topic_cmd_vel,10,std::bind(&MM6Control::velCmdCallback,this,std::placeholders::_1));

}

MM6Control::~MM6Control() 
{

}

void MM6Control::velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if(!msg)
  {
    return;
  }
  macs.write_sdo(0x2201, 0x01, split4(msg->linear.x));
  macs.write_sdo(0x2201, 0x01, split4(msg->angular.z));

}

void MM6Control::initialiseMM6()
{
	int mode= MODE_FIXED_IP;
  std::string ip= IP_MM6;

  // enable the ZbCom logging
  enable_log(log_level_error);

  // initialize random number generator
  srand((int)time(NULL));

  try {
      // controller macs;
      macs = controller(ip, 23);

      std::vector<uint8_t> answer_data;

      // establish a TCP connection to the controller
      macs.connect();

      // read the device name from the SDO with index 0x1008 and subindex 0x00
      std::vector<uint8_t> device_name;
      macs.read_sdo(0x1008, 0x00, &device_name);
      std::cout << "Device Name: ";
      std::string device_name_string(device_name.begin(), device_name.end());
      for (auto &&byte_i : device_name)
          std::cout << byte_i;
      std::cout << std::endl;

      // read the hardware version from the SDO with index 0x1009 and subindex 0x00
      std::vector<uint8_t> hw_version;
      macs.read_sdo(0x1009, 0x00, &hw_version);
      std::cout << "HW Version: " << concatenate4(hw_version, 0) << std::endl;

      // read the software version from the SDO with index 0x100a and subindex 0x00
      std::vector<uint8_t> sw_version;
      macs.read_sdo(0x100a, 0x00, &sw_version);
      std::cout << "SW Version: ";
      for (auto &&byte_i : sw_version)
          std::cout << byte_i;
      std::cout << std::endl;

      // read the serialnumber from the SDO with index 0x1018 and subindex 0x04
      std::vector<uint8_t> serialnumber;
      macs.read_sdo(0x2215, 0x02, &serialnumber);
      std::cout << "Serialnumber: " << concatenate4(serialnumber, 0) << std::endl;

      // stop execution of any program on the controller
      macs.write_sdo(0x2211, 0x01, split4(0));

      // write a random number to the user data
      uint32_t random_number_write = (uint32_t)(rand() % 0xFFFFFFFF);
      macs.write_sdo(0x2201, 0x01, split4(random_number_write));

      // read the previously written number back and verify its correct
      std::vector<uint8_t> random_number_read_vec;
      macs.read_sdo(0x2201, 0x01, &random_number_read_vec);
      uint32_t random_number_read = concatenate4(random_number_read_vec, 0);
      if (random_number_read == random_number_write) {
          std::cout << "SDO read/write user parameter: SUCCESS" << std::endl;
      } else {
          std::cout << "SDO read/write user parameter: FAIL" << std::endl;
      }

      // delete all arrays on the controller
      macs.write_sdo(0x2210, 0x01, split4(3));
      macs.write_sdo(0x2210, 0x03, split4(0xff55));

      // verify all the arrays are deleted
      std::vector<uint8_t> array_count;
      macs.read_sdo(0x21ff, 0x06, &array_count);
      if (concatenate4(array_count, 0) == 0) {
          std::cout << "Array delete: SUCCESS" << std::endl;
      } else {
          std::cout << "Array delete: FAIL" << std::endl;
      }

      // write an array with random numbers through the general array access (0x21FF)
      std::vector<uint8_t> array_no = split4(0);
      int array_len = 3000;
      std::vector<uint8_t> array_len_split = split4(array_len);
      std::vector<uint8_t> array_data_write(array_len);
      for (auto &&byte_i : array_data_write) {
          byte_i = rand() % 0xFFFF;
      }
      macs.write_sdo(0x21FF, 0x01, array_no);
      macs.write_sdo(0x21FF, 0x02, array_len_split);
      macs.write_sdo(0x21FF, 0x03, array_data_write);

      // read the array back and verify its content
      std::vector<uint8_t> array_data_read;
      macs.write_sdo(0x21FF, 0x01, array_no);
      macs.read_sdo(0x21FF, 0x03, &array_data_read);
      bool answer_is_equal = true;
      for (auto &&i : array_data_read) {
          if (array_data_read[i] != array_data_write[i]) {
              answer_is_equal = false;
              break;
          }
      }
      if (answer_is_equal)
          std::cout << "Array read/write: SUCCESS" << std::endl;
      else
          std::cout << "Array read/write: FAIL" << std::endl;

      // delete all arrays on the controller
      macs.write_sdo(0x2210, 0x01, split4(3));
      macs.write_sdo(0x2210, 0x03, split4(0xff55));
	  } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }  
}