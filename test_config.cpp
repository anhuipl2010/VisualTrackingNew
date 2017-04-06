// 2017.3.29 by astraywu
// Use file "config.txt" to test class Config
#include "config.h"
#include <iostream>
int main()
{
	int port = 0;
	std::string ip_address;
	std::string username;
	std::string passwd;
	std::string key_not_in;
	bool enable_use;
	const char *filename_config = "test_config.txt";
	wrj::Config config_setting(filename_config);

	port = config_setting.Read("port", port);
	username = config_setting.Read("username", username);
	passwd = config_setting.Read("passwd", passwd);
	ip_address = config_setting.Read("ip_address", ip_address);
	key_not_in = config_setting.Read("key_not_in", key_not_in);
	//key_not_in = config_setting.Read("key_not_in");
	enable_use = config_setting.Read("enable_use", enable_use);
	std::cout << config_setting;
	std::cout << "port: " << port << std::endl;
	std::cout << "ip_address: " << ip_address << std::endl;
	std::cout << "username: " << username << std::endl;
	std::cout << "passwd: " << passwd << std::endl;
	std::cout << "key_not_in: " << key_not_in << std::endl;
	std::cout << "enable_use: " << enable_use << std::endl;

	return 0;
}