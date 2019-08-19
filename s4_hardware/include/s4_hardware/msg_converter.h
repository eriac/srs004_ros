// OSS
#include <sstream>
#include <string>
// ROS
#include <ros/ros.h>
// SRS004
#include "s4_msgs/CANCode.h"
#include "s4_msgs/SerialCode.h"

int hexconvert(char c1, char c0){
	char value1, value0 =0;

	if('0'<=c1&&c1<='9')value1=c1-'0';
	else if('A'<=c1&&c1<='F')value1=c1-'A'+10;
	else if('a'<=c1&&c1<='f')value1=c1-'a'+10;
	else return -1;

	if('0'<=c0&&c0<='9')value0=c0-'0';
	else if('A'<=c0&&c0<='F')value0=c0-'A'+10;
	else if('a'<=c0&&c0<='f')value0=c0-'a'+10;
	else return -1;

	return value1*16+value0;
}

int hex_convert(std::string input){
	int data_size=input.size();
	int output=0;
	if(data_size==0)return -1;
	for(int i=0;i<data_size;i++){
		if     ('0'<=input[i] && input[i]<='9')output|=(input[i]-'0')<<((data_size-i-1)*4);
		else if('A'<=input[i] && input[i]<='F')output|=(input[i]-'A'+10)<<((data_size-i-1)*4);
		else return -1;
	}
	return output;
}

int available(std::string s0){
	for(int i=0;i<s0.size();i++){
		if(!('0'<=s0[i]&&s0[i]<='9' ||'A'<=s0[i]&&s0[i]<='Z'))return 0;
	}
	return 1;
}

int cancode_to_serialcode(s4_msgs::SerialCode *serialcode, s4_msgs::CANCode cancode) {
  char tmp_data[2];
  s4_msgs::SerialCode midcode;
  midcode.command[0] = "CANLINK";
  midcode.command[1] = cancode.channel;
  midcode.option[0] = "ID";
  sprintf(tmp_data, "%d", cancode.id);
  midcode.suboption[0] = std::string(tmp_data);
  midcode.option[1] = "COM";
  sprintf(tmp_data, "%d", cancode.com);
  midcode.suboption[1] = std::string(tmp_data);
  if (cancode.remote) midcode.option[2] = "REMOTE";
  for (int i = 0; i < cancode.length; i++) {
    midcode.data[i] = cancode.data[i];
  }
  midcode.datanum = cancode.length;
  *serialcode = midcode;
  return 0;
}

int serialcode_to_serial(std::string *outstring, s4_msgs::SerialCode serialcode) {
  char tmp_data[2];
  std::string midstring = "";
  midstring = "#" + serialcode.command[0];
  for (int i = 1; i < 4; i++) {
    if (serialcode.command[i] != "") {
      midstring += "." + serialcode.command[i];
    } else
      break;
  }
  for (int i = 0; i < 4; i++) {
    if (serialcode.option[i] != "") {
      midstring += "-" + serialcode.option[i];
      if (serialcode.suboption[i] != "") {
        midstring += "." + serialcode.suboption[i];
      }
    } else
      break;
  }
  if (serialcode.datanum != 0) {
    midstring += ":";
    for (int i = 0; i < serialcode.datanum; i++) {
      sprintf(tmp_data, "%02X", serialcode.data[i]);
      midstring += std::string(tmp_data);
    }
  }
  midstring += ";";
  *outstring = midstring;
  return 0;
}

int serial_to_serialcode(s4_msgs::SerialCode *serialcode, std::string instring) {
  /*return
  0:corrent
  -1:unavailable char
  -2:too long char
  -3:syntax error
  */
  std::string command = "";
  std::string option = "";
  std::string data = "";

  int f1 = instring.find(":");
  if (f1 >= 0) {
    data = instring.substr(f1 + 1, instring.size() - f1 - 1);
    int f2 = instring.find("-");
    if (f2 >= 0) {
      command = instring.substr(0, f2);
      option = instring.substr(f2, f1 - f2);
    } else
      command = instring.substr(0, f1);
  } else {
    int f2 = instring.find("-");
    if (f2 >= 0) {
      command = instring.substr(0, f2);
      option = instring.substr(f2, instring.size() - f2);
    } else
      command = instring;
  }
  // printf("comm %s\n",command.c_str());
  // printf("optn %s\n",option.c_str());
  // printf("data %s\n",data.c_str());

  std::string split_command[4] = {""};
  std::string split_option[4] = {""};
  std::string split_suboption[4] = {""};
  int split_data[20] = {0};
  int data_size = 0;
  for (int i = 0; i < 4; i++) {
    int f3 = command.find(".");
    if (f3 >= 0) {
      split_command[i] = command.substr(0, f3);
      command = command.substr(f3 + 1, command.size() - f3 - 1);
    } else {
      split_command[i] = command;
      break;
    }
  }
  for (int i = 0; i < 4; i++) {
    if (option[0] == '-') {
      int f4 = option.find("-", 1);
      std::string tmp = "";
      if (f4 >= 0) {
        tmp = option.substr(1, f4 - 1);
        option = option.substr(f4, option.size() - f4);
      } else {
        tmp = option.substr(1, option.size() - 1);
        option.clear();
      }
      int f5 = tmp.find(".");
      if (f5 >= 0) {
        split_option[i] = tmp.substr(0, f5);
        split_suboption[i] = tmp.substr(f5 + 1, tmp.size() - f5 - 1);
      } else
        split_option[i] = tmp;
    } else
      break;
  }
  if (data.size() % 2 == 0) {
    data_size = data.size() / 2;
    for (int i = 0; i < data_size && i < 20; i++) {
      int ret = hexconvert(data[0], data[1]);
      if (ret >= 0) {
        split_data[i] = ret;
        data = data.substr(2, data.size() - 2);
      } else {
        // printf("data %i is not number\n",i);
        return -3;
      }
    }
  } else {
    // printf("data is even\n");
    return -3;
  }

  for (int i = 0; i < 4; i++) {
    if (available(split_command[i]))
      serialcode->command[i] = split_command[i];
    else
      return -1;
  }
  for (int i = 0; i < 4; i++) {
    if (available(split_option[i]) && available(split_suboption[i])) {
      serialcode->option[i] = split_option[i];
      serialcode->suboption[i] = split_suboption[i];
    } else
      return -1;
  }
  for (int i = 0; i < 20; i++) serialcode->data[i] = split_data[i];
  serialcode->datanum = data_size;

  // for(int i=0;i<4;i++)printf("c%i %s\n",i,split_command[i].c_str());
  // for(int i=0;i<4;i++)printf("o%i %s s%i
  // %s\n",i,split_option[i].c_str(),i,split_suboption[i].c_str()); for(int
  // i=0;i<10;i++)printf("d%i %02X\n",i,split_data[i]);
  return 0;
}
