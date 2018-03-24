#ifndef MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#define MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
#pragma once
#include <unordered_map>
#include <memory>
#include <iostream>

namespace rrts{
namespace decision {
using std::string;

#define BLACKBOARD_TYPE_REGISTER(type)                   \
public:                                                  \
void Set##type(std::string key, type value){             \
  type##s_[key]= value;                                  \
}                                                        \
bool Get##type(std::string key) const{                   \
  return Get<type>(type##s_, key);                       \
}                                                        \
private:                                                 \
  std::unordered_map<std::string, type> type##s_;        \


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard(){};
  ~Blackboard() {};
  void MapShow(){};
 private:
  template<typename T>
  T Get(const std::unordered_map<std::string, T> &map, std::string key) const{
    auto search = map.find(key);
	if (search != map.end())
	{
		return search->second;
	}
	std::cerr << "Failed to find value with key: " << key.c_str() << std::endl;
	return T();
  };

  BLACKBOARD_TYPE_REGISTER(bool);
  BLACKBOARD_TYPE_REGISTER(int);
  BLACKBOARD_TYPE_REGISTER(float);
  BLACKBOARD_TYPE_REGISTER(double);
//  BLACKBOARD_TYPE_REGISTER(string);
};
} //namespace decision
} //namespace rrts
#endif //MODULE_DECISION_BEHAVIOR_TREE_BLACKBOARD_H
