#pragma once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>


class ParameterServer
{
public:
	template <class T>
	bool param(std::string path, T & v, T def);

	bool param(std::string path, YAML::Node & v);

	template <class T>
	bool param(std::string path, T & v);

	// TODO write

	YAML::Node top;

	// append config
	bool append(std::string path, std::string prefix = "");

	YAML::Node solve(std::string path);

	YAML::Node solveparent(std::string path);

};

class ParameterSpace
{
public:
	ParameterSpace(ParameterServer & srv, std::string prefix): srv_(srv), prefix_(prefix)
	{	
		if(!prefix_.empty() && prefix_[prefix_.size()-1] != '/')
			prefix_ += '/';
	}	

	ParameterSpace(ParameterSpace & other, std::string prefix): srv_(other.srv_), prefix_(other.prefix_ + prefix)
	{	
		if(!prefix_.empty() && prefix_[prefix_.size()-1] != '/')
			prefix_ += '/';
	}	

	template <class T>
	bool param(std::string path, T & v, T def)
	{
		return srv_.param(path[0] == '/' ? path : prefix_ + path,v,def);
	}

	bool param(std::string path, YAML::Node & v)
	{
		return srv_.param(path[0] == '/' ? path : prefix_ + path,v);
	}

	template <class T>
	bool param(std::string path, T & v)
	{
		return srv_.param(path[0] == '/' ? path : prefix_ + path,v);
	}

	ParameterServer & srv_;
	std::string prefix_; 
};