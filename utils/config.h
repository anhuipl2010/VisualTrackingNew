// 2017.3.29 by astraywu
// src: [C++编写Config类读取配置文件]

#ifndef CONFIG_H__
#define CONFIG_H__

#include <iostream>
#include <map>
#include <string>
#include <sstream>
namespace wrj {
class Config {
protected:
	std::map<std::string, std::string> contents_;	// extracted keys and values
	std::string delimiter_;	// separator between key and vale
	std::string delimiter_comment_;	// separator between value and comment
	typedef std::map<std::string, std::string>::iterator mapit;
	typedef std::map<std::string, std::string>::const_iterator mapcit;

	// Method
public:
	Config(const std::string filename, const std::string delimiter = "=", const std::string delimiter_comment = "#");
	Config();
	template<typename T> T Read(const std::string &key) const;	//! The template shouldn't be correct
	template<typename T> T Read(const std::string &key, const T &value_default) const;	// Search the key and read value or optional default value
	template<typename T> bool ReadInto(T &var, const std::string &key);
	template<typename T> bool ReadInto(T &var, const std::string &key, const T &value_default);
	bool FileExist(const std::string filename) const ;
	void ReadFile(const std::string filename, const std::string delimiter = "=", const std::string delimiter_comment = "#");

	// Check whether key exists in configuration
	bool KeyExist(const std::string &key) const;	

	// Modify keys and value
	template<typename T> void Add(const std::string &key, const T &value);
	void Remove(const std::string &key);

	// Check or change configuration syntax
	std::string delimiter() const { return delimiter_; }
	std::string delimiter_comment() const{ return delimiter_comment_; }
	std::string set_delimiter(const std::string &delimiter) { const std::string old_delimiter = delimiter_; delimiter_ = delimiter; return old_delimiter; }
	std::string set_delimiter_comment(const std::string &delimiter_comment) { const std::string old_delimiter_comment = delimiter_comment_; delimiter_comment_ = delimiter_comment; return old_delimiter_comment; }

	// Write or read configuration
	friend std::ostream &operator<<(std::ostream &os, const Config &cf);
	friend std::istream &operator>>(std::istream &is, Config &cf);

protected:
	template<typename T> static std::string T_as_string(const T &t);
	template<typename T> static T string_as_T(const std::string &s);
	static std::string Trim(const std::string &s);

	// exception types
	// only print the default error, int nested class should be only declared in header file
public:
	struct File_not_found {
		std::string filename_;
		File_not_found(const std::string filename = std::string()) : filename_(filename) { std::cerr << "ERROR: Open file failed: " << filename_ << std::endl; }
	};
	struct Key_not_found {
		std::string key_;
		Key_not_found(const std::string key = std::string()) : key_(key) { std::cerr << "ERROR: key is not found: " << key_ << std::endl; }
	};
};

// static
template<typename T>
std::string Config::T_as_string(const T &t) {
	// Convert T to a string
	// Type T must support << operator
	std::ostringstream oss;
	oss << t;
	return oss.str();
}

// static
template<typename T>
T Config::string_as_T(const std::string &s) {
	// Convert from a string to T
	// Type T must support >> operator
	T t;
	std::istringstream iss(s);
	iss >> t;
	return t;
}


// static
//! static member template should add inline, otherwise it occurs mistake in compile.
template<>
inline std::string Config::string_as_T<std::string>(const std::string &s) {
	// Do nothing
	return s;
}

// static
template<>
inline bool Config::string_as_T<bool>(const std::string &s) {
	// Convert from a string to a bool
	// Interpret "false", "no", "n", "0", "none", "F" as false
	// Interpret "true", "yes", "y", "1" or anything else as true
	bool b = true;
	std::string sup = s;
	for (std::string::iterator it = sup.begin(); it != sup.end(); ++it) {
		*it = toupper(*it);
	}
	if (sup == std::string("FALSE") ||
		sup == std::string("NO") ||
		sup == std::string("N") ||
		sup == std::string("0") ||
		sup == std::string("NONE") ||
		sup == std::string("F")) {
		b = false;
	}
	return b;
}

template<typename T>
T Config::Read(const std::string &key) const {
	// Read the value corresponding to key
	mapcit cit = contents_.find(key);
	if (cit == contents_.cend()) throw Key_not_found(key);
	return string_as_T<T>(cit->second);
}

template<typename T>
T Config::Read(const std::string &key, const T &value_default) const {
	// Return the value corresponding to key or given default value
	// if key is not found
	mapcit cit = contents_.find(key);
	if (cit == contents_.cend()) return value_default;
	return string_as_T<T>(cit->second);
}

template<typename T>
bool Config::ReadInto(T &var, const std::string &key) {
	// Get the value corresponding to key and store in var
	// Return true if key is found
	// Otherwise leave var untouched
	mapcit cit = contents_.find(key);
	bool found = (cit != contents_.cend());
	if (found) {
		var = string_as_T<T>(cit->second);
	}
	return found;
}

template<typename T>
bool Config::ReadInto(T &var, const std::string &key, const T &value_default) {
	// Get the value corresponding to key and store in var
	// Return true if key is found
	// Otherwise set var to given default value
	mapcit cit = contents_.find(key);
	bool found = (cit != contents_.cend());
	if (found) {
		var = string_as_T<T>(cit->second);
	}	else {
		var = value_default;
	}
	return found;
}

template<class T>
void Config::Add(const std::string &key, const T &value) {
	// Add a key with given value
	std::string v = T_as_string(value);
	std::string k = key;
	contents_[Trim(k)] = Trim(v);
}
}	// namespace wrj

#endif	// CONFIG_H__