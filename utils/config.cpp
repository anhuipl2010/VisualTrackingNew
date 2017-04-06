#include "config.h"
#include <fstream>
#include <iostream>
#include <string>

namespace wrj {

Config::Config(const std::string filename, const std::string delimiter, const std::string delimiter_comment)
		: delimiter_(delimiter), delimiter_comment_(delimiter_comment) {
	std::ifstream fin(filename.c_str());
	if (!fin.is_open()) throw File_not_found(filename);
	fin >> (*this);
}

Config::Config()
	: delimiter_(std::string(1, '=')), delimiter_comment_(std::string(1, '#')) {
	// Construct a Config without a file
}

bool Config::KeyExist(const std::string &key) const {
	return (contents_.find(Trim(key)) != contents_.cend());
}


void Config::Remove(const std::string &key) {
	contents_.erase(contents_.find(Trim(key)));
}

bool Config::FileExist(const std::string filename) const {
	bool exist = false;
	std::ifstream fin(filename.c_str());
	if (fin.is_open()) {
		exist = true;
	}
	fin.close();
	return exist;
}

// static
std::string Config::Trim(const std::string &s) {
	// Remove leading and tailing whitespace
	std::string str(s);
	static const char whitespace[] = " \n\t\v\r\f";
	str.erase(0, str.find_first_not_of(whitespace));
	str.erase(str.find_last_not_of(whitespace) + 1U);
	return str;
}

std::ostream &operator<<(std::ostream &os, const Config &config) {
	for (Config::mapcit cit = config.contents_.cbegin(); cit != config.contents_.end(); ++cit) {
		os << cit->first << " " << config.delimiter() << " " << cit->second << std::endl;
	}
	return os;
}

std::istream &operator >> (std::istream &is, Config &config) {
	// Load a Config from is
	// Read in keys and values, keeping internal whitespace

	typedef std::string::size_type pos;
	const std::string &kDelimiter = config.delimiter_;
	const std::string &kDelimiterComment = config.delimiter_comment_;
	const pos kSkip = kDelimiter.length();

	std::string line;
	while (std::getline(is, line)) {
		// Skip empty lines
		if (line.empty()) continue;
		line = line.substr(0, line.find(kDelimiterComment));
		pos delim_pos = line.find(kDelimiter);
		// Not find delimiter, skip
		if (delim_pos == std::string::npos) continue;
		const std::string key = line.substr(0, delim_pos);
		const std::string value = line.substr(delim_pos + kSkip);
		const std::string k = Config::Trim(key);
		const std::string v = Config::Trim(value);
		if (k.empty() || v.empty()) continue;
		config.contents_[k] = v;
	}
	return is;
}

void Config::ReadFile(const std::string filename, const std::string delimiter, const std::string delimiter_comment) {
	delimiter_ = delimiter;
	delimiter_comment_ = delimiter_comment;
	std::ifstream fin(filename);
	if (!fin.is_open()) throw File_not_found(filename);
	fin >> (*this);
}

}