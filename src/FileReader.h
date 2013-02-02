#ifndef FILEREADER_H_
#define FILEREADER_H_

#include <iostream>
#include <fstream>
#include <vector>

class FileReader{
	std::ifstream * file;
public:
	FileReader(std::string name);
	~FileReader();
	bool readLine(std::vector<std::string> * strings);
	static void splitString(std::string str, std::string splitter, std::vector<std::string> * strings);
	static std::string cleanString(std::string str);
	bool good();
	bool is_open();
};


#endif /* FILEREADER_H_ */
