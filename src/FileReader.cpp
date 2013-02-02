#ifndef FILEREADER_CPP_
#define FILEREADER_CPP_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "FileReader.h"

FileReader::FileReader(std::string name){
	this->file = new std::ifstream(name.c_str());
}

FileReader::~FileReader(){
	this->file->close();
}

void FileReader::splitString(std::string str, std::string splitter, std::vector<std::string> * strings){
	size_t pos=0;

	pos = str.find(splitter,0);				// find the end of the first word
	while(pos != std::string::npos){
		if(pos>0){
			strings->push_back(str.substr(0,pos));	// add the word to the vector
		}
		str = str.substr(pos+splitter.size());	// jump to the next non-splitter word
		pos = str.find(splitter,0);				// find the end of the next word
	}
	strings->push_back(str);	// add the last word
}

std::string FileReader::cleanString(std::string str){
	if(str.size() < 1){
		return str;
	}

//	std::cout << "removing leading spaces\n";
	while(str.substr(0,1).compare(" ") == 0){
		str = str.substr(1,str.size());
	}

//	std::cout << "removing tailing spaces\n";
	while(str.substr(str.size()-1,str.size()).compare(" ") == 0){
		str = str.substr(0,str.size()-1);
	}

	return str;
}

bool FileReader::good(){
	return this->file->good();
}

bool FileReader::is_open(){
	return this->file->is_open();
}

bool FileReader::readLine(std::vector<std::string> * strings){
	if(!this->good()){
		return false;
	}
	std::string line;
	std::getline(*file,line);
	if(!this->good()){
		return false;
	}

	line = cleanString(line);

//	std::cout << "splitting string\n";
	FileReader::splitString(line," ",strings);

	return true;
}

//int main(int argc, char**argv){
//
//	if(argc<2){
//		std::cout << "usage: FileReader <textfile name>";
//	}
//
//	FileReader fr(argv[1]);
//
//	while(fr.good()){
//		std::vector<std::string> vettore;
//		fr.readLine(&vettore);
//
////		std::cout << "read a line\n";
//
//		std::cout << "<frase>\n";
//		for(unsigned int i=0; i<vettore.size(); i++){
//			std::cout << vettore[i] << '\n';
//		}
//		std::cout << "</frase>\n\n";
//	}
//
//	return 0;
//}

#endif /* FILEREADER_CPP_ */
