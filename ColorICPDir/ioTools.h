#pragma once

#include <iostream>
#include "io.h"
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::endl;

void getJustCurrentFile(string path, vector<string>& files) {
	//文件句柄
	long long hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				;
			}
			else {
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
struct FileParts
{
	std::string path;
	std::string name;
	std::string ext;
};

FileParts fileparts(std::string filename)
{
	int idx0 = filename.rfind("/");
	int idx1 = filename.rfind(".");

	FileParts fp;
	fp.path = filename.substr(0, idx0 + 1);
	fp.name = filename.substr(idx0 + 1, idx1 - idx0 - 1);
	fp.ext = filename.substr(idx1);

	return fp;
}
FileParts filepartsWin(std::string filename)
{
	int idx0 = filename.rfind("\\");
	int idx1 = filename.rfind(".");

	FileParts fp;
	fp.path = filename.substr(0, idx0 + 1);
	fp.name = filename.substr(idx0 + 1, idx1 - idx0 - 1);
	fp.ext = filename.substr(idx1);

	return fp;
}
