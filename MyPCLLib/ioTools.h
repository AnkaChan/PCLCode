#pragma once

#include <iostream>
#include "io.h"
#include <string>
#include <vector>
#include <algorithm>
#include <boost\filesystem.hpp>
using std::string;
using std::vector;
using std::cout;
using std::endl;

namespace fs = boost::filesystem;

int get_filenames(const std::string& dir, std::vector<std::string>& filenames, bool gotoChildrenDirs = false)
{
	fs::path path(dir);
	if (!fs::exists(path))
	{
		return -1;
	}

	fs::directory_iterator end_iter;
	for (fs::directory_iterator iter(path); iter != end_iter; ++iter)
	{
		if (fs::is_regular_file(iter->status()))
		{
			filenames.push_back(iter->path().string());
		}

		if (gotoChildrenDirs && fs::is_directory(iter->status()))
		{
			get_filenames(iter->path().string(), filenames);
		}
	}

	return filenames.size();
}



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

std::string make2StandardPath(std::string oPath) {
	std::replace(oPath.begin(), oPath.end(), '\\', '/'); // replace all '\' to '/', fuck Microsophtte
	if (oPath.back() != '/') {
		oPath.push_back('/');
	}
	return oPath;
}

void createDir(std::string newDir) {
	boost::filesystem::path dir(newDir.c_str());
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exists: " << newDir << std::endl;

		if (boost::filesystem::create_directory(dir))
			std::cout << "Successfully Created: " << newDir << std::endl;
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
	std::replace(filename.begin(), filename.end(), '\\', '/'); // replace all '\' to '/', fuck Microsophtte

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
