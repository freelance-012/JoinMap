//
// Created by shanggaoxing on 2/3/21.
//

#ifndef VIDEOPROCESSTOOLS_FILEUTIL_H
#define VIDEOPROCESSTOOLS_FILEUTIL_H

#include <string>
#include <cstring>
#include <iostream>
#include <map>
#include <fstream>

#ifdef __linux__

#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>

#elif defined(_WIN32)

#include <io.h> // _access fun
#include <direct.h> // _mkdir fun

#endif


using namespace std;

class FileUtil {
public:
    static void readConfigFile(string strConfigFilePath, map<string, string>& configMap);

    static bool isFolderExist(const char* folder);
    static bool isFileExist(const char* file);
    static int32_t createDirectory(char* directoryPath);

private:
    FileUtil();
};


#endif //VIDEOPROCESSTOOLS_FILEUTIL_H
