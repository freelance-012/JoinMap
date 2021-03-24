//
// Created by shanggaoxing on 2/3/21.
//

#include "FileUtil.h"

bool FileUtil::isFolderExist(const char *folder) {
    bool ret = false;
#ifdef __linux__
    DIR *dp;
    if( NULL == (dp = opendir(folder))) {
        cerr << "cannot open folder : " << folder << endl;
        ret = false;
    }
    else{
        ret = true;
    }
    closedir(dp);

#elif defined(_WIN32)
    if(_access(folder, 0) == 0){
        ret = true;
    }
    else {
        ret = false;
    }
#endif

    return ret;
}

bool FileUtil::isFileExist(const char *file) {
    bool ret = false;
#ifdef __linux__
    if(access(file, F_OK) == 0) {
        ret = true;
    }
    else {
        ret = false;
    }

#elif defined(_WIN32)
    if(_access(file, 0) == 0){
        ret = true;
    }
    else {
        ret = false;
    }
#endif
    return ret;
}

int32_t FileUtil::createDirectory(char *directoryPath) {
    uint32_t  ret = -1;
    uint32_t dirPathlen = 0;
    if(NULL != directoryPath) {
        dirPathlen = strlen(directoryPath);
    }
    if(dirPathlen > FILENAME_MAX) {
        return -1;
    }
    char tmpDirPath[FILENAME_MAX] = {0};
    for(uint32_t i = 0; i < dirPathlen; ++i) {
        tmpDirPath[i] = directoryPath[i];
        if(tmpDirPath[i] == '\\' || tmpDirPath[i] == '/') {
            if(!isFolderExist(tmpDirPath)){
#ifdef __linux__
                ret = mkdir(tmpDirPath, S_IRWXU|S_IRWXG|S_IRWXO);
#elif defined(_WIN32)
                ret = _mkdir(tmpDirPath);
#endif
            }
        }
    }

    return ret;  // 0表示顺利
}

// 私有
FileUtil::FileUtil() {

}

void FileUtil::readConfigFile(string strConfigFilePath, map<string, string>& configMap) {
    //cout << "hello readconfig" << endl;

    // 读取配置文件 获取图片路径、相机参数等信息
    ifstream config_reader;
    config_reader.open(strConfigFilePath, ios::in);

    if (!config_reader.is_open())
    {
        cerr << "could not open strConfigFilePath: " << strConfigFilePath << endl;
    }

    string temp;
    string key, value;
    while (getline(config_reader, temp))
    {
        //cout << "temp = " << temp << endl;
        if (temp == "" || temp.at(0) != '[')
        {
            continue;
        }

        int pos = temp.find("=");
        //cout << "pos = " << pos << endl;
        key = temp.substr(1, pos - 2); //  [1,pos-2]
        value = temp.substr(pos + 1);

        //cout << "key = " << key << "\t value = " << value << endl;
        configMap.insert(make_pair(key, value));

    }
}
