//
// Created by shanggaoxing on 3/16/21.
//

#include "CsvFileReader.h"
#include <fstream>


bool CsvFileReader::csvFileReader(string csvPath, vector<vector<double>>& dispVec) {


    ifstream _csvInput(csvPath);

    // 这里需要价格判断  文件是否为空


    //vector<vector<double>> dispVec;

    // 一行代表图像一行，用逗号隔开
    // 按行读  用split(",")分割成数组string[]。 然后转换成 double[]
    //
    bool isFirstLine = true;
    string oneLine;
    while(getline(_csvInput, oneLine)) {
        if(isFirstLine) {
            isFirstLine = false;
            continue;
        }

        vector<string> vecLines;
        _split(oneLine, ",", vecLines);

        // string 转 double
        vector<double> dLinesVec;
        for(string ele : vecLines) {
            dLinesVec.push_back( atof(ele.c_str()) );
        }

        dispVec.push_back(dLinesVec);

    }

    return true;
}

bool CsvFileReader::_split(string str, string pattern, vector<string>& vecResults) {

    // 将 str 按“pattern”分割 装入vecResults中
    string::size_type pos;
    str += pattern;
    int size = str.size();
    for(int i=0; i<size; ++i) {
        pos = str.find(pattern, i);
        if(pos < size) {
            string subStr = str.substr(i, pos-i);
            vecResults.push_back(subStr);

            i = pos + pattern.size() - 1;
        }
    }
    vecResults.push_back(str.substr(0, str.find(pattern)) );

    return false;
}