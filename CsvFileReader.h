//
// Created by shanggaoxing on 3/16/21.
//

#ifndef POINTCLOUD_CSVFILEREADER_H
#define POINTCLOUD_CSVFILEREADER_H

#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

class CsvFileReader {

public:
    static bool csvFileReader(string csvPath, vector<vector<double>>& dispVec);

private:
    static bool _split(string str, string pattern, vector<string>& vecResults);

};


#endif //POINTCLOUD_CSVFILEREADER_H
