//
// Created by shanggaoxing on 2/5/21.
//

#include "Config.h"


bool Config::parseConfigFile(map<string, string> configMap) {

    // 遍历config中的所有参数  逐个读取到程序中
    for(auto it = configMap.begin(); it != configMap.end(); ++it) {

        string key, value;
        key = it->first;
        value = it->second;

        if(key == DEF_ORIGIN_IMG_PATH)
        {
            m_strOriginImgPath = value;
        }
        else if(key == DEF_GREY_OR_RGB)
        {
            m_strGreyOrRGB = value;
        }
        else if(key == DEF_DISP_IMG_PATH)
        {
            m_strDispImgPath = value;
        }
        else if(key == DEF_IS_BY_CSV)
        {
            m_bByCSV = (value=="yes")?true:false;
        }
        else if(key == DEF_DISP_CSV_PATH)
        {
            m_strDispCSVPath = value;
        }
        else if(key == DEF_PCD_SAVE_PATH)
        {
            m_strPcdSavePath = value;
        }
        else if(key == DEF_DEGREE)
        {
            m_degree = atof(value.c_str());
        }
        else if(key == DEF_DISTANCE_Z)
        {
            m_distance_z = atof(value.c_str());
        }
        else if(key == DEF_BASELINE)
        {
            m_baseLine = atof(value.c_str());
        }
        else if(key == DEF_CAMERA_FACTOR)
        {
            m_factor = atof(value.c_str());
        }
        else if(key == DEF_CAMERA_CX)
        {
            m_cx = atof(value.c_str());
        }
        else if(key == DEF_CAMERA_CY)
        {
            m_cy = atof(value.c_str());
        }
        else if(key == DEF_CAMERA_FX)
        {
            m_fx = atof(value.c_str());
        }
        else if(key == DEF_CAMERA_FY)
        {
            m_fy = atof(value.c_str());
        }
        else if(key == DEF_START_INDEX)
        {
            m_StartIndex = atoi(value.c_str());
        }
        else if(key == DEF_END_INDEX)
        {
            m_EndIndex = atoi(value.c_str());
        }
        else if(key == DEF_ORIGIN_SUFFIX)
        {
            m_originSuffix = value;
        }
        else if(key == DEF_DISP_SUFFIX)
        {
            m_dispSuffix = value;
        }

    }


    return true;
}



const string &Config::getStrOriginImgPath() const {
    return m_strOriginImgPath;
}

void Config::setStrOriginImgPath(const string &mStrOriginImgPath) {
    m_strOriginImgPath = mStrOriginImgPath;
}

const string &Config::getStrDispImgPath() const {
    return m_strDispImgPath;
}

void Config::setStrDispImgPath(const string &mStrDispImgPath) {
    m_strDispImgPath = mStrDispImgPath;
}

const string &Config::getStrPcdSavePath() const {
    return m_strPcdSavePath;
}

void Config::setStrPcdSavePath(const string &mStrPcdSavePath) {
    m_strPcdSavePath = mStrPcdSavePath;
}


float Config::getDegree() const {
    return m_degree;
}

void Config::setDegree(float mDegree) {
    m_degree = mDegree;
}

float Config::getDistanceZ() const {
    return m_distance_z;
}

void Config::setDistanceZ(float mDistanceZ) {
    m_distance_z = mDistanceZ;
}

float Config::getBaseLine() const {
    return m_baseLine;
}

void Config::setBaseLine(float mBaseLine) {
    m_baseLine = mBaseLine;
}

double Config::getFactor() const {
    return m_factor;
}

void Config::setFactor(double mFactor) {
    m_factor = mFactor;
}

double Config::getCx() const {
    return m_cx;
}

void Config::setCx(double mCx) {
    m_cx = mCx;
}

double Config::getCy() const {
    return m_cy;
}

void Config::setCy(double mCy) {
    m_cy = mCy;
}

double Config::getFx() const {
    return m_fx;
}

void Config::setFx(double mFx) {
    m_fx = mFx;
}

double Config::getFy() const {
    return m_fy;
}

void Config::setFy(double mFy) {
    m_fy = mFy;
}

int Config::getStartIndex() const {
    return m_StartIndex;
}

void Config::setStartIndex(int mStartIndex) {
    m_StartIndex = mStartIndex;
}

int Config::getEndIndex() const {
    return m_EndIndex;
}

void Config::setEndIndex(int mEndIndex) {
    m_EndIndex = mEndIndex;
}

const string &Config::getStrGreyOrRgb() const {
    return m_strGreyOrRGB;
}

void Config::setStrGreyOrRgb(const string &mStrGreyOrRgb) {
    m_strGreyOrRGB = mStrGreyOrRgb;
}

string Config::getDispSuffix() const {
    return m_dispSuffix;
}

void Config::setDispSuffix(string mDispSuffix) {
    m_dispSuffix = mDispSuffix;
}

string Config::getOriginSuffix() const {
    return m_originSuffix;
}

void Config::setOriginSuffix(string mOriginSuffix) {
    m_originSuffix = mOriginSuffix;
}

Config::Config() {
    m_StartIndex = 0;
    m_EndIndex = 1000;
    m_dispSuffix = ".png";
    m_originSuffix = ".png";
    m_baseLine = 0;
    m_cx = 0;
    m_cy = 0;
    m_fx = 0;
    m_fy = 0;
    m_factor = 0;
    m_degree = 0;
    m_distance_z = 0;
    m_strDispImgPath = "";
    m_strGreyOrRGB = "";
    m_strOriginImgPath = "";
    m_strPcdSavePath = "";
    m_strDispCSVPath = "";

    m_bByCSV = false;
}

const string &Config::getStrDispCsvPath() const {
    return m_strDispCSVPath;
}

void Config::setStrDispCsvPath(const string &mStrDispCsvPath) {
    m_strDispCSVPath = mStrDispCsvPath;
}

bool Config::isByCsv() const {
    return m_bByCSV;
}

void Config::setByCsv(bool mBByCsv) {
    m_bByCSV = mBByCsv;
}
