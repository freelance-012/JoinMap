//
// Created by shanggaoxing on 2/5/21.
//

#ifndef IMAGE_UNDISTORED_CONFIG_H
#define IMAGE_UNDISTORED_CONFIG_H

#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>


#define DEF_ORIGIN_IMG_PATH "origin_image_path"
#define DEF_GREY_OR_RGB     "grey_or_rgb"
#define DEF_DISP_IMG_PATH   "disparity_image_path"
#define DEF_IS_BY_CSV       "is_by_csv"
#define DEF_DISP_CSV_PATH   "disp_csv_path"
#define DEF_PCD_SAVE_PATH   "pcd_save_path"
#define DEF_DEGREE          "degree"
#define DEF_DISTANCE_Z      "distance_z"

#define DEF_BASELINE        "baseline"
#define DEF_CAMERA_FACTOR   "camera_factor"
#define DEF_CAMERA_CX       "camera_cx"
#define DEF_CAMERA_CY       "camera_cy"
#define DEF_CAMERA_FX       "camera_fx"
#define DEF_CAMERA_FY       "camera_fy"

#define DEF_START_INDEX     "startIndex"
#define DEF_END_INDEX       "endIndex"
#define DEF_ORIGIN_SUFFIX          "origin_suffix" // 后缀
#define DEF_DISP_SUFFIX             "disp_suffix"

#define DEF_VALUE_GREY      "grey"
#define DEF_VALUE_RGB       "rgb"

using namespace std;



class Config {

public:
    Config();

    bool parseConfigFile(map<string, string> configMap);

private:

public:
    const string &getStrOriginImgPath() const;

    void setStrOriginImgPath(const string &mStrOriginImgPath);

    const string &getStrDispImgPath() const;

    void setStrDispImgPath(const string &mStrDispImgPath);

    bool isByCsv() const;

    void setByCsv(bool mBByCsv);

    const string &getStrDispCsvPath() const;

    void setStrDispCsvPath(const string &mStrDispCsvPath);

    const string &getStrPcdSavePath() const;

    void setStrPcdSavePath(const string &mStrPcdSavePath);

    const string &getStrGreyOrRgb() const;

    void setStrGreyOrRgb(const string &mStrGreyOrRgb);


    float getDegree() const;

    void setDegree(float mDegree);

    float getDistanceZ() const;

    void setDistanceZ(float mDistanceZ);

    float getBaseLine() const;

    void setBaseLine(float mBaseLine);

    double getFactor() const;

    void setFactor(double mFactor);

    double getCx() const;

    void setCx(double mCx);

    double getCy() const;

    void setCy(double mCy);

    double getFx() const;

    void setFx(double mFx);

    double getFy() const;

    void setFy(double mFy);

    int getStartIndex() const;

    void setStartIndex(int mStartIndex);

    int getEndIndex() const;

    void setEndIndex(int mEndIndex);

    string getDispSuffix() const;

    void setDispSuffix(string mDispSuffix);

    string getOriginSuffix() const;

    void setOriginSuffix(string mOriginSuffix);

private:

    string m_strOriginImgPath;
    string m_strDispImgPath;
    string m_strPcdSavePath;
    string m_strGreyOrRGB;

    bool   m_bByCSV;
    string m_strDispCSVPath;
    float  m_degree;
    float  m_distance_z;

    // 相机内参
    float  m_baseLine;
    double m_factor;
    double m_cx;
    double m_cy;
    double m_fx;
    double m_fy;

    //
    int m_StartIndex;
    int m_EndIndex;
    string m_dispSuffix;
    string m_originSuffix;
};


#endif //IMAGE_UNDISTORED_CONFIG_H
