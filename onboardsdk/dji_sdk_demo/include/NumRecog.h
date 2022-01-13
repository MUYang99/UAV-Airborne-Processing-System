#ifndef NUMRECOG_H
#define NUMRECOG_H

#include "opencv2/opencv.hpp"
#include "svm.h"
#include <fstream>

#define srcfeature

using namespace std;
using namespace cv;

static char *tem_line = NULL;
static int max_line_len;


class NumRecog
{
public:
  NumRecog();
  ~NumRecog();
//模型训练
  void readTrainFileList(vector<string> &trainImageList,  vector<int> &trainLabelList);
  void processHogFeature(vector<string> trainImageList,vector<int> trainLabelList, CvMat * &dataMat,CvMat * &labelMat);
  void trainLibSVM(CvMat *& dataMat, CvMat * & labelMat);
  char* readline(FILE *input);
  void trainmodel();
  
  //识别数字
  void loadModel(const char *model_file_name);
  int getNumber(Mat& image);
  
private:
  const char* trainImageFile= "/home/exbot/Downloads/Recognization_Num/数字识别/SVMMinistRecognization/train_list.txt";
  const char* testImageFile = "/home/exbot/Downloads/Recognization_Num/数字识别/SVMMinistRecognization/trainlabel.txt";
  //string testBasePath = "/home/kaiwang/testimage/";
  //string trainBasePath = "/home/kaiwang/testimage/";

  CvMat * dataMat;
  CvMat * labelMat;
  svm_model* svm;
};

#endif
