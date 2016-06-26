#ifndef CameraReader_H__
#define CameraReader_H__

#include "BGRAVideoFrame.h"
#include "Marker.hpp"
#include "MarkerDetector.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include"djicam.h"

using namespace cv;
using namespace std;
struct FoundMarker {
  int id;
  double x;
  double y;
  double z;
  double radX;
  double radY;
  double radZ;
  double degreeX;
  double degreeY;
  double degreeZ;

public:
  FoundMarker() {
    id = 0;
    x = 0;
    y = 0;
    z = 0;
    radX = 0;
    radY = 0;
    radZ = 0;
    degreeX = 0;
    degreeY = 0;
    degreeZ = 0;
  }
};

enum CameraType { DJIX3 = 0, BirdUSB = 1,smallUSB=2 };

void *startCameraThread(void *args);
void *startLoopThread(void *args);

class CameraReader {
public:
  CameraReader();
  CameraReader(CameraType cameraType);
  bool Start();
  bool Stop();
  void SetMarkerSize(double LargeMarkerSize, double SmallMarkerSize,double FrontMarkerSize);
  bool isStarted, isImageGot, isMarkerCatched, isCameraBusy;
  double xPoint, yPoint;
  int height,width;
  FoundMarker foundMarker;
  Mat input, inputCut, camera, inputOri;
  CameraType Type;

private:
  int j,lastTime,currentTime;
  bool smallMarkerCatched;
  double dYpoint,dzdistance;
  VideoCapture cap;
  unsigned int FrameCount;
  unsigned char buffer[2764800];
  float fc1, fc2, cc1, cc2, kc1, kc2, kc3, kc4, distorsionCoeff[4];
  CameraCalibration calibration;
  MarkerDetector markerDetector;
  Mat_<float> srcMat, dstMat;
  BGRAVideoFrame frame;
  vector<Transformation> trans;
  vector<Transformation>::iterator tr;
  pthread_t cameraT, loopT;
  /*
  私有函数
  */
  friend void *startCameraThread(void *args);
  friend void *startLoopThread(void *args);
  void cameraThread();
  void loopThread();
};

#endif
