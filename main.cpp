#include "BGRAVideoFrame.h"
#include "CameraReader.h"
#include "Marker.hpp"
#include "MarkerDetector.hpp"
#include "djicam.h"
#include <opencv2/opencv.hpp>
#include <time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int main() {
  CameraReader reader(smallUSB);
  reader.SetMarkerSize(40, 9, 6);
  bool ret = reader.Start();
  if (!ret) {
    cout<<"failed"<<endl;
  };
  Mat large;
  Size dsize;
  dsize.width=1280;
  dsize.height=1024;
  while (true) {
    if (reader.isImageGot) {
        resize(reader.input,large,Size(1280,1024));
      imshow("1", large);
      //resizeWindow("1",1280,1024);
      // cout<<reader.height<<"\t"<<reader.width<<endl;
    }
    if (reader.isImageGot && reader.isMarkerCatched) {
      cout << reader.yPoint << "\t" << reader.foundMarker.z << "\t"
           << reader.foundMarker.id << endl;
    }
    waitKey(30);
  }

  return 0;
}
