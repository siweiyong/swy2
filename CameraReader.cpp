#include "CameraReader.h"
/*
构造函数
*/
CameraReader::CameraReader() : markerDetector(calibration) {
    Type = BirdUSB;
    isStarted = false;
    fc1 = 631.122813013762;
    fc2 = 632.607705586834;
    cc1 = 334.345913160626;
    cc2 = 238.214167384894;
    kc1 = 0.00716713978954894	;
    kc2 = 0.199739591210769;
    kc3 = 0;
    kc4 = 0;
    distorsionCoeff[0] = kc1;
    distorsionCoeff[1] = kc2;
    distorsionCoeff[2] = kc3;
    distorsionCoeff[3] = kc4;
    calibration = CameraCalibration(fc1, fc2, cc1, cc2, distorsionCoeff);
    markerDetector = MarkerDetector(calibration);
}

CameraReader::CameraReader(CameraType type) : markerDetector(calibration) {
    Type = type;
    isStarted = false;
    switch (type) {
    case DJIX3:
        fc1 = 2328.94795134537 * 6 / 25.0;
        fc2 = 2328.07882489417 * 6 / 25.0;
        cc1 = 1973.77902508872 * 6 / 25.0;
        cc2 = 1460.77504859926 * 6 / 25.0;
        kc1 = -0.123631906445824;
        kc2 = 0.0841239992939220;
        kc3 = 0;
        kc4 = 0;
        buffer[0] = '\0';
        break;
    case BirdUSB:
        fc1 = 631.122813013762;
        fc2 = 632.607705586834;
        cc1 = 334.345913160626;
        cc2 = 238.214167384894;
        kc1 = 0.00716713978954894	;
        kc2 = 0.199739591210769;
        kc3 = 0;
        kc4 = 0;
        break;
    case smallUSB:
        fc1 = 730.166036369399 / 1.6;
        fc2 = 729.992389107947 / 1.6;
        cc1 = 489.104940065542 / 1.6;
        cc2 = 383.685983738317 / 1.6;
        kc1 = -0.0116300151234399;
        kc2 = 0.0407467972044594;
        kc3 = 0;
        kc4 = 0;
        break;
    }
    distorsionCoeff[0] = kc1;
    distorsionCoeff[1] = kc2;
    distorsionCoeff[2] = kc3;
    distorsionCoeff[3] = kc4;
    calibration = CameraCalibration(fc1, fc2, cc1, cc2, distorsionCoeff);
    markerDetector = MarkerDetector(calibration);
    srcMat = Mat_<float>(3, 3);
    dstMat = Mat_<float>(1, 3);
}

void CameraReader::SetMarkerSize(double LargeMarkerSize, double SmallMarkerSize,
                                 double FrontMarkerSize) {
    markerDetector.ChangeMarkerSize(LargeMarkerSize, SmallMarkerSize,
                                    FrontMarkerSize);
}

bool CameraReader::Start() {
    if (!isStarted) {
        switch (Type) {
        case DJIX3:
            int ret;
         //   ret=manifold_cam_init(GETBUFFER_MODE|TRANSFER_MODE);
            if (ret == 0) {
                pthread_create(&cameraT, NULL, startCameraThread, this);
                isStarted = true;
                return true;
            } else {
                isStarted = false;
                return false;
            }
            break;
        default:
            cap.open(0);
            if (!cap.isOpened()) {
                cout << "Could not open camera." << endl;
                isStarted = false;
                return false;
            } else {
                cout << "Camera opened." << endl;
                pthread_create(&cameraT, NULL, startCameraThread, this);
                isStarted = true;
                return true;
            }
            break;
        }
    } else {
        cout << "Already running!" << endl;
        return true;
    }
}

bool CameraReader::Stop() {
    isStarted = false;
    switch (Type) {
    /*case DJIX3:
        while (!manifold_cam_exit())
        {
            sleep(1);
        }
        break;*/
    default:
        cap.release();
        break;
    }

    return true;
}

void CameraReader::cameraThread() {
    while (isStarted) {
        pthread_create(&cameraT, NULL, startLoopThread, this);
        usleep(30000);
    }
    return;
}

void CameraReader::loopThread() {
    if (!isCameraBusy) {
        int ret = 0;
        isCameraBusy = true;
        switch (Type) {
        case DJIX3:
            //ret=manifold_cam_read(buffer,&FrameCount,1);
            if (ret > 0) {
                camera = Mat(720 * 3 / 2, 1280, CV_8UC1, buffer);
                cvtColor(camera, inputOri, CV_YUV2BGR_NV12);
                inputCut = inputOri(Range(0, 720), Range(160, 1120));
                input = inputCut;
            }
            break;
        default:

            cap >> input;
            ret = 1;
            break;
        }
        if (ret > 0) {
            isImageGot = true;
            frame.height = input.rows;
            frame.width = input.cols;
            frame.stride = input.step;
            frame.data = input.data;
            height = frame.height;
            width = frame.width;
            markerDetector.processFrame(frame);
            isCameraBusy = false;
            trans = markerDetector.getTransformations();
            int count = trans.size();
            if (count > 0) {
                xPoint = markerDetector.xPoints;
                yPoint = markerDetector.yPoints;
                isMarkerCatched = true;
            } else {
                isMarkerCatched = false;
            }
            smallMarkerCatched = false;
            for (tr = trans.begin(); tr != trans.end(); tr++) {
                for (int col = 0; col < 3; col++) {
                    for (int row = 0; row < 3; row++) {
                        srcMat(row, col) = tr.base()->r().mat[row][col];
                    }
                }
                Rodrigues(srcMat, dstMat);
                foundMarker.id = tr.base()->id();
                if (tr.base()->id() == 1) {
                    foundMarker.radX = dstMat(0, 0);
                    foundMarker.radY = dstMat(0, 1);
                    foundMarker.radZ = dstMat(0, 2);
                    foundMarker.degreeX = foundMarker.radX * 180 / M_PI;
                    foundMarker.degreeY = foundMarker.radY * 180 / M_PI;
                    foundMarker.degreeZ = foundMarker.radZ * 180 / M_PI;
                    foundMarker.x = tr.base()->t().data[0];
                    foundMarker.y = tr.base()->t().data[1];
                    foundMarker.z = tr.base()->t().data[2];
                    break;
                }
                if (tr.base()->id() == 2) {
                    smallMarkerCatched = true;
                    foundMarker.radX = dstMat(0, 0);
                    foundMarker.radY = dstMat(0, 1);
                    foundMarker.radZ = dstMat(0, 2);
                    foundMarker.degreeX = foundMarker.radX * 180 / M_PI;
                    foundMarker.degreeY = foundMarker.radY * 180 / M_PI;
                    foundMarker.degreeZ = foundMarker.radZ * 180 / M_PI;
                    foundMarker.x = tr.base()->t().data[0];
                    foundMarker.y = tr.base()->t().data[1];
                    foundMarker.z = tr.base()->t().data[2];
                }
                if (tr.base()->id() == 3) {
                    if (smallMarkerCatched) {
                        dYpoint = markerDetector.yPointFront - yPoint;
                        dzdistance = tr.base()->t().data[2] - foundMarker.z;
                    } else {
                        yPoint = markerDetector.yPointFront - dYpoint;
                        foundMarker.z = tr.base()->t().data[2] - dzdistance;
                    }
                    foundMarker.radX = dstMat(0, 0);
                    foundMarker.radY = dstMat(0, 1);
                    foundMarker.radZ = dstMat(0, 2);
                    foundMarker.degreeX = foundMarker.radX * 180 / M_PI;
                    foundMarker.degreeY = foundMarker.radY * 180 / M_PI;
                    foundMarker.degreeZ = foundMarker.radZ * 180 / M_PI;
                    foundMarker.x = tr.base()->t().data[0];
                    foundMarker.y = tr.base()->t().data[1];
                }
            }
        } else {
            isImageGot = false;
        }
        isCameraBusy = false;
    }
    pthread_detach(pthread_self());
    return;
}

void *startCameraThread(void *args) {
    CameraReader *reader = (CameraReader *)args;
    reader->cameraThread();
    return NULL;
}

void *startLoopThread(void *args) {
    CameraReader *reader = (CameraReader *)args;
    reader->loopThread();
    return NULL;
}
