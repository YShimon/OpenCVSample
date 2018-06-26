#pragma once

using namespace std;

#include <vector>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/legacy/legacy.hpp"

#if _DEBUG
#pragma comment(lib, "opencv_core247d.lib")
#pragma comment(lib, "opencv_imgproc247d.lib")
#pragma comment(lib, "opencv_highgui247d.lib")
#pragma comment(lib, "opencv_calib3d247d.lib")
#else
#pragma comment(lib, "opencv_core247.lib")
#pragma comment(lib, "opencv_imgproc247.lib")
#pragma comment(lib, "opencv_highgui247.lib")
#pragma comment(lib, "opencv_calib3d247.lib")
#endif		

class Calibrate
{
public:
	// コンストラクタ
	Calibrate();

	// デストラクタ
	~Calibrate();

	// 画像の読込・登録
	void SetImage(vector<string> images);

	void FindTemplate();
	void CalibrateCamera();
	void Store(); 
	
private:
	// パターンサイズ TODO:一時的コードの為、要修正
	int patternX = 0;
	int patternY = 0;
	int numPattern = 0;
	double chessSize = 24.0f;
	CvSize patternSize;

	// 画像数
	size_t numImage = 0;

	// キャリブレーション用画像
	vector<IplImage*> src_img;

	// 画像ごとの特徴点数
	int *p_count = nullptr;

	// 画像上の特徴点座標(チェスボードの場合は、コーナー) TODO:配列ではなくてvectorにする
	CvPoint2D32f *corners = nullptr;

	// 実際の特徴点座標(チェスボードの場合は、コーナー) TODO:配列ではなくてvectorにする
	CvPoint3D32f* objects = nullptr;

	int corner_count, found;
	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;
	CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);
};


