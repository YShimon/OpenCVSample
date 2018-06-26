#include "Calibrate.h"

using namespace std;

// コンストラクタ
Calibrate::Calibrate()
{
	patternX = 6;	// パターンの列数
	patternY = 9;	// パターンの行数
	numPattern = patternX * patternY;
	patternSize = cvSize(patternX, patternY);

	numImage = 0;
}

// デストラクタ
Calibrate::~Calibrate()
{
	// 画像用メモリ解放
	for (IplImage* image : src_img) { cvReleaseImage(&image); }

	cvFree(&corners);
	cvFree(&p_count);
}

// 画像の読込・登録
void Calibrate::SetImage(vector<string> images)
{
	// 画像ファイル読込
	for (string path : images)
	{
		src_img.push_back(cvLoadImage(path.c_str(), CV_LOAD_IMAGE_COLOR));
		if (src_img.back() == nullptr) throw new exception();
	}

	// 読込画像数更新
	numImage += src_img.size();

	// TODO:リサイズが必要,nullチェック
	corners = (CvPoint2D32f *)cvAlloc(sizeof(CvPoint2D32f)* numImage * numPattern);
	p_count = (int *)cvAlloc(sizeof(int)* numImage);
	objects = (CvPoint3D32f *)cvAlloc(sizeof(CvPoint3D32f)* numImage * numPattern);
}

void Calibrate::FindTemplate()
{
	int found_num = 0;
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	for (int i = 0; i < numImage; i++)
	{
		// (3.1)チェスボード（キャリブレーションパターン）のコーナー検出
		//	FindTemplate::FindChessboardCorners();
		found = cvFindChessboardCorners(src_img[i], patternSize, &corners[i * numPattern], &corner_count);
		fprintf(stderr, "Try...%02d...", i);
		if (found)
		{
			fprintf(stderr, "Detect OK\n");
			found_num++;
		}
		else
		{
			fprintf(stderr, "Detect Fail\n");
		}

		// (3.2)コーナー位置をサブピクセル精度に修正，描画
		//	FindTemplate::FindCornerSubPix();
		IplImage *src_gray = cvCreateImage(cvGetSize(src_img[i]), IPL_DEPTH_8U, 1);
		cvCvtColor(src_img[i], src_gray, CV_BGR2GRAY);
		cvFindCornerSubPix(src_gray, &corners[i * numPattern], corner_count,
			cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cvDrawChessboardCorners(src_img[i], patternSize, &corners[i * numPattern], corner_count, found);
		p_count[i] = corner_count;
		cvShowImage("Calibration", src_img[i]);
		cvWaitKey(0);
		cvReleaseImage(&src_gray);
	}
	cvDestroyWindow("Calibration");

	if (found_num != numImage) return;

	// ※image_pointsとcornersは浅いコピーなので注意(corners,p_countは消せない)
	cvInitMatHeader(&image_points, numImage*numPattern, 1, CV_32FC2, corners);
	cvInitMatHeader(&point_counts, numImage, 1, CV_32SC1, p_count);
}

void Calibrate::CalibrateCamera()
{
	// (3)3次元空間座標の設定
	// 1辺の長さを24mmとする(高さ0)
	// チェスボード(列6x行9)　2枚分の画像の初期化
	for (int i = 0; i < numImage; i++) {
		for (int j = 0; j < patternY; j++) {
			for (int k = 0; k < patternX; k++) {
				objects[i * numPattern + j * patternX + k].x = j * chessSize;
				objects[i * numPattern + j * patternX + k].y = k * chessSize;
				objects[i * numPattern + j * patternX + k].z = 0.0;
			}
		}
	}
	cvInitMatHeader(&object_points, numImage*numPattern, 3, CV_32FC1, objects);

	// (4)内部パラメータ，歪み係数の推定
	//	Calibrate::CalibrateCamera(); (4)(5)をまとめる
	cvCalibrateCamera2(&object_points, &image_points, &point_counts, cvSize(640, 480), intrinsic, distortion);

	// (5)外部パラメータの推定
	CvMat sub_image_points, sub_object_points;
	int base = 0;
	cvGetRows(&image_points, &sub_image_points, base * numPattern, (base + 1) * numPattern);
	cvGetRows(&object_points, &sub_object_points, base * numPattern, (base + 1) * numPattern);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);
}

// カメラパラメータを出力
void Calibrate::Store()
{
	// (6)XMLファイルへの書き出し
	CvFileStorage *fs;
	fs = cvOpenFileStorage("xml/Camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);
}