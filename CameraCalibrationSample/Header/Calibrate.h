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
	// �R���X�g���N�^
	Calibrate();

	// �f�X�g���N�^
	~Calibrate();

	// �摜�̓Ǎ��E�o�^
	void SetImage(vector<string> images);

	void FindTemplate();
	void CalibrateCamera();
	void Store(); 
	
private:
	// �p�^�[���T�C�Y TODO:�ꎞ�I�R�[�h�ׁ̈A�v�C��
	int patternX = 0;
	int patternY = 0;
	int numPattern = 0;
	double chessSize = 24.0f;
	CvSize patternSize;

	// �摜��
	size_t numImage = 0;

	// �L�����u���[�V�����p�摜
	vector<IplImage*> src_img;

	// �摜���Ƃ̓����_��
	int *p_count = nullptr;

	// �摜��̓����_���W(�`�F�X�{�[�h�̏ꍇ�́A�R�[�i�[) TODO:�z��ł͂Ȃ���vector�ɂ���
	CvPoint2D32f *corners = nullptr;

	// ���ۂ̓����_���W(�`�F�X�{�[�h�̏ꍇ�́A�R�[�i�[) TODO:�z��ł͂Ȃ���vector�ɂ���
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


