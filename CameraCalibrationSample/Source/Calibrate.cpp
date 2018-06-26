#include "Calibrate.h"

using namespace std;

// �R���X�g���N�^
Calibrate::Calibrate()
{
	patternX = 6;	// �p�^�[���̗�
	patternY = 9;	// �p�^�[���̍s��
	numPattern = patternX * patternY;
	patternSize = cvSize(patternX, patternY);

	numImage = 0;
}

// �f�X�g���N�^
Calibrate::~Calibrate()
{
	// �摜�p���������
	for (IplImage* image : src_img) { cvReleaseImage(&image); }

	cvFree(&corners);
	cvFree(&p_count);
}

// �摜�̓Ǎ��E�o�^
void Calibrate::SetImage(vector<string> images)
{
	// �摜�t�@�C���Ǎ�
	for (string path : images)
	{
		src_img.push_back(cvLoadImage(path.c_str(), CV_LOAD_IMAGE_COLOR));
		if (src_img.back() == nullptr) throw new exception();
	}

	// �Ǎ��摜���X�V
	numImage += src_img.size();

	// TODO:���T�C�Y���K�v,null�`�F�b�N
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
		// (3.1)�`�F�X�{�[�h�i�L�����u���[�V�����p�^�[���j�̃R�[�i�[���o
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

		// (3.2)�R�[�i�[�ʒu���T�u�s�N�Z�����x�ɏC���C�`��
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

	// ��image_points��corners�͐󂢃R�s�[�Ȃ̂Œ���(corners,p_count�͏����Ȃ�)
	cvInitMatHeader(&image_points, numImage*numPattern, 1, CV_32FC2, corners);
	cvInitMatHeader(&point_counts, numImage, 1, CV_32SC1, p_count);
}

void Calibrate::CalibrateCamera()
{
	// (3)3������ԍ��W�̐ݒ�
	// 1�ӂ̒�����24mm�Ƃ���(����0)
	// �`�F�X�{�[�h(��6x�s9)�@2�����̉摜�̏�����
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

	// (4)�����p�����[�^�C�c�݌W���̐���
	//	Calibrate::CalibrateCamera(); (4)(5)���܂Ƃ߂�
	cvCalibrateCamera2(&object_points, &image_points, &point_counts, cvSize(640, 480), intrinsic, distortion);

	// (5)�O���p�����[�^�̐���
	CvMat sub_image_points, sub_object_points;
	int base = 0;
	cvGetRows(&image_points, &sub_image_points, base * numPattern, (base + 1) * numPattern);
	cvGetRows(&object_points, &sub_object_points, base * numPattern, (base + 1) * numPattern);
	cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);
}

// �J�����p�����[�^���o��
void Calibrate::Store()
{
	// (6)XML�t�@�C���ւ̏����o��
	CvFileStorage *fs;
	fs = cvOpenFileStorage("xml/Camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation);
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);
}