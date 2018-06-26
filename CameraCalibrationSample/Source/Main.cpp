#include <stdio.h>
#include <Windows.h>
#include "Main.h"
#include "Calibrate.h"


using namespace cv;

int main(int argc, char *argv[])
{
	vector<string> images{"img/00.jpg", "img/01.jpg"};

	// (0)������
	Calibrate calibrate;

	// (1)�L�����u���[�V�����摜�̓ǂݍ���
	calibrate.SetImage(images);

	// (2)�R�[�i�[���o
	calibrate.FindTemplate();

	// (3)3������ԍ��W�̐ݒ�
	// (4)�����p�����[�^�C�c�݌W���̐���
	// (5)�O���p�����[�^�̐���
	calibrate.CalibrateCamera();

	// (6)XML�t�@�C���ւ̏����o��
	calibrate.Store(); 

	fprintf(stderr, "Finish\n");
	return 0;
}
