#include <stdio.h>
#include <Windows.h>
#include "Main.h"
#include "Calibrate.h"


using namespace cv;

int main(int argc, char *argv[])
{
	vector<string> images{"img/00.jpg", "img/01.jpg"};

	// (0)初期化
	Calibrate calibrate;

	// (1)キャリブレーション画像の読み込み
	calibrate.SetImage(images);

	// (2)コーナー検出
	calibrate.FindTemplate();

	// (3)3次元空間座標の設定
	// (4)内部パラメータ，歪み係数の推定
	// (5)外部パラメータの推定
	calibrate.CalibrateCamera();

	// (6)XMLファイルへの書き出し
	calibrate.Store(); 

	fprintf(stderr, "Finish\n");
	return 0;
}
