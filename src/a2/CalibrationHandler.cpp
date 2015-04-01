#include "CalibrationHandler.hpp"
#include "CoordinateConverter.hpp"
#include "ColorRecognizer.hpp"
#include "Constants.hpp"
#include "BlobDetector.hpp"
#include "Arm.hpp"
#include "math/point.hpp"
#include "math/angle_functions.hpp"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <fstream>

using namespace eecs467;

CalibrationHandler* CalibrationHandler::_instance = new CalibrationHandler;

CalibrationHandler::CalibrationHandler() : _state(IDLE),
	_imageToGlobal(3, 3), _validImageToGlobal(false) {
	if (pthread_mutex_init(&_calibMutex, NULL)) {
		printf("calibration mutex not initialized\n");
		exit(1);
	}
	readCalibrationDataFromFile();
}

CalibrationHandler* CalibrationHandler::instance() {
	return _instance;
}

void CalibrationHandler::handleMouseEvent(double x, double y, image_u32_t* im) {
	std::array<int, 2> imageIndices;
	std::array<uint8_t, 3> rgbVals;

	if (_state != IDLE) {
		imageIndices = CoordinateConverter::screenToImage({{(float)x, (float)y}});
		if (imageIndices[0] < 0) {
			imageIndices[0] = 0;
		}
		if (imageIndices[1] < 0) {
			imageIndices[1] = 0;
		}
		if (imageIndices[0] > im->width - 1) {
			imageIndices[0] = im->width - 1;
		}
		if (imageIndices[1] > im->height - 1) {
			imageIndices[1] = im->height - 1;
		}
		int pixelVal = im->buf[imageIndices[1] * im->stride + imageIndices[0]];
		rgbVals = CoordinateConverter::imageValToRgb(pixelVal);
	}
	pthread_mutex_lock(&_calibMutex);
	switch (_state) {
		case IDLE:
			break;
		case MASKING1:
			_calibrationInfo.maskXRange[0] = imageIndices[0];
			_calibrationInfo.maskYRange[1] = imageIndices[1];
			_state = MASKING2;
			break;
		case MASKING2:
			_calibrationInfo.maskXRange[1] = imageIndices[0];
			_calibrationInfo.maskYRange[0] = imageIndices[1];
			_state = IDLE;
			break;
		case HSV1: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);
			
			_calibrationInfo.redBallHue[0] = hsv[0] - objectHueRedHalfRange;
			_calibrationInfo.redBallHue[1] = hsv[0] + objectHueRedHalfRange;
			_calibrationInfo.sat[0] = hsv[1];			
			_calibrationInfo.val[0] = hsv[2];
				
			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);

			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			_state = HSV2;
			break; }
		case HSV2: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);

			_calibrationInfo.greenBallHue[0] = hsv[0] - objectHueGreenHalfRange;
			_calibrationInfo.greenBallHue[1] = hsv[0] + objectHueGreenHalfRange;
			_calibrationInfo.sat[0] += hsv[1];			
			_calibrationInfo.val[0] += hsv[2];

			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);

			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			_state = HSV3;
			break; }
		case HSV3: {
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgbVals);

			_calibrationInfo.blueSquareHue[0] = hsv[0] - objectHueBlueHalfRange;
			_calibrationInfo.blueSquareHue[1] = hsv[0] + objectHueBlueHalfRange;
			_calibrationInfo.sat[0] += hsv[1];			
			_calibrationInfo.val[0] += hsv[2];


			_calibrationInfo.sat[0] /= 3;
			_calibrationInfo.val[0] /= 3;
			_calibrationInfo.sat[0] *= noObjectThresholdSat;
			_calibrationInfo.val[0] *= noObjectThresholdVal;

			_calibrationInfo.sat[1] = 1;
			_calibrationInfo.val[1] = 1;

			printf("rgb: (%d, %d, %d)\n", rgbVals[0], rgbVals[1], rgbVals[2]);
			printf("hsv: (%f, %f, %f)\n", hsv[0], hsv[1], hsv[2]);

			if (!putCalibrationToFile()) {
				printf("Couldn't save calibration to file\n");
			}

			_state = IDLE;
			break; }
		case BOARD1:
			_imageClicks.push_back(Point<int>{imageIndices[0],
				imageIndices[1]});
			_state = BOARD2;
			break;
		case BOARD2: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_state = BOARD3;
			break; }
		case BOARD3:
			_imageClicks.push_back(Point<int>{imageIndices[0],
				imageIndices[1]});
			_state = BOARD4;
			break;
		case BOARD4: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_state = BOARD5;
			break; }
		case BOARD5:
			_imageClicks.push_back(Point<int>{imageIndices[0],
				imageIndices[1]});
			_state = BOARD6;
			break;
		case BOARD6: {
			std::array<float, 2> loc;
			if (!Arm::instance()->forwardKinematics(loc)) {
				std::cout << "Rexarm hasn't been initialized\n";
				break;
			}
			_armLocations.push_back(Point<float>{loc[0], loc[1]});
			_state = BOARD7;
			break; }
		case BOARD7: {
			// get blobs
			std::vector<BlobDetector::Blob> blobs =
				BlobDetector::findBlobs(im, 
					_calibrationInfo,
					blobMinPixels);

			// only get blue blobs
			std::vector<BlobDetector::Blob> blueBlobs;
			for (auto& blob : blobs) {
				if (blob.type == BLUESQUARE) {
					blueBlobs.push_back(blob);
				}
			}
			if (blueBlobs.size() != 4) {
				std::cout << "Not Enough Blue Squares found\n";
				break;
			}

			// getting blob coordinates from clicks
			std::vector<Point<int>> blobCoords;
			for (auto& pt : _imageClicks) {
				blobCoords.push_back(getBlobClosestToClick(pt,
					blueBlobs));
			}

			if (!createAfflineTransform(blobCoords, _armLocations,
				_imageToGlobal)) {
				std::cout << "Affline Transform error\n";
				break;
			}

			_validImageToGlobal = true;
			_state = IDLE;
			break; }
		case COORDTRANSFORM: 
			printf("Screen Coords: (%f, %f)\n", x, y);
			printf("Image Coords: (%d, %d)\n", imageIndices[0], imageIndices[1]);
			Matrix<float> imageCoords(3, 1);
			imageCoords(0, 0) = imageIndices[0];
			imageCoords(1, 0) = imageIndices[1];
			imageCoords(2, 0) = 1;
			Matrix<float> globalCoords = _imageToGlobal * imageCoords;
			printf("Global Coords: (%f, %f)\n", globalCoords(0, 0), globalCoords(1, 0));
			_state = IDLE;
			break;
	}
	pthread_mutex_unlock(&_calibMutex);
}

bool CalibrationHandler::isIdle() {
	pthread_mutex_lock(&_calibMutex);
	bool ret = _state == IDLE;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

void CalibrationHandler::calibrateMask() {
	pthread_mutex_lock(&_calibMutex);
	_state = MASKING1;
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::calibrateHsv() {
	pthread_mutex_lock(&_calibMutex);
	_state = HSV1;
	pthread_mutex_unlock(&_calibMutex);
}

CalibrationInfo CalibrationHandler::getCalibration() {
	pthread_mutex_lock(&_calibMutex);
	CalibrationInfo ret = _calibrationInfo;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

int CalibrationHandler::imageWidth() {
	pthread_mutex_lock(&_calibMutex);
	int ret = _imageWidth;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

int CalibrationHandler::imageHeight() {
	pthread_mutex_lock(&_calibMutex);
	int ret = _imageHeight;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

void CalibrationHandler::calibrateImageSize(int imageHeight, int imageWidth, bool mask) {
	pthread_mutex_lock(&_calibMutex);
	_imageHeight = imageHeight;
	_imageWidth = imageWidth;
	if (mask) {
		_calibrationInfo.maskXRange[0] = 0;
		_calibrationInfo.maskXRange[1] = imageWidth;
		_calibrationInfo.maskYRange[0] = 0;
		_calibrationInfo.maskYRange[1] = imageHeight;
	}
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::calibrateBoardTransform() {
	pthread_mutex_lock(&_calibMutex);
	_state = BOARD1;
	_validImageToGlobal = false;
	pthread_mutex_unlock(&_calibMutex);
}

void CalibrationHandler::coordTransform() {
	pthread_mutex_lock(&_calibMutex);
	_state = COORDTRANSFORM;
	pthread_mutex_unlock(&_calibMutex);
}

bool CalibrationHandler::boardTransformValid() {
	pthread_mutex_lock(&_calibMutex);
	bool ret = _validImageToGlobal;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

const Matrix<float>& CalibrationHandler::getImageToBoardTransform() {
	pthread_mutex_lock(&_calibMutex);
	Matrix<float>& ret = _imageToGlobal;
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

std::string CalibrationHandler::getMessage() {
	pthread_mutex_lock(&_calibMutex);
	std::string ret;
	switch(_state) {
		case IDLE:
			ret = ""; break;
		case MASKING1:
			ret = "click bottom left corner"; break;
		case MASKING2:
			ret = "click top right corner"; break;
		case HSV1:
			ret = "click red ball"; break;
		case HSV2:
			ret = "click green ball"; break;
		case HSV3:
			ret = "click blue square"; break;
		case BOARD1:
			ret = "click square to left of arm"; break;
		case BOARD2:
			ret = "move arm to left square, then click"; break;
		case BOARD3:
			ret = "click square to right of arm"; break;
		case BOARD4:
			ret = "move arm to right square then click"; break;
		case BOARD5:
			ret = "click square to far left"; break;
		case BOARD6:
			ret = "move arm far left square, then click"; break;
		case BOARD7:
			ret = "move arm out of board, then click"; break;
		case COORDTRANSFORM:
			ret = "click a point"; break;
		default:
			ret = ""; break;
	}
	pthread_mutex_unlock(&_calibMutex);
	return ret;
}

void CalibrationHandler::clipImage(image_u32_t* im) {
	pthread_mutex_lock(&_calibMutex);
	std::array<int, 2> maskXRange = _calibrationInfo.maskXRange;
	std::array<int, 2> maskYRange = _calibrationInfo.maskYRange;
	pthread_mutex_unlock(&_calibMutex);

	for (int i = 0; i < im->height; ++i) {
		for (int j = 0; j < im->width; ++j) {
			if (i < maskYRange[0] || 
				i > maskYRange[1] || 
				j < maskXRange[0] || 
				j > maskXRange[1]) {
				im->buf[i * im->stride + j] = 0;
			}
		}
	}
}

bool CalibrationHandler::createAfflineTransform(
		const std::vector<Point<int>>& imageCoords,
		const std::vector<Point<float>>& armCoords,
		Matrix<float>& afflineTransform) {

	if (imageCoords.size() != armCoords.size() || imageCoords.size() < 3) {
		// std::cout << "ERROR IN AFFLINE\n";
		return false;
	}

	Matrix<float> imageMatrix(imageCoords.size() * 2, 6);
	Matrix<float> armVector(armCoords.size() * 2, 1);
	imageMatrix.fill(0);
	for (size_t i = 0; i < imageCoords.size(); ++i) {
		int index = 2 * i;
		imageMatrix(index, 0) = imageCoords[i].x;
		imageMatrix(index, 1) = imageCoords[i].y;
		imageMatrix(index, 2) = 1;
		imageMatrix(index + 1, 3) = imageCoords[i].x;
		imageMatrix(index + 1, 4) = imageCoords[i].y;
		imageMatrix(index + 1, 5) = 1;

		armVector(index) = armCoords[i].x;
		armVector(index + 1) = armCoords[i].y;
	}

	Matrix<float> x(6, 1);
	Matrix<float>::leastSquares(imageMatrix, armVector, x);

	afflineTransform.fill(0);
	afflineTransform(2, 2) = 1;
	for (int i = 0; i < 6; ++i) {
		afflineTransform(i) = x(i);
	}

	return true;
}

Point<int> CalibrationHandler::getBlobClosestToClick(Point<int> click,
		std::vector<BlobDetector::Blob> blobs) {
	BlobDetector::Blob closest = *std::min_element(blobs.begin(),
		blobs.end(),
		[&](const BlobDetector::Blob& A,
			const BlobDetector::Blob& B) {
			// just using manhattan distance
			int dist1 = abs(A.x - click.x) + 
				abs(A.y - click.y);

			int dist2 = abs(B.x - click.x) +
				abs(B.y - click.y);
			return dist1 < dist2;
		});
	Point<int> ret{closest.x, closest.y};
	return ret;
}

bool CalibrationHandler::readCalibrationDataFromFile(){
	std::string line;
	std::ifstream inputFile (colorCalibrationFileName.c_str());
	if(!inputFile.is_open()) return false;
	
	inputFile >> _calibrationInfo.redBallHue[0];
	inputFile >> _calibrationInfo.redBallHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	inputFile >> _calibrationInfo.greenBallHue[0];
	inputFile >> _calibrationInfo.greenBallHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	inputFile >> _calibrationInfo.blueSquareHue[0];
	inputFile >> _calibrationInfo.blueSquareHue[1];
	inputFile >> _calibrationInfo.sat[0];
	inputFile >> _calibrationInfo.sat[1];
	inputFile >> _calibrationInfo.val[0];
	inputFile >> _calibrationInfo.val[1];

	return true;

}
bool CalibrationHandler::putCalibrationToFile(){
	std::ofstream outputFile;
	outputFile.open(colorCalibrationFileName.c_str());
	if(!outputFile.is_open()) return false;

	// Red //
	outputFile << _calibrationInfo.redBallHue[0] << " " << _calibrationInfo.redBallHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";
	// Green //
	outputFile << _calibrationInfo.greenBallHue[0] << " " << _calibrationInfo.greenBallHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";
	// Blue //
	outputFile << _calibrationInfo.blueSquareHue[0] << " " << _calibrationInfo.blueSquareHue[1] << " " 
		   << _calibrationInfo.sat[0] << " " << _calibrationInfo.sat[1] << " " 
		   << _calibrationInfo.val[0] << " " << _calibrationInfo.val[1] << "\n";

	outputFile.close();
	return true;
}
