#pragma once
#include <opencv2/opencv.hpp>
#include <windows.h>

class SapAcqDevice;
class SapBuffer;
class SapBayer;
class SapView;
class SapTransfer;
class SapXferCallbackInfo;

class GenieCamera
{
public:
	GenieCamera(void);
	~GenieCamera(void);
	void RelaeaseCamera();
	
	char filename[100];	
	std::vector<cv::Mat> capture_buf;
	std::vector<double> record_time;

	// buffer variable
	std::vector<unsigned*> addresOfBuffer[2];			// m_Buffers's Address
	int countOfbuff;				// size of buffers

	// camera variable
	int exposure;
	int gain;
	int fps;

	// bayer variable
	float m_Gamma[2];
	float m_RedGain[2];
	float m_GreenGain[2];
	float m_BlueGain[2];

	int cur_frame;
	int num_record_frame;
	int num_cameras;	
	int record_delay;

	bool isOnRecord;
	bool isCameraActive;
	bool isLoadimg;
	bool iscaptureCV;
	
	void InitCamera(HWND hwnd1 = (HWND)-1, HWND hwnd2 = (HWND)-1);
	void ToggleCamera();
	void Record();
	void RecordStop();
	void out2OpenCV(std::vector<cv::Mat> *img_buf);	
	void capture();
	void capture2OpenCV();
	void updataParam();
	void updataFPS();
	
private:
	// Sapera variables
	SapAcqDevice	*m_AcqDevice[2];
	SapBuffer		*m_Buffers[2];
	SapBuffer		*m_BayerBuffer[2];
	SapBayer		*m_Bayer[2];
	SapView			*m_View[2];
	SapTransfer		*m_Xfer[2];	
	
	void CreateSapera();
	void DestroySapera();
	void DeleteSapera();

	static void XferCallback(SapXferCallbackInfo *pInfo);
	static void XferCallback2(SapXferCallbackInfo *pInfo);	
};

