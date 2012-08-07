#include "GenieCamera.h"
#include <SapClassBasic.h>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

#define NUMFRAMES		150				//200
#define EXPOSURE		1000
#define GAIN			12
#define FPS				60000
#define RECORD_DELAY	0

#define DISPLAY_WIDTH	700
#define DISPLAY_HEIGHT	700
#define DISPLAY_OFFSETX	int((1400-DISPLAY_WIDTH)*0.5)
#define DISPLAY_OFFSETY	int((1024-DISPLAY_HEIGHT)*0.5)

static GenieCamera* m_ptr = NULL;

GenieCamera::GenieCamera(void)
	: countOfbuff(NUMFRAMES), exposure(EXPOSURE), gain(GAIN), fps(FPS)
	, cur_frame(0), num_record_frame(0), record_delay(RECORD_DELAY)
	, isOnRecord(false), isCameraActive(false), isLoadimg(false)
{
	m_ptr = this;
	// initialize camera variable
	for (int i = 0; i<2; ++i)
	{
		m_AcqDevice[i]		= NULL;
		m_Buffers[i]		= NULL;
		m_BayerBuffer[i]	= NULL;
		m_Bayer[i]			= NULL;
		m_Xfer[i]			= NULL;
		m_View[i]			= NULL;

		m_Gamma[i]			= 1.0f;
		m_RedGain[i]		= 1.07011f;
		m_GreenGain[i]		= 1.0f;
		m_BlueGain[i]		= 1.7537f;		
	}
	m_BlueGain[1] = 1.80543f;
}

GenieCamera::~GenieCamera(void)
{	
	RelaeaseCamera();
}
void GenieCamera::RelaeaseCamera(void)
{
	if (m_AcqDevice[0])
	{
		if (m_Xfer[0]->IsGrabbing()) m_Xfer[0]->Freeze();
		m_Xfer[0]->Wait(100);
		DestroySapera();
		DeleteSapera();
		for (int i = 0; i<num_cameras; ++i)
		{
			if (m_AcqDevice[i]) m_AcqDevice[i]->Destroy();		
			delete m_AcqDevice[i];
		}
		for (int i = 0; i<2; ++i)
		{
			m_AcqDevice[i]		= NULL;
			m_Buffers[i]		= NULL;
			m_BayerBuffer[i]	= NULL;
			m_Bayer[i]			= NULL;
			m_Xfer[i]			= NULL;
			m_View[i]			= NULL;
		}
	}
}
void GenieCamera::InitCamera(HWND hwnd1, HWND hwnd2)
{
	char text[50];
	num_cameras = SapManager::GetServerCount()-1;

	if (num_cameras == 0)
	{
		sprintf_s(text, "There is no camera.\n");
		cout<<text<<endl;
		countOfbuff = 0;
		return;
	}
	
	sprintf_s(text, "There is %d camera(s).\n", num_cameras);
	cout<<text<<endl;

	char main_serialNumberName[9] = "S1011423"; //<-- 여기에 main camera serial number를 적어넣으시오.
	char sub_serialNumberName[9] = "S1011422"; //<-- 여기에 sub camera serial number를 적어넣으시오.
	char tem_serialNumberName[9];
	
	m_AcqDevice[0] = new SapAcqDevice(SapLocation("Genie_HC1400_1" , 0));
	m_AcqDevice[0]->Create();	
	m_AcqDevice[0]->GetFeatureValue("DeviceID", tem_serialNumberName, sizeof(tem_serialNumberName));	
	if (num_cameras ==2)
	{
		if (strcmp(tem_serialNumberName, main_serialNumberName)!=0)
		{
			m_AcqDevice[1] = m_AcqDevice[0];
			m_AcqDevice[0] = new SapAcqDevice(SapLocation("Genie_HC1400_2" , 0));
			m_AcqDevice[0]->Create();
			m_AcqDevice[0]->GetFeatureValue("DeviceID", tem_serialNumberName, sizeof(tem_serialNumberName));
			cout<< "main camera serial number : "<<tem_serialNumberName <<endl;
			m_AcqDevice[1]->GetFeatureValue("DeviceID", tem_serialNumberName, sizeof(tem_serialNumberName));
			cout<< "sub camera serial number : "<< tem_serialNumberName <<endl;
		}
		else
		{
			cout<< "main camera serial number : "<<tem_serialNumberName <<endl;
			m_AcqDevice[1] = new SapAcqDevice(SapLocation("Genie_HC1400_2" , 0)); // sub camera <-- 이부분을 CamExpert 프로그램을 사용해서 확인한 수 수정해 주세요.
			m_AcqDevice[1]->Create();
			m_AcqDevice[1]->GetFeatureValue("DeviceID", tem_serialNumberName, sizeof(tem_serialNumberName));
			cout<< "sub camera serial number : "<< tem_serialNumberName <<endl;
		}
	}

	for (int i = 0; i<num_cameras; ++i)
	{
		// ROI Setting
		m_AcqDevice[i]->SetFeatureValue("Width", DISPLAY_WIDTH);
		m_AcqDevice[i]->SetFeatureValue("Height", DISPLAY_HEIGHT);
		m_AcqDevice[i]->SetFeatureValue("OffsetX", DISPLAY_OFFSETX);
		m_AcqDevice[i]->SetFeatureValue("OffsetY", DISPLAY_OFFSETY);

		// Sensor Setting
		m_AcqDevice[i]->SetFeatureValue("ExposureTime", exposure);
		m_AcqDevice[i]->SetFeatureValue("Gain", gain*10);

		m_Buffers[i] =  new SapBuffer(countOfbuff+5, m_AcqDevice[i]);
		m_Bayer[i] = new SapBayer(m_Buffers[i]);
		if (i == 0)
		{			
			m_View[i] = new SapView(NULL, hwnd1/*ui.label_camera1->winId()*/);
			m_Xfer[i] = new SapAcqDeviceToBuf(m_AcqDevice[i], m_Buffers[i], XferCallback, m_View[i]);

			// I/O Setting
			m_AcqDevice[0]->SetFeatureValue("FrameRate", fps);
			m_AcqDevice[0]->SetFeatureValue("TriggerEnable",0);
			m_AcqDevice[0]->SetFeatureValue("OutputSelectorOutput_1", 4);
			m_AcqDevice[0]->SetFeatureValue("PolarityOutput_1", 0);
		}
		else if (i == 1)
		{
			m_View[i] = new SapView(NULL, hwnd2/*ui.label_camera2->winId()*/);
			m_Xfer[i] = new SapAcqDeviceToBuf(m_AcqDevice[i], m_Buffers[i], XferCallback2, m_View[i]);

			m_AcqDevice[1]->SetFeatureValue("TriggerEnable", 1);
			m_AcqDevice[1]->SetFeatureValue("FramesPerTrigger", 1);
			m_AcqDevice[1]->SetFeatureValue("TriggerSource", 0);
			m_AcqDevice[1]->SetFeatureValue("OutputSelectorOutput_1", 2);
			m_AcqDevice[1]->SetFeatureValue("PolarityOutput_1", 0);
		}		
	}
	CreateSapera();

	capture_buf.resize(num_cameras);

	if (m_Xfer[1]) m_Xfer[1]->Grab();
	m_Xfer[0]->Grab();
}

void GenieCamera::CreateSapera()
{
	for (int i = 0; i<num_cameras; ++i)
	{
		if (m_AcqDevice[i])
		{
			m_Buffers[i]->Create();	

			m_Bayer[i]->Enable(1,0);
			m_Bayer[i]->SetBayerBufferCount(m_Buffers[i]->GetCount());
			m_Bayer[i]->Create();

			m_Bayer[i]->SetGamma(m_Gamma[i]);
			m_Bayer[i]->SetWBGain(SapDataFRGB(m_RedGain[i], m_GreenGain[i], m_BlueGain[i]));

			m_BayerBuffer[i] = m_Bayer[i]->GetBayerBuffer();
			m_View[i]->SetBuffer(m_BayerBuffer[i]);
//	 		m_View[i]->SetBuffer(m_Buffers);

			m_View[i]->Create();
			m_Xfer[i]->Create();
		}
	}
}
void GenieCamera::DestroySapera()
{
	for (int i = 0; i<num_cameras; ++i)
	{
		if(m_Xfer[i]) m_Xfer[i]->Destroy();
		if(m_Bayer[i]) m_Bayer[i]->Destroy();
		if(m_Buffers[i]) m_Buffers[i]->Destroy();
		if(m_View[i]) m_View[i]->Destroy();
	}
}
void GenieCamera::DeleteSapera()
{
	for (int i = 0; i<num_cameras; ++i)
	{
		delete m_Xfer[i];
		delete m_Bayer[i];
		delete m_Buffers[i];
		delete m_View[i];
	}
}

void GenieCamera::XferCallback( SapXferCallbackInfo *pInfo )
{		
	m_ptr->m_Bayer[0]->Convert();
	m_ptr->m_View[0]->Show();

	m_ptr->cur_frame = m_ptr->m_Buffers[0]->GetIndex();

	if (m_ptr->isOnRecord)
	{
		m_ptr->record_time.push_back((double)clock() /CLOCKS_PER_SEC);
		unsigned* tem;
		m_ptr->m_BayerBuffer[0]->GetParameter(CORBUFFER_PRM_ADDRESS, &tem);
		m_ptr->addresOfBuffer[0].push_back(tem);
		
		if (m_ptr->addresOfBuffer[0].size() == m_ptr->countOfbuff)
		{
			m_ptr->m_Xfer[0]->Freeze();
			m_ptr->m_Xfer[0]->Wait(100);
			m_ptr->num_record_frame = m_ptr->addresOfBuffer[0].size();
			m_ptr->isOnRecord = false;
			cout<<"record stop"<<endl;
		}
	}

}
void GenieCamera::XferCallback2( SapXferCallbackInfo *pInfo )
{
	m_ptr->m_Bayer[1]->Convert();
	m_ptr->m_View[1]->Show(); 
	if (m_ptr->isOnRecord)
	{		
		unsigned* tem;
		m_ptr->m_BayerBuffer[1]->GetParameter(CORBUFFER_PRM_ADDRESS, &tem);
		m_ptr->addresOfBuffer[1].push_back(tem);
// 		printf("%d\n", m_ptr->addresOfBuffer[1].size());
	}

}

void GenieCamera::ToggleCamera()
{
	isOnRecord = false;

	if (!m_Xfer[0]->IsGrabbing())
	{		
		m_Xfer[0]->Grab();
		isCameraActive = true;
		if (num_record_frame>0) num_record_frame = 0;
	}
	else
	{		
		m_Xfer[0]->Freeze();
		m_Xfer[0]->Wait(100);
		isCameraActive = false;
	}
}
void GenieCamera::Record()
{
	cur_frame = 0;
	num_record_frame = 0;

	record_time.clear();
	record_time.reserve(countOfbuff+10);

	m_Xfer[0]->Freeze();
	m_Xfer[0]->Wait(100);

	for (int i =0; i<num_cameras; ++i)
	{
		m_Xfer[i]->Init();
// 		m_Buffers[i]->SetIndex(0);
// 		m_BayerBuffer[i]->SetIndex(0);
 		m_BayerBuffer[i]->SetIndex(m_Buffers[i]->GetIndex());
		addresOfBuffer[i].clear();
		addresOfBuffer[i].reserve(countOfbuff+5);
	}

	isOnRecord = true;	

	Sleep(record_delay);
	
	cout<<"camera record start"<<endl;
	m_Xfer[0]->Grab();
}
void GenieCamera::RecordStop()
{
	if (isOnRecord)
	{	
		m_Xfer[0]->Freeze();
		m_Xfer[0]->Wait(100);
		num_record_frame = addresOfBuffer[0].size();
		isOnRecord = false;
		cout<<"record stop"<<endl;
	}
}
void GenieCamera::out2OpenCV( std::vector<cv::Mat> *img_buf)
{
	if (num_record_frame>0)
	{
		num_record_frame = min(addresOfBuffer[0].size(), addresOfBuffer[1].size());
			
		for (int i = 0; i<num_cameras; ++i)
		{
			img_buf[i].clear();
			img_buf[i].reserve(num_record_frame);
			for (int j = 0; j<num_record_frame; ++j)
			{
// 				m_BayerBuffer[i]->SetIndex(j+from);
// 				m_BayerBuffer[i]->GetParameter(CORBUFFER_PRM_ADDRESS, &process[i]);
				Mat tem(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC4, addresOfBuffer[i][j]);
				Mat tem2;
				cvtColor(tem, tem2, CV_BGRA2BGR);
				img_buf[i].push_back(tem2);
			}
		}
	}
}
void GenieCamera::capture()
{
	char filename[50];
	for (int i = 0; i<num_cameras; ++i)
	{	
		sprintf_s(filename, "capture\\%d.bmp", (2-i)*1000+m_BayerBuffer[i]->GetIndex());
		m_BayerBuffer[i]->Save(filename, "-format jpg");		
	}
}
void GenieCamera::capture2OpenCV()
{	
	for (int i = 0; i<num_cameras; ++i)
	{
		unsigned* tem;
		m_BayerBuffer[i]->GetParameter(CORBUFFER_PRM_ADDRESS, &tem);
		capture_buf[i] = cv::Mat(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC4, tem);
	}	
}

void GenieCamera::updataParam()
{
	for (int i = 0; i<num_cameras; ++i)
	{
		m_AcqDevice[i]->SetFeatureValue("ExposureTime", exposure);
		m_AcqDevice[i]->SetFeatureValue("Gain", gain*10);
	}
}
void GenieCamera::updataFPS()
{
	if (!m_Xfer[0]->IsGrabbing())
	{	
		m_AcqDevice[0]->SetFeatureValue("FrameRate", fps);
	}
	else
	{		
		m_Xfer[0]->Freeze();
		m_Xfer[0]->Wait(100);
		m_AcqDevice[0]->SetFeatureValue("FrameRate", fps);
		m_Xfer[0]->Grab();
	}
}

