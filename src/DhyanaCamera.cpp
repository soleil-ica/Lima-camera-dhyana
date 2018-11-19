//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2014
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
//#include <chrono>
#include <climits>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "lima/MiscUtils.h"
#include "DhyanaCamera.h"

using namespace lima;
using namespace lima::Dhyana;
using namespace std;


//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera():
m_depth(16),
m_trig_mode(IntTrig),
m_status(Ready),
m_acq_frame_nb(0),
m_temperature_target(0),
m_new_acquisition_done(false),
m_hThdEvent(NULL),
m_waiting(false)
{

	DEB_CONSTRUCTOR();
	init();
	m_acq_thread = new AcqThread(*this);
	m_acq_thread->start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	DEB_TRACE() << "Close TUCAM API ...";
	TUCAM_Dev_Close(m_opCam.hIdxTUCam);// Close camera
	DEB_TRACE() << "Uninitialize TUCAM API ...";
	TUCAM_Api_Uninit();// Uninitialize SDK API environment

	delete m_acq_thread;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::init()
{
	DEB_MEMBER_FUNCT();

	DEB_TRACE() << "Initialize TUCAM API ...";
	m_itApi.pstrConfigPath = "c:\\DeviceServers";
	m_itApi.uiCamCount = 0;

	if(TUCAMRET_SUCCESS != TUCAM_Api_Init(&m_itApi))
	{
		// Initializing SDK API environment failed
		THROW_HW_ERROR(Error) << "Unable to initialize TUCAM_Api !";
	}

	if(0 == m_itApi.uiCamCount)
	{
		// No camera
		THROW_HW_ERROR(Error) << "Unable to locate the camera !";
	}

	DEB_TRACE() << "Open TUCAM API ...";
	m_opCam.hIdxTUCam = 0;
	m_opCam.uiIdxOpen = 0;

	if(TUCAMRET_SUCCESS != TUCAM_Dev_Open(&m_opCam))
	{
		// Failed to open camera
		THROW_HW_ERROR(Error) << "Unable to Open the camera !";
	}

	if(NULL == m_opCam.hIdxTUCam)
	{
		THROW_HW_ERROR(Error) << "Unable to open the camera !";
	}

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::reset()
{
	DEB_MEMBER_FUNCT();
	stopAcq();
	//@BEGIN : other stuff on Driver/API
	//...
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex lock(m_cond.mutex());
	//@BEGIN : Ensure that Acquisition is Stopped . Sometimes API TUcam doesn't close correctly !!
	Timestamp t0 = Timestamp::now();
	DEB_TRACE() << "Ensure that Acquisition is Stopped ";
	m_status = Camera::Exposure;
	m_waiting = true;
	//@BEGIN : Ensure that Acquisition is Started before return ...
	DEB_TRACE() << "Ensure that Acquisition is Started  ";
	if(NULL == m_hThdEvent)
	{		
		m_frame.pBuffer = NULL;
		m_frame.ucFormatGet = TUFRM_FMT_RAW;
		m_frame.uiRsdSize = 1;// how many frames do you want

		DEB_TRACE() << "TUCAM_Buf_Alloc";
		TUCAM_Buf_Alloc(m_opCam.hIdxTUCam, &m_frame);// Alloc buffer after set resolution or set ROI attribute

		DEB_TRACE() << "TUCAM_Cap_Start";
		if(m_trigger_mode == IntTrig)
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_SOFTWARE);// Start capture in software trigger
		else if(m_trigger_mode == ExtTrigSingle)
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_STANDARD);// Start capture in external trigger STANDARD

		DEB_TRACE() << "CreateEvent";
		m_hThdEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		DEB_TRACE() << "_beginthread";
		_beginthread(grabThread, 0, this);// Start drawing thread	
	}
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "prepareAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex lock(m_cond.mutex());
	Timestamp t0 = Timestamp::now();
	DEB_TRACE() << "startAcq ...";
	m_acq_frame_nb = 0;
	m_new_acquisition_done = false;
	StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	//@BEGIN : tigger the acquisition
	DEB_TRACE() << "DoSoftwareTrigger ";
	TUCAM_Cap_DoSoftwareTrigger(m_opCam.hIdxTUCam);
	//@END

	//Start acquisition thread
	m_wait_flag = false;
	m_quit = false;
	m_cond.broadcast();
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "startAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "stopAcq ...";
	AutoMutex aLock(m_cond.mutex());
	// Don't do anything if acquisition is idle.
	if(m_thread_running == true)
	{
		m_wait_flag = true;
		m_cond.broadcast();
		DEB_TRACE() << "stop requested ";
	}

	//@BEGIN : Ensure that Acquisition is Stopped before return ...			
	m_waiting = false;
	Timestamp t0 = Timestamp::now();
	if(NULL != m_hThdEvent)
	{
		TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam);// If you called TUCAM_Buf_WaitForFrames()
		WaitForSingleObject(m_hThdEvent, INFINITE);
		CloseHandle(m_hThdEvent);
		m_hThdEvent = NULL;
		TUCAM_Cap_Stop(m_opCam.hIdxTUCam);// Stop capture   
		TUCAM_Buf_Release(m_opCam.hIdxTUCam);// Release alloc buffer after stop capture and quit drawing thread
	}
	m_status = Camera::Ready;	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "stopAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	//@END	
	DEB_TRACE() << "Ensure that Acquisition is Stopped ";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
	DEB_MEMBER_FUNCT();

	status = m_status;

	DEB_RETURN() << DEB_VAR1(status);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::readFrame(void *bptr, int& frame_nb)
{
	DEB_MEMBER_FUNCT();
	Timestamp t0 = Timestamp::now();
	DEB_TRACE() << "Camera::readFrame() ";
	//@BEGIN : Get frame from Driver/API & copy it into bptr already allocated 
	DEB_TRACE() << "Copy Buffer image into Lima Frame Ptr";
	memcpy((unsigned short *) bptr, (unsigned short *) (m_frame.pBuffer + m_frame.usOffset), m_frame.uiImgSize);//we need a nb of BYTES .		
	frame_nb = m_frame.uiIndex;
	//@END	
	m_status = Camera::Readout;
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "readFrame : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	return false;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::grabThread(LPVOID lParam)
{
	std::cout << "Camera::grabThread() [BEGIN]" << std::endl;
	Camera *pIn = (Camera *) lParam;
//	while(pIn->m_status != Camera::Ready)
	while(pIn->m_waiting)
	{
		if(TUCAMRET_SUCCESS == TUCAM_Buf_WaitForFrame(pIn->m_opCam.hIdxTUCam, &pIn->m_frame))
		{
			//The based information
			//std::cout<<"pIn->m_frame.szSignature = "	<<pIn->m_frame.szSignature<<std::endl;		// [out]Copyright+Version: TU+1.0 ['T', 'U', '1', '\0']		
			//std::cout<<"pIn->m_frame.usHeader = "		<<pIn->m_frame.usHeader<<std::endl;			// [out] The frame header size
			//std::cout<<"pIn->m_frame.usOffset = "		<<pIn->m_frame.usOffset<<std::endl;			// [out] The frame data offset
			std::cout << "pIn->m_frame.usWidth = " << pIn->m_frame.usWidth << std::endl;// [out] The frame width
			std::cout << "pIn->m_frame.usHeight = " << pIn->m_frame.usHeight << std::endl;// [out] The frame height
			//std::cout<<"pIn->m_frame.uiWidthStep = "	<<pIn->m_frame.uiWidthStep<<std::endl;		// [out] The frame width step
			//std::cout<<"pIn->m_frame.ucDepth = "		<<pIn->m_frame.ucDepth<<std::endl;			// [out] The frame data depth 
			//std::cout<<"pIn->m_frame.ucFormat = "		<<pIn->m_frame.ucFormat<<std::endl;			// [out] The frame data format                  
			//std::cout<<"pIn->m_frame.ucChannels = "		<<pIn->m_frame.ucChannels<<std::endl;		// [out] The frame data channels
			//std::cout<<"pIn->m_frame.ucElemBytes = "	<<pIn->m_frame.ucElemBytes<<std::endl;		// [out] The frame data bytes per element
			//std::cout<<"pIn->m_frame.ucFormatGet = "	<<pIn->m_frame.ucFormatGet<<std::endl;		// [in]  Which frame data format do you want    see TUFRM_FORMATS
			std::cout << "pIn->m_frame.uiIndex = " << pIn->m_frame.uiIndex << std::endl;// [in/out] The frame index number
			std::cout << "pIn->m_frame.uiImgSize = " << pIn->m_frame.uiImgSize << std::endl;// [out] The frame size
			std::cout << "pIn->m_frame.uiRsdSize = " << pIn->m_frame.uiRsdSize << std::endl;// [in]  The frame reserved size    (how many frames do you want)
			//std::cout<<"pIn->m_frame.uiHstSize = "		<<pIn->m_frame.uiHstSize<<std::endl;		// [out] The frame histogram size	
			pIn->m_new_acquisition_done = true;
		}
		else
		{
			std::cout << "Unable to get the frame from the camera !" << std::endl;
		}
	}
	std::cout << "setEvent" << std::endl;
	SetEvent(pIn->m_hThdEvent);

	std::cout << "_endthread" << std::endl;
	_endthread();

	std::cout << "Camera::grabThread() [END]";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();
	//	AutoMutex aLock(m_cam.m_cond.mutex());
	StdBufferCbMgr& buffer_mgr = m_cam.m_bufferCtrlObj.getBuffer();

	while(!m_cam.m_quit)
	{
		while(m_cam.m_wait_flag && !m_cam.m_quit)
		{
			DEB_TRACE() << "AcqThread : Wait for start acquisition";
			m_cam.m_thread_running = false;
			AutoMutex lock(m_cam.m_cond.mutex());
			m_cam.m_cond.wait();
		}

		if(m_cam.m_quit)
			return;
		DEB_TRACE() << "AcqThread : Running";
		m_cam.m_thread_running = true;
		//auto t1 = Clock::now();

		bool continueFlag = true;
		while(continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames))
		{
			// Check first if acq. has been stopped
			DEB_TRACE() << "AcqThread : Check first if acq. has been stopped ";
			if(m_cam.m_wait_flag)
			{
				DEB_TRACE() << "AcqThread: has been stopped from user ";
				continueFlag = false;
				continue;
			}
			
			//Generate software triger for each frame, except for the first image
			if(m_cam.m_acq_frame_nb != 0)
			{
				DEB_TRACE() << "AcqThread : Wait latency time : " << m_cam.m_lat_time * 1000000 << " (us) ";
				usleep((DWORD)(m_cam.m_lat_time * 1000000));
				DEB_TRACE() << "AcqThread : DoSoftwareTrigger ";
				TUCAM_Cap_DoSoftwareTrigger(m_cam.m_opCam.hIdxTUCam);
			}

			if(m_cam.m_new_acquisition_done)
			{
				//Prepare Lima Frame Ptr 
				DEB_TRACE() << "AcqThread : Prepare  Lima Frame Ptr";
				void* bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);

				//Read Frame From API/Driver/Etc ... & Copy it into Lima Frame Ptr
				DEB_TRACE() << "AcqThread : Read Frame From API/Driver/Etc ....";
				int frame_nb = 0;
				m_cam.readFrame(bptr, frame_nb);
				//Push the image buffer through Lima 
				DEB_TRACE() << "AcqThread : Declare a Lima new Frame Ready (" << m_cam.m_acq_frame_nb << ")";
				HwFrameInfoType frame_info;
				frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
				continueFlag = buffer_mgr.newFrameReady(frame_info);
				m_cam.m_acq_frame_nb++;
				m_cam.m_new_acquisition_done = false;
			}
		}
		//auto t2 = Clock::now();		
		//DEB_TRACE() << "Delta t2-t1: " << std::chrono::duration_cast < std::chrono::nanoseconds > (t2 - t1).count() << " nanoseconds";
		//stopAcq only if this is not already done
		DEB_TRACE() << "AcqThread : stopAcq only if this is not already done ";
		if(!m_cam.m_wait_flag)
		{
			DEB_TRACE() << " AcqThread: StopAcq";
			m_cam.stopAcq();
		}
		DEB_TRACE() << " AcqThread: Setting thread running flag to false";
		AutoMutex lock(m_cam.m_cond.mutex());
		m_cam.m_thread_running = false;
		m_cam.m_wait_flag = true;
		m_cam.m_status = Camera::Ready;
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::AcqThread::AcqThread(Camera& cam):
m_cam(cam)
{
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_wait_flag = true;
	m_cam.m_quit = false;
	aLock.unlock();
	pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::AcqThread::~AcqThread()
{
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_quit = true;
	m_cam.m_cond.broadcast();
	aLock.unlock();
	join();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Fix the image type (pixel depth) into Driver/API		
	switch(m_depth)
	{
		case 16: type = Bpp16;
			break;
		default:
			THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
			break;
	}
	//@END	
	return;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(type);
	//@BEGIN : Fix the image type (pixel depth) into Driver/API	
	switch(type)
	{
		case Bpp16:
			m_depth = 16;
			break;
		default:
			THROW_HW_ERROR(Error) << "This pixel format of the camera is not managed, only 16 bits cameras are already managed!";
			break;
	}
	//@END	
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Get Detector type from Driver/API
	type = "Tucsen - Dhyana";
	//@END	

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorModel(std::string& model)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Get Detector model/type from Driver/API
	stringstream ss;
	TUCAM_VALUE_INFO valInfo;
	valInfo.nID = TUIDI_CAMERA_MODEL;
	if(TUCAMRET_SUCCESS != TUCAM_Dev_GetInfo(m_opCam.hIdxTUCam, &valInfo))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDI_CAMERA_MODEL from the camera !";
	}
	ss << valInfo.pText;
	model = ss.str();
	//@END		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();

	//@BEGIN : Get Detector size in pixels from Driver/API
	size = Size(PIXEL_NB_WIDTH, PIXEL_NB_HEIGHT);
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Get Pixels size in micron from Driver/API			
	sizex = PIXEL_SIZE_WIDTH_MICRON;
	sizey = PIXEL_SIZE_HEIGHT_MICRON;
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj()
{
	return &m_bufferCtrlObj;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	bool valid_mode;

	switch(mode)
	{
		case IntTrig:
		case ExtTrigSingle:
			valid_mode = true;
			break;
		case ExtTrigReadout:
		case ExtGate:
		case ExtTrigMult:
		case IntTrigMult:
		default:
			valid_mode = false;
			break;
	}
	return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTrigMode() " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
	TUCAM_TRIGGER_ATTR tgrAttr;
	tgrAttr.nTgrMode = 0;
	tgrAttr.nFrames = 1;
	tgrAttr.nDelayTm = 0;
	tgrAttr.nExpMode = TUCTE_EXPTM;
	tgrAttr.nEdgeMode = TUCTD_RISING;


	switch(mode)
	{
		case IntTrig:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE";
			break;
		case ExtTrigSingle:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD";
			break;
		case ExtTrigMult:
		case ExtGate:
		case IntTrigMult:
		case ExtTrigReadout:
		default:
			THROW_HW_ERROR(NotSupported) << DEB_VAR1(mode);
	}
	m_trigger_mode = mode;

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_trigger_mode;
	DEB_RETURN() << DEB_VAR1(mode);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	double dbVal;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, &dbVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDP_EXPOSURETM from the camera !";
	}
	m_exp_time = dbVal / 1000;//TUCAM use (ms), but lima use (second) as unit 
	exp_time = m_exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setExpTime() " << DEB_VAR1(exp_time);
	if(TUCAMRET_SUCCESS != TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, exp_time * 1000))//TUCAM use (ms), but lima use (second) as unit 
	{
		THROW_HW_ERROR(Error) << "Unable to Write TUIDP_EXPOSURETM to the camera !";
	}
	m_exp_time = exp_time;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setLatTime() " << DEB_VAR1(lat_time);
	m_lat_time = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	m_lat_time = lat_time;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
	DEB_MEMBER_FUNCT();
	min_expo = 0.;
	max_expo = 10;//10s
	DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
	DEB_MEMBER_FUNCT();
	// --- no info on min latency
	min_lat = 0.;
	// --- do not know how to get the max_lat, fix it as the max exposure time
	max_lat = 10;//10s
	DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setNbFrames() " << DEB_VAR1(nb_frames);
	if(nb_frames < 0)
	{
		THROW_HW_ERROR(Error) << "Number of frames to acquire has not been set";
	}
	if(nb_frames > 1)
	{
		//THROW_HW_ERROR(Error) << "Unable to acquire more than 1 frame ! ";	//T@D@ allowed only for test 
	}
	m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getNbFrames";
	DEB_RETURN() << DEB_VAR1(m_nb_frames);
	nb_frames = m_nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Camera::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	return m_acq_frame_nb;
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();

	//@BEGIN : check available values of binning H/V
	int x = hw_bin.getX();
	int y = hw_bin.getY();
	if(x != 1 || y != 1)
	{
		DEB_ERROR() << "Binning values not supported";
		THROW_HW_ERROR(Error) << "Binning values not supported = " << DEB_VAR1(hw_bin);
	}
	//@END

	hw_bin = Bin(x, y);
	DEB_RETURN() << DEB_VAR1(hw_bin);
}
//-----------------------------------------------------
// @brief set the new binning mode
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
	DEB_MEMBER_FUNCT();

	//@BEGIN : set binning H/V to the Driver/API
	//...
	//@END
	m_bin = set_bin;

	DEB_RETURN() << DEB_VAR1(set_bin);
}

//-----------------------------------------------------
// @brief return the current binning mode
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
	DEB_MEMBER_FUNCT();

	//@BEGIN : get binning from Driver/API
	int bin_x = 1;
	int bin_y = 1;
	//@END
	Bin tmp_bin(bin_x, bin_y);

	hw_bin = tmp_bin;
	m_bin = tmp_bin;

	DEB_RETURN() << DEB_VAR1(hw_bin);
}

//-----------------------------------------------------
//! Camera::checkRoi()
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::checkRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : check available values of Roi
	//@END
	hw_roi = set_roi;

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getRoi";
	//@BEGIN : get Roi from the Driver/API
	TUCAM_ROI_ATTR roiAttr;
	if(TUCAMRET_SUCCESS != TUCAM_Cap_GetROI(m_opCam.hIdxTUCam, &roiAttr))
	{
		THROW_HW_ERROR(Error) << "Unable to GetRoi from  the camera !";
	}
	hw_roi = Roi(roiAttr.nHOffset,
				roiAttr.nVOffset,
				roiAttr.nWidth,
				roiAttr.nHeight);
	//@END

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::setRoi()
//---------------------------------------------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : set Roi from the Driver/API	
	if(!set_roi.isActive())
	{
		DEB_TRACE() << "Roi is not Enabled : so set full frame";
		/*
		//set Roi to Driver/API
		Size size;
		getDetectorImageSize(size);
		TUCAM_ROI_ATTR roiAttr;
		roiAttr.bEnable = TRUE;
		roiAttr.nHOffset = 0;
		roiAttr.nVOffset = 0;
		roiAttr.nWidth = size.getWidth();
		roiAttr.nHeight = size.getHeight();

		if(TUCAMRET_SUCCESS != TUCAM_Cap_SetROI(m_opCam.hIdxTUCam, roiAttr))
		{
			THROW_HW_ERROR(Error) << "Unable to SetRoi to the camera !";
		}		
		 */
	}
	else
	{
		DEB_TRACE() << "Roi is Enabled";
		//set Roi to Driver/API
		TUCAM_ROI_ATTR roiAttr;
		roiAttr.bEnable = TRUE;
		roiAttr.nHOffset = set_roi.getTopLeft().x;
		roiAttr.nVOffset = set_roi.getTopLeft().y;
		roiAttr.nWidth = set_roi.getSize().getWidth();
		roiAttr.nHeight = set_roi.getSize().getHeight();

		if(TUCAMRET_SUCCESS != TUCAM_Cap_SetROI(m_opCam.hIdxTUCam, roiAttr))
		{
			THROW_HW_ERROR(Error) << "Unable to SetRoi to the camera !";
		}
	}
	//@END	
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool Camera::isAcqRunning() const
{
	DEB_MEMBER_FUNCT();
	//	AutoMutex aLock(m_cond.mutex());
	DEB_TRACE() << "isAcqRunning - " << DEB_VAR1(m_thread_running) << "---------------------------";
	return m_thread_running;
}

///////////////////////////////////////////////////////
// Dhyana specific stuff now
///////////////////////////////////////////////////////

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTemperatureTarget(double temp)
{
	DEB_MEMBER_FUNCT();
	TUCAM_PROP_ATTR attrProp;
	attrProp.nIdxChn = 0;// Current channel (camera monochrome = 0) . VERY IMPORTANT, doesn't work otherwise !!!!!
	attrProp.idProp = TUIDP_TEMPERATURE;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_GetAttr(m_opCam.hIdxTUCam, &attrProp))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDP_TEMPERATURE range from the camera !";
	}
	DEB_TRACE() << "Temperature range [" << (int) attrProp.dbValMin << " , " << (int) attrProp.dbValMax << "]";

	int temp_middle = (int) attrProp.dbValMax / 2;
	if(((int) temp + temp_middle)<((int) attrProp.dbValMin) || ((int) temp + temp_middle)>((int) attrProp.dbValMax))
	{
		THROW_HW_ERROR(Error) << "Unable to set the Temperature Target !\n"
		 << "It is out of range : "
		 << "["
		 << (int) attrProp.dbValMin - temp_middle
		 << ","
		 << (int) attrProp.dbValMax - temp_middle
		 << "]";
	}

	int nVal = (int) temp + temp_middle;//temperatureTarget is a delta according to the middle temperature !!
	if(TUCAMRET_SUCCESS != TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_TEMPERATURE, nVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Write TUIDP_TEMPERATURE to the camera !";
	}
	m_temperature_target = (double) temp;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTemperatureTarget(double& temp)
{
	DEB_MEMBER_FUNCT();

	temp = m_temperature_target;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTemperature(double& temp)
{
	DEB_MEMBER_FUNCT();

	double dbVal = 0.0f;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, TUIDP_TEMPERATURE, &dbVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDP_TEMPERATURE from the camera !";
	}
	temp = dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setFanSpeed(unsigned speed)
{
	DEB_MEMBER_FUNCT();

	int nVal = (int) speed;

	if(TUCAMRET_SUCCESS != TUCAM_Capa_SetValue(m_opCam.hIdxTUCam, TUIDC_FAN_GEAR, nVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Write TUIDC_FAN_GEAR to the camera !";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getFanSpeed(unsigned& speed)
{
	DEB_MEMBER_FUNCT();

	int nVal;
	if(TUCAMRET_SUCCESS != TUCAM_Capa_GetValue(m_opCam.hIdxTUCam, TUIDC_FAN_GEAR, &nVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDC_FAN_GEAR from the camera !";
	}
	speed = (unsigned) nVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setGlobalGain(unsigned gain)
{
	DEB_MEMBER_FUNCT();

	if(gain != 0 && gain != 1 && gain != 2)
	{
		THROW_HW_ERROR(Error) << "Available gain values are : 0:HDR\n1:HIGH\n2: LOW !";
	}

	double dbVal = (double) gain;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_GLOBALGAIN, dbVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Write TUIDP_GLOBALGAIN to the camera !";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getGlobalGain(unsigned& gain)
{
	DEB_MEMBER_FUNCT();

	double dbVal;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, TUIDP_GLOBALGAIN, &dbVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDP_GLOBALGAIN from the camera !";
	}
	gain = (unsigned) dbVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTucamVersion(std::string& version)
{
	DEB_MEMBER_FUNCT();
	TUCAM_VALUE_INFO valInfo;
	valInfo.nID = TUIDI_VERSION_API;
	if(TUCAMRET_SUCCESS != TUCAM_Dev_GetInfo(m_opCam.hIdxTUCam, &valInfo))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDI_VERSION_API from the camera !";
	}
	version = valInfo.pText;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getFirmwareVersion(std::string& version)
{
	DEB_MEMBER_FUNCT();
	TUCAM_VALUE_INFO valInfo;
	valInfo.nID = TUIDI_VERSION_FRMW;
	if(TUCAMRET_SUCCESS != TUCAM_Dev_GetInfo(m_opCam.hIdxTUCam, &valInfo))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDI_VERSION_FRMW from the camera !";
	}
	version = valInfo.nValue;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  