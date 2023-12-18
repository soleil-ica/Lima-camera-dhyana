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
#include "DhyanaTimer.h"
#include "DhyanaCamera.h"

using namespace lima;
using namespace lima::Dhyana;
using namespace std;

//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera(unsigned short timer_period_ms):
m_depth(16),
m_trigger_mode(IntTrig),
m_status(Ready),
m_acq_frame_nb(0),
m_temperature_target(0),
m_timer_period_ms(timer_period_ms),
m_fps(0.0),
m_tucam_trigger_mode(kTriggerStandard),
m_tucam_trigger_edge_mode(kEdgeRising)
{

	DEB_CONSTRUCTOR();	
	//Init TUCAM	
	init();		
	//create the acquisition thread
	DEB_TRACE() << "Create the acquisition thread";
	m_acq_thread = new AcqThread(*this);
	DEB_TRACE() <<"Create the Internal Trigger Timer";
	m_internal_trigger_timer = new CSoftTriggerTimer(m_timer_period_ms, *this);
	m_acq_thread->start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	// Close camera
	DEB_TRACE() << "Close TUCAM API ...";
	TUCAM_Dev_Close(m_opCam.hIdxTUCam);
	m_opCam.hIdxTUCam = NULL;
	// Uninitialize SDK API environment
	DEB_TRACE() << "Uninitialize TUCAM API ...";
	TUCAM_Api_Uninit();
	//delete the acquisition thread
	DEB_TRACE() << "Delete the acquisition thread";
	delete m_acq_thread;
	//delete the Internal Trigger Timer
	DEB_TRACE() << "Delete the Internal Trigger Timer";
	delete m_internal_trigger_timer;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::init()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Initialize TUCAM API ...";
	m_itApi.pstrConfigPath = "./";//Camera parameters input saving path is not defined
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

	DEB_TRACE() << "Open TUCAM API ...(nb. camera : "<<m_itApi.uiCamCount<<")";
	m_opCam.hIdxTUCam = NULL;
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
	
	//initialize TUCAM Event used when Waiting for Frame
	m_hThdEvent = NULL;

	m_tgroutAttr1.nTgrOutPort = 0;
	m_tgroutAttr1.nTgrOutMode = TucamSignal::kSignalReadEnd;
	m_tgroutAttr1.nEdgeMode = TucamSignalEdge::kSignalEdgeRising;
	m_tgroutAttr1.nDelayTm = 0;
	m_tgroutAttr1.nWidth = 5000;

	m_tgroutAttr2.nTgrOutPort = 1;
	m_tgroutAttr2.nTgrOutMode = TucamSignal::kSignalReadEnd;
	m_tgroutAttr2.nEdgeMode = TucamSignalEdge::kSignalEdgeRising;
	m_tgroutAttr2.nDelayTm = 0;
	m_tgroutAttr2.nWidth = 5000;

	m_tgroutAttr3.nTgrOutPort = 2;
	m_tgroutAttr3.nTgrOutMode = TucamSignal::kSignalReadEnd;
	m_tgroutAttr3.nEdgeMode = TucamSignalEdge::kSignalEdgeRising;
	m_tgroutAttr3.nDelayTm = 0;
	m_tgroutAttr3.nWidth = 5000;

	createParametersMap();
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
	Timestamp t0 = Timestamp::now();

	//@BEGIN : Ensure that Acquisition is Started before return ...
	DEB_TRACE() << "prepareAcq ...";
	DEB_TRACE() << "Ensure that Acquisition is Started";
	setStatus(Camera::Exposure, false);
	if(NULL == m_hThdEvent)
	{
		m_frame.pBuffer = NULL;
		m_frame.ucFormatGet = TUFRM_FMT_USUAl;
		m_frame.uiRsdSize = 1;// how many frames do you want

		// Alloc buffer after set resolution or set ROI attribute
		DEB_TRACE() << "TUCAM_Buf_Alloc";
		TUCAM_Buf_Alloc(m_opCam.hIdxTUCam, &m_frame);

		DEB_TRACE() << "TUCAM_Cap_Start";
		if(m_trigger_mode == IntTrig)
		{
			// Start capture in software trigger
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_SOFTWARE);
		}
		if(m_trigger_mode == IntTrigMult)
		{
			// Start capture in software trigger
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_SOFTWARE);
		}		
		else if(m_trigger_mode == ExtTrigMult)
		{
			// Start capture in external trigger STANDARD (EXPOSURE SOFT)
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_STANDARD);
		}
		else if(m_trigger_mode == ExtGate)
		{
			// Start capture in external trigger STANDARD (EXPOSURE WIDTH)
			TUCAM_Cap_Start(m_opCam.hIdxTUCam, TUCCM_TRIGGER_STANDARD);
		}
		
		////DEB_TRACE() << "TUCAM CreateEvent";
		m_hThdEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	}
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrig)	
	{
		DEB_TRACE() <<"Start Internal Trigger Timer (Single)";
		m_internal_trigger_timer->disable_oneshot_mode();
		m_internal_trigger_timer->start();
	}

	
	//@END
	if(m_trigger_mode == IntTrigMult)
	{
	  _startAcq();
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

	StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrigMult)	
	{
		DEB_TRACE() <<"Start Internal Trigger Timer (Multi)";
		m_internal_trigger_timer->enable_oneshot_mode();
		m_internal_trigger_timer->start();
		return;
	}
	//@END
	m_acq_frame_nb = 0;
	m_fps = 0.0;	
	DEB_TRACE() << "Ensure that Acquisition is Started  & wait thread to be started";
	setStatus(Camera::Exposure, false);		
	////setStatus(Camera::Ready, false);		
	//Start acquisition thread & wait 
	{
		m_wait_flag = false;
		m_quit = false;
		m_cond.broadcast();
		m_cond.wait();
	}
	
	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "startAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::_startAcq()
{
  DEB_MEMBER_FUNCT();
  m_acq_frame_nb = 0;
  m_fps = 0.0;	
  //Start acqusition thread
  AutoMutex aLock(m_cond.mutex());
  m_wait_flag = false;
  m_cond.broadcast();
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cond.mutex());
	DEB_TRACE() << "stopAcq ...";
	// Don't do anything if acquisition is idle.
	if(m_thread_running == true)
	{
		m_wait_flag = true;
		m_cond.broadcast();		
	}

	//@BEGIN : Ensure that Acquisition is Stopped before return ...			
	Timestamp t0 = Timestamp::now();
	if(NULL != m_hThdEvent)
	{
		DEB_TRACE() << "TUCAM_Buf_AbortWait";
		TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam);
		WaitForSingleObject(m_hThdEvent, INFINITE);
		CloseHandle(m_hThdEvent);
		m_hThdEvent = NULL;
		// Stop capture   
		DEB_TRACE() << "TUCAM_Cap_Stop";
		TUCAM_Cap_Stop(m_opCam.hIdxTUCam);
		// Release alloc buffer after stop capture
		DEB_TRACE() << "TUCAM_Buf_Release";
		TUCAM_Buf_Release(m_opCam.hIdxTUCam);
	}
	//@END	
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrig)	
	{
		DEB_TRACE() <<"Stop Internal Trigger Timer (Single)";
		m_internal_trigger_timer->stop();
	}
	//@END
	
	//@BEGIN : trigger the acquisition
	if(m_trigger_mode == IntTrigMult)	
	{
		DEB_TRACE() <<"Stop Internal Trigger Timer (Multi)";
		m_internal_trigger_timer->stop();
	}
	//@END
	
	//@BEGIN
	//now detector is ready
	DEB_TRACE() << "Ensure that Acquisition is Stopped";
	setStatus(Camera::Ready, false);
	//@END	
	
	Timestamp t1 = Timestamp::now();
	double delta_time = t1 - t0;
	DEB_TRACE() << "stopAcq : elapsed time = " << (int) (delta_time * 1000) << " (ms)";		
}

//-----------------------------------------------------
// @brief set the new camera status
//-----------------------------------------------------
void Camera::setStatus(Camera::Status status, bool force)
{
	DEB_MEMBER_FUNCT();
	//AutoMutex aLock(m_cond.mutex());
	if(force || m_status != Camera::Fault)
		m_status = status;
	//m_cond.broadcast();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cond.mutex());
	if(m_trigger_mode == IntTrigMult)
		m_status = Camera::Ready;
		
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

	//@BEGIN : Get frame from Driver/API & copy it into bptr already allocated 
	////DEB_TRACE() << "Copy Buffer image into Lima Frame Ptr";
	memcpy((unsigned short *) bptr, (unsigned short *) (m_frame.pBuffer + m_frame.usOffset), m_frame.uiImgSize);//we need a nb of BYTES .		
	frame_nb = m_frame.uiIndex;
	//@END	

	////Timestamp t1 = Timestamp::now();
	////double delta_time = t1 - t0;
	////DEB_TRACE() << "readFrame : elapsed time = " << (int) (delta_time * 1000) << " (ms)";
	return false;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cam.m_cond.mutex());
	StdBufferCbMgr& buffer_mgr = m_cam.m_bufferCtrlObj.getBuffer();

	while(!m_cam.m_quit)
	{
		while(m_cam.m_wait_flag && !m_cam.m_quit)
		{
			DEB_TRACE() << "Wait for start acquisition ...";
			m_cam.m_thread_running = false;
			m_cam.m_cond.broadcast();
			m_cam.m_cond.wait();
		}

		//if quit is requested (requested only by destructor)
		if(m_cam.m_quit)
			return;

		DEB_TRACE() << "Running ...";
		m_cam.m_thread_running = true;
		m_cam.m_cond.broadcast();
		aLock.unlock();		

		Timestamp t0_capture = Timestamp::now();
		Timestamp t0_fps, t1_fps, delta_fps;

		//@BEGIN 
		DEB_TRACE() << "Capture all frames ...";
		bool continueFlag = true;
		t0_fps = Timestamp::now();
		while(continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames))
		{
			// Check first if acq. has been stopped
			if(m_cam.m_wait_flag)
			{
				DEB_TRACE() << "AcqThread has been stopped from user";
				continueFlag = false;
				continue;
			}

			//set status to exposure
			////m_cam.setStatus(Camera::Exposure, false);
			
			//wait frame from TUCAM API ...
			if(m_cam.m_acq_frame_nb == 0)//display TRACE only once ...
			{				
				DEB_TRACE() << "TUCAM_Buf_WaitForFrame ...";
			}
			
			if(TUCAMRET_SUCCESS == TUCAM_Buf_WaitForFrame(m_cam.m_opCam.hIdxTUCam, &m_cam.m_frame))
			{
				/*
				//The based information
				DEB_TRACE() << "m_cam.m_frame.szSignature = "	<< m_cam.m_frame.szSignature	;		// [out]Copyright+Version: TU+1.0 ['T', 'U', '1', '\0']		
				DEB_TRACE() << "m_cam.m_frame.usHeader = "		<< m_cam.m_frame.usHeader		;		// [out] The frame header size
				DEB_TRACE() << "m_cam.m_frame.usOffset = "		<< m_cam.m_frame.usOffset		;		// [out] The frame data offset
				DEB_TRACE() << "m_cam.m_frame.usWidth = "		<< m_cam.m_frame.usWidth		;		// [out] The frame width
				DEB_TRACE() << "m_cam.m_frame.usHeight = "		<< m_cam.m_frame.usHeight		;		// [out] The frame height
				DEB_TRACE() << "m_cam.m_frame.uiWidthStep = "	<< m_cam.m_frame.uiWidthStep	;		// [out] The frame width step
				DEB_TRACE() << "m_cam.m_frame.ucDepth = "		<< m_cam.m_frame.ucDepth		;		// [out] The frame data depth 
				DEB_TRACE() << "m_cam.m_frame.ucFormat = "		<< m_cam.m_frame.ucFormat		;		// [out] The frame data format                  
				DEB_TRACE() << "m_cam.m_frame.ucChannels = "	<< m_cam.m_frame.ucChannels		;		// [out] The frame data channels
				DEB_TRACE() << "m_cam.m_frame.ucElemBytes = "	<< m_cam.m_frame.ucElemBytes	;		// [out] The frame data bytes per element
				DEB_TRACE() << "m_cam.m_frame.ucFormatGet = "	<< m_cam.m_frame.ucFormatGet	;		// [in]  Which frame data format do you want    see TUFRM_FORMATS
				DEB_TRACE() << "m_cam.m_frame.uiIndex = "		<< m_cam.m_frame.uiIndex		;		// [in/out] The frame index number
				DEB_TRACE() << "m_cam.m_frame.uiImgSize = "		<< m_cam.m_frame.uiImgSize		;		// [out] The frame size
				DEB_TRACE() << "m_cam.m_frame.uiRsdSize = "		<< m_cam.m_frame.uiRsdSize		;		// [in]  The frame reserved size    (how many frames do you want)
				DEB_TRACE() << "m_cam.m_frame.uiHstSize = "		<< m_cam.m_frame.uiHstSize		;		// [out] The frame histogram size	
				*/

				// Grabbing was successful, process image
				m_cam.setStatus(Camera::Readout, false);

				//Prepare Lima Frame Ptr 
				void* bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);

				//Copy Frame into Lima Frame Ptr
				int frame_nb = 0;
				m_cam.readFrame(bptr, frame_nb);
		
				//Push the image buffer through Lima 
				Timestamp t0 = Timestamp::now();
				DEB_TRACE() << "Declare a Lima new Frame Ready (" << m_cam.m_acq_frame_nb << ")";
				HwFrameInfoType frame_info;
				frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
				continueFlag = buffer_mgr.newFrameReady(frame_info);
				m_cam.m_acq_frame_nb++;
				
				////Timestamp t1 = Timestamp::now();
				////double delta_time = t1 - t0;			

				//wait latency after each frame , except for the last image 
				if((!m_cam.m_nb_frames) || (m_cam.m_acq_frame_nb < m_cam.m_nb_frames) && (m_cam.m_lat_time))
				{
					DEB_TRACE() << "Wait latency time : " << m_cam.m_lat_time * 1000 << " (ms) ...";
					usleep((DWORD) (m_cam.m_lat_time * 1000000));
				}		
				////DEB_TRACE() << "newFrameReady+latency : elapsed time = " << (int) (delta_time * 1000) << " (ms)";						
			}
			else
			{
				if(m_cam.m_trigger_mode == IntTrigMult)
					continue;
				DEB_TRACE() << "Unable to get the frame from the camera !";
			}


			t1_fps = Timestamp::now();
			delta_fps = t1_fps - t0_fps;
			if (delta_fps > 0)
			{
				m_cam.m_fps = m_cam.m_acq_frame_nb / delta_fps;
			}
		}

		//
		////DEB_TRACE() << "TUCAM SetEvent";
		SetEvent(m_cam.m_hThdEvent);
		//@END
		
		//stopAcq only if this is not already done		
		DEB_TRACE() << "stopAcq only if this is not already done";
		if(!m_cam.m_wait_flag)
		{
			////DEB_TRACE() << "stopAcq";
			m_cam.stopAcq();
		}

		//now detector is ready
		m_cam.setStatus(Camera::Ready, false);
		DEB_TRACE() << "AcqThread is no more running";		
		
		Timestamp t1_capture = Timestamp::now();
		double delta_time_capture = t1_capture - t0_capture;

		DEB_TRACE() << "Capture all frames elapsed time = " << (int) (delta_time_capture * 1000) << " (ms)";				

		aLock.lock();
		m_cam.m_thread_running = false;
		m_cam.m_wait_flag = true;
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
	m_cam.m_wait_flag = true;
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
	DEB_TRACE() << "setImageType - " << DEB_VAR1(type);
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
	std::string model="UNKNOWN";
	getDetectorModel(model);
	if (model == "Dhyana 95"  || model.find("Dhyana 95 V2") != std::string::npos)
	{
		size = Size(PIXEL_NB_WIDTH_MODEL_95, PIXEL_NB_HEIGHT_MODEL_95);
	}
	else if (model.find("4040") != std::string::npos)
	{
		size = Size(PIXEL_NB_WIDTH_MODEL_4040, PIXEL_NB_HEIGHT_MODEL_4040);
	}	
	else
	{
		THROW_HW_ERROR(NotSupported) << model;
	}
	//@END
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN : Get Pixels size in micron from Driver/API	
	std::string model="UNKNOWN";
	getDetectorModel(model);	
	if (model == "Dhyana 95"  || model.find("Dhyana 95 V2") != std::string::npos)
	{
		sizex = PIXEL_SIZE_WIDTH_MICRON_MODEL_95;
		sizey = PIXEL_SIZE_HEIGHT_MICRON_MODEL_95;
	}
	else if (model.find("4040") != std::string::npos)
	{
		sizex=PIXEL_SIZE_WIDTH_MICRON_MODEL_4040;
		sizey=PIXEL_SIZE_HEIGHT_MICRON_MODEL_4040;
	}
	else
	{
		THROW_HW_ERROR(NotSupported) << model;
	}	
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
	//@BEGIN
	switch(mode)
	{
		case IntTrig:
		case IntTrigMult:
		case ExtTrigMult:
		case ExtGate:
			valid_mode = true;
			break;
		case ExtTrigReadout:
		case ExtTrigSingle:
		default:
			valid_mode = false;
			break;
	}
	//@END
	return valid_mode;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setTrigMode() " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
	//@BEGIN
	TUCAM_TRIGGER_ATTR tgrAttr;
	tgrAttr.nTgrMode = -1;//NOT DEFINED (see below)
	tgrAttr.nFrames = 1;
	tgrAttr.nDelayTm = 0;
	tgrAttr.nExpMode = -1;//NOT DEFINED (see below)
	tgrAttr.nEdgeMode = TUCTD_RISING;

	switch(mode)
	{
		case IntTrig:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE)";
			break;
		case IntTrigMult:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE) (MULTI)";
			break;			
		case ExtTrigMult :
			tgrAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD (EXPOSURE SOFTWARE: "<<tgrAttr.nExpMode<<")";
			break;
		case ExtGate:		
			tgrAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
			tgrAttr.nExpMode = TUCTE_WIDTH;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD (EXPOSURE TRIGGER WIDTH: "<<tgrAttr.nExpMode<<")";
			break;			
		case ExtTrigSingle :		
		case ExtTrigReadout:
		default:
			THROW_HW_ERROR(NotSupported) << DEB_VAR1(mode);
	}
	m_trigger_mode = mode;
	//@END

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_trigger_mode;
}

void Camera::getTriggerMode(TucamTriggerMode &mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_tucam_trigger_mode;
}

void Camera::setTriggerMode(TucamTriggerMode mode)
{
	DEB_MEMBER_FUNCT();
	m_tucam_trigger_mode = mode;
}

void Camera::getTriggerEdge(TucamTriggerEdge &edge)
{
	DEB_MEMBER_FUNCT();
	edge = m_tucam_trigger_edge_mode;
}

void Camera::setTriggerEdge(TucamTriggerEdge edge)
{
	DEB_MEMBER_FUNCT();
	m_tucam_trigger_edge_mode = edge;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	double dbVal;
	if(TUCAMRET_SUCCESS != TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, &dbVal))
	{
		THROW_HW_ERROR(Error) << "Unable to Read TUIDP_EXPOSURETM from the camera !";
	}
	m_exp_time = dbVal / 1000;//TUCAM use (ms), but lima use (second) as unit 
	//@END
	exp_time = m_exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setExpTime() " << DEB_VAR1(exp_time);
	//@BEGIN
	if(TUCAMRET_SUCCESS != TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, exp_time * 1000))//TUCAM use (ms), but lima use (second) as unit 
	{
		THROW_HW_ERROR(Error) << "Unable to Write TUIDP_EXPOSURETM to the camera !";
	}
	//@END
	m_exp_time = exp_time;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setLatTime() " << DEB_VAR1(lat_time);
	m_lat_time = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	//@END
	m_lat_time = lat_time;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	min_expo = 0.;
	max_expo = 10;//10s
	//@END
	DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
	DEB_MEMBER_FUNCT();
	//@BEGIN
	// --- no info on min latency
	min_lat = 0.;
	// --- do not know how to get the max_lat, fix it as the max exposure time
	max_lat = 10;//10s
	//@END
	DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setNbFrames() " << DEB_VAR1(nb_frames);
	//@BEGIN
	if(nb_frames < 0)
	{
		THROW_HW_ERROR(Error) << "Number of frames to acquire has not been set";
	}
	if(nb_frames > 1)
	{
		//THROW_HW_ERROR(Error) << "Unable to acquire more than 1 frame ! ";	//T@D@ allowed only for test 
	}
	//@END
	m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(m_nb_frames);
	//@BEGIN
	//@END
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
	DEB_TRACE() << "checkRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : check available values of Roi
	if(set_roi.isActive())
	{
		hw_roi = set_roi;
	}
	else
	{
		hw_roi = set_roi;
	}
	//@END


	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
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
	DEB_TRACE() << "setRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	//@BEGIN : set Roi from the Driver/API	
	if(!set_roi.isActive())
	{
		DEB_TRACE() << "Roi is not Enabled : so set full frame";

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

//-----------------------------------------------------------------------------
/// Get the frame rate
//-----------------------------------------------------------------------------
void Camera::getFPS(double& fps) ///< [out] last computed fps
{
    DEB_MEMBER_FUNCT();

    fps = m_fps;
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::setTecMode(unsigned mode)
{
	DEB_MEMBER_FUNCT();

	if(mode != 0 && mode != 1)
	{
		THROW_HW_ERROR(Error) << "Available mode values are :\n0:OFF\n1:ON";
	}

	int nVal = (int) mode;
	if(TUCAMRET_SUCCESS != TUCAM_Capa_SetValue(m_opCam.hIdxTUCam, TUIDC_ENABLETEC, nVal))
	{
		DEB_TRACE() << "Unable to Write TUIDC_ENABLETEC from the camera!";
		THROW_HW_ERROR(Error) << "Unable to Write TUIDC_ENABLETEC to the camera !";
	}
}

//-----------------------------------------------------
//
//-----------------------------------------------------  
void Camera::getTecMode(unsigned& mode)
{
	DEB_MEMBER_FUNCT();

	int nVal;
	if(TUCAMRET_SUCCESS != TUCAM_Capa_GetValue(m_opCam.hIdxTUCam, TUIDC_ENABLETEC, &nVal))
	{
		DEB_TRACE() << "Unable to Read TUIDC_ENABLETEC from the camera!";
		THROW_HW_ERROR(Error) << "Unable to Read TUIDC_ENABLETEC from the camera !";
	}
	mode = (unsigned) nVal;
}

//-----------------------------------------------------
// Set trigger outpout on selected port
//-----------------------------------------------------  
void Camera::setOutputSignal(int port, TucamSignal signal, TucamSignalEdge edge, int delay, int width)
{
	DEB_MEMBER_FUNCT();

	TUCAM_TRGOUT_ATTR tgroutAttr;

	switch (port)
	{
	case 0:
		m_tgroutAttr1.nTgrOutMode = signal;
		m_tgroutAttr1.nEdgeMode = edge;
		m_tgroutAttr1.nDelayTm = delay * 1000;
		m_tgroutAttr1.nWidth = width * 1000;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_SetTriggerOut(m_opCam.hIdxTUCam, m_tgroutAttr1))
		{
			THROW_HW_ERROR(Error) << "Unable to set Output signal port " << port;
		}

		tgroutAttr.nTgrOutPort = port;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_GetTriggerOut(m_opCam.hIdxTUCam, &tgroutAttr))
		{
			THROW_HW_ERROR(Error) << "Unable to get Output signal port " << port;
		}

		m_tgroutAttr1.nTgrOutMode = tgroutAttr.nTgrOutMode;
		m_tgroutAttr1.nEdgeMode = tgroutAttr.nEdgeMode;
		m_tgroutAttr1.nDelayTm = tgroutAttr.nDelayTm;
		m_tgroutAttr1.nWidth = tgroutAttr.nWidth;
		break;

	case 1:

		m_tgroutAttr2.nTgrOutMode = signal;
		m_tgroutAttr2.nEdgeMode = edge;
		m_tgroutAttr2.nDelayTm = delay * 1000;
		m_tgroutAttr2.nWidth = width * 1000;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_SetTriggerOut(m_opCam.hIdxTUCam, m_tgroutAttr2))
		{
			THROW_HW_ERROR(Error) << "Unable to set Output signal port " << port;
		}

		tgroutAttr.nTgrOutPort = port;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_GetTriggerOut(m_opCam.hIdxTUCam, &tgroutAttr))
		{
			THROW_HW_ERROR(Error) << "Unable to get Output signal port " << port;
		}
		m_tgroutAttr2.nTgrOutMode = tgroutAttr.nTgrOutMode;
		m_tgroutAttr2.nEdgeMode = tgroutAttr.nEdgeMode;
		m_tgroutAttr2.nDelayTm = tgroutAttr.nDelayTm;
		m_tgroutAttr2.nWidth = tgroutAttr.nWidth;
		break;

	case 2:

		m_tgroutAttr3.nTgrOutMode = signal;
		m_tgroutAttr3.nTgrOutMode = signal;
		m_tgroutAttr3.nEdgeMode = edge;
		m_tgroutAttr3.nDelayTm = delay * 1000;
		m_tgroutAttr3.nWidth = width * 1000;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_SetTriggerOut(m_opCam.hIdxTUCam, m_tgroutAttr3))
		{
			THROW_HW_ERROR(Error) << "Unable to set Output signal port " << port;
		}

		tgroutAttr.nTgrOutPort = port;

		if (TUCAMRET_SUCCESS != TUCAM_Cap_GetTriggerOut(m_opCam.hIdxTUCam, &tgroutAttr))
		{
			THROW_HW_ERROR(Error) << "Unable to get Output signal port " << port;
		}
		m_tgroutAttr3.nTgrOutMode = tgroutAttr.nTgrOutMode;
		m_tgroutAttr3.nEdgeMode = tgroutAttr.nEdgeMode;
		m_tgroutAttr3.nDelayTm = tgroutAttr.nDelayTm;
		m_tgroutAttr3.nWidth = tgroutAttr.nWidth;
		break;

	default:
		THROW_HW_ERROR(Error) << "Unable to set Output signal port " << port;
		break;
	}
}

//-----------------------------------------------------
// Get trigger outpout on selected port
//----------------------------------------------------- 
void Camera::getOutputSignal(int port, TucamSignal& signal, TucamSignalEdge& edge, int& delay, int& width)
{
  DEB_MEMBER_FUNCT();

  switch (port)
  {
  case 0:
	  signal = (TucamSignal)m_tgroutAttr1.nTgrOutMode;
	  edge = (TucamSignalEdge)m_tgroutAttr1.nEdgeMode;
	  delay = m_tgroutAttr1.nDelayTm;
	  width = m_tgroutAttr1.nWidth;
	  break;

  case 1:
	  signal = (TucamSignal)m_tgroutAttr2.nTgrOutMode;
	  edge = (TucamSignalEdge)m_tgroutAttr2.nEdgeMode;
	  delay = m_tgroutAttr2.nDelayTm;
	  width = m_tgroutAttr2.nWidth;
	  break;

  case 2:
	  signal = (TucamSignal)m_tgroutAttr3.nTgrOutMode;
	  edge = (TucamSignalEdge)m_tgroutAttr3.nEdgeMode;
	  delay = m_tgroutAttr3.nDelayTm;
	  width = m_tgroutAttr3.nWidth;
	  break;

  default:
	  THROW_HW_ERROR(Error) << "Unable to set Output signal port " << port;
	  break;
  }
}

//-----------------------------------------------------
// is trigger out attribute is available
//----------------------------------------------------- 
bool Camera::is_trigOutput_available()
{
	DEB_MEMBER_FUNCT();	
	bool is_trigOutput_available = true;

	TUCAM_TRGOUT_ATTR tgroutAttr;
	if (TUCAMRET_SUCCESS != TUCAM_Cap_GetTriggerOut(m_opCam.hIdxTUCam, &tgroutAttr))
	{
		is_trigOutput_available = false;
		DEB_TRACE() << "Unable to get trigger out attribute from the camera!";
	}
	
	return is_trigOutput_available;
}


//-----------------------------------------------------
// Get selected parameter value
//----------------------------------------------------- 
std::string Camera::getParameter(std::string parameter_name)
{
	DEB_MEMBER_FUNCT();
    std::stringstream result;
	result.str("");

	std::map<std::string, int>::const_iterator it = m_parameters_map.find(parameter_name);
	//Check if parameter_name exists
	if(it == m_parameters_map.end())
	{
		THROW_HW_ERROR(Error) << parameter_name << " is not an available parameter for the camera !";
	}
	else
	{
		// it->second is the parameter ID
		result << getParameterValue(parameter_name, it->second).str();
	}
		
    return result.str();
}

//-------------------------------------------
// @brief Get the list of all parameters and belong values 
//--------------------------------------------
std::string Camera::getAllParameters()
{
	DEB_MEMBER_FUNCT();
    std::stringstream result; 
	std::stringstream temp_result; 
	result.str("");
	temp_result.str("");

	std::map<std::string, int>::const_iterator it;
	for (it = m_parameters_map.begin(); it != m_parameters_map.end(); ++it)
	{		
		//Get paramter current value
		temp_result << getParameter(it->first);
		//We didplay only parameters with values successfully read
		if(!temp_result.str().empty())
		{
			result << it->first << "=" <<  temp_result.str();
			temp_result.str("");
		}
	}

	return result.str();
}

//-----------------------------------------------------
// @brief Set the value of the specified parameter
//-----------------------------------------------------
void Camera::setParameter(std::string parameter_name, std::string value_str)
{
	DEB_MEMBER_FUNCT();

	std::map<std::string, int>::const_iterator it = m_parameters_map.find(parameter_name);
	//Check if the parameter name exists
	if(it == m_parameters_map.end())
	{
		THROW_HW_ERROR(Error) << parameter_name << " is not an available parameter for the camera !";
	}
	else
	{
		//Convert the value from string to double
		std::stringstream str_stream;
		str_stream << value_str;
		double value = 0.0;
		str_stream >> value;
		
		//Set a control parameter
		if(parameter_name.find("TUIDP_") != std::string::npos)
		{
			if(TUCAMRET_SUCCESS != TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, it->second, value))
			{
				THROW_HW_ERROR(Error) << "Unable to Write " << parameter_name << " to the camera !";
			}
		}
		//Set a capability parameter
		else if(parameter_name.find("TUIDC_") != std::string::npos)
		{
			if(TUCAMRET_SUCCESS != TUCAM_Capa_SetValue(m_opCam.hIdxTUCam, it->second, (int)value))
			{
				THROW_HW_ERROR(Error) << "Unable to Write " << parameter_name << " to the camera !";
			}	
		}
		//Set a process image parameter
		else if(parameter_name.find("TUIDPP_") != std::string::npos)
		{
			if(TUCAMRET_SUCCESS != TUCAM_Proc_Prop_SetValue(m_opCam.hIdxTUCam, it->second, value))
			{
				THROW_HW_ERROR(Error) << "Unable to Write " << parameter_name << " to the camera !";
			}
		}
		//Set a vendor control parameter
		else if(parameter_name.find("TUIDV_") != std::string::npos)
		{
			if(TUCAMRET_SUCCESS != TUCAM_Vendor_Prop_SetValue(m_opCam.hIdxTUCam, it->second, value))
			{
				THROW_HW_ERROR(Error) << "Unable to Write " << parameter_name << " to the camera !";
			}
		}
	}

}

//---------------------------------------
// @brief Get parameter current value 
//---------------------------------------
std::stringstream Camera::getParameterValue(std::string parameter_name, int parameter_id)
{
	DEB_MEMBER_FUNCT();
	
	std::stringstream result;
	result.str("");
	double double_value = 0.0f;
	int int_value = 0;

	if(parameter_name.find("TUIDP_") != std::string::npos)
	{
		if(TUCAMRET_SUCCESS != TUCAM_Prop_GetValue(m_opCam.hIdxTUCam, parameter_id, &double_value))
		{
			DEB_TRACE() << "Unable to Read " << parameter_name <<  " from the camera!";
		}
		else
		{
			result << double_value << std::endl;
		}
	}
	else if(parameter_name.find("TUIDC_") != std::string::npos)
	{
		if(TUCAMRET_SUCCESS != TUCAM_Capa_GetValue(m_opCam.hIdxTUCam, parameter_id, &int_value))
		{
			DEB_TRACE() << "Unable to Read " << parameter_name <<  " from the camera!";
		}
		else
		{
			result << int_value << std::endl;
		}
	}
	else if(parameter_name.find("TUIDPP_") != std::string::npos)
	{
		if(TUCAMRET_SUCCESS != TUCAM_Proc_Prop_GetValue(m_opCam.hIdxTUCam, parameter_id, &double_value))
		{
			DEB_TRACE() << "Unable to Read " << parameter_name <<  " from the camera!";
		}
		else
		{
			result <<  double_value << std::endl;
		}
	}
	else if(parameter_name.find("TUIDV_") != std::string::npos)
	{
		if(TUCAMRET_SUCCESS != TUCAM_Vendor_Prop_GetValue(m_opCam.hIdxTUCam, parameter_id, &double_value))
		{
			DEB_TRACE() << "Unable to Read " << parameter_name <<  " from the camera!";
		}
		else
		{
			result << double_value << std::endl;
		}
	}

	return result;
}

//--------------------------------------------------------
// @brief Create a map including all availbla parameters 
//--------------------------------------------------------
void Camera::createParametersMap()
{
	DEB_MEMBER_FUNCT();
	
	// TUCAM_IDPROP: Control Properties
	m_parameters_map["TUIDP_GLOBALGAIN"]         = 0x00;
    m_parameters_map["TUIDP_EXPOSURETM"]         = 0x01;
    m_parameters_map["TUIDP_BRIGHTNESS"]         = 0x02;
    m_parameters_map["TUIDP_BLACKLEVEL"]         = 0x03;
    m_parameters_map["TUIDP_TEMPERATURE"]        = 0x04;
    m_parameters_map["TUIDP_SHARPNESS"]          = 0x05;
    m_parameters_map["TUIDP_NOISELEVEL"]         = 0x06;
    m_parameters_map["TUIDP_HDR_KVALUE"]         = 0x07;
    m_parameters_map["TUIDP_GAMMA"]         	 = 0x08;
    m_parameters_map["TUIDP_CONTRAST"]           = 0x09;
    m_parameters_map["TUIDP_LFTLEVELS"]          = 0x0A;
    m_parameters_map["TUIDP_RGTLEVELS"]          = 0x0B;
    m_parameters_map["TUIDP_CHNLGAIN"]         	 = 0x0C;
    m_parameters_map["TUIDP_SATURATION"]         = 0x0D;
    m_parameters_map["TUIDP_CLRTEMPERATURE"]     = 0x0E;
    m_parameters_map["TUIDP_CLRMATRIX "]         = 0x0F;
    m_parameters_map["TUIDP_DPCLEVEL"]         	 = 0x10;
    m_parameters_map["TUIDP_BLACKLEVELHG"]       = 0x11;
    m_parameters_map["TUIDP_BLACKLEVELLG"]       = 0x12;
	m_parameters_map["TUIDP_POWEEFREQUENCY"]     = 0x13;
	m_parameters_map["TUIDP_HUE"]         		 = 0x14;
	m_parameters_map["TUIDP_LIGHT"]         	 = 0x15;
    m_parameters_map["TUIDP_ENHANCE_STRENGTH"]   = 0x16;
    m_parameters_map["TUIDP_NOISELEVEL_3D"]      = 0x17;
    m_parameters_map["TUIDP_FOCUS_POSITION"]     = 0x18;
	m_parameters_map["TUIDP_FRAME_RATE"]         = 0x19;
	m_parameters_map["TUIDP_START_TIME"]         = 0x1A;
	m_parameters_map["TUIDP_FRAME_NUMBER"]       = 0x1B;
	m_parameters_map["TUIDP_INTERVAL_TIME"]      = 0x1C;
	m_parameters_map["TUIDP_GPS_APPLY"]          = 0x1D;
	m_parameters_map["TUIDP_AMB_TEMPERATURE"]    = 0x1E;
	m_parameters_map["TUIDP_AMB_HUMIDITY"]       = 0x1F;
	m_parameters_map["TUIDP_AUTO_CTRLTEMP"]      = 0x20;
	m_parameters_map["TUIDP_AVERAGEGRAY"]        = 0x21;
	m_parameters_map["TUIDP_AVERAGEGRAYTHD"]     = 0x22;
	m_parameters_map["TUIDP_ENHANCETHD"]         = 0x23;
	m_parameters_map["TUIDP_ENHANCEPARA"]        = 0x24;
	m_parameters_map["TUIDP_EXPOSUREMAX"]        = 0x25;
	m_parameters_map["TUIDP_EXPOSUREMIN"]        = 0x26;
	m_parameters_map["TUIDP_GAINMAX"]        	 = 0x27;
	m_parameters_map["TUIDP_GAINMIN"]            = 0x28;
	m_parameters_map["TUIDP_THROUGHFOGPARA"]     = 0x29;
    m_parameters_map["TUIDP_ENDPROPERTY"]        = 0x2A;
	
	//TUCAM_IDCAPA: Capability Properties 
	m_parameters_map["TUIDC_RESOLUTION"]            = 0x00;
    m_parameters_map["TUIDC_PIXELCLOCK"]            = 0x01;
    m_parameters_map["TUIDC_BITOFDEPTH"]            = 0x02;
    m_parameters_map["TUIDC_ATEXPOSURE"]            = 0x03;
    m_parameters_map["TUIDC_HORIZONTAL"]            = 0x04;
    m_parameters_map["TUIDC_VERTICAL"]              = 0x05;
    m_parameters_map["TUIDC_ATWBALANCE"]            = 0x06;
    m_parameters_map["TUIDC_FAN_GEAR"]              = 0x07;
    m_parameters_map["TUIDC_ATLEVELS"]              = 0x08;
    m_parameters_map["TUIDC_SHIFT"]                 = 0x09;
    m_parameters_map["TUIDC_HISTC"]                 = 0x0A;
    m_parameters_map["TUIDC_CHANNELS"]              = 0x0B;
    m_parameters_map["TUIDC_ENHANCE"]               = 0x0C;
    m_parameters_map["TUIDC_DFTCORRECTION"]         = 0x0D;
    m_parameters_map["TUIDC_ENABLEDENOISE"]         = 0x0E;
    m_parameters_map["TUIDC_FLTCORRECTION"]         = 0x0F;
    m_parameters_map["TUIDC_RESTARTLONGTM"]         = 0x10;
    m_parameters_map["TUIDC_DATAFORMAT"]            = 0x11;
    m_parameters_map["TUIDC_DRCORRECTION"]          = 0x12;
    m_parameters_map["TUIDC_VERCORRECTION"]         = 0x13;
    m_parameters_map["TUIDC_MONOCHROME"]            = 0x14;
    m_parameters_map["TUIDC_BLACKBALANCE"]          = 0x15;
    m_parameters_map["TUIDC_IMGMODESELECT"]         = 0x16;
    m_parameters_map["TUIDC_CAM_MULTIPLE"]          = 0x17;
	m_parameters_map["TUIDC_ENABLEPOWEEFREQUENCY"]  = 0x18;
	m_parameters_map["TUIDC_ROTATE_R90"]			= 0x19;
	m_parameters_map["TUIDC_ROTATE_L90"]			= 0x1A;
	m_parameters_map["TUIDC_NEGATIVE"]			    = 0x1B;
	m_parameters_map["TUIDC_HDR"]				    = 0x1C;
    m_parameters_map["TUIDC_ENABLEIMGPRO"]          = 0x1D;
    m_parameters_map["TUIDC_ENABLELED"]             = 0x1E;
    m_parameters_map["TUIDC_ENABLETIMESTAMP"]       = 0x1F;
    m_parameters_map["TUIDC_ENABLEBLACKLEVEL"]      = 0x20;
    m_parameters_map["TUIDC_ATFOCUS"]               = 0x21;
    m_parameters_map["TUIDC_ATFOCUS_STATUS"]        = 0x22;
	m_parameters_map["TUIDC_PGAGAIN"]               = 0x23;
    m_parameters_map["TUIDC_ATEXPOSURE_MODE"]       = 0x24;
    m_parameters_map["TUIDC_BINNING_SUM"]           = 0x25;
    m_parameters_map["TUIDC_BINNING_AVG"]           = 0x26;
    m_parameters_map["TUIDC_FOCUS_C_MOUNT"]         = 0x27;
	m_parameters_map["TUIDC_ENABLEPI"]              = 0x28;
    m_parameters_map["TUIDC_ATEXPOSURE_STATUS"]     = 0x29;
    m_parameters_map["TUIDC_ATWBALANCE_STATUS"]     = 0x2A;
	m_parameters_map["TUIDC_TESTIMGMODE"]           = 0x2B;
	m_parameters_map["TUIDC_SENSORRESET"]           = 0x2C;
	m_parameters_map["TUIDC_PGAHIGH"]               = 0x2D;
	m_parameters_map["TUIDC_PGALOW"]                = 0x2E;
	m_parameters_map["TUIDC_PIXCLK1_EN"]            = 0x2F;
	m_parameters_map["TUIDC_PIXCLK2_EN"]            = 0x30;
	m_parameters_map["TUIDC_ATLEVELGEAR"]           = 0x31;
	m_parameters_map["TUIDC_ENABLEDSNU"]            = 0x32;
	m_parameters_map["TUIDC_ENABLEOVERLAP"]         = 0x33;
	m_parameters_map["TUIDC_CAMSTATE"]              = 0x34;
	m_parameters_map["TUIDC_ENABLETRIOUT"]          = 0x35;
	m_parameters_map["TUIDC_ROLLINGSCANMODE"]       = 0x36;
	m_parameters_map["TUIDC_ROLLINGSCANLTD"]        = 0x37;
	m_parameters_map["TUIDC_ROLLINGSCANSLIT"]       = 0x38;
	m_parameters_map["TUIDC_ROLLINGSCANDIR"]        = 0x39;
	m_parameters_map["TUIDC_ROLLINGSCANRESET"]      = 0x3A;
	m_parameters_map["TUIDC_ENABLETEC"]           	= 0x3B;
	m_parameters_map["TUIDC_ENABLEBLC"]           	= 0x3C;
	m_parameters_map["TUIDC_ENABLETHROUGHFOG"]      = 0x3D;
	m_parameters_map["TUIDC_ENABLEGAMMA"]           = 0x3E;
	m_parameters_map["TUIDC_CAMPARASAVE"]           = 0x3F;
	m_parameters_map["TUIDC_CAMPARALOAD"]           = 0x40;
    m_parameters_map["TUIDC_ENDCAPABILITY"]         = 0x41;
	
	//TUCAM_IDPPROP : Process image Properties
	m_parameters_map["TUIDPP_EDF_QUALITY"] 		= 0;
	m_parameters_map["TUIDPP_STITCH_SPEED"] 	= 1;
	m_parameters_map["TUIDPP_STITCH_BGC_RED"] 	= 2;
	m_parameters_map["TUIDPP_STITCH_BGC_GREEN"] = 3;
	m_parameters_map["TUIDPP_STITCH_BGC_BLUE"] 	= 4;
	m_parameters_map["TUIDPP_STITCH_VALID"] 	= 5;
	m_parameters_map["TUIDPP_STITCH_AREA_X"] 	= 6;
	m_parameters_map["TUIDPP_STITCH_AREA_Y"] 	= 7;
	m_parameters_map["TUIDPP_STITCH_NEXT_X"] 	= 8;
	m_parameters_map["TUIDPP_STITCH_NEXT_Y"] 	= 9;
	m_parameters_map["TUIDPP_ENDPPROPERTY"] 	= 10;
	
	//TUCAM_IDVPROP : Vendor Properties
 	m_parameters_map["TUIDV_ADDR_FLASH"]        = 0x00;
	m_parameters_map["TUIDV_ODDEVENH"]          = 0x01;
	m_parameters_map["TUIDV_ODDEVENL"]          = 0x02;
	m_parameters_map["TUIDV_HDRHGBOFFSET"]      = 0x03;
	m_parameters_map["TUIDV_HDRLGBOFFSET"]      = 0x04;
	m_parameters_map["TUIDV_CMSHGBOFFSET"]      = 0x05;
	m_parameters_map["TUIDV_CMSLGBOFFSET"]      = 0x06;
	m_parameters_map["TUIDV_FPNENABLE"]      	= 0x07;
	m_parameters_map["TUIDV_WORKING_TIME"]      = 0x08;
	m_parameters_map["TUIDV_CALC_DSNU"]      	= 0x09;
	m_parameters_map["TUIDV_CALC_PRNU"]      	= 0x0A;
	m_parameters_map["TUIDV_CALC_DPC"]      	= 0x0B;
	m_parameters_map["TUIDV_CALC_STOP"]      	= 0x0C;
	m_parameters_map["TUIDV_CALC_STATE"]      	= 0x0D;
	m_parameters_map["TUIDV_HDR_LVALUE"]      	= 0x0E;
	m_parameters_map["TUIDV_HDR_HVALUE"]      	= 0x0F;
	m_parameters_map["TUIDV_FW_CHECK"]      	= 0x10;
	m_parameters_map["TUIDV_HIGHSPEEDHGBOFFSET"]= 0x11;
	m_parameters_map["TUIDV_HIGHSPEEDLGBOFFSET"]= 0x12;
	m_parameters_map["TUIDV_TEMPERATURE_OFFSET"]= 0x13;
    m_parameters_map["TUIDV_ENDVPROPERTY"]      = 0x14;
}


