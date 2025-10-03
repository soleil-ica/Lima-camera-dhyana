//#################
// This main file was used to troubleshoot significant delays when using Tucsen SDK with Dhyana cameras.
// It can trigger an acquisition with direct calls to the SDK, or run the acquisition through Lima.
//#################
#include <iostream>
#include <yat/utils/XString.h>
#include <yat/time/Timer.h>
#include <yat/utils/StringTokenizer.h>
#include <yat/utils/String.h>
#include <yat/threading/Task.h>
#include <exception>
#include <chrono>
#include <thread>

#include <lima/HwInterface.h>
#include <lima/CtControl.h>
#include <lima/CtEvent.h>
#include <lima/CtBuffer.h>
#include <lima/CtImage.h>
#include <lima/CtAcquisition.h>
#include <DhyanaBinCtrlObj.h>
#include <DhyanaInterface.h>

#include <ctime>

#include "TUCamApi.h"
#include "TUDefine.h"


#define TASK_PREPARE_ACQ 10000

//global variables

TUCAM_INIT m_itApi;
TUCAM_OPEN m_opCam;
TUCAM_FRAME m_frame;

lima::Dhyana::Camera* m_camera;
lima::Dhyana::Interface* m_interface;
lima::CtControl* m_control;

unsigned m_exp_time_ms = 100;
unsigned m_nb_frames = 2;
unsigned m_nb_loops = 3;
std::string m_file_target = "DO_NOT_SAVE_FILE";


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//measure duration of execution of a function functionCall:
//- if function return  TUCAMRET_SUCCESS, then compute and print duration of execution of this funct_name
//- if function return !TUCAMRET_SUCCESS, then throw exception containing funct_name msg
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MEASURE_TIME(functionCall, funct_name)                   							\
do 																							\
{                                                         									\
	clock_t start_time = clock();                            								\
	functionCall;																					\
	clock_t end_time = clock();                              								\
	double duration = static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC; \
	std::cout << "[Elapsed time "<< funct_name <<" :  "<< duration << " ms]" << std::endl; 	\
}																							\
while (0)				

																	\





void init_lima_device()
{
	// Logs init
	std::vector<std::string> debugModules = {"Control", "Camera"};
	lima::DebParams::setModuleFlagsNameList(debugModules);
	std::vector<std::string> debugLevels = {"Fatal", "Error", "Warning", "Trace"};
	lima::DebParams::setTypeFlagsNameList(debugLevels);
	std::vector<std::string> debugFormats = {"DateTime", "Module", "Funct"};
	lima::DebParams::setFormatFlagsNameList(debugFormats);

    unsigned short time_period = 1;
    m_camera = new lima::Dhyana::Camera(time_period);
    m_interface = new lima::Dhyana::Interface(*(static_cast<lima::Dhyana::Camera*> (m_camera)));
    m_control = new lima::CtControl(m_interface);
	
    m_control->event()->resetEventList();
    m_control->buffer()->setMaxMemory((short) 70);
    m_control->image()->reset();
    m_control->acquisition()->setAcqNbFrames(5);

	lima::Roi roi;
	m_camera->getRoi(roi);
}

void lima_snap()
{
	std::cout << "prepareAcq" << std::endl;
	MEASURE_TIME(m_control->prepareAcq(), "Lima PrepareAcq");
	std::cout << "startAcq" << std::endl;
	MEASURE_TIME(m_control->startAcq(), "Lima startAcq");
	std::cout << "snap finished" << std::endl;
}

bool prepare_acq()
{
	std::cout << "prepare_acq ..." << std::endl;
	MEASURE_TIME(TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, m_exp_time_ms),"TUCAM_Prop_SetValue");//fix exposure time

    m_frame.pBuffer = NULL;
	m_frame.ucFormatGet = TUFRM_FMT_USUAl; 	// Frame data format
    m_frame.uiRsdSize = 1; 					// number of frames captured at a time

	MEASURE_TIME(TUCAM_Buf_Alloc(m_opCam.hIdxTUCam, &m_frame),"TUCAM_Buf_Alloc");

    MEASURE_TIME(TUCAM_Cap_Start(m_opCam.hIdxTUCam, (UINT32)TUCCM_TRIGGER_SOFTWARE),"TUCAM_Cap_Start");
	MEASURE_TIME(TUCAM_Cap_Stop(m_opCam.hIdxTUCam),"TUCAM_Cap_Stop");
	MEASURE_TIME(TUCAM_Cap_Start(m_opCam.hIdxTUCam, (UINT32)TUCCM_TRIGGER_SOFTWARE),"TUCAM_Cap_Start");
    std::cout << "prepare_acq done\n" << std::endl;

    return true;
}

class YatTask : public yat::Task
{
	public:
	YatTask::YatTask ()
	  : yat::Task(Config(true,     //- enable timeout msg
						 1000,     //- every second (i.e. 1000 msecs)
						 false,    //- disable periodic msgs
						 0,        //- no exec period for periodic msgs (disabled)
						 false,    //- don't lock the internal mutex while handling a msg (recommended setting)
						 128,   //- msgQ low watermark value
						 512,   //- msgQ high watermark value
						 false,    //- do not throw exception on post msg timeout (msqQ saturated)
						 0))      //- user data (same for all msgs) - we don't use it here

	{

	}

	// ============================================================================
	// Consumer::handle_message
	// ============================================================================
	void YatTask::handle_message (yat::Message& _msg)
	{
		std::cout << "YatTask handling message" <<std::endl;
		//- handle msg
	  switch (_msg.type())
		{
		  //- TASK_INIT ----------------------
		  case yat::TASK_INIT:
			{
			//- "initialization" code goes here
			std::cout << "Consumer::handle_message::TASK_INIT::task is starting up" <<std::endl;
		  }
			  break;
			//- TASK_PERIODIC ------------------
			case yat::TASK_PERIODIC:
			  {
			  //- code relative to the task's periodic job goes here
			  std::cout << "Consumer::handle_message::handling TASK_PERIODIC msg" <<std::endl;
		  }
			  break;
			  //- TASK_PREPARE_ACQ ------------------
			  case TASK_PREPARE_ACQ:
			  std::cout << "Consumer::handle_message::TASK_PREPARE_ACQ::task is starting up" <<std::endl;
			  prepare_acq();
		default:
			std::cout << "Consumer::handle_message::unhandled msg type received" <<std::endl;
			break;
		}
	}

};


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void init()
{
	std::cout << "init ..." << std::endl;

	//Api_Init

    m_itApi.pstrConfigPath = NULL;

	m_itApi.uiCamCount = 0;

	MEASURE_TIME(TUCAM_Api_Init(&m_itApi), "TUCAM_Api_Init");

	//Drv_Open
    m_opCam.hIdxTUCam = 0;
	m_opCam.uiIdxOpen = 0;
	MEASURE_TIME(TUCAM_Dev_Open(&m_opCam), "TUCAM_Dev_Open");

	//display TUCAM version
	TUCAM_VALUE_INFO valInfo;
	valInfo.nID = TUIDI_VERSION_API;
	MEASURE_TIME(TUCAM_Dev_GetInfo(m_opCam.hIdxTUCam, &valInfo),"TUCAM_Dev_GetInfo");
	std::cout<<"TUCAM version : "<<std::string(valInfo.pText)<<std::endl;
	//MEASURE_TIME(TUCAM_Prop_SetValue(m_opCam.hIdxTUCam, TUIDP_EXPOSURETM, m_exp_time_ms),"TUCAM_Prop_SetValue");//fix exposure time
	
	lima::TrigMode mode = lima::IntTrig;
	
	TUCAM_TRIGGER_ATTR tgrAttr;
	tgrAttr.nTgrMode = -1;//NOT DEFINED (see below)
	tgrAttr.nFrames = 1;
	tgrAttr.nDelayTm = 0;
	tgrAttr.nExpMode = -1;//NOT DEFINED (see below)
	tgrAttr.nEdgeMode = TUCTD_RISING;
	switch(mode)
	{
		case lima::IntTrig:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			//DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE)";
			break;
		case lima::IntTrigMult:
			tgrAttr.nTgrMode = TUCCM_TRIGGER_SOFTWARE;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			//DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_SOFTWARE (EXPOSURE SOFTWARE) (MULTI)";
			break;			
		case lima::ExtTrigMult :
			tgrAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
			tgrAttr.nExpMode = TUCTE_EXPTM;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			//DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD (EXPOSURE SOFTWARE: "<<tgrAttr.nExpMode<<")";
			break;
		case lima::ExtGate:		
			tgrAttr.nTgrMode = TUCCM_TRIGGER_STANDARD;
			tgrAttr.nExpMode = TUCTE_WIDTH;
			TUCAM_Cap_SetTrigger(m_opCam.hIdxTUCam, tgrAttr);
			//DEB_TRACE() << "TUCAM_Cap_SetTrigger : TUCCM_TRIGGER_STANDARD (EXPOSURE TRIGGER WIDTH: "<<tgrAttr.nExpMode<<")";
			break;			
		// case ExtTrigSingle :		
		// case ExtTrigReadout:
		// default:
			// THROW_HW_ERROR(NotSupported) << DEB_VAR1(mode);
			// break;
	}

	std::cout << "init done\n" << std::endl;

	return ;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void uninit()
{
	std::cout << "uninit..." << std::endl;

	MEASURE_TIME(TUCAM_Dev_Close(m_opCam.hIdxTUCam), "TUCAM_Dev_Close");
	MEASURE_TIME(TUCAM_Api_Uninit(), "TUCAM_Api_Uninit");	

	m_opCam.hIdxTUCam = NULL;	
	std::cout << "uninit done\n" << std::endl;
	return ;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_acq(unsigned loop_num)
{
	std::cout << "start_acq ..." << std::endl;

	for(int i = 0; i < m_nb_frames; i++)
	{
		MEASURE_TIME(TUCAM_Cap_DoSoftwareTrigger(m_opCam.hIdxTUCam),"TUCAM_Cap_DoSoftwareTrigger");
	
		std::cout << "TUCAM_Buf_WaitForFrame " << i <<std::endl;		

		auto start = std::chrono::high_resolution_clock::now();
		TUCAMRET ret = TUCAM_Buf_WaitForFrame(m_opCam.hIdxTUCam, &m_frame,5000);
		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duree = end - start;
		std::cout << "Temps d'execution de TUCAM_Buf_WaitForFrame : " << duree.count() << " ms" << std::endl;

		if(ret == TUCAMRET_SUCCESS)
		{			
			if(m_file_target!="DO_NOT_SAVE_FILE")
			{
				//save image to file png if requested by user
				char file_name[256] = ""; 
				sprintf(file_name, "%s_%03d_%05d", m_file_target.c_str(), loop_num, i);
				TUCAM_FILE_SAVE fileSave;
				fileSave.nSaveFmt = TUFMT_PNG; // Save Tiff format
				fileSave.pFrame = &m_frame; // Pointer to the frame to be saved
				fileSave.pstrSavePath = const_cast<char*>(file_name); // path contains filename (without extension)
				MEASURE_TIME(TUCAM_File_SaveImage(m_opCam.hIdxTUCam, fileSave), "TUCAM_File_SaveImage");
			}
		}
		else
		{
			std::cout << "TUCAM_Buf_WaitForFrame ERROR ! " << std::hex << ret << std::endl;
		}
	}

	std::cout << "start_acq done\n" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void stop_acq()
{
	std::cout << "stop_acq ..." << std::endl;
    MEASURE_TIME(TUCAM_Buf_AbortWait(m_opCam.hIdxTUCam),"TUCAM_Buf_AbortWait");//abort 
    MEASURE_TIME(TUCAM_Cap_Stop(m_opCam.hIdxTUCam),"TUCAM_Cap_Stop"); // stop data capture
    MEASURE_TIME(TUCAM_Buf_Release(m_opCam.hIdxTUCam),"TUCAM_Buf_Release"); // Free the allocated memory
	std::cout << "stop_acq done\n" << std::endl;
}

void vanilla_snap()
{
	//Api Init & Dev Open
	init();    

	// m_nb_loops acquisition of m_nb_frames images
	for(int i=0; i<m_nb_loops;i++)
	{
		std::cout<<"\n============== loop["<<i<< "]=========="<<std::endl;
		
		///////////
		std::cout << "Running prepare_acq in main thread..." << std::endl;
		prepare_acq();
		start_acq(i);

		//////////
		//std::cout << "Running prepare_acq in a separate thread..." << std::endl;
		//std::thread prepare_thread(prepare_acq);
		//prepare_thread.join();
		//std::thread start_thread(start_acq, i);
		//start_thread.join();

		//////////
		// try
		// {
			// std::cout << "Running prepare in a yat task" << std::endl;
			// YatTask* task = new YatTask();
			// yat::Message* initMsg = yat::Message::allocate(yat::TASK_INIT, INIT_MSG_PRIORITY, true);
			// task->go(initMsg);
			// std::cout << "Running prepare in a yat task 2" << std::endl;
			// yat::Message* msg = yat::Message::allocate(TASK_PREPARE_ACQ, DEFAULT_MSG_PRIORITY, true);
			// std::cout << "Running prepare in a yat task 3" << std::endl;
			// task->wait_msg_handled(msg, 10000);
			// std::cout << "Running prepare in a yat task 4" << std::endl;
			// start_acq(i);
			// stop_acq();
		// }
		// catch (const std::exception& ex)
		// {
			// std::cout << ex.what() << std::endl;
		// }
	}

	uninit();   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	std::cout<<"usage : MainDhyana.exe exptime_ms nbframes nbloops [path+filename to save image, if this arg is empty, then saving is disabled]\n"<<std::endl;
    try
	{
		//decode program user inputs 
		if(argc > 1)
			m_exp_time_ms 		= yat::StringUtil::to_num<unsigned>(std::string(argv[1]));				
		if(argc > 2)
			m_nb_frames 		= yat::StringUtil::to_num<unsigned>(std::string(argv[2]));		
		if(argc > 3)
			m_nb_loops 			= yat::StringUtil::to_num<unsigned>(std::string(argv[3]));
		if(argc > 4)
			m_file_target 		= std::string(argv[4]);

		m_file_target = ((m_file_target=="DO_NOT_SAVE_FILE")?"DO_NOT_SAVE_FILE":(m_file_target.substr(0, m_file_target.find_last_of("."))));

		std::cout<<"m_exp_time_ms\t: "	<<	m_exp_time_ms		<<std::endl;
		std::cout<<"m_nb_frames\t: "	<<	m_nb_frames			<<std::endl;
		std::cout<<"m_nb_loops\t: " 	<<	m_nb_loops			<<std::endl;
		std::cout<<"m_file_target\t: "	<<  m_file_target		<<std::endl;
		std::cout<<""<<std::endl;

        init_lima_device();
		lima_snap();
	
		//vanilla_snap();
	}
	catch(std::exception& ex)	
	{
		TUCAM_Api_Uninit();
        TUCAM_Dev_Close(m_opCam.hIdxTUCam);     // close camera
        m_opCam.hIdxTUCam = NULL;		
		std::cerr<<"An exception is occured : "<<ex.what()<<std::endl;
	}
}
