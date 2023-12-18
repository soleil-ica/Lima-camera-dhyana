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
#include <climits>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "lima/MiscUtils.h"
#include "DhyanaTimer.h"

using namespace lima;
using namespace lima::Dhyana;
using namespace std;

//-----------------------------------------------------
// @brief  ctor
//-----------------------------------------------------    
CBaseTimer::CBaseTimer(int period) :
m_period_ms(period),
m_is_oneshot(true)
{
	DEB_CONSTRUCTOR();		
	// Set resolution to the minimum supported by the system
    TIMECAPS tc;
    if(timeGetDevCaps(&tc, sizeof(TIMECAPS))!=TIMERR_NOERROR)
	{
		throw std::exception("Erreur timeGetDevCaps");
	}
    m_resolution = min(max(tc.wPeriodMin, 0), tc.wPeriodMax);
	DEB_TRACE()<<"Timer resolution : "	<< m_resolution	<<" (ms)";
	DEB_TRACE()<<"Timer period : "		<< m_period_ms	<<" (ms)";
    timeBeginPeriod(m_resolution);	
};

//-----------------------------------------------------
// @brief  dtor
//-----------------------------------------------------   
CBaseTimer::~CBaseTimer()
{
	DEB_DESTRUCTOR();		
	stop();
};

//-----------------------------------------------------
// @brief  set_oneshot
//----------------------------------------------------- 
void CBaseTimer::enable_oneshot_mode()
{
	DEB_MEMBER_FUNCT();
	m_is_oneshot = true;
}

//-----------------------------------------------------
// @brief  set_oneshot
//----------------------------------------------------- 
void CBaseTimer::disable_oneshot_mode()
{
	DEB_MEMBER_FUNCT();
	m_is_oneshot = false;
}

//-----------------------------------------------------
// @brief  start
//-----------------------------------------------------   
void CBaseTimer::start()
{
	DEB_MEMBER_FUNCT();
	////Timestamp t0 = Timestamp::now();		
	m_timer_id = timeSetEvent(m_period_ms, m_resolution, base_timer_proc, (DWORD_PTR)this, TIME_PERIODIC);
	if (m_timer_id == NULL)
	{
		throw std::exception("Erreur timeSetEvent");
	}
	////Timestamp t1 = Timestamp::now();
	////double delta_time = t1 - t0;
	////DEB_TRACE() << "timeSetEvent : elapsed time = " << (int) (delta_time * 1000) << " (ms)";		
}

//-----------------------------------------------------
// @brief  stop
//-----------------------------------------------------   
void CBaseTimer::stop()
{
	DEB_MEMBER_FUNCT();
	timeKillEvent(m_timer_id);
	timeEndPeriod(m_resolution);
}

///////////////////////////////////////////////////////
// USER CSoftTriggerTimer
///////////////////////////////////////////////////////

//-----------------------------------------------------
// @brief  ctor
//-----------------------------------------------------   
CSoftTriggerTimer::CSoftTriggerTimer(int period, Camera& cam) :
CBaseTimer(period),
m_cam(cam)
{
	DEB_CONSTRUCTOR();		
}

//-----------------------------------------------------
// @brief  dtor
//-----------------------------------------------------   
CSoftTriggerTimer::~CSoftTriggerTimer()
{
	DEB_DESTRUCTOR();	
};

//-----------------------------------------------------
// @brief  on_timer
//-----------------------------------------------------   
void CSoftTriggerTimer::on_timer()
{
	DEB_MEMBER_FUNCT();

	////Timestamp t0 = Timestamp::now();						
	////DEB_TRACE() << "CSoftTriggerTimer::on_timer : TUCAM_Cap_DoSoftwareTrigger";
	TUCAM_Cap_DoSoftwareTrigger(m_cam.m_opCam.hIdxTUCam);	
	if(m_is_oneshot)//for internal_multi
	{
		stop();
	}
	////Timestamp t1 = Timestamp::now();
	////double delta_time = t1 - t0;
	////DEB_TRACE() << "TUCAM_Cap_DoSoftwareTrigger : elapsed time = " << (int) (delta_time * 1000) << " (ms)";					
}

//-----------------------------------------------------
//
//-----------------------------------------------------  