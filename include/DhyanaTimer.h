//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
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
//
// DhyanaCamera.h
// Created on: October 24, 2018
// Author: Arafat NOUREDDINE

#ifndef DHYANATIMER_H_
#define DHYANATIMER_H_


#include <ostream>
#include <map>
#include <process.h>
#include <windows.h>
#include <stdio.h>
#include <Mmsystem.h>
#pragma comment(lib, "Winmm.lib" )

#include "DhyanaCompatibility.h"
#include "lima/Debug.h"
#include "DhyanaCamera.h"

using namespace std;

namespace lima
{

	namespace Dhyana
	{

		class Camera;

		////////////////////////////////////////////////////////////////////
		//CBaseTimer class
		////////////////////////////////////////////////////////////////////
		
		class CBaseTimer
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Camera", "CBaseTimer");
		public:
			// ctor
			//------------------------------------------------------------
			CBaseTimer(int period = 1000);

			// dtor
			//------------------------------------------------------------
			~CBaseTimer();

			//------------------------------------------------------------
			static void CALLBACK base_timer_proc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
			{
				//std::cout<<" ----> base_timer_proc <------- [BEGIN]"<<std::endl;
				CBaseTimer* owner = (CBaseTimer*)dwUser;
				owner->on_timer();
				//std::cout<<" ----> base_timer_proc <------- [END]"<<std::endl;
			}
			//------------------------------------------------------------
			void enable_oneshot_mode();
			//------------------------------------------------------------
			void disable_oneshot_mode();
			//------------------------------------------------------------
			void start();

			//------------------------------------------------------------
			void stop();

			//------------------------------------------------------------
			virtual void on_timer() = 0;

		protected:
			int  m_timer_id;
			long m_period_ms;
			UINT m_resolution;
			bool m_is_oneshot;
		};

		////////////////////////////////////////////////////////////////////
		//CSoftTriggerTimer class
		////////////////////////////////////////////////////////////////////		
		class CSoftTriggerTimer : public CBaseTimer
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Camera", "CSoftTriggerTimer");
		public:
			//ctor
			//------------------------------------------------------------
			CSoftTriggerTimer(int period, Camera& cam);

			//dtor
			//------------------------------------------------------------
			~CSoftTriggerTimer();

			//------------------------------------------------------------
			void on_timer();

		public:
			Camera& m_cam;
		};

	} // namespace Dhyana
} // namespace lima

#endif /* DHYANATIMER_H_ */
