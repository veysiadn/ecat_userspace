/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the Wrapped IgH EtherCAT master userspace program 
 * for control applications.
 *
 *  The Wrapped IgH EtherCAT master userspace program for control application
 *  in userspace is free software; you canredistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation; version 2 of the License.
 *
 *  The Wrapped IgH EtherCAT master userspace program for control application
 *  is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 *  PURPOSE.  
 *  See the  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the Wrapped IgH EtherCAT master userspace program for control application. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*******************************************************************************
 * \file  xboxController.hpp
 * \brief Contains blueprint for reading values from XboxController
 *******************************************************************************/

/*
 *	Xbox Controller code has been downloaded from  ; 
 *	https://github.com/msch26/scholtyssek-blogspot/tree/master/xboxControllerClient/src
 *  as a C code, this code has been modified to C++ code, and class implementation
 *  added by 
 *  VeysiADN
 *  01/18/2020 
 *  veysi.adin@kist.re.kr
 */

#ifndef XBOXCONTROLLER_H_
#define XBOXCONTROLLER_H_

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <iostream>

#define IDENTIFIER_SIZE			64

#define STICK_LEFT_X_ID			0x00
#define STICK_LEFT_Y_ID			0x01
#define STICK_RIGHT_X_ID		0x03
#define STICK_RIGHT_Y_ID		0x04
#define STICK_LEFT_TOP_ID		0x02
#define STICK_RIGHT_TOP_ID		0x05
#define CROSS_LR_ID				0x06
#define CROSS_FB_ID				0x07

#define BTN_BACK_ID				0x06
#define BTN_START_ID			0x07
#define BTN_XBOX_ID				0x08
#define BTN_LEFT_STICK_ID		0x09
#define BTN_RIGHT_STICK_ID		0x0A

#define BTN_A_ID				0x00
#define BTN_B_ID				0x01
#define BTN_X_ID				0x02
#define BTN_Y_ID				0x03
#define BTN_TOP_LEFT_ID			0x04
#define BTN_TOP_RIGHT_ID		0x05

static std::string s = "/dev/input/js0";
static char* XBOX_DEVICE = const_cast<char*>(s.c_str());
/// Structure for reading values from XboxController
typedef struct {
	int numOfAxis;
	int numOfButtons;
	char identifier[IDENTIFIER_SIZE];
	int stk_LeftX; 		/* left stick */
	int stk_LeftY; 		/* left stick */
	int stk_RightX; 	/* right stick */
	int stk_RightY;		
	int stk_crossFB; 	/* cross front/back */
	int stk_crossLR; 	/* cross left/right */
	int stk_LeftTop; 	/* left top stick */
	int stk_RightTop; 	/* right top stick */
	int btn_back;
	int btn_start;
	int btn_leftStk;
	int btn_rightStk;
	int btn_A;
	int btn_B;
	int btn_X;
	int btn_Y;
	int btn_xbox;
	int btn_leftTop;
	int btn_rightTop;
} xboxCtrl;
/**
 *  \class   XboxController
 *  \brief   Class for opening joystick and reading values from XboxController
 */
class XboxController{
public: 
	struct js_event js;
	char joysickIdentifier[IDENTIFIER_SIZE];
	int fd = -1;
	xboxCtrl* xbox = NULL;
	/**
	 * @brief Opens xbox controller via file descriptor.
	 * 
	 * @param dev name of the device to be open.
	 * @return 0 if successfull, otherwise -1. 
	 */
	int initXboxController(char* dev);
	/**
	 * @brief Closes opened controller.
	 * 
	 * @param xbox controller to be closed.
	 */
	void deinitXboxController(xboxCtrl* xbox);
	/**
	 * @brief Allocate mememory for XboxDataStruct
	 * 
	 * @return xboxCtrl* 
	 */
	xboxCtrl* getXboxDataStruct(void);
	/**
	 * @brief Reads xbox controller information, e.g number of axis and buttons.
	 * 
	 * @param xbox xbox instance to read information from.
	 */
	void readXboxControllerInformation(xboxCtrl* xbox);
	/**
	 * @brief Reads actual values from xbox controller such as axis data and button data.
	 * 
	 * @param xbox xbox instance to read values from.
	 */
	void readXboxData(xboxCtrl* xbox);
	/**
	 * @brief Assing acquired values to controller struct.
	 * 
	 * @param xbox instance to write values acquired from xbox controller.
	 * @param js button or axis event
	 */
	void setXboxCtrlValue(xboxCtrl* xbox, struct js_event* js);
	/**
	 * @brief Prints acquired values from xbox controller.
	 * 
	 * @param xbox instance.
	 */
	void printXboxCtrlValues(xboxCtrl* xbox);
};
#endif /* XBOXCONTROLLER_H_ */
