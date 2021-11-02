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

class XboxController{
public: 
	struct js_event js;
	char joysickIdentifier[IDENTIFIER_SIZE];
	int fd = -1;
	xboxCtrl* xbox = NULL;
	int initXboxController(char* dev);
	void deinitXboxController(xboxCtrl* xbox);
	xboxCtrl* getXboxDataStruct(void);
	void readXboxControllerInformation(xboxCtrl* xbox);
	void readXboxData(xboxCtrl* xbox);
	void setXboxCtrlValue(xboxCtrl* xbox, struct js_event* js);
	void printXboxCtrlValues(xboxCtrl* xbox);
};
#endif /* XBOXCONTROLLER_H_ */
