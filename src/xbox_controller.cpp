/*
 * xboxController.c
 *
 */
#include "xbox_controller.hpp"



void XboxController::DeinitXboxController(XboxCtrl* xbox){
	free(xbox);
	close(fd);
}

int XboxController::InitXboxController(char* dev){
	if(fd == -1){
		if ((fd = open(dev, O_RDONLY)) < 0) {
			printf("device %s could not opened.\n", dev);
			return -1;
		}
		fcntl(fd, F_SETFL, O_NONBLOCK); /* use non-blocking mode */
	}
	return 0;
}

XboxCtrl* XboxController::GetXboxDataStruct(){
	if(xbox == NULL){
		printf("Allocationg memory for Xbox\n");
		xbox = (XboxCtrl*) malloc(sizeof(XboxCtrl));
	}
	return xbox;
}

void XboxController::ReadXboxControllerInformation(XboxCtrl* xbox){
	ioctl(fd, JSIOCGAXES, xbox->numOfAxis);
	ioctl(fd, JSIOCGBUTTONS, xbox->numOfButtons);
	ioctl(fd, JSIOCGNAME(IDENTIFIER_SIZE), xbox->identifier);

	if (xbox->numOfButtons > 1000 || xbox->numOfButtons < -1000 ) {
		/*
		 * this happens, if the process is started more the one time.
		 * then the number of buttons can not be read.
		 */
		printf("problem while reading the button count. exiting.");
	}
}

void XboxController::ReadXboxData(XboxCtrl* xbox){
	read(fd, &js, sizeof(struct js_event));
	SetXboxCtrlValue(xbox, &js);
}
void XboxController::SetXboxCtrlValue(XboxCtrl* xbox, struct js_event* js) {
	int event = js->type & ~JS_EVENT_INIT;

	if (event == JS_EVENT_AXIS) {
		switch (js->number) {
		case STICK_LEFT_X_ID:
			xbox->stk_LeftX = js->value;
			break;
		case STICK_LEFT_Y_ID:
			xbox->stk_LeftY = js->value;
			break;
		case STICK_RIGHT_X_ID:
			xbox->stk_RightX = js->value;
			break;
		case STICK_RIGHT_Y_ID:
			xbox->stk_RightY = js->value;
			break;
		case STICK_LEFT_TOP_ID:
			xbox->stk_LeftTop = js->value;
			break;
		case STICK_RIGHT_TOP_ID:
			xbox->stk_RightTop = js->value;
			break;
		case CROSS_LR_ID:
			xbox->stk_crossLR = js->value;
			break;
		case CROSS_FB_ID:
			xbox->stk_crossFB = js->value;
			break;
		}
	} else if (event == JS_EVENT_BUTTON)
		switch (js->number) {
		case BTN_BACK_ID:
			xbox->btn_back = js->value;
			break;
		case BTN_START_ID:
			xbox->btn_start = js->value;
			break;
		case BTN_XBOX_ID:
			xbox->btn_xbox = js->value;
			break;
		case BTN_LEFT_STICK_ID:
			xbox->btn_leftStk = js->value;
			break;
		case BTN_RIGHT_STICK_ID:
			xbox->btn_rightStk = js->value;
			break;
		case BTN_A_ID:
			xbox->btn_A = js->value;
			break;
		case BTN_B_ID:
			xbox->btn_B = js->value;
			break;
		case BTN_X_ID:
			xbox->btn_X = js->value;
			break;
		case BTN_Y_ID:
			xbox->btn_Y = js->value;
			break;
		case BTN_TOP_LEFT_ID:
			xbox->btn_leftTop = js->value;
			break;
		case BTN_TOP_RIGHT_ID:
			xbox->btn_rightTop = js->value;
			break;
		}
}

void XboxController::PrintXboxCtrlValues(XboxCtrl* xbox) {
	/* print sticks values */
	printf("X0: %6d Y0: %6d Z0: %6d ", xbox->stk_LeftX, xbox->stk_LeftY, xbox->stk_LeftTop);
	printf("X1: %6d Y1: %6d Z1: %6d ", xbox->stk_RightX, xbox->stk_RightY, xbox->stk_RightTop); // right stick
	printf("C_LR: %6d  C_FB: %6d ", xbox->stk_crossLR, xbox->stk_crossFB); // cross left/right and front/back


	/* print button values */
	printf("  back:%d  xbox:%d  start:%d  A:%d  B:%d  X:%d  Y:%d  TL:%d  TR:%d  STK_L:%d  STK_R:%d",
				xbox->btn_back,
				xbox->btn_xbox,
				xbox->btn_start,
				xbox->btn_A,
				xbox->btn_B,
				xbox->btn_X,
				xbox->btn_Y,
				xbox->btn_leftTop,
				xbox->btn_rightTop,
				xbox->btn_leftStk,
				xbox->btn_rightStk
				);
	printf("\n");
	fflush(stdout);
}
