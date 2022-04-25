/****************************************************************************/
#include "ecat_node.hpp"
/****************************************************************************/
#include "ecat_lifecycle.hpp"
#include "xbox_controller.hpp"

void signalHandler();
void SetRealTimeSettings();

int main(int argc, char **argv)
{
  XboxController Controller;
  SetRealTimeSettings();
  std::unique_ptr<EthercatLifeCycleNode::EthercatLifeCycle> ecat_lifecycle_node;
  ecat_lifecycle_node = std::make_unique<EthercatLifeCycleNode::EthercatLifeCycle>();

  if(ecat_lifecycle_node ->on_configure())
  {
    return -1;
  }
  if(ecat_lifecycle_node->on_activate())
  {
    return -1;
  }
  
  if (Controller.InitXboxController(XBOX_DEVICE) >= 0) {
    Controller.xbox = Controller.GetXboxDataStruct();
    Controller.ReadXboxControllerInformation(Controller.xbox);
    printf("xbox controller detected\n\naxis:\t\t%d\nbuttons:\t%d\nidentifier:\t%s\n",
    Controller.xbox ->numOfAxis, Controller.xbox ->numOfButtons, Controller.xbox->identifier);
    while (sig){
        Controller.ReadXboxData(Controller.xbox );
        ecat_lifecycle_node->controller_.left_x_axis_  = float(Controller.xbox->stk_LeftX / 32767.0);
        ecat_lifecycle_node->controller_.right_x_axis_  = float(Controller.xbox->stk_RightX/32767.0);
        ecat_lifecycle_node->controller_.left_y_axis_ = float(Controller.xbox->stk_LeftY / 32767.0);
        ecat_lifecycle_node->controller_.red_button_ = Controller.xbox->btn_B;
        ecat_lifecycle_node->controller_.blue_button_ = Controller.xbox->btn_X;
        ecat_lifecycle_node->controller_.green_button_ = Controller.xbox->btn_A;
        ecat_lifecycle_node->controller_.yellow_button_ = Controller.xbox->btn_Y;
        ecat_lifecycle_node->controller_.xbox_button_ = Controller.xbox->btn_xbox; 
        ecat_lifecycle_node->controller_.left_rb_button_ = Controller.xbox->btn_leftTop;
        ecat_lifecycle_node->controller_.right_rb_button_ = Controller.xbox->btn_rightTop;
        nanosleep((const struct timespec[]){0,PERIOD_NS},NULL);
      }
    Controller.DeinitXboxController(Controller.xbox);
    }else{
    float target_val =  0.0 ;
      while(sig){
        std::cin >> target_val ;
        ecat_lifecycle_node->controller_.left_x_axis_ = target_val;
        ecat_lifecycle_node->controller_.left_x_axis_  = target_val;
        ecat_lifecycle_node->controller_.right_x_axis_  = target_val;
        ecat_lifecycle_node->controller_.left_y_axis_ = target_val;
        ecat_lifecycle_node->controller_.red_button_ = target_val;
        ecat_lifecycle_node->controller_.blue_button_ = target_val;
        ecat_lifecycle_node->controller_.green_button_ = target_val;
        ecat_lifecycle_node->controller_.yellow_button_ = target_val;
        ecat_lifecycle_node->controller_.left_rb_button_ = target_val;
        ecat_lifecycle_node->controller_.right_rb_button_ = target_val;
        nanosleep((const struct timespec[]){0,PERIOD_NS*10},NULL);
      } 
    }
  
  return 0;
}


void signalHandler(int /*signum*/)
{
    sig = 0;
    nanosleep((const struct timespec[]){0,g_kNsPerSec},NULL);
  //  ecat_lifecycle_node->on_shutdown();
}

void SetRealTimeSettings()
{
      // CKim - Configure stdout sream buffer. _IONBF means no buffering. Each I/O operation is written as soon as possible. 
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // CKim - Associate 'signalHandler' function with interrupt signal (Ctrl+C key)
    signal(SIGINT,signalHandler);

    // CKim - Prepare memory for real time performance 
    // https://design.ros2.org/articles/realtime_background.html
    
    // CKim - Lock this processe's memory. Necessary for real time performance....
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        printf( "Mlockall failed, check if you have sudo authority.");
        return ;
    }
    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
    // -----------------------------------------------------------------------------
}