/****************************************************************************/
#include "ecat_node.hpp"
/****************************************************************************/
#include "ecat_lifecycle.hpp"
std::unique_ptr<EthercatLifeCycleNode::EthercatLifeCycle> ecat_lifecycle_node;

void signalHandler(int /*signum*/)
{
    //sig = 0;
    //usleep(1e3);
    ecat_lifecycle_node->on_shutdown();
}

int main(int argc, char **argv)
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
        return -1;
    }
    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
    // -----------------------------------------------------------------------------

    // CKim - Initialize and launch EthercatLifeCycleNode
    ecat_lifecycle_node = std::make_unique<EthercatLifeCycleNode::EthercatLifeCycle>();

    ecat_lifecycle_node->on_configure();
    ecat_lifecycle_node->on_activate();
    ecat_lifecycle_node->on_cleanup();
    ecat_lifecycle_node->on_shutdown();

    
    return 0;
}

