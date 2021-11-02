/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  ecat_lifecycle.hpp
 * \brief Ethercat lifecycle node implementation and real-time communication
 *        loop implemented in this file.
 *******************************************************************************/
#include "ecat_node.hpp"
#include "timing.hpp"
/******************************************************************************/

using namespace EthercatCommunication ; 
namespace EthercatLifeCycleNode
{
class EthercatLifeCycle
{
    public:
        EthercatLifeCycle();
        ~EthercatLifeCycle();
    /**
     * @brief Ethercat lifecycle node configuration function, node will start with this function
     *        For more information about Lifecyclenode and it's interfaces check below link :
     *         https://design.ros2.org/articles/node_lifecycle.html
     *         \seeuint8_t
     * @return Success if configuration succesfull,otherwise FAILURE 
     */
        uint8_t on_configure();
    
    /**
     * @brief Activates Ethercat lifecycle node and starts real-time Ethercat communication.
     *        All publishing is done in real-time loop in this active state.
     * 
     * @return Success if activation succesfull,otherwise FAILURE
     */
       uint8_t on_activate();
   
    /**
     * @brief Deactivates Ethercat lifecycle node, turns of real-time communication.
     * 
     * @return Success if deactivation succesfull,otherwise FAILURE
     */
       uint8_t on_deactivate();
    
    /**
     * @brief Cleans up all variables and datas assigned by Ethercat lifecycle node. 
     * 
     * @return Success if cleanup succesfull,otherwise FAILURE 
     */
       uint8_t on_cleanup();
    
    /**
     * @brief Shuts down EtherCAT lifecycle node, releases Ethercat master.
     * 
     * @return Success if shut down succesfull,otherwise FAILURE 
     */
       uint8_t on_shutdown();
    
    /**
     * @brief There isn't any error recovery functionality for this node, just resets nodes.
     *         Reconfiguration is needed for restarting communication.
     * 
     * @return Success 
     */
       uint8_t on_error();

             
        DataReceived     received_data_;
        DataSent         sent_data_;
        std::unique_ptr<EthercatNode>    ecat_node_;
        
        /**
         * @brief Sets Ethercat communication thread's properties 
         *        After this function called user must call StartEthercatCommunication() function]
         * @return 0 if succesfull, otherwise -1.
         */
        int SetComThreadPriorities();
        
        /**
         * @brief Encapsulates all configuration steps for the EtherCAT communication with default slaves.
         *        And waits for connected slaves to become operational.
         * @return 0 if succesful otherwise -1. 
         */
        int InitEthercatCommunication() ;

        /**
         * @brief Helper function to enter pthread_create, since pthread's are C function it doesn't
         *        accept class member function, to pass class member function this helper function is 
         *        created.
         * 
         * @param arg Pointer to current class instance.
         * @return void* 
         */
        static void *PassCycylicExchange(void *arg);

        /**
         * @brief Starts EtherCAT communcation
         * 
         * @return 0 if succesfull, otherwise -1.
         */
        int  StartEthercatCommunication(); 

        /**
         * @brief Realtime cyclic Pdo exchange function which will constantly read/write values from/to slaves
         * 
         * @param arg Used during pthread_create function to pass variables to realtime task. 
         * @return NULL
         */
        void StartPdoExchange(void *instance); 
        
        /**
         * @brief Gets  master's communication state.
         *  \see ec_al_state_t
         * 
         * @return Application layer state for master.
         */
        int GetComState();
        
        /**
         * @brief Reads data from slaves and updates received data structure to be published
         */
        void ReadFromSlaves();
        
        /**
         * @brief Publishes all data that master received and will be sent
         * 
         * @return 0 if succesfull otherwise -1. 
         */
        int PublishAllData();
        
        /**
         * @brief Enables connected motor drives based on CIA402
         * 
         */
        void EnableMotors();
        
        /**
         * @brief Updates data that will be sent to slaves.
         *        This updated data will be published as well.
         */
        void WriteToSlavesVelocityMode();
        
        /**
         * @brief Writes target position and control word to motor in profile 
         *        position mode.
         */
        void WriteToSlavesInPositionMode();
        
        /**
         * @brief Acquired data from subscribed controller topic will be assigned as 
         *        motor speed parameter.
         */
        void UpdateVelocityModeParameters();

        /**
         * @brief Acquired data from subscribed controller topic will be assigned as 
         *        motor speed parameter.
         */
        void UpdateCyclicVelocityModeParameters(); 

        /**
         * @brief Acquired data from subscribed controller topic will be assigned as 
         *        motor target position parameter.
         */
        void UpdatePositionModeParameters();
        
        /**
         * @brief Acquired data from subscribed controller topic will be assigned as motor
         *        cyclic target position parameter in configured interpolation time.
         */
        void UpdateCyclicPositionModeParameters();
        
        /**
         * @brief Updates motor control world and motor state in velocity mode based on CIA402.
         * 
         */
        void UpdateMotorStateVelocityMode();
        /**
         * @brief Updates motor control word and motor state in position mode based on CIA402 state machine,
         */
        void UpdateMotorStatePositionMode();
        
        /**
         * @brief CKim - This function checks status word and returns
         *        state of the motor driver
         * 
         */
        int GetDriveState(const int& statusWord);

        /**
         * @brief CKim - This function checks status word, clears
         *        any faults and enables torque of the motor driver
         * 
         */
        int EnableDrivers();

    public : 
        /// pthread create required parameters.
        pthread_t ethercat_thread_;
        struct sched_param ethercat_sched_param_ = {};
        pthread_attr_t ethercat_thread_attr_;
        int32_t err_;
        /// Application layer of slaves seen by master.(INIT/PREOP/SAFEOP/OP)
        uint8_t al_state_ = 0; 
        uint32_t motor_state_[g_kNumberOfServoDrivers];
        uint32_t command_ = 0x004F;
        Controller controller_;
        /// Values will be sent by controller node and will be assigned to variables below.
        uint8_t gui_node_data_ = 1;
        uint8_t emergency_status_ = 1 ;
        // Will be used as a parameter for taking timing measurements.
        std::int32_t measurement_time = 0 ; 
        Timing timer_info_ ; 
};
}