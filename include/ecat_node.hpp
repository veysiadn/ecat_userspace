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
/******************************************************************************
 *  \file   ecat_node.hpp
 *  \brief  IgH EtherCAT library functionality wrapper class header file.
 * 
 *   This header file contains blueprint of EthercatNode class which will be 
 *   responsible for encapsulating IgH EtherCAT library's functionality.
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include "ecat_globals.hpp"
/******************************************************************************/
/// Forward declaration of EthercatSlave class.
class EthercatSlave ;
#include "ecat_slave.hpp"
/******************************************************************************/

namespace EthercatCommunication
{
class EthercatNode
{
    public:
        EthercatNode();
        ~EthercatNode();
    EthercatSlave slaves_[NUM_OF_SLAVES];
/**
 * @brief Requests master instance and creates a domain for a master.
 * @note  Keep in mind that created master and domain are global variables.
 * @return 0 if succesful otherwise -1.
 */
    int  ConfigureMaster();
/**
 * @brief Defines default connected slaves based on number of slaves.
 *        Specifies its position, vendor id , product code etc.
 *  Default connected slaves considered implementation specific. In our case it will be 3 motors and 
 *  one EasyCAT slave.
 */
    void DefineDefaultSlaves();
/**
 * @brief Passes your defined slave to EthercatNode class.
 * 
 * @param c_slave first create your own EthercatSlave instance and modify it then pass it to configuration.
 * @param position specify the physical connection position for your custom configured slave.
 */
    void SetCustomSlave(EthercatSlave c_slave, int position);
/**
 * @brief Obtains slave configuration for all slaves w.r.t master.
 * @return 0 if succesfull, otherwise -1. 
 */
    int  ConfigureSlaves();
/**
 * @brief Set mode to ProfilePositionMode with specified parameters for servo drive on that position.
 *       
 * @param P Profile position parameter structure specified by user.
 * @param position Slave position
 * @return 0 if succesfull, otherwise -1.
 */
    int  SetProfilePositionParameters(ProfilePosParam& P , int position);
/**
 * @brief Set the mode to ProfilePositionMode with specified Parameters for all servo drives on the bus.
 * 
 * @param P Profile position parameter structure specified by user.
 * @return 0 if succesful, otherwise -1.
 */
    int SetProfilePositionParametersAll(ProfilePosParam& P);
/**
 * @brief Set mode to ProfileVelocityMode with specified parameters for servo drive on that position.
 * 
 * @param P Profile velocity parameter structure specified by user.
 * @param position Slave position
 * @return 0 if succesful, -1 otherwise.
 */
    int SetProfileVelocityParameters(ProfileVelocityParam& P,int position);
/**
 * @brief Set mode to ProfileVelocityMode with specified parameters for all servo drives on the bus
 * 
 * @param P Profile velocity parameter structure specified by user.
 * @return 0 if succesfull, -1 otherwise.
 * @todo Add error code to all functions.Instead of returning -1. 
 */
    int SetProfileVelocityParametersAll(ProfileVelocityParam& P);
/**
 * @brief Set the Cyclic Sync Position Mode Parameters for slave in specified physical position w.r.t. master.
 * 
 * @param P Cyclic Sync. Position Mode Parameters.
 * @param position Physical position of slave to be configured
 * @return 0 if sucessfull, otherwise -1.
 */
    int SetCyclicSyncPositionModeParameters(CSPositionModeParam& P, int position);
/**
 * @brief Sets the Cyclic Synchronous Position Mode Parameters for all connected motor driver slaves
 * 
 * @return 0 if sucessful, otherwise -1.
 */
    int SetCyclicSyncPositionModeParametersAll(CSPositionModeParam &P);

/**
 * @brief Set the Cyclic Sync Velocity Mode Parameters for slave in specified physical position w.r.t. master.
 * 
 * @param P Cyclic Sync. Velocity Mode Parameters.
 * @param position Physical position of slave to be configured
 * @return 0 if sucessfull, otherwise -1.
 */
    int SetCyclicSyncVelocityModeParameters(CSVelocityModeParam& P, int position);
/**
 * @brief Sets the Cyclic Synchronous Velocity Mode Parameters for all connected motor driver slaves
 * 
 * @return 0 if sucessful, otherwise -1.
 */
    int SetCyclicSyncVelocityModeParametersAll(CSVelocityModeParam &P);

/**
 * @brief Maps default PDOs for our spine surgery robot implementation.
 * @note This method is specific for our spinerobot implementation.
 * If you have different topology or different servo drives use 
 * \see MapCustomPdos() function of modify this function based on your needs.
 * @return 0 if succesfull, otherwise -1.
 */
    int MapDefaultPdos();
/**
 * @brief Map Custom PDO based on your PDO mappings
 * @note  You have to specify slave syncs and slave pdo registers before using function
 * @param c_slave EthercatSlave instance
 * @param position Physical position of your slave w.r.t master
 * @return 0 if succesfull, -1 otherwise.
 */
    int MapCustomPdos(EthercatSlave c_slave, int position);
/**
 * @brief Configures DC sync for our default configuration
 * 
 */
    void ConfigDcSyncDefault();
/**
 * @brief Configures DC synchronization for specified slave position
 * 
 * @param assign_activate Activating DC synchronization for slave.
 * 0x300 for Elmo | and same for EasyCAT
 * @note Assign activate parameters specified in slaves ESI file 
 * 
 * @param position
 */
    void ConfigDcSync(uint16_t assign_activate, int position);
/**
 * @brief This function will check slave's application layer states. (INIT/PREOP/SAFEOP/OP)
 */
    void CheckSlaveConfigurationState();
/**
 * @brief This function will check master's state, in terms of number of responding slaves and their application layer states
 * 
 * @return 0 if succesful, otherwise -1 
 * \see ec_master_state_t structure.
 **/
    int  CheckMasterState();
/**
 * @brief  Reads the state of a domain.
 * Stores the domain state in the given state structure.
 * Using this method, the process data exchange can be monitored in realtime.
 * */
    void CheckMasterDomainState();
/**
 * @brief Activates master, after this function call realtime operation can start.
 * \warning Before activating master all configuration should be done
 * \warning After calling this function you have to register domain(s) and start realtime task.
 * @return 0 if succesful, otherwise -1. 
 */
    int  ActivateMaster();
/**
 * @brief Registers domain for each slave.
 *  This method has to be called after ecrt_master_activate() to get the mapped domain process data memory. 
 * @return 0 if succeful , otherwise -1 
 */
    int  RegisterDomain();
/**
 * @brief Puts all slave to operational mode. User must call this before entering real-time operation.
 *        Reason for this function is that, master and slave has to do several exchange before becoming operational.
 *        So this function does exchange between master and slaves for up to 10 sec, could finish earlier.
 *        If timeout occurs it will return -1.
 * @return 0 if succesfull, otherwise -1.
 */
    int  WaitForOperationalMode();

/**
 * @brief Opens EtherCAT master via command line tool if it's not already on.
 * 
 * @return 0 if succesfull, otherwise -1.
 */ 
    int OpenEthercatMaster();
/**
 * @brief Get the Number Of physically Connected Slaves to the bus.And checks if specified NUM_OF_SLAVES
 *        is correct.
 * @return 0 if NUM_OF_SLAVES setting is correct, otherwise -1.
 */
    int GetNumberOfConnectedSlaves();
/**
 * @brief Get the information of physically connected slaves to the master.
 *        This function will return connected slave's vendor id, product code.
 */
    void GetAllSlaveInformation();

/**
 * @brief Deactivates slaves and can be called in real-time.
 */
    void DeactivateCommunication();
/**
 * @brief Deactivates and releases master shouldn't be called in real-time.
 */
    void ReleaseMaster();
/**
 * @brief Shutdowns EtherCAT master via command line tool if it's not already off.
 * 
 * @return 0 if succesfull, otherwise -1.
 */ 
    int ShutDownEthercatMaster();
    private:
    /// File descriptor to open and wake  master from CLI.
    int  fd;
    
};
}