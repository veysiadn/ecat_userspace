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
/******************************************************************************
 *  \file   ecat_slave.hpp
 *  \brief  Ethercat slave class implementation header file.
 * 
 *   This header file contains blueprint of EtherCAT slave's which will be 
 *   used to  communicate with EtherCAT master.
 *******************************************************************************/
#pragma once

#include "ecat_globals.hpp"


/**
 *  \class   EthercatSlave
 *  \brief   Contains EtherCAT slave parameters for configuration.
 */
class EthercatSlave
{
    public:
        EthercatSlave();
        ~EthercatSlave();
    /**
     * @brief This function will check slave's application layer states.
     *        (INIT/PREOP/SAFEOP/OP) 
     * @note This function shouldn't be called in real time context.For diagnosis 
     *       you can use CheckDomainState() encapsulation in ecat_node.
     * @return 0 if succesful. 
     */
    int CheckSlaveConfigState();

 
    /// DC sync shift setting, zero will give best synchronization.
    const static uint32_t   kSync0_shift_ = 0;

    /// Slave configuration parameters, assinged to each slave.
    ec_slave_config_t       * slave_config_ ;

    /// Slave state handle to check if slave is online and slaves state machine status(INIT/PREOP/SAFEOP/0P)
    ec_slave_config_state_t  slave_config_state_ ;
    
    /// For custom PDO configuration, check \see MapDefaultPdos() function @file ethercat_node.cpp as an example 
    ec_pdo_info_t          * slave_pdo_info_ ;
    /// For custom PDO configuration, check \see MapDefaultPdos() function @file ethercat_node.cpp as an example 
    ec_pdo_entry_info_t    * slave_pdo_entry_info_;
    /// For custom PDO configuration, check \see MapDefaultPdos() function @file ethercat_node.cpp as an example 
    ec_sync_info_t         * slave_sync_info_;
    /// For custom PDO configuration, check \see MapDefaultPdos() function @file ethercat_node.cpp as an example 
    ec_pdo_entry_reg_t     * slave_pdo_entry_reg_ ;
    /// PDO domain for data exchange
    uint8_t                * slave_pdo_domain_ ;

    /// Variable for checking motor state 
    int32_t                  motor_state_ ;
    /**
     * @brief Slave information data structure.
     *      This structure contains all information related to slave.
     *      It will be used to get slave's information from master.
     */
    ec_slave_info_t         slave_info_;
    /// Offset for PDO entries to assign pdo registers.
    OffsetPDO                 offset_ ;
    /// Received data from servo drivers.
    DataReceived              data_ ;
    /// Slave velocity parameters.
    ProfileVelocityParam    velocity_param_ ;
    /// Slave position parameters.
    ProfilePosParam         position_param_ ;
    // Slave homing parameters. 
    HomingParam             homing_param_ ;

};// EthercatSlave class