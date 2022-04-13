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
 * \file  object_dictionary.hpp
 * \brief Contains object dictionary and state machine definitions based on CIA402
 * 		  Some index and subindexes might be specific to Maxon EPOS4.
 * 		  Check indexes if you're using different slave.
 *******************************************************************************/

#pragma once

#define OD_DEVICE_TYPE					0x1000,0X00  // RO: uint32_t
#define OD_ERROR_REGISTER				0X1001,0X00  // RO: uint8_t Maxon EPOS4 Compact 50/5 \see ErrorRegisterBits struct 
#define OD_MANUFACTURER_DEVICE_NAME 	0X1008,0X00  // RO: VISIBLE_STRING
#define OD_STORE_PARAMETERS				0X1010,0X00  // Check subindex
#define OD_SAVE_ALL_PARAMETERS			0X1010,0X01  // RW: uint32_t 
#define OD_RESTORE_PARAMETERS			0X1011,0X00  // Check subindex 
#define OD_RESTORE_ALL_PARAMETERS		0X1011,0X01	 // RW: uint32_t 
#define OD_IDENTITY_OBJECT				0X1018,0X00  // Check subindexes
#define OD_VENDOR_ID					0X1018,0X01  // RO : uint32_t
#define OD_PRODUCT_CODE					0X1018,0X02  // RO : uint32_t
#define OD_REVISION_NUMBER				0X1018,0X03  // RO : uint32_t
#define OD_SERIAL_NUMBER				0X1018,0X04  // RO : uint32_t
#define OD_DIAGNOSIS_HISTORY            0X10F3,0X00  // Check subindexes.
#define OD_MAXIMUM_MESSAGES				0X10F3,0X01  // RO : uint8_t
#define OD_NEWEST_MESSAGE				0X10F3,0X02  // RO : uint8_t
#define OD_NEWEST_ACK_MESSAGE		    0X10F3,0X03  // RW : uint8_t
#define OD_NEW_MESSAGE_AVAILABLE		0X10F3,0X04  // RO : bool
#define OD_FLAGS						0X10F3,0X05  // RW : uint16_t
#define OD_DIAGNOSIS_MESSAGE_1			0X10F3,0X06  // RW : Octet_string
#define OD_DIAGNOSIS_MESSAGE_2			0X10F3,0X07  // RW : Octet_string
#define OD_DIAGNOSIS_MESSAGE_3			0X10F3,0X08  // RW : Octet_string
#define OD_DIAGNOSIS_MESSAGE_4			0X10F3,0X09  // RW : Octet_string
#define OD_DIAGNOSIS_MESSAGE_5			0X10F3,0X0A  // RW : Octet_string

#define OD_NUM_OF_RPDO_1_MAPPING		0X1600,0x00  // RW : uint8_t
#define OD_NUM_OF_RPDO_2_MAPPING		0X1601,0x00  // RW : uint8_t
#define OD_NUM_OF_RPDO_3_MAPPING		0X1602,0x00  // RW : uint8_t
#define OD_NUM_OF_RPDO_4_MAPPING		0X1603,0x00  // RW : uint8_t

#define OD_NUM_OF_TPDO_1_MAPPING		0X1A00,0x00  // RW : uint8_t
#define OD_NUM_OF_TPDO_2_MAPPING		0X1A01,0x00  // RW : uint8_t
#define OD_NUM_OF_TPDO_3_MAPPING		0X1A02,0x00  // RW : uint8_t
#define OD_NUM_OF_TPDO_4_MAPPING		0X1A03,0x00  // RW : uint8_t

#define OD_NUM_OF_SYNC_MANAGER_CHANNELS		0X1C00,0x00  // RO : uint8_t
#define OD_COM_TYPE_SYNC_CHANNEL_0			0X1C00,0x01  // RO : uint8_t Master > slave SDO
#define OD_COM_TYPE_SYNC_CHANNEL_1 			0X1C00,0X02  // RO : uint8_t Master < slave SDO
#define OD_COM_TYPE_SYNC_CHANNEL_2		    OX1C00,0X03  // RO : uint8_t Master > slave PDO
#define OD_COM_TYPE_SYNC_CHANNEL_3			0X1C00,0X04  // RO : uint8_t Master > slave PDO

#define OD_NUM_OF_ASSIGNED_RxPDOS			0X1C12,0x00  // RW : uint8_t
#define OD_1ST_ASSIGNED_RxPDO				0X1C12,0X00  // RW : uint16_t

#define OD_NUM_OF_ASSIGNED_TxPDOS			0X1C13,0X00  // RW : uint8_t
#define OD_1ST_ASSIGNED_TxPDO				0X1C13,0X01  // RW : uint16_t

#define OD_SYNC_TYPE_OUTPUTS	   			0X1C32,0X01  // RO : uint16_t
#define OD_CYCLE_TIME_OUTPUTS				0X1C32,0X02  // RO : uint32_t
#define OD_SUPPORTED_SYNC_TYPES_OUTPUTS		0X1C32,0X04  // RO : uint16_t
#define	OD_MIN_CYCLE_TIME_OUTPUTS			0X1C32,0X05  // RO : uint32_t
#define	OD_CALC_AND_COPY_TIME_OUTPUTS		0X1C32,0X06  // RO : uint32_t
#define	OD_DELAY_TIME_OUTPUTS				0X1C32,0X09  // RO : uint32_t
#define	OD_SM_EVENT_MISSED_OUTPUTS			0X1C32,0X0B  // RO : uint16_t
#define OD_CYCLE_TIME_TOO_SMALL_OUTPUTS		0X1C32,0X0C  // RO : uint16_t

#define OD_SYNC_TYPE_INPUTS					0X1C33,0X01  // RO : uint16_t
#define OD_CYCLE_TIME_INPUTS				0X1C33,0X02  // RO : uint32_t
#define OD_SUPPORTED_SYNC_TYPES_INPUTS		0X1C33,0X04  // RO : uint16_t
#define	OD_MIN_CYCLE_TIME_INPUTS			0X1C33,0X05  // RO : uint32_t
#define	OD_CALC_AND_COPY_TIME_INPUTS		0X1C33,0X06  // RO : uint32_t
#define	OD_SM_EVENT_MISSED_INPUTS			0X1C33,0X0B  // RO : uint16_t
#define OD_CYCLE_TIME_TOO_SMALL_INPUTS		0X1C33,0X0C  // RO : uint16_t

#define OD_POWER_SUPPLY_VOLTAGE				0X2200,0X01	 // RO : uint16_t
#define OD_ACTIVE_FIELDBUS					0X2010,0X00  // R0: uint8_t - 0: None , 1 : CANOpen -  2:EtherCAT 
#define OD_CUSTOM_PERSISTENT_MEMORY_1		0X210C,0X01	 // RW: int32_t to store configuration values, up to 4 bytes.
#define OD_CUSTOM_PERSISTENT_MEMORY_2		0X210C,0X02	 // RW: int32_t to store configuration values, up to 4 bytes.
#define OD_CUSTOM_PERSISTENT_MEMORY_3		0X210C,0X03	 // RW: int32_t to store configuration values, up to 4 bytes.
#define OD_CUSTOM_PERSISTENT_MEMORY_4		0X210C,0X04	 // RW: int32_t to store configuration values, up to 4 bytes.
#define OD_POWER_SUPPLY_VOLTAGE				0X2200,0X01	 // RO: uint16_t 
/// Object Dictionary for Axis configurations 
#define OD_SENSOR_CONFIGURATION							0X3000,0X01	 // RW: uint32_t \see SensorConfig struct.
#define OD_CONTROL_STRUCTURE 							0X3000,0X02  // RW: uint32_t \see IMPORTANT! EPOS4-Firmware-Specification pg. 139
// EOF OD_AXIS_CONFIURATIONS
#define OD_MOTOR_DATA_NOMINAL_CURRENT					0X3001,0X01	 // RW: uint32_t  unit is mA
#define OD_MOTOR_DATA_OUTPUT_CURRENT_LIMIT				0X3001,0X02	 // RW: uint32_t
#define OD_MOTOR_DATA_NUMBER_OF_POLE_PAIRS				0X3001,0X03	 // RW: uint8_t
#define OD_MOTOR_DATA_THERMAL_TIME_CONSTANT_WINDINGS	0X3001,0X04	 // RW: uint16_t
#define OD_MOTOR_DATA_TORQUE_CONSTANT					0X3001,0X05	 // RW: uint32_t
#define OD_GEAR_REDUCTION_NUMERATOR 					0X3003,0X01  // RW: uint32_t
#define OD_GEAR_REDUCTION_DENOMINATOR					0X3003,0X02	 // RW: uint32_t
#define OD_GEAR_MAX_INPUT_SPEED							0X3003,0X03  // RW: uint32_t
#define OD_GEAR_MISC_CONFIGURATION						0X3003,0X04  // RW: uint32_t // Gear direction 0 normal 1 inverted.
#define OD_DIGITAL_INCREMENTAL_ENCODER_1_TYPE			0X3010,0X02  // RW: uint16_t // pg. 157 EPOS4-Firmware-Specifications

#define OD_CURRENT_CONTROLLER_PGAIN						0X30A0,0X01   // RW: uint32_t
#define OD_CURRENT_CONTROLLER_IGAIN						0X30A0,0X02   // RW: uint32_t

#define OD_POSITION_CONTROLLER_PGAIN					0X30A1,0X01	  // RW: uint32_t
#define OD_POSITION_CONTROLLER_IGAIN					0X30A1,0X02	  // RW: uint32_t
#define OD_POSITION_CONTROLLER_DGAIN					0X30A1,0X03	  // RW: uint32_t
#define OD_POSITION_CONTROLLER_FF_VEL_GAIN				0X30A1,0X04	  // RW: uint32_t
#define OD_POSITION_CONTROLLER_FF_ACC_GAIN				0X30A1,0X05	  // RW: uint32_t

#define OD_VELOCITY_CONTROLLER_PGAIN		      		0x30A2,0x01	  // RW: uint32_t  \see EPOS4-Firmware-Specification pg. 167
#define OD_VELOCITY_CONTROLLER_IGAIN		      		0x30A2,0x02	  // RW: uint32_t  \see EPOS4-Firmware-Specification pg. 167
#define OD_VELOCITY_CONTROLLER_FF_VEL_GAIN				0x30A2,0x03	  // RW: uint32_t  \see EPOS4-Firmware-Specification pg. 167
#define OD_VELOCITY_CONTROLLER_FF_ACC_GAIN				0x30A2,0x04	  // RW: uint32_t  \see EPOS4-Firmware-Specification pg. 167

#define OD_CONTROL_WORD             0x6040,0x00
#define OD_STATUS_WORD              0x6041,0x00

#define OD_OPERATION_MODE           0x6060,0x00
#define OD_OPERATION_MODE_DISPLAY   0x6061,0x00

#define OD_TARGET_POSITION        	0x607A,0x00
#define OD_POSITION_ACTUAL_VAL    	0x6064,0x00
#define OD_POSITION_DEMAND        	0x6062,0x00
#define OD_POSITON_FOLLOWING_ERROR 	0x60F4,0X00
#define OD_POSITONCOUNTS		 	0x6063,0x00
#define OD_MAX_FOLLOWING_ERROR     	0x6065,0x00

#define OD_VELOCITY_ACTUAL_VALUE    0x606C,0x00
#define OD_VELOCITY_ADDRESS         0x6069,0x00
#define OD_TARGET_VELOCITY          0x60FF,0x00
#define	OD_VELOCITY_OFFSET		    0x60B1,0x00
#define OD_PROFILE_VELOCITY                     		0x6081,0x00
#define OD_MAX_MOTOR_SPEED	                			0x6080,0x00
#define OD_MAX_PROFILE_VELOCITY							0X607F,0X00
#define OD_PROFILE_ACCELERATION                			0x6083,0x00
#define OD_PROFILE_DECELERATION               			0x6084,0x00
#define OD_QUICK_STOP_DECELERATION             			0x6085,0x00
#define OD_MOTION_PROFILE_TYPE                  		0x6086,0x00
#define OD_VELOCITY_ENCODER_RESOLUTION_NUM      		0x6094,0x01
#define OD_VELOCITY_ENCODER_RESOLUTION_DEN      		0x6094,0x02

#define OD_INTERPOLATION_TIME_PERIOD 					0X60C2,0X01	 // RW: uint8_t   check EPOS4 Firmware Specification pg. 247
#define OD_INTERPOLATION_TIME_UNIT						0X60C2,0X02  // RW: uint8_t   set this to -3(default) for ms unit.

#define OD_DIGITAL_INPUTS			   0x60FD,0x00
#define OD_DIGITAL_OUTPUTS 			   0x60FE,0x01

#define OD_DC_CIRCUIT_LINK_VOLTAGE	   0x6079,0x00
#define OD_TARGET_TORQUE               0x6071,0x00 
#define OD_TORQUE_MAX                  0x6072,0x00
#define OD_TORQUE_ACTUAL_VALUE		   0x6077,0x00
#define OD_TORQUE_OFFSET               0x60b2,0x00

#define OD_MAX_CURRENT				   0x6073, 0x00
#define OD_CURRENT_ACTUAL_VALUE		   0x6078, 0x00
#define OD_ERROR_CODE				   0x603F, 0x00   // 2 bit

#define OD_QUICK_STOP_MODE             0x605A,0x00
#define OD_STOP_OPTION_CODE            0x605D,0x00

/*****************************************************************/
// CIA 402 State machine definitions start. 

#define SM_COMM_RESET              	0x81
#define SM_FULL_RESET              	0x82
#define SM_START                  	0x01
#define SM_GO_READY_TO_SWITCH_ON    0x06
#define SM_GO_SWITCH_ON            	0x07
#define SM_GO_ENABLE               	0X0F
#define SM_GO_SWITCH_ON_DISABLE    	0x00
#define SM_RUN                    	0x1F
#define SM_EXPEDITE               	0x3F       //like run, but dont finish actual position profile
#define SM_QUICKSTOP              	0x02 
#define SM_RELATIVE_POS				0X7F

/* From CiA402,  - State coding
	Statusword      |      PDS FSA state
xxxx xxxx x0xx 0000 | Not ready to switch on
xxxx xxxx x1xx 0000 | Switch on disabled
xxxx xxxx x01x 0001 | Ready to switch on
xxxx xxxx x01x 0011 | Switched on
xxxx xxxx x01x 0111 | Operation enabled
xxxx xxxx x00x 0111 | Quick stop active
xxxx xxxx x0xx 1111 | Fault reaction active
xxxx xxxx x0xx 1000 | Fault
*/
#define SM_FSAFROMSTATUSWORD(SW) 		(SW & 0x006f)
#define SM_NOT_READY_TO_SWITCH_ON   	0b00000000
#define SM_NOT_READY_TO_SWITCH_ON_2  	0b00100000
#define SM_SWITCH_ON_DISABLED     		0b01000000
#define SM_SWITCH_ON_DISABLED_2    		0b01100000
#define SM_READY_TO_SWITCH_ON      		0b00100001
#define SM_SWITCHED_ON           		0b00100011
#define SM_OPERATION_ENABLED     		0b00100111
#define SM_QUICK_STOP_ACTIVE      		0b00000111
#define SM_FAULT_REACTION_ACTIVE  		0b00001111
#define SM_FAULTREACTIONACTIVE2 		0b00101111
#define SM_FAULT                		0b00001000
#define SM_FAULT2               		0b00101000

// SatusWord bits :
#define SM_SW_READY_TO_SWITCH_ON    	0x0001
#define SM_SW_SWITCHED_ON          		0x0002
#define SM_SW_OPERATION_ENABLED    		0x0004
#define SM_SW_FAULT               		0x0008
#define SM_SW_VOLTAGE_ENABLED     		0x0010
#define SM_SW_QUICK_STOP           		0x0020
#define SM_SW_SWITCH_ON_DISABLED    	0x0040
#define SM_SW_WARNING             		0x0080
#define SM_SW_REMOTE              		0x0200
#define SM_SW_TARGET_REACHED       		0x0400
#define SM_SW_INTERNAL_LIMIT_ACTIVE 	0x0800

// ControlWord bits :
#define SM_CW_SWITCH_ON        	0x0001
#define SM_CW_ENABLE_VOLTAGE   	0x0002
#define SM_CW_QUICK_STOP       	0x0004
#define SM_CW_ENABLE_OPERATION 	0x0008
#define SM_CW_FAULT_RESET      	0x0080
#define SM_CW_OD_HALT         	0x0100

/* CiA402 statemachine definition end */
/*****************************************************************/
