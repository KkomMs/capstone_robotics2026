/*
 * ©2022 Zebra Technologies LLC. All rights reserved.
 */

#ifndef USERDEFS_H
#define USERDEFS_H

/*! \file CsUserDefs.h
    \brief Definitions related to CoreScanner interface
*/

/*!\enum
 * \brief  CMD OpCode type
 */
#ifndef CMD_OPCODE
#define CMD_OPCODE
enum CmdOpcode
{
    CMD_GET_VERSION						= 0x3E8, /**1000< Gets the version of CoreScanner Driver */
    CMD_REGISTER_FOR_EVENTS					= 0x3E9, /**1001< Register for API events described API Events beginning on */
    CMD_CLAIM_DEVICE						= 0x5DC, /**1500< Claim a specified device */
    CMD_RELEASE_DEVICE						= 0x5DD, /**1501< Release a claimed device */
    CMD_DEVICE_ABORT_UPDATE_FIRMWARE                            = 0x7D1, /**2001< Abort Firmware updates process of a specified scanner while it is progressing */
    CMD_DEVICE_AIM_OFF						= 0x7D2, /**2002< Turn off the aiming of a specified scanner */
    CMD_DEVICE_AIM_ON						= 0x7D3, /**2003< Turn on the aiming of a specified scanner */
    CMD_DEVICE_LED_OFF						= 0x7D9, /**2009< Turn off LED of a specified scanner */
    CMD_DEVICE_LED_ON						= 0x7DA, /**2010< Turn on LED of specified scanner */
    CMD_DEVICE_PULL_TRIGGER					= 0x7DB, /**2011< Pull the trigger of a specified scanner */
    CMD_DEVICE_RELEASE_TRIGGER                                  = 0x7DC, /**2012< Release the pulled trigger of a specified scanner */
    CMD_DEVICE_SCAN_DISABLE					= 0x7DD, /**2013< Disable scanning on a specified scanner */
    CMD_DEVICE_SCAN_ENABLE					= 0x7DE, /**2014< Enable scanning on a specified scanner */
    CMD_DEVICE_BEEP_CONTROL					= 0x7E2, /**2018< Control the beep */
    CMD_REBOOT_SCANNER						= 0x7E3, /**2019< Reboot a specified scanner. */

    CMD_DEVICE_CAPTURE_IMAGE                                    = 0xBB8, /**3000< Change a specified scanner to snapshot mode. */
    CMD_DEVICE_CAPTURE_BARCODE                                  = 0xDAC, /**3500< Change a specified scanner to decode mode */
    CMD_DEVICE_CAPTURE_VIDEO                                    = 0xFA0, /**4000< Change a specified scanner to video mode.  */

    CMD_RSM_ATTR_GETALL						= 0x1388, /**5000< Get all the attributes of a specified scanner. */
    CMD_RSM_ATTR_GET						= 0x1389, /**5001< Query the values of attribute(s) of a specified scanner. */
    CMD_RSM_ATTR_GETNEXT					= 0x138A, /**5002< Query the value of the next attribute to a given attribute of a specified scanner. */
    CMD_RSM_ATTR_SET						= 0x138C, /**5004< Set the values of attribute(s) of a specified scanner.  */
    CMD_RSM_ATTR_STORE						= 0x138D, /**5005< Store the values of attribute(s) of a specified scanner. */
    CMD_GET_DEVICE_TOPOLOGY					= 0x138E, /**5006< Get the topology of devices that are connected to the calling system */
    CMD_REFRESH_TOPOLOGY					= 0x138F, /**5007< Refresh the topology of devices that are connected to the calling system */

    CMD_START_NEW_FIRMWARE					= 0x1396, /**5014< Start the updated firmware. This reboots the scanner */
    CMD_DEVICE_UPDATE_FIRMWARE                                  = 0x1398, /**5016< Update the firmware of the specified scanner. */
    CMD_DEVICE_UPDATE_FIRMWARE_FROM_PLUGIN                      = 0x1399, /**5017< Update the firmware of the specified scanner using a scanner plug-in. */
    CMD_LOAD_CONFIGURATION                                      = 0x139C, /**5020< Load configuration (.SCNCFG) file configurations to scanner device */
    CMD_GET_DEVICE_POWER_CYCLE_STATS                            = 0x144F, /**5199< Get the scanner's power cycle stats according to defined attribute id set */

    CMD_RTA_GET_SUPPORTED_EVENTS                                = 0x157C, /**5500< Get Supported RTA events. */
    CMD_RTA_REGISTRATION                                        = 0x157D, /**5501< Register for RTA events. */
    CMD_RTA_UN_REGISTRATION                                     = 0x157E, /**5502< Unregister RTA events, */
    CMD_RTA_GET_EVENT_STATUS                                    = 0x157F, /**5503< Get RTA event Status. */
    CMD_RTA_SET_EVENT_STATUS                                    = 0x1580, /**5504< Set RTA event Status. */
    CMD_RTA_SUSPEND_EVENTS                                      = 0x1581, /**5505< Sunspend RTA events. */
    CMD_RTA_STATE                                               = 0x1582, /**5506< Get RTA State. */
    CMD_DEVICE_SWITCH_HOST_MODE                                 = 0x1838, /**< 6200 Switch the USB host mode of a specified scanner */
    CMD_SWITCH_CDC_DEVICES                                      = 0x1839, /**< Switch CDC devices */
    CMD_KEYBOARD_EMULATOR_ENABLE                                = 0x189C, /**6300< Enable / Disable Hidkb Simulation */
    CMD_KEYBOARD_EMULATOR_GET_XML                               = 0x189E, /**6302< Get Hidkb Simulation settings*/

    CMD_RETRIEVE_CRADLE_CONTACT_STATE_OF_HEALTH        		= 0x1450  /**  5200< Retrieves the health status of the cradle.*/
    
};
#endif // CMD_OPCODE


/*!\enum
 * \brief  Status Code type
 */
#ifndef STATUS_ID
#define STATUS_ID
enum StatusID
{
    STATUS_OK                                   = 0, /**< Generic success */
    STATUS_ERROR                                = 1, /**< Generic error */
    STATUS_ERROR_PARTIAL_SUCCESS                = 8, /**< Partial success */
    STATUS_ERROR_DEVICE_NOT_AVAILABLE           = 4, /**< Device not available */
    STATUS_ERROR_COMMAND_RESPONSE_TIMED_OUT     = 7, /**< Command respone timeout*/
    STATUS_ERROR_COMMLIB_UNAVAILABLE            = 101, /**< Required Comm Lib is unavailable to support the requested Type. */
    STATUS_ERROR_INCORRECT_BUFFER_SIZE          = 104, /**< When registering for events, inXml event count is not eaqual to actual no. of events*/
    STATUS_ERROR_INVALID_ARG                    = 107, /**< Invalid argument*/
    STATUS_ERROR_INVALID_SCANNERID              = 108, /**< Invalid scanner ID*/
    STATUS_ERROR_INCORRECT_NUMBER_OF_EVENTS     = 109, /**< Incorrect value for number of Event IDs*/
    STATUS_ERROR_DUPLICATE_EVENTID              = 110, /**< Event IDs are duplicated*/
    STATUS_ERROR_INVALID_EVENTID                = 111, /**< Invalid value for Event ID*/
    STATUS_ERROR_DEVICE_UNAVAILABLE             = 112, /**< Required device is unavailable*/
    STATUS_ERROR_INVALID_OPCODE                 = 113, /**< Opcode is invalid*/
    STATUS_ERROR_INVALID_TYPE                   = 114, /**< Invalid value for Type*/
    STATUS_ERROR_ASYNC_NOT_SUPPORTED            = 115, /**< OpCode does not support asynchronous method*/
    STATUS_ERROR_OPCODE_NOT_SUPPORTED           = 116, /**< Device does not support the OpCode*/
    STATUS_ERROR_OPERATION_FAILED               = 117, /**< Operation failed in device*/
    STATUS_ERROR_REQUEST_FAILED                 = 118, /**< Request failed in CoreScanner*/
    STATUS_ERROR_OPERATION_NOT_SUPPORTED_FOR_AUXILIARY_SCANNERS = 119,

    STATUS_ERROR_DEVICE_BUSY                    = 120, /**< Device Busy. Applications should retry command.*/   
    
    STATUS_ERROR_CDC_SCANNERS_NOT_FOUND         = 150, /**< No CDC scanners found in the system.*/
    STATUS_ERROR_UNABLE_TO_OPEN_CDC_COM_PORT    = 151, /**< Unable to open CDC port.*/

    STATUS_ERROR_ALREADY_OPENED                 = 200, /**< CoreScanner is already opened*/
    STATUS_ERROR_ALREADY_CLOSED                 = 201, /**< CoreScanner is already closed*/
    STATUS_ERROR_CLOSED                         = 202, /**< CoreScanner is closed*/
    
    STATUS_ERROR_INVALID_INXML                  = 300, /**< Malformed in-XML*/
    
    ERROR_FW_INVALID_DATFILE                    = 500, /**< Invalid firmware file*/
    ERROR_FW_UPDATE_FAILED_IN_SCN               = 501, /**< FW Update failed in scanner*/
    ERROR_FW_READ_FAILED_DATFILE                = 502, /**< Failed to read DAT file*/
    ERROR_FW_UPDATE_INPROGRESS                  = 503, /**< Firmware Update is in progress (cannot proceed another FW Update or another command)*/
    ERROR_FW_UPDATE_ALREADY_ABORTED             = 504, /**< Firmware update is already aborted*/
    ERROR_FW_UPDATE_ABORTED                     = 505, /**< FW Update aborted*/
    ERROR_FW_SCN_DETTACHED                      = 506, /**< Scanner is disconnected while updating firmware*/
    STATUS_FW_SWCOMP_RESIDENT                   = 600, /**< The software component is already resident in the scanner*/
            
    STATUS_ERROR_INVALID_CONFIG_FILE            = 601,  /**Configuration file hash check failed. Tampered .scncfg file*/
    STATUS_ERROR_INCOMPATIBLE_CONFIG_FILE       = 602,  /**Configuration file is not compatible with selected scanner*/
    STATUS_ERROR_CONFIG_FILE_SYNTAX_VALIDATION_FAILED = 603   /**Configuration file syntax validation failed */
            
};
#endif // STATUS_ID

/*!\enum
 * \brief Event subscription flags types
 */
#ifndef SUBSCRIPTION_FLAG
#define SUBSCRIPTION_FLAG
enum EventSubscriptionFlags
{
    SUBSCRIBE_NONE              = 0,/**< No event subscription */
    SUBSCRIBE_BARCODE           = 1,/**< Subscribe for Barcode events */
    SUBSCRIBE_IMAGE             = 2,/**< Subscribe for Image events*/
    SUBSCRIBE_VIDEO             = 4,/**< Subscribe for Video events */
    SUBSCRIBE_RMD               = 8,/**< Subscribe for RMD events */
    SUBSCRIBE_PNP               = 16,/**< Subscribe for PNP events */
    SUBSCRIBE_CMD_REPONSE       = 32,/**< Subscribe for CMD events*/
    SUBSCRIBE_DIO               = 64,/**< Subscribe for DIO events */
    SUBSCRIBE_NOTIFICATION      = 128/**< Subscribe for Notification events */
};
#endif // SUBSCRIPTION_FLAG



/*!\enum
 * \brief Firmware download events types
 */
#ifndef UF_EVENT_ID
#define UF_EVENT_ID
enum UpdateFirmwareEventID
{
    SCANNER_EVENT_RTA              = 10, /**< Triggered when Real Time Alert Recieved */
    
    SCANNER_UF_SESS_START          = 11, /**< Triggered when flash download session starts */
    SCANNER_UF_DL_START            = 12, /**< Triggered when Component download starts */
    SCANNER_UF_DL_PROGRESS         = 13, /**< Triggered when block(s) of flash completed */
    SCANNER_UF_DL_END              = 14, /**< Triggered when component download ends */
    SCANNER_UF_SESS_END            = 15, /**< Triggered when flash download session end */
    SCANNER_UF_STATUS              = 16, /**< Triggered when update error or status */
            
    SCANNER_UC_SESS_START          = 17, /**< Triggered when configuration update session starts */
    SCANNER_UC_PROGRESS            = 18, /**< Reports configuration push progress */
    SCANNER_UC_SESS_END            = 19, /**< Triggered when configuration update session end */

    SCANNER_EVENT_RSM_DECODE       = 48  /**Triggered when RSM decode event is recieved - Initialised to resolve SSDK-21244 */ 
};
#endif

/*!\enum
 * \brief Plug-and-play event type
 */
enum PnpEventType
{
    SCANNER_ATTACHED = 0, /**< Zebra scanner is attached */
    SCANNER_DETACHED = 1  /**< Zebra scanner is detached. */
};

/*!\enum
 * \brief Image Event Types
 */
enum ImageEventType{
    IMAGE_COMPLETE = 1, /**< A complete image is captured */
    IMAGE_TRAN_STATUS = 2 /**< Image error or status */
};

/*!\enum
 * \brief Video event types
 */
enum VideoEventType{
    VIDEO_FRAME_COMPLETE = 1 /**< A complete video frame is captured */
};


/*!\enum
 * \brief Image Formats types
 */
enum ImageType{
    BMP_FILE_SECTION = 3, /**< BMP */
    TIFF_FILE_SECTION = 4,/**< TIFF */
    JPEG_FILE_SECTION = 1,/**< JPEG */
    ISO_IEC_15434_TYPE = 5/**< IDC */
};

/*!\enum
 * \brief  Scanner types
 */
#ifndef SCANNER_TYPES
#define SCANNER_TYPES
enum ScannerType
{
    SCANNER_TYPE_ALL        = 0xffff,/**< All Scanners */
    SCANNER_TYPE_SNAPI      = 1,/**< SNAPI Scanners */
    SCANNER_TYPE_IBMHID     = 2,/**< IBM Hand-held Scanners */
    SCANNER_TYPE_IBMTT      = 4,/**< IBM Table-top Scanners */
    SCANNER_TYPE_HIDKB      = 8,/**< USB HID Keyboard scanners */
    SCANNER_TYPE_INVALID    = 0/**<Invalid Scanners */
};
#endif // SCANNER_TYPES


/*!\enum
 * \brief Scanner Notification Event Type
 */
enum ScannerNotificationType{
    SCANNER_NOTIFICATION_UKNOWN =0,
    SCANNER_NOTIFICATION_DECODE_MODE, /**< Triggered when a scanner changes its operation mode to decode */
    SCANNER_NOTIFICATION_SNAPSHOT_MODE, /**< Triggered  when a scanner changes its operation mode to image mode */
    SCANNER_NOTIFICATION_VIDEO_MODE, /**< Triggered when a scanner changes its operation mode to video mode*/
    SCANNER_NOTIFICATION_NO_DECODE_EVENT = 1001 /** Triggers when no-decode event is detected */
};

enum UifCode{
    GREEN_LED_ON = 0x2B,
    RED_LED_ON = 0x2F,
    YELLOW_LED_ON = 0x2D,
    GREEN_LED_OFF = 0x2A,
    RED_LED_OFF = 0x30,
    YELLOW_LED_OFF = 0x2E
};

#endif // USERDEFS_H
