/*
 * ©2022 Zebra Technologies LLC. All rights reserved.
 */

#ifndef SCANNERCLIENTAPI_H
#define SCANNERCLIENTAPI_H

#include <cstddef>
#include <vector>
#include <string>

#include <zebra-scanner/CsUserDefs.h>
#include <zebra-scanner/CsIEventListenerXml.h>

/*! \file Cslibcorescanner_xml-release.h
    \brief Define the methods provided by CoreScanner.
*/


/**
 * Opens a connection to the CoreScanner
 * @return status of the method
 */
unsigned short Open(
    IEventListenerXml* pEventListener/**< [in] Pointer to the eventListener */,
    unsigned int scannerTypeFlags/**< [in] Scanner type flags */,
    StatusID *status/**< [out] Status of the method execution */);


/**
 * Retrieves the list of connected scanners of interested  type
 * @return status of the method
 */
unsigned short GetScanners(
    unsigned short *numberOfScanners/**< [out] Number Of Scanners */,
    std::vector<unsigned int> *scannerIDList/**< [out][in] Scanner ID List */,
    std::string& outXML/**< [out] Output XML */,
    StatusID *status/**< [out] Status of the method execution */);

/**
 * Synchronously executes a command.
 * Function is timed blocking
 * @return status of the method
 */
unsigned short ExecCommand(
    unsigned int opcode /**< [in] Opcode */,
    const std::string inXML /**< [in] Input XML */,
    std::string& outXML/**< [out] Output XML */,
    StatusID *status/**< [out] Status of the method execution */);

//@cond
/**
 * Asynchronously executes a command
 * Functions will return regardless of completion of the command.
 * If user has registered for events with SUBSCRIBE_CMD_REPONSE option,
 * then suitable handler of the provided "IScannerEventListener" object would be invoked.
 * @return status of the method
 */
unsigned short ExecCommandAsync(
        unsigned int opcode /**< [in] Opcode */,
        const std::string inXML /**< [in] Input XML */,
        StatusID *status /**< [out] Status of the method execution */);
//@endcond

/**
 * Closes the client's connection to core scanner
 * @return status of the method
 */
unsigned short Close(
    unsigned int appHandle/**< [in] Application Handler */,
    StatusID *status/**< [Out] Status of the method execution */);


#endif // SCANNERCLIENTAPI_H

