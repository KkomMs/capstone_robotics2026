/*
 * ©2022 Zebra Technologies LLC. All rights reserved.
 */

#ifndef IEVENTLISTENERXML_H
#define IEVENTLISTENERXML_H

#include <string>

/*! \file CsIEventListenerXml.h
    \brief Define the events provided by CoreScanner.
*/
class IEventListenerXml {
  
public:

    /**
     * Triggers when an imaging scanner captures images in image mode.
     * @param eventType Type of image event received. This is always 1 at the moment.
     * @param size Size of image data buffer
     * @param imageFormat Format of image. 1 for JPEG, 3 for BMP, 4 for TIFF
     * @param sfimageData Image data buffer
     * @param dataLength Data length
     * @param pScannerData Scanner information XML
     * @deprecated
     */
    virtual void OnImageEvent(short eventType, int size, short imageFormat, char* sfimageData, int dataLength, std::string& pScannerData) = 0;

    /**
     * Triggers when an imaging scanner captures images in image mode.
     * @param eventType Type of image event received. This is always 1 at the moment.
     * @param size Size of image data buffer
     * @param imageFormat Format of image. 1 for JPEG, 3 for BMP, 4 for TIFF
     * @param sfimageData Image data buffer
     * @param pScannerData Scanner information XML
     */
    virtual void OnImageEvent(short eventType, int size, short imageFormat, char *sfimageData, std::string& pScannerData) {
        // if no new method implemented then call the deprecated method //
        this->OnImageEvent(eventType, size, imageFormat, sfimageData, size, pScannerData);
    }

    /**
     * Triggers when an IDC-supported imaging scanner captures an image in Intelligent Document Capture (IDC).
     * @param eventType Type of Binary Data event received.
     * @param dataLength Size of the BinaryData data buffer
     * @param dataFormat The format of the Binary DataEvent. This is always 0xB5 at the moment
     * @param sfBinaryData IDC data buffer
     * @param pScannerData Scanner information XML
     */
    virtual void OnBinaryDataEvent(short eventType, int dataLength, short dataFormat, unsigned char* sfBinaryData, std::string& pScannerData) = 0;

    /**
     * Triggers when an imaging scanner captures video in video mode.
     * @param eventType Type of video event received . This is always 1 at the moment.
     * @param size Size of video frame
     * @param sfvideoData Video data buffer
     * @param dataLength Size of video frame
     * @param pScannerData Scanner information XML
     */
    virtual void OnVideoEvent(short eventType, int size, char* sfvideoData, int dataLength, std::string& pScannerData) = 0;

    /**
     * Triggers when a scanner captures barcode.
     * @param eventType Type of barcode event received . This is always 1 at the moment.
     * @param pscanData Barcode string that contains information about the scanner that triggered the barcode event including data type, data label and raw data of the scanned barcode.
     */
    virtual void OnBarcodeEvent(short eventType, std::string& pscanData) = 0;

    /**
     * Triggers when a scanner attaches to the system or detaches from the system. The pairing of a Bluetooth scanner to a cradle does not trigger a PnP event. To receive information about a newly paired device, the GetScanners command must be called again.
     * @param eventType Type of PnP event received. 0 for scanner attaches 1 for scanner detaches. 
     * @param ppnpData PnP information string containing the asset tracking information of the attached or detached device
     */
    virtual void OnPNPEvent(short eventType, std::string ppnpData) = 0;

    //@cond 
    /**
     * Not supported yet
     * @param status
     * @param prspData
     */
    virtual void OnCommandResponseEvent(short status, std::string& prspData) = 0;
    //@endcond
    
    /**
     * Triggers when a SNAPI scanner changes it's operational mode.
     * @param notificationType Type of the notification event received. 1 for Decode mode, 2 for image mode, 3 for video mode
     * @param pScannerData Scanner information XML
     */
    virtual void OnScannerNotification(short notificationType, std::string& pScannerData) = 0;

    //@cond 
    /**
     * Not supported yet
     * @param type
     * @param data
     */

    virtual void OnIOEvent(short type, unsigned char data) = 0;
    //@endcond
    
    /**
     * The Event which receives when updating scanner firmware
     * @param eventType Type of RMD event received
     * @param prmdData Scanner information XML
     */
    virtual void OnScanRMDEvent(short eventType, std::string& prmdData) = 0;

    /**
     * Receives when CoreScanner service disconnects from client library
     */
    virtual void OnDisconnect() = 0;

};


#endif // IEVENTLISTENERXML_H

