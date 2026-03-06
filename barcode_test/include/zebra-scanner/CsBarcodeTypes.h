/*
 * ©2022 Zebra Technologies LLC. All rights reserved.
 */

#ifndef BARCODETYPES_H
#define BARCODETYPES_H

#include <tr1/unordered_map>
/*! \file CsBarcodeTypes.h
    \brief Barcode types receiving from OnBarcodeEvent.
*/

/*
 * Macro definition
 * Holds the number of elements in the "g_BarcodeTypeInfoMap" array
 * Must increase if new elements added to the array.
 */
#define NO_OF_ELEMENTS_IN_BARCODE_ELEMENT_INFO_MAP 209

/*!\enum
 * \brief Barcode Symbologies/Types
 */
enum SymbolType
{
    ST_NOT_APP                  = 0x00,/**< Not applicable */
    ST_CODE_39                  = 0x01,/**< Code 39 */
    ST_CODABAR                  = 0x02,/**< Codabar */
    ST_CODE_128                 = 0x03,/**< Code 128 */
    ST_D2OF5                    = 0x04,/**< Discreet 2 of 5*/
    ST_IATA                     = 0x05,/**< IATA */
    ST_I2OF5                    = 0x06,/**< Interleaved 2 of 5  */
    ST_CODE93                   = 0x07,/**< Code 93 */
    ST_UPCA                     = 0x08,/**< UPC-A */
    ST_UPCE0                    = 0x09,/**< UPC-E0 */
    ST_EAN8                     = 0x0a,/**< EAN-8 */
    ST_EAN13                    = 0x0b,/**< EAN-13 */
    ST_CODE11                   = 0x0c,/**< Code 11 */
    ST_CODE49                   = 0x0d,/**< Code 49 */
    ST_MSI                      = 0x0e,/**< MSI */
    ST_EAN128                   = 0x0f,/**< EAN-128  */
    ST_UPCE1                    = 0x10,/**< UPC-E1 */
    ST_PDF417                   = 0x11,/**< PDF-417 */
    ST_CODE16K                  = 0x12,/**< Code 16K */
    ST_C39FULL                  = 0x13,/**< Code 39 Full ASCII */
    ST_UPCD                     = 0x14,/**< UPC-D */
    ST_TRIOPTIC                 = 0x15,/**< Code 39 Trioptic */
    ST_BOOKLAND                 = 0x16,/**< Bookland */
    ST_COUPON                   = 0x17,/**< Coupon Code */
    ST_NW7                      = 0x18,/**< NW-7 */
    ST_ISBT128                  = 0x19,/**< ISBT-128 */
    ST_MICRO_PDF                = 0x1a,/**< Micro PDF */
    ST_DATAMATRIX               = 0x1b,/**< DataMatrix */
    ST_QR_CODE                  = 0x1c,/**< QR Code */
    ST_MICRO_PDF_CCA            = 0x1d,/**< Micro PDF CCA */
    ST_POSTNET_US		= 0x1e,/**< PostNet US */
    ST_PLANET_CODE		= 0x1f,/**< Planet Code */
    ST_CODE_32			= 0x20,/**< Code 32 */
    ST_ISBT128_CON		= 0x21,/**< ISBT-128 Con */
    ST_JAPAN_POSTAL		= 0x22,/**< Japan Postal */
    ST_AUS_POSTAL               = 0x23,/**< Australian Postal */
    ST_DUTCH_POSTAL		= 0x24,/**< Dutch Postal */
    ST_MAXICODE			= 0x25,/**< MaxiCode */
    ST_CANADIN_POSTAL           = 0x26,/**< Canadian Postal */
    ST_UK_POSTAL                = 0x27,/**< UK Postal */
    ST_MACRO_PDF                = 0x28,/**< Macro PDF */
    ST_RSS14			= 0x30,/**< GS1 Databar (RSS-14) */
    ST_RSS_LIMITET		= 0x31,/**< RSS Limited */
    ST_RSS_EXPANDED		= 0x32,/**< GS1 Databar Expanded (RSS Expanded) */
    ST_SCANLET			= 0x37,/**< Scanlet */
    ST_UPCA_2                   = 0x48,/**< UPC-A + 2 Supplemental */
    ST_UPCE0_2			= 0x49,/**< UPC-E0 + 2 Supplemental */
    ST_EAN8_2			= 0x4a,/**< EAN-8 + 2 Supplemental */
    ST_EAN13_2			= 0x4b,/**< EAN-13 + 2 Supplemental */
    ST_UPCE1_2			= 0x50,/**< UPC-E1 + 2 Supplemental */
    ST_CCA_EAN128		= 0x51,/**< CCA EAN-128 */
    ST_CCA_EAN13		= 0x52,/**< CCA EAN-13 */
    ST_CCA_EAN8			= 0x53,/**< CCA EAN-8 */
    ST_CCA_RSS_EXPANDED         = 0x54,/**< CCA RSS Expanded */
    ST_CCA_RSS_LIMITED          = 0x55,/**< CCA RSS Limited */
    ST_CCA_RSS14		= 0x56,/**< CCA RSS-14 */
    ST_CCA_UPCA			= 0x57,/**< CCA UPC-A */
    ST_CCA_UPCE			= 0x58,/**< CCA UPC-E */
    ST_CCC_EAN128		= 0x59,/**< CCC EAN-128 */
    ST_TLC39			= 0x5A,/**< TLC-39 */
    ST_CCB_EAN128		= 0x61,/**< CCB EAN-128 */
    ST_CCB_EAN13		= 0x62,/**< CCB EAN-13 */
    ST_CCB_EAN8			= 0x63,/**< CCB EAN-8 */
    ST_CCB_RSS_EXPANDED		= 0x64,/**< CCB RSS Expanded */
    ST_CCB_RSS_LIMITED		= 0x65,/**< CCB RSS Limited */
    ST_CCB_RSS14		= 0x66,/**< CCB RSS-14 */
    ST_CCB_UPCA                 = 0x67,/**< CCB UPC-A */
    ST_CCB_UPCE                 = 0x68,/**< CCB UPC-E */
    ST_SIGNATURE_CAPTURE        = 0x69,/**< Signature Capture */
    ST_MATRIX2OF5               = 0x71,/**< Matrix 2 of 5 */
    ST_CHINESE2OF5              = 0x72,/**< Chinese 2 of 5 */
    ST_UPCA_5                   = 0x88,/**< UPC-A + 5 Supplemental */
    ST_UPCE0_5                  = 0x89,/**< UPC-E0 + 5 Supplemental */
    ST_EAN8_5                   = 0x8a,/**< EAN-8 + 5 Supplemental */
    ST_EAN13_5                  = 0x8b,/**< EAN-13 + 5 Supplemental */
    ST_UPCE1_5                  = 0x90,/**< UPC-E1 + 5 Supplemental */
    ST_MACRO_MICRO_PDF          = 0x9A,/**< Macro Micro PDF */
    ST_MICRO_QR_CODE            = 0x2c,/**< Micro QR code */
    ST_AZTEC                    = 0x2d,/**< Aztec */
    ST_HAN_XIN_CODE             = 0xB7,/**<    */
    ST_GS1_DATAMATRIX           = 0xC1,/**< GS1 DATA MATRIX */
    ST_GS1_QR                   = 0xC2,/**<    */
    ST_DOTCODE                  = 0xC4,/**< 196 = DotCode  */
    ST_GRIDMATRIX               = 0xC8, /**< 200 = Grid Matrix  */
    ST_RECTANGULAR_MICRO_QR     = 0xD1 /**< 209 = Rectangular Micro QR Code */

};


/**
 * Get the name of the provided symbology
 * @param eType Symbology
 * @return Name type
 */
extern const char* GetNameOfType( SymbolType eType );


#endif // BARCODETYPES_H

