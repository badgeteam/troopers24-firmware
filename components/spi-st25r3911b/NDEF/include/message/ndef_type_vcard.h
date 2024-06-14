/******************************************************************************
  * @attention
  *
  * COPYRIGHT 2019 STMicroelectronics, all rights reserved
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*
 *      PROJECT:   NDEF firmware
 *      Revision:
 *      LANGUAGE:  ISO C99
 */

/*! \file
 *
 *  \author
 *
 *  \brief NDEF MIME vCard type header file
 *
 * NDEF MIME type provides functionalities to handle vCard records.
 *
 * \addtogroup NDEF
 * @{
 *
 */

#ifndef NDEF_TYPE_VCARD_H
#define NDEF_TYPE_VCARD_H


/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_record.h"
#include "ndef_buffer.h"


/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */


#define NDEF_VCARD_PROPERTY_COUNT       16U    /*!< Number of properties that can be decoded */


/*
 ******************************************************************************
 * GLOBAL TYPES
 ******************************************************************************
 */

/*! vCard Record Type buffer */
extern const ndefConstBuffer8 bufMediaTypeVCard; /*! vCard Record Type buffer */


/*! NDEF Type vCard */
typedef struct
{
    const uint8_t* propertyBuffer[NDEF_VCARD_PROPERTY_COUNT]; /*!< vCard property buffers  */
    uint16_t       propertyLength[NDEF_VCARD_PROPERTY_COUNT]; /*!< vCard property buffers length */
} ndefTypeVCard;


/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/*!
 *****************************************************************************
 * Compare two NDEF buffers
 *
 * \param[in]  buf1: reference buffer
 * \param[in]  buf2: buffer to compare to the reference buffer
 *
 * \return true if both buffers content and length match
 *****************************************************************************
 */
bool ndefBufferMatch(const ndefConstBuffer* buf1, const ndefConstBuffer* buf2);


/***************
 * vCard
 ***************
 */


/*!
 *****************************************************************************
 * Parse a vCard property to get the type, subtype if any, and its value
 *
 * \param[in]  bufProperty: vCard Property to parse
 * \param[out] bufType:    property type
 * \param[out] bufSubtype: property subtype
 * \param[out] bufValue:   property value
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardParseProperty(const ndefConstBuffer* bufProperty, ndefConstBuffer* bufType, ndefConstBuffer* bufSubtype, ndefConstBuffer* bufValue);


/*!
 *****************************************************************************
 * Add a property to the vCard type
 *
 * \param[in] vCard:       vCard type
 * \param[in] bufProperty: vCard Property to add, contain the type, subtype if any and its value
 *
 * \return ERR_NONE if successful, ERR_NOMEM or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardSetProperty(ndefTypeVCard* vCard, const ndefConstBuffer* bufProperty);


/*!
 *****************************************************************************
 * Get a pointer to a vCard property
 *
 * \param[in]  vCard:       vCard type
 * \param[in]  bufType:     Type to find
 * \param[out] bufProperty: The vCard property matching bufType: contain the type, subtype if any and the property value
 *
 * \return ERR_NONE if successful, ERR_NOTFOUND or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardGetProperty(const ndefTypeVCard* vCard, const ndefConstBuffer* bufType, ndefConstBuffer* bufProperty);


/*!
 *****************************************************************************
 * Reset a vCard type
 *
 * \param[in] vCard: vCard type
 *
 * This function resets each field of the ndefTypeVCard structure
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardReset(ndefTypeVCard* vCard);


/*!
 *****************************************************************************
 * Initialize a vCard
 *
 * \param[out] type:  Type to initialize
 * \param[in]  vCard: vCard type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardInit(ndefType* type, const ndefTypeVCard* vCard);


/*!
 *****************************************************************************
 * Get vCard content
 *
 * \param[in]  type:  vCard type to get information from
 * \param[out] vCard: vCard type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefGetVCard(const ndefType* type, ndefTypeVCard* vCard);


/*!
 *****************************************************************************
 * Convert an NDEF record to a vCard
 *
 * \param[in]  record: Record to convert
 * \param[out] type:   The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRecordToVCard(const ndefRecord* record, ndefType* type);


/*!
 *****************************************************************************
 * Convert a vCard type to an NDEF record
 *
 * \param[in]  type:   Type to convert
 * \param[out] record: The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefVCardToRecord(const ndefType* type, ndefRecord* record);


#endif /* NDEF_TYPE_VCARD_H */

/**
  * @}
  *
  */
