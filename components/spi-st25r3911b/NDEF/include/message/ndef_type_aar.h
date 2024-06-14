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
 *  \brief NDEF RTD Android Application Record (AAR) type header file
 *
 * NDEF RTD provides functionalities to handle RTD AAR records.
 *
 * \addtogroup NDEF
 * @{
 *
 */

#ifndef NDEF_TYPE_AAR_H
#define NDEF_TYPE_AAR_H


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


/*
 ******************************************************************************
 * GLOBAL TYPES
 ******************************************************************************
 */

/*! RTD Record Type buffers */
extern const ndefConstBuffer8 bufRtdTypeAar;        /*! AAR (Android Application Record) Record Type buffer */


/*! RTD Android Application Record External Type */
typedef struct
{
    ndefConstBuffer8 bufType;    /*!< AAR type    */
    ndefConstBuffer  bufPayload; /*!< AAR payload */
} ndefTypeRtdAar;


/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/*******************
 * AAR External Type
 *******************
 */

/*!
 *****************************************************************************
 * Initialize an RTD Android Application Record External type
 *
 * \param[out] aar:        Type to initialize
 * \param[in]  bufPayload: Payload buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRtdAarInit(ndefType* aar, const ndefConstBuffer* bufPayload);


/*!
 *****************************************************************************
 * Get RTD Android Application Record type content
 *
 * \param[in]  aar:          Type to get information from
 * \param[out] bufAarString: AAR string buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefGetRtdAar(const ndefType* aar, ndefConstBuffer* bufAarString);


/*!
 *****************************************************************************
 * Convert an NDEF record to an RTD Android Application Record External type
 *
 * \param[in]  record: Record to convert
 * \param[out] aar:    The converted AAR external type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRecordToRtdAar(const ndefRecord* record, ndefType* aar);


/*!
 *****************************************************************************
 * Convert an RTD Android Application Record External type to an NDEF record
 *
 * \param[in]  aar:    AAR External type to convert
 * \param[out] record: The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRtdAarToRecord(const ndefType* aar, ndefRecord* record);


#endif /* NDEF_TYPE_AAR_H */

/**
  * @}
  *
  */
