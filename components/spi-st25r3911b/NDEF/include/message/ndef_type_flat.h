/******************************************************************************
  * @attention
  *
  * COPYRIGHT 2020 STMicroelectronics, all rights reserved
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
 *  \brief NDEF Flat payload type header file
 *
 * NDEF RTD provides functionalities to handle flat payload records.
 *
 * \addtogroup NDEF
 * @{
 *
 */

#ifndef NDEF_TYPE_FLAT_H
#define NDEF_TYPE_FLAT_H


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


/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/***************
 * Flat type
 ***************
 */

/*!
 *****************************************************************************
 * Initialize a flat payload type
 *
 * \param[out] type:       Type to initialize
 * \param[in]  bufPayload: Payload buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefFlatPayloadTypeInit(ndefType* type, const ndefConstBuffer* bufPayload);


/*!
 *****************************************************************************
 * Initialize a flat payload type
 *
 * \param[out] type:       Type to get data from
 * \param[in]  bufPayload: Payload buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefGetFlatPayloadType(const ndefType* type, ndefConstBuffer* bufPayload);


/*!
 *****************************************************************************
 * Convert an NDEF record to a flat payload type
 *
 * \param[in]  record: Record to convert
 * \param[out] type:  The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRecordToFlatPayloadType(const ndefRecord* record, ndefType* type);


/*!
 *****************************************************************************
 * Convert a flat payload type to an NDEF record
 *
 * \param[in]  type:   Type to convert
 * \param[out] record: The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefFlatPayloadTypeToRecord(const ndefType* type, ndefRecord* record);


#endif /* NDEF_TYPE_FLAT_H */

/**
  * @}
  *
  */
