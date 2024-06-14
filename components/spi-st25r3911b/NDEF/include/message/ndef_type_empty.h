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
 *  \brief NDEF Empty type header file
 *
 * NDEF RTD provides functionalities to handle empty records.
 *
 * \addtogroup NDEF
 * @{
 *
 */

#ifndef NDEF_TYPE_EMPTY_H
#define NDEF_TYPE_EMPTY_H


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
 * Empty type
 ***************
 */

/*!
 *****************************************************************************
 * Initialize an Empty type
 *
 * \param[out] empty: Type to initialize
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefEmptyTypeInit(ndefType* empty);


/*!
 *****************************************************************************
 * Convert an NDEF record to an Empty type
 *
 * \param[in]  record: Record to convert
 * \param[out] empty:  The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRecordToEmptyType(const ndefRecord* record, ndefType* empty);


/*!
 *****************************************************************************
 * Convert an Empty type to an NDEF record
 *
 * \param[in]  empty:  Type to convert
 * \param[out] record: The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefEmptyTypeToRecord(const ndefType* empty, ndefRecord* record);


#endif /* NDEF_TYPE_EMPTY_H */

/**
  * @}
  *
  */
