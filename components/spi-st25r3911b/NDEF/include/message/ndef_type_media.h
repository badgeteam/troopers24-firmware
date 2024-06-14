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
 *  \brief NDEF MIME Media type header file
 *
 * NDEF MIME type provides functionalities to handle generic MIME Media records.
 *
 * \addtogroup NDEF
 * @{
 *
 */

#ifndef NDEF_TYPE_MEDIA_H
#define NDEF_TYPE_MEDIA_H


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


/*! Media Type */
typedef struct
{
    ndefConstBuffer8 bufType;    /*!< Media type    */
    ndefConstBuffer  bufPayload; /*!< Media payload */
} ndefTypeMedia;


/*
 ******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/***************
 * Media
 ***************
 */

/*!
 *****************************************************************************
 * Initialize a Media type
 *
 * \param[out] media:      Media type to initialize
 * \param[in]  bufType:    Type buffer
 * \param[in]  bufPayload: Payload buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefMediaInit(ndefType* media, const ndefConstBuffer8* bufType, const ndefConstBuffer* bufPayload);


/*!
 *****************************************************************************
 * Get Media type content
 *
 * \param[in]  media:      Media type to get information from
 * \param[out] bufType:    Type buffer
 * \param[out] bufPayload: Payload buffer
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefGetMedia(const ndefType* media, ndefConstBuffer8* bufType, ndefConstBuffer* bufPayload);


/*!
 *****************************************************************************
 * Convert an NDEF record to a Media type
 *
 * \param[in]  record: Record to convert
 * \param[out] media:  The converted Media type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefRecordToMedia(const ndefRecord* record, ndefType* media);


/*!
 *****************************************************************************
 * Convert a Media type to an NDEF record
 *
 * \param[in]  media:  Type to convert
 * \param[out] record: The converted type
 *
 * \return ERR_NONE if successful or a standard error code
 *****************************************************************************
 */
ndefStatus ndefMediaToRecord(const ndefType* media, ndefRecord* record);


#endif /* NDEF_TYPE_MEDIA_H */

/**
  * @}
  *
  */
