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
 *  \brief NDEF Empty type
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_record.h"
#include "ndef_types.h"
#include "ndef_type_empty.h"
#include "utils.h"


#if NDEF_TYPE_EMPTY_SUPPORT


/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */


/*
 ******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */


/*
 * Empty record
 */


/*****************************************************************************/
static uint32_t ndefEmptyTypePayloadGetLength(const ndefType* empty)
{
    NO_WARNING(empty);

    return 0;
}


/*****************************************************************************/
static const uint8_t* ndefEmptyTypePayloadItem(const ndefType* empty, ndefConstBuffer* bufItem, bool begin)
{
    NO_WARNING(begin);

    if ( (empty == NULL) || (empty->id != NDEF_TYPE_ID_EMPTY) )
    {
        return NULL;
    }

    if (bufItem != NULL)
    {
        bufItem->buffer = NULL;
        bufItem->length = 0;
    }

    return NULL;
}


/*****************************************************************************/
ndefStatus ndefEmptyTypeInit(ndefType* empty)
{
    if (empty == NULL)
    {
        return ERR_PARAM;
    }

    empty->id               = NDEF_TYPE_ID_EMPTY;
    empty->getPayloadLength = ndefEmptyTypePayloadGetLength;
    empty->getPayloadItem   = ndefEmptyTypePayloadItem;
    empty->typeToRecord     = ndefEmptyTypeToRecord;

    return ERR_NONE;
}


/*****************************************************************************/
ndefStatus ndefRecordToEmptyType(const ndefRecord* record, ndefType* empty)
{
    ndefConstBuffer8 bufEmpty = { NULL, 0 };

    if ( (record == NULL) || (empty == NULL) )
    {
        return ERR_PARAM;
    }

    if ( ! ndefRecordTypeMatch(record, NDEF_TNF_EMPTY, &bufEmpty))
    {
        return ERR_PARAM;
    }

    if ( (record->idLength          != 0U) || (record->id                != NULL) ||
         (record->bufPayload.length != 0U) || (record->bufPayload.buffer != NULL) )
    {
        return ERR_PARAM;
    }

    return ndefEmptyTypeInit(empty);
}


/*****************************************************************************/
ndefStatus ndefEmptyTypeToRecord(const ndefType* empty, ndefRecord* record)
{
    if ( (empty  == NULL) || (empty->id != NDEF_TYPE_ID_EMPTY) ||
         (record == NULL) )
    {
        return ERR_PARAM;
    }

    (void)ndefRecordReset(record);

    if (ndefRecordSetNdefType(record, empty) != ERR_NONE)
    {
        return ERR_PARAM;
    }

    return ERR_NONE;
}

#endif
