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
 *  \brief NDEF message
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_record.h"
#include "ndef_message.h"
#include "utils.h"


/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */

#define NDEF_MAX_RECORD          10U    /*!< Maximum number of records */

/*
 ******************************************************************************
 * GLOBAL TYPES
 ******************************************************************************
 */


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
static uint8_t ndefRecordPoolIndex = 0;


/*
 ******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 ******************************************************************************
 */


/*****************************************************************************/
static ndefRecord* ndefAllocRecord(void)
{
    static ndefRecord ndefRecordPool[NDEF_MAX_RECORD];

    if (ndefRecordPoolIndex >= NDEF_MAX_RECORD)
    {
        return NULL;
    }

    return &ndefRecordPool[ndefRecordPoolIndex++];
}


/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */
/*****************************************************************************/


ndefStatus ndefMessageInit(ndefMessage* message)
{
    if (message == NULL)
    {
        return ERR_PARAM;
    }

    message->record           = NULL;
    message->info.length      = 0;
    message->info.recordCount = 0;

    ndefRecordPoolIndex = 0;

    return ERR_NONE;
}


/*****************************************************************************/
ndefStatus ndefMessageGetInfo(const ndefMessage* message, ndefMessageInfo* info)
{
    ndefRecord* record;
    uint32_t    length      = 0;
    uint32_t    recordCount = 0;

    if ( (message == NULL) || (info == NULL) )
    {
        return ERR_PARAM;
    }

    record = message->record;

    while (record != NULL)
    {
        length += ndefRecordGetLength(record);
        recordCount++;

        record = record->next;
    }

    info->length      = length;
    info->recordCount = recordCount;

    return ERR_NONE;
}


/*****************************************************************************/
uint32_t ndefMessageGetRecordCount(const ndefMessage* message)
{
    ndefMessageInfo info;

    if (ndefMessageGetInfo(message, &info) == ERR_NONE)
    {
        return info.recordCount;
    }

    return 0;
}


/*****************************************************************************/
ndefStatus ndefMessageAppend(ndefMessage* message, ndefRecord* record)
{
    if ( (message == NULL) || (record == NULL) )
    {
        return ERR_PARAM;
    }

    record->next = NULL;

    if (message->record == NULL)
    {
        message->record = record;

        /* Set both Message Begin and Message End bits */
        ndefHeaderSetMB(record);
        /* Record is appended so it is the last in the list, set the Message End bit */
        ndefHeaderSetME(record);
    }
    else
    {
        ndefRecord* current = message->record;

        /* Go through the list of records */
        while (current->next != NULL)
        {
            current = current->next;
        }

        /* Clear the Message End bit to the record before the one being appended */
        ndefHeaderClearME(current);

        /* Append to the last record */
        current->next = record;

        /* Clear the Message Begin bit of appended record */
        ndefHeaderClearMB(record);
        /* Record is appended so it is the last in the list, set the Message End bit */
        ndefHeaderSetME(record);
    }

    message->info.length      += ndefRecordGetLength(record);
    message->info.recordCount += 1U;

    return ERR_NONE;
}


/*****************************************************************************/
ndefStatus ndefMessageDecode(const ndefConstBuffer* bufPayload, ndefMessage* message)
{
    ndefStatus err;
    uint32_t offset;

    if ( (bufPayload == NULL) || (bufPayload->buffer == NULL) )
    {
        return ERR_PARAM;
    }

    err = ndefMessageInit(message);
    if (err != ERR_NONE)
    {
        return err;
    }

    offset = 0;
    while (offset < bufPayload->length)
    {
        ndefConstBuffer bufRecord;
        ndefRecord* record = ndefAllocRecord();
        if (record == NULL)
        {
            return ERR_NOMEM;
        }
        bufRecord.buffer = &bufPayload->buffer[offset];
        bufRecord.length =  bufPayload->length - offset;
        err = ndefRecordDecode(&bufRecord, record);
        if (err != ERR_NONE)
        {
            return err;
        }
        offset += ndefRecordGetLength(record);

        err = ndefMessageAppend(message, record);
        if (err != ERR_NONE)
        {
            return err;
        }
    }

    return ERR_NONE;
}


#if NDEF_FEATURE_FULL_API
/*****************************************************************************/
ndefStatus ndefMessageEncode(const ndefMessage* message, ndefBuffer* bufPayload)
{
    ndefStatus      err;
    ndefMessageInfo info = {.length = 0U};
    ndefRecord*     record;
    uint32_t        offset;
    uint32_t        remainingLength;

    if ( (bufPayload == NULL) || (bufPayload->buffer == NULL) )
    {
        return ERR_PARAM;
    }

    err = ndefMessageGetInfo(message, &info);
    if ( (err != ERR_NONE) || (bufPayload->length < info.length) )
    {
        bufPayload->length = info.length;
        return ERR_NOMEM;
    }

    /* Get the first record */
    record          = ndefMessageGetFirstRecord(message);
    offset          = 0;
    remainingLength = bufPayload->length;

    while (record != NULL)
    {
        ndefBuffer bufRecord;
        bufRecord.buffer = &bufPayload->buffer[offset];
        bufRecord.length = remainingLength;
        err = ndefRecordEncode(record, &bufRecord);
        if (err != ERR_NONE)
        {
            bufPayload->length = info.length;
            return err;
        }
        offset          += bufRecord.length;
        remainingLength -= bufRecord.length;

        record = ndefMessageGetNextRecord(record);
    }

    bufPayload->length = offset;
    return ERR_NONE;
}
#endif


#if NDEF_FEATURE_FULL_API
/*****************************************************************************/
ndefRecord* ndefMessageFindRecordType(ndefMessage* message, uint8_t tnf, const ndefConstBuffer8* bufType)
{
    ndefRecord* record;

    record = ndefMessageGetFirstRecord(message);

    while (record != NULL)
    {
        if (ndefRecordTypeMatch(record, tnf, bufType) == true)
        {
            return record;
        }

        record = ndefMessageGetNextRecord(record);
    }

    return record;
}
#endif
