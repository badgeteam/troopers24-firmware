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
 *  \brief NDEF MIME types
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_record.h"
#include "ndef_types.h"
#include "ndef_type_media.h"
#include "utils.h"


#if NDEF_TYPE_MEDIA_SUPPORT


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
 * Media
 */


/*****************************************************************************/
ndefStatus ndefMediaInit(ndefType* media, const ndefConstBuffer8* bufType, const ndefConstBuffer* bufPayload)
{
    ndefTypeMedia* typeMedia;

    if ( (media == NULL) || (bufType == NULL) || (bufPayload == NULL) )
    {
        return ERR_PARAM;
    }

    media->id               = NDEF_TYPE_ID_MEDIA;
    media->getPayloadLength = NULL;
    media->getPayloadItem   = NULL;
    media->typeToRecord     = ndefMediaToRecord;
    typeMedia               = &media->data.media;

    typeMedia->bufType.buffer    = bufType->buffer;
    typeMedia->bufType.length    = bufType->length;
    typeMedia->bufPayload.buffer = bufPayload->buffer;
    typeMedia->bufPayload.length = bufPayload->length;

    return ERR_NONE;
}


/*****************************************************************************/
ndefStatus ndefGetMedia(const ndefType* media, ndefConstBuffer8* bufType, ndefConstBuffer* bufPayload)
{
    const ndefTypeMedia* typeMedia;

    if ( (media   == NULL) || (media->id != NDEF_TYPE_ID_MEDIA) ||
         (bufType == NULL) || (bufPayload == NULL) )
    {
        return ERR_PARAM;
    }

    typeMedia = &media->data.media;

    bufType->buffer    = typeMedia->bufType.buffer;
    bufType->length    = typeMedia->bufType.length;

    bufPayload->buffer = typeMedia->bufPayload.buffer;
    bufPayload->length = typeMedia->bufPayload.length;

    return ERR_NONE;
}


/*****************************************************************************/
ndefStatus ndefRecordToMedia(const ndefRecord* record, ndefType* media)
{
    const ndefType* type;
    ndefConstBuffer8 bufType;

    if ( (record == NULL) || (media == NULL) )
    {
        return ERR_PARAM;
    }

    if (ndefHeaderTNF(record) != NDEF_TNF_MEDIA_TYPE)
    {
        return ERR_PROTO;
    }

    type = ndefRecordGetNdefType(record);
    if ( (type != NULL) && (type->id == NDEF_TYPE_ID_MEDIA) )
    {
        (void)ST_MEMCPY(media, type, sizeof(ndefType));
        return ERR_NONE;
    }

    bufType.buffer = record->type;
    bufType.length = record->typeLength;

    return ndefMediaInit(media, &bufType, &record->bufPayload);
}


/*****************************************************************************/
ndefStatus ndefMediaToRecord(const ndefType* media, ndefRecord* record)
{
    const ndefTypeMedia* typeMedia;

    if ( (media  == NULL) || (media->id != NDEF_TYPE_ID_MEDIA) ||
         (record == NULL) )
    {
        return ERR_PARAM;
    }

    typeMedia = &media->data.media;

    (void)ndefRecordReset(record);

    (void)ndefRecordSetType(record, NDEF_TNF_MEDIA_TYPE, &typeMedia->bufType);

    (void)ndefRecordSetPayload(record, &typeMedia->bufPayload);

    return ERR_NONE;
}

#endif
