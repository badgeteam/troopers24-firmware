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
 *  \brief NDEF RTD Device Information type
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_record.h"
#include "ndef_types.h"
#include "ndef_type_deviceinfo.h"
#include "utils.h"


#if NDEF_TYPE_RTD_DEVICE_INFO_SUPPORT


/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */


/*! Device Information payload defines */
#define NDEF_RTD_DEVICE_INFO_PAYLOAD_MIN     (2U * (sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t))) /*!< Device Information minimum length (2 required TLV structures) */
#define NDEF_RTD_DEVICE_INFO_PAYLOAD_MAX     ((4U * (sizeof(uint8_t) + sizeof(uint8_t) + 255U)) + (sizeof(uint8_t) + sizeof(uint8_t) + 16U)) /*!< Device Information maximum length */
#define NDEF_RTD_DEVICE_INFO_TLV_LENGTH_MIN  (sizeof(uint8_t) + sizeof(uint8_t))  /*!< Device Information minimum TLV length */


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */


/*! RTD Device Information Type string */
static const uint8_t ndefRtdTypeDeviceInfo[]     = "Di";              /*!< Device Information Record Type {0x44, 0x69} */

const ndefConstBuffer8 bufRtdTypeDeviceInfo      = { ndefRtdTypeDeviceInfo, sizeof(ndefRtdTypeDeviceInfo) - 1U }; /*!< Device Information Record Type buffer */


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
 * Device Information
 */


/*****************************************************************************/
static uint32_t ndefRtdDeviceInfoPayloadGetLength(const ndefType* devInfo)
{
    const ndefTypeRtdDeviceInfo* rtdDevInfo;
    uint32_t payloadLength = 0;

    if ( (devInfo == NULL) || (devInfo->id != NDEF_TYPE_ID_RTD_DEVICE_INFO) )
    {
        return 0;
    }

    rtdDevInfo = &devInfo->data.deviceInfo;

    for (uint32_t i = 0; i < NDEF_DEVICE_INFO_TYPE_COUNT; i++)
    {
        if (rtdDevInfo->devInfo[i].length != 0U)
        {
                             /* sizeof(type) + sizeof(length) + actual length */
            payloadLength += sizeof(rtdDevInfo->devInfo[i].type) + sizeof(rtdDevInfo->devInfo[i].length) + (uint32_t)rtdDevInfo->devInfo[i].length;
        }
    }

    return payloadLength;
}


/*****************************************************************************/
static const uint8_t* ndefRtdDeviceInfoToPayloadItem(const ndefType* devInfo, ndefConstBuffer* bufItem, bool begin)
{
    static uint32_t item = 0;
    const ndefTypeRtdDeviceInfo* rtdDevInfo;
    uint32_t index;

    if ( (devInfo == NULL) || (devInfo->id != NDEF_TYPE_ID_RTD_DEVICE_INFO) ||
         (bufItem == NULL) )
    {
        return NULL;
    }

    rtdDevInfo = &devInfo->data.deviceInfo;

    if (begin == true)
    {
        item = 0;
    }

    bufItem->buffer = NULL;
    bufItem->length = 0;

    index = item / 3U;

    /* Stop streaming on first empty entry */
    if (rtdDevInfo->devInfo[index].length > 0U)
    {
        switch (item % 3U)
        {
        case 0:
            bufItem->buffer = &rtdDevInfo->devInfo[index].type;
            bufItem->length = sizeof(rtdDevInfo->devInfo[index].type);
            break;
        case 1:
            bufItem->buffer = &rtdDevInfo->devInfo[index].length;
            bufItem->length = sizeof(rtdDevInfo->devInfo[index].length);
            break;
        case 2:
            bufItem->buffer = rtdDevInfo->devInfo[index].buffer;
            bufItem->length = rtdDevInfo->devInfo[index].length;
            break;
        /*NOTREACHED*/
        default:
            bufItem->buffer = NULL;
            bufItem->length = 0;
            break;
        }
    }

    /* Move to next item for next call */
    item++;

    return bufItem->buffer;
}


/*****************************************************************************/
ndefStatus ndefRtdDeviceInfoInit(ndefType* devInfo, const ndefDeviceInfoEntry* devInfoData, uint8_t devInfoDataCount)
{
    ndefTypeRtdDeviceInfo* rtdDevInfo;
    uint8_t  count;
    uint8_t  manufacturerNameIndex;
    uint8_t  modelNameIndex;

    if ( (devInfo     == NULL)    ||
         (devInfoData == NULL)    || (devInfoData->length == 0U) ||
         (devInfoDataCount == 0U) || (devInfoDataCount > NDEF_DEVICE_INFO_TYPE_COUNT) )
    {
        return ERR_PARAM;
    }

    devInfo->id               = NDEF_TYPE_ID_RTD_DEVICE_INFO;
    devInfo->getPayloadLength = ndefRtdDeviceInfoPayloadGetLength;
    devInfo->getPayloadItem   = ndefRtdDeviceInfoToPayloadItem;
    devInfo->typeToRecord     = ndefRtdDeviceInfoToRecord;
    rtdDevInfo                = &devInfo->data.deviceInfo;

    /* Clear the Device Information structure before parsing */
    for (uint32_t i = 0; i < NDEF_DEVICE_INFO_TYPE_COUNT; i++)
    {
        rtdDevInfo->devInfo[i].type   = 0;
        rtdDevInfo->devInfo[i].length = 0;
        rtdDevInfo->devInfo[i].buffer = NULL;
    }

    /* Read Type, Length and Value fields */
    /* Not checking multiple occurences of a given field, use the last one */
    count = 0;
    manufacturerNameIndex = 0;
    modelNameIndex = 0;

    while (count < devInfoDataCount)
    {
        uint8_t type   = devInfoData[count].type;
        uint8_t length = devInfoData[count].length;
        if ((type == NDEF_DEVICE_INFO_UUID) && (length != NDEF_UUID_LENGTH))
        {
            return ERR_PROTO;
        }
        if ( (type > NDEF_DEVICE_INFO_TYPE_COUNT) || (length == 0U) )
        {
            return ERR_PROTO;
        }
        if (type == NDEF_DEVICE_INFO_MANUFACTURER_NAME)
        {
            manufacturerNameIndex = count;
        }
        else
        {
            if (type == NDEF_DEVICE_INFO_MODEL_NAME)
            {
                modelNameIndex = count;
            }
        }

        rtdDevInfo->devInfo[count].type   = type;
        rtdDevInfo->devInfo[count].length = length;
        rtdDevInfo->devInfo[count].buffer = devInfoData[count].buffer;
        count++;
    }

    /* Check that both required fields are there */
    if ( (manufacturerNameIndex != modelNameIndex) &&
         (rtdDevInfo->devInfo[manufacturerNameIndex].buffer != NULL) &&
         (rtdDevInfo->devInfo[modelNameIndex].buffer        != NULL) )
    {
        return ERR_NONE;
    }
    else
    {
        return ERR_PARAM;
    }
}


/*****************************************************************************/
ndefStatus ndefGetRtdDeviceInfo(const ndefType* devInfo, ndefTypeRtdDeviceInfo* devInfoData)
{
    const ndefTypeRtdDeviceInfo* rtdDevInfo;

    if ( (devInfo     == NULL) || (devInfo->id != NDEF_TYPE_ID_RTD_DEVICE_INFO) ||
         (devInfoData == NULL) )
    {
        return ERR_PARAM;
    }

    rtdDevInfo = &devInfo->data.deviceInfo;

    for (uint32_t i = 0; i < NDEF_DEVICE_INFO_TYPE_COUNT; i++)
    {
        devInfoData->devInfo[i].type   = rtdDevInfo->devInfo[i].type;
        devInfoData->devInfo[i].length = rtdDevInfo->devInfo[i].length;
        devInfoData->devInfo[i].buffer = rtdDevInfo->devInfo[i].buffer;
    }

    return ERR_NONE;
}


/*****************************************************************************/
static ndefStatus ndefPayloadToRtdDeviceInfo(const ndefConstBuffer* bufDevInfo, ndefType* devInfo)
{
    ndefTypeRtdDeviceInfo* rtdDevInfo;
    uint32_t offset;
    uint8_t  count;
    uint8_t  manufacturerNameIndex;
    uint8_t  modelNameIndex;

    if ( (bufDevInfo == NULL) || (bufDevInfo->buffer == NULL) ||
         (devInfo    == NULL) )
    {
        return ERR_PARAM;
    }

    devInfo->id               = NDEF_TYPE_ID_RTD_DEVICE_INFO;
    devInfo->getPayloadLength = ndefRtdDeviceInfoPayloadGetLength;
    devInfo->getPayloadItem   = ndefRtdDeviceInfoToPayloadItem;
    devInfo->typeToRecord     = ndefRtdDeviceInfoToRecord;
    rtdDevInfo                = &devInfo->data.deviceInfo;

    if ( (bufDevInfo->length < NDEF_RTD_DEVICE_INFO_PAYLOAD_MIN) ||
         (bufDevInfo->length > NDEF_RTD_DEVICE_INFO_PAYLOAD_MAX) )
    {
        return ERR_PROTO;
    }

    /* Extract device information from the buffer */

    /* Clear the Device Information structure before parsing */
    for (uint32_t i = 0; i < NDEF_DEVICE_INFO_TYPE_COUNT; i++)
    {
        rtdDevInfo->devInfo[i].type   = 0;
        rtdDevInfo->devInfo[i].length = 0;
        rtdDevInfo->devInfo[i].buffer = NULL;
    }

    /* Read Type, Length and Value fields */
    /* Not checking multiple occurences of a given field, use the last one */
    offset = 0;
    count = 0;
    manufacturerNameIndex = 0;
    modelNameIndex = 0;

    while ( ((offset + NDEF_RTD_DEVICE_INFO_TLV_LENGTH_MIN) < bufDevInfo->length)
            && (count < NDEF_DEVICE_INFO_TYPE_COUNT) )
    {
        uint8_t type   =  bufDevInfo->buffer[offset];
        uint8_t length =  bufDevInfo->buffer[offset + 1U];
        if ((type == NDEF_DEVICE_INFO_UUID) && (length != NDEF_UUID_LENGTH))
        {
            return ERR_PROTO;
        }
        if ( (type > NDEF_DEVICE_INFO_TYPE_COUNT) || (length == 0U) )
        {
            return ERR_PROTO;
        }
        if (type == NDEF_DEVICE_INFO_MANUFACTURER_NAME)
        {
            manufacturerNameIndex = count;
        }
        else
        {
            if (type == NDEF_DEVICE_INFO_MODEL_NAME)
            {
                modelNameIndex = count;
            }
        }

        rtdDevInfo->devInfo[count].type   = type;
        rtdDevInfo->devInfo[count].length = length;
        rtdDevInfo->devInfo[count].buffer = &bufDevInfo->buffer[offset + 2U];
        count++;

        /* Next entry */
        offset += sizeof(uint8_t) + sizeof(uint8_t) + (uint32_t)length;
    }

    /* Check both required fields are there */
    if ( (manufacturerNameIndex != modelNameIndex) &&
         (rtdDevInfo->devInfo[manufacturerNameIndex].buffer != NULL) &&
         (rtdDevInfo->devInfo[modelNameIndex].buffer        != NULL) )
    {
        return ERR_NONE;
    }
    else
    {
        return ERR_PARAM;
    }
}


/*****************************************************************************/
ndefStatus ndefRecordToRtdDeviceInfo(const ndefRecord* record, ndefType* devInfo)
{
    const ndefType* type;

    if ( (record == NULL) || (devInfo == NULL) )
    {
        return ERR_PARAM;
    }

    if ( ! ndefRecordTypeMatch(record, NDEF_TNF_RTD_WELL_KNOWN_TYPE, &bufRtdTypeDeviceInfo)) /* "Di" */
    {
        return ERR_PROTO;
    }

    type = ndefRecordGetNdefType(record);
    if ( (type != NULL) && (type->id == NDEF_TYPE_ID_RTD_DEVICE_INFO) )
    {
        (void)ST_MEMCPY(devInfo, type, sizeof(ndefType));
        return ERR_NONE;
    }

    return ndefPayloadToRtdDeviceInfo(&record->bufPayload, devInfo);
}


/*****************************************************************************/
ndefStatus ndefRtdDeviceInfoToRecord(const ndefType* devInfo, ndefRecord* record)
{
    if ( (devInfo == NULL) || (devInfo->id != NDEF_TYPE_ID_RTD_DEVICE_INFO) ||
         (record  == NULL) )
    {
        return ERR_PARAM;
    }

    (void)ndefRecordReset(record);

    /* "Di" */
    (void)ndefRecordSetType(record, NDEF_TNF_RTD_WELL_KNOWN_TYPE, &bufRtdTypeDeviceInfo);

    if (ndefRecordSetNdefType(record, devInfo) != ERR_NONE)
    {
        return ERR_PARAM;
    }

    return ERR_NONE;
}

#endif
