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
 *  \brief Provides NDEF methods and definitions to access NFC Forum Tags
 *
 *  This module provides an interface to handle NDEF message
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_poller.h"
#include "ndef_t2t.h"
#include "ndef_t3t.h"
#include "ndef_t4t.h"
#include "ndef_t5t_hal.h"
#include "ndef_t5t.h"
#include "utils.h"

/*
 ******************************************************************************
 * ENABLE SWITCH
 ******************************************************************************
 */

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
 * GLOBAL MACROS
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
 * GLOBAL VARIABLE DEFINITIONS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */


/*******************************************************************************/
ndefStatus ndefPollerContextInitialization(ndefContext *ctx, const ndefDevice *dev)
{
    ndefDeviceType type;

#if NDEF_FEATURE_T1T
    static const ndefPollerWrapper ndefT1TWrapper =
    {
        NULL, /* ndefT1TPollerContextInitialization, */
        NULL, /* ndefT1TPollerNdefDetect,            */
        NULL, /* ndefT1TPollerReadBytes,             */
        NULL, /* ndefT1TPollerReadRawMessage,        */
#if NDEF_FEATURE_FULL_API
        NULL, /* ndefT1TPollerWriteBytes,            */
        NULL, /* ndefT1TPollerWriteRawMessage,       */
        NULL, /* ndefT1TPollerTagFormat,             */
        NULL, /* ndefT1TPollerWriteRawMessageLen     */
        NULL, /* ndefT1TPollerCheckPresence          */
        NULL, /* ndefT1TPollerCheckAvailableSpace    */
        NULL, /* ndefT1TPollerBeginWriteMessage      */
        NULL, /* ndefT1TPollerEndWriteMessage        */
        NULL  /* ndefT1TPollerSetReadOnly            */
#endif /* NDEF_FEATURE_FULL_API */
    };
#endif /* NDEF_FEATURE_T1T */

#if NDEF_FEATURE_T2T
    static const ndefPollerWrapper ndefT2TWrapper =
    {
        ndefT2TPollerContextInitialization,
        ndefT2TPollerNdefDetect,
        ndefT2TPollerReadBytes,
        ndefT2TPollerReadRawMessage,
#if NDEF_FEATURE_FULL_API
        ndefT2TPollerWriteBytes,
        ndefT2TPollerWriteRawMessage,
        ndefT2TPollerTagFormat,
        ndefT2TPollerWriteRawMessageLen,
        ndefT2TPollerCheckPresence,
        ndefT2TPollerCheckAvailableSpace,
        ndefT2TPollerBeginWriteMessage,
        ndefT2TPollerEndWriteMessage,
        ndefT2TPollerSetReadOnly
#endif /* NDEF_FEATURE_FULL_API */
    };
#endif /* NDEF_FEATURE_T2T */

#if NDEF_FEATURE_T3T
    static const ndefPollerWrapper ndefT3TWrapper =
    {
        ndefT3TPollerContextInitialization,
        ndefT3TPollerNdefDetect,
        ndefT3TPollerReadBytes,
        ndefT3TPollerReadRawMessage,
#if NDEF_FEATURE_FULL_API
        ndefT3TPollerWriteBytes,
        ndefT3TPollerWriteRawMessage,
        ndefT3TPollerTagFormat,
        ndefT3TPollerWriteRawMessageLen,
        ndefT3TPollerCheckPresence,
        ndefT3TPollerCheckAvailableSpace,
        ndefT3TPollerBeginWriteMessage,
        ndefT3TPollerEndWriteMessage,
        ndefT3TPollerSetReadOnly
#endif /* NDEF_FEATURE_FULL_API */
    };
#endif /* NDEF_FEATURE_T3T */

#if NDEF_FEATURE_T4T
    static const ndefPollerWrapper ndefT4TWrapper =
    {
        ndefT4TPollerContextInitialization,
        ndefT4TPollerNdefDetect,
        ndefT4TPollerReadBytes,
        ndefT4TPollerReadRawMessage,
#if NDEF_FEATURE_FULL_API
        ndefT4TPollerWriteBytes,
        ndefT4TPollerWriteRawMessage,
        ndefT4TPollerTagFormat,
        ndefT4TPollerWriteRawMessageLen,
        ndefT4TPollerCheckPresence,
        ndefT4TPollerCheckAvailableSpace,
        ndefT4TPollerBeginWriteMessage,
        ndefT4TPollerEndWriteMessage,
        ndefT4TPollerSetReadOnly
#endif /* NDEF_FEATURE_FULL_API */
    };
#endif /* NDEF_FEATURE_T4T */

#if NDEF_FEATURE_T5T
    static const ndefPollerWrapper ndefT5TWrapper =
    {
        ndefT5TPollerContextInitialization,
        ndefT5TPollerNdefDetect,
        ndefT5TPollerReadBytes,
        ndefT5TPollerReadRawMessage,
#if NDEF_FEATURE_FULL_API
        ndefT5TPollerWriteBytes,
        ndefT5TPollerWriteRawMessage,
        ndefT5TPollerTagFormat,
        ndefT5TPollerWriteRawMessageLen,
        ndefT5TPollerCheckPresence,
        ndefT5TPollerCheckAvailableSpace,
        ndefT5TPollerBeginWriteMessage,
        ndefT5TPollerEndWriteMessage,
        ndefT5TPollerSetReadOnly
#endif /* NDEF_FEATURE_FULL_API */
    };
#endif /* NDEF_FEATURE_T5T */

    static const ndefPollerWrapper *ndefPollerWrappers[] =
    {
        NULL,            /* No device */
#if NDEF_FEATURE_T1T
        &ndefT1TWrapper,
#else
        NULL,
#endif
#if NDEF_FEATURE_T2T
        &ndefT2TWrapper,
#else
        NULL,
#endif
#if NDEF_FEATURE_T3T
        &ndefT3TWrapper,
#else
        NULL,
#endif
#if NDEF_FEATURE_T4T
        &ndefT4TWrapper,
#else
        NULL,
#endif
#if NDEF_FEATURE_T5T
        &ndefT5TWrapper,
#else
        NULL,
#endif
    };

    if( (ctx == NULL) || (dev == NULL) )
    {
        return ERR_PARAM;
    }

    type = ndefGetDeviceType(dev);

    if( (type == NDEF_DEV_NONE) || ((uint32_t)type >= SIZEOF_ARRAY(ndefPollerWrappers)) )
    {
        return ERR_PARAM;
    }

    ctx->ndefPollWrapper = ndefPollerWrappers[type];

    /* ndefPollWrapper is NULL when support of a given tag type is not enabled */
    if( (ctx->ndefPollWrapper == NULL) || (ctx->ndefPollWrapper->pollerContextInitialization == NULL) )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerContextInitialization)(ctx, dev);
}

/*******************************************************************************/
ndefStatus ndefPollerNdefDetect(ndefContext *ctx, ndefInfo *info)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerNdefDetect == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerNdefDetect)(ctx, info);
}

/*******************************************************************************/
ndefStatus ndefPollerReadRawMessage(ndefContext *ctx, uint8_t *buf, uint32_t bufLen, uint32_t *rcvdLen, bool single)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerReadRawMessage == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerReadRawMessage)(ctx, buf, bufLen, rcvdLen, single);
}

/*******************************************************************************/
ndefStatus ndefPollerReadBytes(ndefContext *ctx, uint32_t offset, uint32_t len, uint8_t *buf, uint32_t *rcvdLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerReadBytes == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerReadBytes)(ctx, offset, len, buf, rcvdLen);
}

#if NDEF_FEATURE_FULL_API

/*******************************************************************************/
ndefStatus ndefPollerWriteRawMessage(ndefContext *ctx, const uint8_t *buf, uint32_t bufLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerWriteRawMessage == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerWriteRawMessage)(ctx, buf, bufLen);
}

/*******************************************************************************/
ndefStatus ndefPollerTagFormat(ndefContext *ctx, const ndefCapabilityContainer *cc, uint32_t options)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerTagFormat == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerTagFormat)(ctx, cc, options);
}

/*******************************************************************************/
ndefStatus ndefPollerWriteRawMessageLen(ndefContext *ctx, uint32_t rawMessageLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerWriteRawMessageLen == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerWriteRawMessageLen)(ctx, rawMessageLen, true);
}

/*******************************************************************************/
ndefStatus ndefPollerWriteBytes(ndefContext *ctx, uint32_t offset, const uint8_t *buf, uint32_t len)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerWriteBytes == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerWriteBytes)(ctx, offset, buf, len, false, false);
}

/*******************************************************************************/
ndefStatus ndefPollerCheckPresence(ndefContext *ctx)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerCheckPresence == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerCheckPresence)(ctx);
}

/*******************************************************************************/
ndefStatus ndefPollerCheckAvailableSpace(const ndefContext *ctx, uint32_t messageLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerCheckAvailableSpace == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerCheckAvailableSpace)(ctx, messageLen);
}

/*******************************************************************************/
ndefStatus ndefPollerBeginWriteMessage(ndefContext *ctx, uint32_t messageLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerBeginWriteMessage == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerBeginWriteMessage)(ctx, messageLen);
}

/*******************************************************************************/
ndefStatus ndefPollerEndWriteMessage(ndefContext *ctx, uint32_t messageLen)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerEndWriteMessage == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerEndWriteMessage)(ctx, messageLen, true);
}

/*******************************************************************************/
ndefStatus ndefPollerSetReadOnly(ndefContext *ctx)
{
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }

    if( ctx->ndefPollWrapper == NULL )
    {
        return ERR_WRONG_STATE;
    }

    if( ctx->ndefPollWrapper->pollerSetReadOnly == NULL )
    {
        return ERR_NOTSUPP;
    }

    return (ctx->ndefPollWrapper->pollerSetReadOnly)(ctx);
}

#endif /* NDEF_FEATURE_FULL_API */
