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
 *  This module provides an interface to handle the device type
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "ndef_poller.h"

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
 *****************************************************************************
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
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */

/*******************************************************************************/
ndefDeviceType ndefGetDeviceType(const ndefDevice *dev)
{
    ndefDeviceType type = NDEF_DEV_NONE;

    if( dev != NULL )
    {
        switch( dev->type )
        {
        case RFAL_NFC_LISTEN_TYPE_NFCA:
            switch( dev->dev.nfca.type )
            {
                case RFAL_NFCA_T1T:
                    type = NDEF_DEV_T1T;
                    break;
                case RFAL_NFCA_T2T:
                    type = NDEF_DEV_T2T;
                    break;
                case RFAL_NFCA_T4T:
                    type = NDEF_DEV_T4T;
                    break;
                default:
                    break;
            }
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCB:
            type = NDEF_DEV_T4T;
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCF:
            type = NDEF_DEV_T3T;
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCV:
            type = NDEF_DEV_T5T;
            break;
        default:
            break;
        }
    }

    return type;
}
