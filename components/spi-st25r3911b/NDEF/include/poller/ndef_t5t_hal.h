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
 *  NDEF provides several functionalities required to perform NFC NDEF activities. 
 *  <br>The NDEF encapsulates the different tag technologies (T2T, T3T, T4AT, T4BT, T5T)
 *  into a common and easy to use interface.
 *
 *  This file provides the hardware-dependent API to adapt to a given platform
 *  to benefit from the NDEF generic functionalities.
 *  The main functions are:
 *
 *      <br>&nbsp; ndefT5TisSTDevice()
 *      <br>&nbsp; ndefT5TisT5TDeviceType()
 *      <br>&nbsp; ndefT5TPollerAccessMode()
 *      <br>&nbsp; ndefT5TGetBlockLength()
 *      <br>&nbsp; ndefT5TGetMemoryConfig()
 *      <br>&nbsp; ndefT5TIsMultipleBlockReadSupported()
 *      <br>&nbsp; ndefT5TIsDevicePresent()
 *      <br>&nbsp; ndefT5TLockDevice()
 *
 * \addtogroup NDEF
 * @{
 *
 */


#ifndef NDEF_T5T_HAL_H
#define NDEF_T5T_HAL_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "ndef_config.h"
#include "ndef_poller.h"
#include "ndef_t5t.h"


/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */


/*
 ******************************************************************************
 * GLOBAL MACROS
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


/*!
 *****************************************************************************
 * \brief Return whether the device type is a STMicroelectronics device
 *
 * \param[in] dev: ndef Device
 *
 * \return true if it is a STMicroelectronics device
 *****************************************************************************
 */
bool ndefT5TisSTDevice(const ndefDevice *dev);


/*!
 *****************************************************************************
 * \brief Return whether the device type is a T5T device
 *
 * \param[in] dev: ndef Device
 *
 * \return true if it is a T5T device
 *****************************************************************************
 */
bool ndefT5TisT5TDevice(const ndefDevice *dev);


/*!
 *****************************************************************************
 * \brief Set T5T device access mode
 *
 * \param[in] ctx: ndef Context
 * \param[in] dev: ndef Device
 * \param[in] mode: Acces mode
 *
 * \return ERR_PARAM        : Invalid parameter
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ndefStatus ndefT5TPollerAccessMode(ndefContext *ctx, const ndefDevice *dev, ndefT5TAccessMode mode);


/*!
 *****************************************************************************
 * \brief Get Block length
 *
 * This function returns the block length, in bytes
 *
 * \param[in] ctx: ndef Context
 *
 * \return 0 if invalid parameter or the block length
 *****************************************************************************
 */
uint8_t ndefT5TGetBlockLength(ndefContext *ctx);


/*!
 *****************************************************************************
 * \brief Get the memory configuration
 *
 * This function provides the number of blocks and the block size in the
 * T5T system information structure.
 *
 * \param[in] ctx: ndef Context
 *
 * \return ERR_PARAM: Invalid parameter
 * \return ERR_NONE : No error
 *****************************************************************************
 */
ndefStatus ndefT5TGetMemoryConfig(ndefContext *ctx);


/*!
 *****************************************************************************
 * \brief Return whether the device supports Multiple block read
 *
 * \param[in] ctx: ndef Context
 *
 * \return true or false
 *****************************************************************************
 */
bool ndefT5TIsMultipleBlockReadSupported(ndefContext *ctx);


/*!
 *****************************************************************************
 * \brief Check Presence
 *
 * This method checks whether an NFC tag is still present in the operating field
 *
 * \param[in] ctx : ndef Context
 *
 * \return ERR_WRONG_STATE  : Library not initialized or mode not set
 * \return ERR_PARAM        : Invalid parameter
 * \return ERR_PROTO        : Protocol error
 * \return ERR_NONE         : No error
 *****************************************************************************
 */
ndefStatus ndefT5TIsDevicePresent(ndefContext *ctx);


/*!
 *****************************************************************************
 * \brief This function locks the device
 *
 * \param[in] ctx: ndef Context
 *
 * \return ERR_PARAM      : Invalid parameter
 * \return ERR_NONE       : No error
 *****************************************************************************
 */
ndefStatus ndefT5TLockDevice(ndefContext *ctx);


#endif /* NDEF_T5T_HAL_H */

/**
  * @}
  *
  */
