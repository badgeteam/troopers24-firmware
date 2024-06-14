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
 *  \brief NDEF buffer type structures
 *
 */

#ifndef NDEF_BUFFER_H
#define NDEF_BUFFER_H


/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include <stdint.h>


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


/*! NDEF structure to handle const buffers */
typedef struct
{
    const uint8_t* buffer; /*!< Pointer to const buffer */
    uint32_t       length; /*!< buffer length           */
} ndefConstBuffer;


/*! NDEF structure to handle buffers */
typedef struct
{
    uint8_t* buffer; /*!< Pointer to buffer */
    uint32_t length; /*!< buffer length     */
} ndefBuffer;


/*! NDEF structure to handle const buffers limited to 256 bytes */
typedef struct
{
    const uint8_t* buffer; /*!< Pointer to const buffer */
    uint8_t        length; /*!< buffer length           */
} ndefConstBuffer8;


/*! NDEF structure to handle buffers limited to 256 bytes */
typedef struct
{
    uint8_t* buffer; /*!< Pointer to buffer */
    uint8_t  length; /*!< buffer length     */
} ndefBuffer8;


#endif /* NDEF_BUFFER_H */
