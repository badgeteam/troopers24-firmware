
/******************************************************************************
  * @attention
  *
  * COPYRIGHT 2016 STMicroelectronics, all rights reserved
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
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */


#ifndef ST25R3911_DYNAMICPOWER_H
#define ST25R3911_DYNAMICPOWER_H

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "rfal_dpo.h"


/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

 /*! Default DPO table */
const uint8_t rfalDpoDefaultSettings [] = {
                0x00, 255, 200,
                0x01, 210, 150,
                0x02, 160, 100,
                0x03, 110, 50,
};

#endif /* ST25R3911_DYNAMICPOWER_H */
