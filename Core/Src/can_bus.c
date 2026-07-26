/*
 * can_bus.c
 *
 *  CAN transport layer for the VCU (RTOS version).
 *
 *  RX path (classic ISR -> queue -> task pattern):
 *    frame arrives -> CAN1 FIFO0 -> RX0 interrupt
 *      -> HAL_CAN_RxFifo0MsgPendingCallback: read frame, post to queue
 *      -> CanRxTask: CanBus_ProcessRx() takes from queue, decodes into
 *         the status structs while holding the status mutex.
 *
 *  ControlTask reads those structs under the same mutex
 *  (CanBus_LockStatus / CanBus_UnlockStatus) -> no torn reads.
 */

#include "can_bus.h"
#include "cmsis_os.h"

#include "bms_can.h"
#include "motor_can.h"
#include "charger_can.h"

/* One raw received CAN frame, carried through the queue. */
typedef struct
{
  uint32_t id;
  uint8_t  data[8];
  uint8_t  len;
} CanRxFrame_t;

static CAN_HandleTypeDef *s_hcan = 0;
static osMessageQueueId_t  s_rxQueue = NULL;
static osMutexId_t         s_statusMutex = NULL;

void CanBus_Init(CAN_HandleTypeDef *hcan)
{
  s_hcan = hcan;
  s_rxQueue = osMessageQueueNew(16, sizeof(CanRxFrame_t), NULL);
  s_statusMutex = osMutexNew(NULL);
}

void CanBus_Start(void)
{
  /* Accept-all filter into FIFO0 (decoders reject IDs they don't own). */
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  HAL_CAN_ConfigFilter(s_hcan, &filter);
  HAL_CAN_Start(s_hcan);
  HAL_CAN_ActivateNotification(s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CanBus_LockStatus(void)
{
  if (s_statusMutex != NULL)
  {
    osMutexAcquire(s_statusMutex, osWaitForever);
  }
}

void CanBus_UnlockStatus(void)
{
  if (s_statusMutex != NULL)
  {
    osMutexRelease(s_statusMutex);
  }
}

void CanBus_ProcessRx(uint32_t timeout)
{
  CanRxFrame_t f;

  if (osMessageQueueGet(s_rxQueue, &f, NULL, timeout) == osOK)
  {
    /* Decode under the mutex so ControlTask never reads a half-updated
       status struct. Each decoder ignores IDs it doesn't own. */
    CanBus_LockStatus();
    BmsCan_HandleRx(f.id, f.data, f.len);
    MotorCan_HandleRx(f.id, f.data, f.len);
    ChargerCan_HandleRx(f.id, f.data, f.len);
    CanBus_UnlockStatus();
  }
}

uint8_t CanBus_SendFrame(uint32_t canId, const uint8_t *data, uint8_t len)
{
  if (s_hcan == 0 || data == 0 || len > 8U)
  {
    return 0U;
  }

  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;

  txHeader.StdId = canId;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(s_hcan, &txHeader, (uint8_t *)data, &txMailbox) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

/*
 * RX FIFO0 interrupt callback: read the frame and post it to the queue.
 * No decoding / no mutex here -- ISRs must stay short. CanRxTask does
 * the heavy work. osMessageQueuePut with timeout 0 is ISR-safe.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  CanRxFrame_t f;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, f.data) != HAL_OK)
  {
    return;
  }

  f.id  = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
  f.len = (uint8_t)rxHeader.DLC;

  if (s_rxQueue != NULL)
  {
    (void)osMessageQueuePut(s_rxQueue, &f, 0U, 0U);
  }
}
