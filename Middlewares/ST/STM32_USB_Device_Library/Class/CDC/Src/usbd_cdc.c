/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team, modified for dual CDC ACM
  * @brief   USB CDC class driver exposing two virtual COM ports.
  ******************************************************************************
  */

#include "usbd_cdc.h"
#include "usbd_ctlreq.h"

static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);

USBD_ClassTypeDef USBD_CDC =
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL,
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_CDC_GetHSCfgDesc,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetOtherSpeedCfgDesc,
  USBD_CDC_GetDeviceQualifierDescriptor,
};

__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0xEF,
  0x02,
  0x01,
  0x40,
  0x01,
  0x00,
};

#define CDC_FUNCTION_DESC(cmd_ep, in_ep, out_ep, comm_if, data_if, cmd_interval, data_packet_size) \
  0x08, 0x0B, comm_if, 0x02, 0x02, 0x02, 0x01, 0x00, \
  0x09, USB_DESC_TYPE_INTERFACE, comm_if, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, \
  0x05, 0x24, 0x00, 0x10, 0x01, \
  0x05, 0x24, 0x01, 0x00, data_if, \
  0x04, 0x24, 0x02, 0x02, \
  0x05, 0x24, 0x06, comm_if, data_if, \
  0x07, USB_DESC_TYPE_ENDPOINT, cmd_ep, 0x03, LOBYTE(CDC_CMD_PACKET_SIZE), HIBYTE(CDC_CMD_PACKET_SIZE), cmd_interval, \
  0x09, USB_DESC_TYPE_INTERFACE, data_if, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00, \
  0x07, USB_DESC_TYPE_ENDPOINT, out_ep, 0x02, LOBYTE(data_packet_size), HIBYTE(data_packet_size), 0x00, \
  0x07, USB_DESC_TYPE_ENDPOINT, in_ep, 0x02, LOBYTE(data_packet_size), HIBYTE(data_packet_size), 0x00

#define CDC_DUAL_CONFIG_DESC(packet_size, interval) \
  0x09, USB_DESC_TYPE_CONFIGURATION, LOBYTE(USB_CDC_CONFIG_DESC_SIZ), HIBYTE(USB_CDC_CONFIG_DESC_SIZ), \
  0x04, 0x01, 0x00, 0xC0, 0x32, \
  CDC_FUNCTION_DESC(CDC0_CMD_EP, CDC0_IN_EP, CDC0_OUT_EP, 0x00, 0x01, interval, packet_size), \
  CDC_FUNCTION_DESC(CDC1_CMD_EP, CDC1_IN_EP, CDC1_OUT_EP, 0x02, 0x03, interval, packet_size)

__ALIGN_BEGIN uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  CDC_DUAL_CONFIG_DESC(CDC_DATA_HS_MAX_PACKET_SIZE, CDC_HS_BINTERVAL)
};

__ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  CDC_DUAL_CONFIG_DESC(CDC_DATA_FS_MAX_PACKET_SIZE, CDC_FS_BINTERVAL)
};

__ALIGN_BEGIN uint8_t USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  CDC_DUAL_CONFIG_DESC(CDC_DATA_FS_MAX_PACKET_SIZE, CDC_FS_BINTERVAL)
};

static uint8_t USBD_CDC_PortFromInterface(uint16_t interface, uint8_t *port)
{
  if (interface <= 1U)
  {
    *port = CDC_FOCUSER_PORT;
    return USBD_OK;
  }
  if (interface <= 3U)
  {
    *port = CDC_BULB_PORT;
    return USBD_OK;
  }
  return USBD_FAIL;
}

static uint8_t USBD_CDC_PortFromOutEp(uint8_t epnum, uint8_t *port)
{
  if (epnum == (CDC0_OUT_EP & 0x0FU))
  {
    *port = CDC_FOCUSER_PORT;
    return USBD_OK;
  }
  if (epnum == (CDC1_OUT_EP & 0x0FU))
  {
    *port = CDC_BULB_PORT;
    return USBD_OK;
  }
  return USBD_FAIL;
}

static uint8_t USBD_CDC_PortFromInEp(uint8_t epnum, uint8_t *port)
{
  if (epnum == (CDC0_IN_EP & 0x0FU))
  {
    *port = CDC_FOCUSER_PORT;
    return USBD_OK;
  }
  if (epnum == (CDC1_IN_EP & 0x0FU))
  {
    *port = CDC_BULB_PORT;
    return USBD_OK;
  }
  return USBD_FAIL;
}

static uint8_t USBD_CDC_GetInEp(uint8_t port)
{
  return (port == CDC_BULB_PORT) ? CDC1_IN_EP : CDC0_IN_EP;
}

static uint8_t USBD_CDC_GetOutEp(uint8_t port)
{
  return (port == CDC_BULB_PORT) ? CDC1_OUT_EP : CDC0_OUT_EP;
}

static uint8_t USBD_CDC_GetCmdEp(uint8_t port)
{
  return (port == CDC_BULB_PORT) ? CDC1_CMD_EP : CDC0_CMD_EP;
}

static uint16_t USBD_CDC_GetDataPacketSize(USBD_HandleTypeDef *pdev)
{
  return (pdev->dev_speed == USBD_SPEED_HIGH) ? CDC_DATA_HS_MAX_PACKET_SIZE : CDC_DATA_FS_MAX_PACKET_SIZE;
}

static uint8_t USBD_CDC_OpenPort(USBD_HandleTypeDef *pdev, uint8_t port)
{
  uint16_t packet_size = USBD_CDC_GetDataPacketSize(pdev);
  uint8_t in_ep = USBD_CDC_GetInEp(port);
  uint8_t out_ep = USBD_CDC_GetOutEp(port);
  uint8_t cmd_ep = USBD_CDC_GetCmdEp(port);

  USBD_LL_OpenEP(pdev, in_ep, USBD_EP_TYPE_BULK, packet_size);
  pdev->ep_in[in_ep & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, out_ep, USBD_EP_TYPE_BULK, packet_size);
  pdev->ep_out[out_ep & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, cmd_ep, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[cmd_ep & 0x0FU].is_used = 1U;

  return USBD_OK;
}

static void USBD_CDC_ClosePort(USBD_HandleTypeDef *pdev, uint8_t port)
{
  uint8_t in_ep = USBD_CDC_GetInEp(port);
  uint8_t out_ep = USBD_CDC_GetOutEp(port);
  uint8_t cmd_ep = USBD_CDC_GetCmdEp(port);

  USBD_LL_CloseEP(pdev, in_ep);
  pdev->ep_in[in_ep & 0x0FU].is_used = 0U;

  USBD_LL_CloseEP(pdev, out_ep);
  pdev->ep_out[out_ep & 0x0FU].is_used = 0U;

  USBD_LL_CloseEP(pdev, cmd_ep);
  pdev->ep_in[cmd_ep & 0x0FU].is_used = 0U;
}

static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t ret = USBD_OK;
  uint16_t packet_size = USBD_CDC_GetDataPacketSize(pdev);
  USBD_CDC_HandleTypeDef *hcdc;

  UNUSED(cfgidx);

  USBD_CDC_OpenPort(pdev, CDC_FOCUSER_PORT);
  USBD_CDC_OpenPort(pdev, CDC_BULB_PORT);

  pdev->pClassData = USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));
  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  for (uint8_t port = 0U; port < CDC_PORT_COUNT; port++)
  {
    hcdc->Port[port].TxState = 0U;
    hcdc->Port[port].RxState = 0U;
    hcdc->Port[port].CmdOpCode = 0xFFU;
    hcdc->Port[port].CmdLength = 0U;
    hcdc->Port[port].RxLength = 0U;
    hcdc->Port[port].TxLength = 0U;
    hcdc->Port[port].RxBuffer = NULL;
    hcdc->Port[port].TxBuffer = NULL;

    if (((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init(port) != USBD_OK)
    {
      ret = USBD_FAIL;
    }
  }

  if (ret == USBD_OK)
  {
    for (uint8_t port = 0U; port < CDC_PORT_COUNT; port++)
    {
      USBD_LL_PrepareReceive(pdev, USBD_CDC_GetOutEp(port), hcdc->Port[port].RxBuffer, packet_size);
    }
  }

  return ret;
}

static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_CDC_ClosePort(pdev, CDC_FOCUSER_PORT);
  USBD_CDC_ClosePort(pdev, CDC_BULB_PORT);

  if (pdev->pClassData != NULL)
  {
    for (uint8_t port = 0U; port < CDC_PORT_COUNT; port++)
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit(port);
    }
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  uint8_t port = CDC_FOCUSER_PORT;
  uint8_t ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      if ((hcdc == NULL) || (USBD_CDC_PortFromInterface(req->wIndex, &port) != USBD_OK))
      {
        USBD_CtlError(pdev, req);
        return USBD_FAIL;
      }

      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U)
        {
          ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(port, req->bRequest,
                                                            (uint8_t *)(void *)hcdc->Port[port].data,
                                                            req->wLength);
          USBD_CtlSendData(pdev, (uint8_t *)(void *)hcdc->Port[port].data, req->wLength);
        }
        else
        {
          hcdc->Port[port].CmdOpCode = req->bRequest;
          hcdc->Port[port].CmdLength = (uint8_t)req->wLength;
          USBD_CtlPrepareRx(pdev, (uint8_t *)(void *)hcdc->Port[port].data, req->wLength);
        }
      }
      else
      {
        ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(port, req->bRequest,
                                                          (uint8_t *)(void *)req, 0U);
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            USBD_CtlSendData(pdev, &ifalt, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state != USBD_STATE_CONFIGURED)
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  uint8_t port = CDC_FOCUSER_PORT;

  if ((hcdc == NULL) || (USBD_CDC_PortFromInEp(epnum, &port) != USBD_OK))
  {
    return USBD_FAIL;
  }

  if ((pdev->ep_in[epnum].total_length > 0U) &&
      ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
  {
    pdev->ep_in[epnum].total_length = 0U;
    USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    hcdc->Port[port].TxState = 0U;
  }

  return USBD_OK;
}

static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  uint8_t port = CDC_FOCUSER_PORT;

  if ((hcdc == NULL) || (USBD_CDC_PortFromOutEp(epnum, &port) != USBD_OK))
  {
    return USBD_FAIL;
  }

  hcdc->Port[port].RxLength = USBD_LL_GetRxDataSize(pdev, epnum);
  ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Receive(port, hcdc->Port[port].RxBuffer,
                                                    &hcdc->Port[port].RxLength);

  return USBD_OK;
}

static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((pdev->pUserData == NULL) || (hcdc == NULL))
  {
    return USBD_FAIL;
  }

  for (uint8_t port = 0U; port < CDC_PORT_COUNT; port++)
  {
    if (hcdc->Port[port].CmdOpCode != 0xFFU)
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(port, hcdc->Port[port].CmdOpCode,
                                                        (uint8_t *)(void *)hcdc->Port[port].data,
                                                        (uint16_t)hcdc->Port[port].CmdLength);
      hcdc->Port[port].CmdOpCode = 0xFFU;
      hcdc->Port[port].CmdLength = 0U;
      break;
    }
  }

  return USBD_OK;
}

static uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_CDC_CfgFSDesc);
  return USBD_CDC_CfgFSDesc;
}

static uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_CDC_CfgHSDesc);
  return USBD_CDC_CfgHSDesc;
}

static uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_CDC_OtherSpeedCfgDesc);
  return USBD_CDC_OtherSpeedCfgDesc;
}

uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = sizeof(USBD_CDC_DeviceQualifierDesc);
  return USBD_CDC_DeviceQualifierDesc;
}

uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return USBD_FAIL;
  }

  pdev->pUserData = fops;
  return USBD_OK;
}

uint8_t USBD_CDC_SetTxBufferEx(USBD_HandleTypeDef *pdev, uint8_t port, uint8_t *pbuff, uint16_t length)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((hcdc == NULL) || (port >= CDC_PORT_COUNT))
  {
    return USBD_FAIL;
  }

  hcdc->Port[port].TxBuffer = pbuff;
  hcdc->Port[port].TxLength = length;

  return USBD_OK;
}

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint16_t length)
{
  return USBD_CDC_SetTxBufferEx(pdev, CDC_FOCUSER_PORT, pbuff, length);
}

uint8_t USBD_CDC_SetRxBufferEx(USBD_HandleTypeDef *pdev, uint8_t port, uint8_t *pbuff)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((hcdc == NULL) || (port >= CDC_PORT_COUNT))
  {
    return USBD_FAIL;
  }

  hcdc->Port[port].RxBuffer = pbuff;

  return USBD_OK;
}

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  return USBD_CDC_SetRxBufferEx(pdev, CDC_FOCUSER_PORT, pbuff);
}

uint8_t USBD_CDC_TransmitPacketEx(USBD_HandleTypeDef *pdev, uint8_t port)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  uint8_t in_ep = USBD_CDC_GetInEp(port);

  if ((hcdc == NULL) || (port >= CDC_PORT_COUNT))
  {
    return USBD_FAIL;
  }

  if (hcdc->Port[port].TxState != 0U)
  {
    return USBD_BUSY;
  }

  hcdc->Port[port].TxState = 1U;
  pdev->ep_in[in_ep & 0x0FU].total_length = hcdc->Port[port].TxLength;
  USBD_LL_Transmit(pdev, in_ep, hcdc->Port[port].TxBuffer, (uint16_t)hcdc->Port[port].TxLength);

  return USBD_OK;
}

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  return USBD_CDC_TransmitPacketEx(pdev, CDC_FOCUSER_PORT);
}

uint8_t USBD_CDC_ReceivePacketEx(USBD_HandleTypeDef *pdev, uint8_t port)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

  if ((hcdc == NULL) || (port >= CDC_PORT_COUNT))
  {
    return USBD_FAIL;
  }

  USBD_LL_PrepareReceive(pdev, USBD_CDC_GetOutEp(port), hcdc->Port[port].RxBuffer,
                         USBD_CDC_GetDataPacketSize(pdev));

  return USBD_OK;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  return USBD_CDC_ReceivePacketEx(pdev, CDC_FOCUSER_PORT);
}
