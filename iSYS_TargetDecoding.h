
typedef unsigned char uint8_t;
//typedef unsigned int uint16_t;
typedef int sint16_t;
typedef unsigned long uint32_t;
typedef long sint32_t;
typedef float float32_t;

// {SD, LE, LEr, SDr, DA(dest addr), SA (source addr), FC (funcioncode), FSC (cheqsum from bit 4 on),ED (endDelimiter 0x16) }
// MESSAGES FOR SENSOR ON DEFAULT ID 128 (0x80)
uint8_t iSYS_startAcquisiton[11] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD1, 0x00, 0x00, 0x52, 0x16};
uint8_t iSYS_stopAcquisiton[11] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD1, 0x00, 0x01, 0x53, 0x16};
uint8_t iSYS_requestMeasurement[11] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xDA, 0x01, 0x20, 0x7C, 0x16};

uint8_t iSYS_getChannel[11] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD2, 0x00, 0x04, 0x57, 0x16};
uint8_t iSYS_setChannelX[13] = {0x68, 0x07, 0x07, 0x68, 0x80, 0x01, 0xD3, 0x00, 0x04, 0x00, 0x00, 0x58, 0x16}; // ADD CHANNEL NR TO BYTES 10 & 11


uint8_t iSYS_getPoti[11] = {0x68, 0x05, 0x05, 0x68, 0x80, 0x01, 0xD4, 0x06, 0x80, 0xDB, 0x16};
uint8_t iSYS_setPotiX[13] = {0x68, 0x07, 0x07, 0x68, 0x80, 0x01, 0xD5, 0x06, 0x80, 0x00, 0x00, 0xDC, 0x16};

uint8_t iSYS_saveAllToEEPROM[10] = {0x68, 0x04, 0x04, 0x68, 0x80, 0x01, 0xDF, 0x04, 0x64, 0x16};

//#include <string.h> /* memset */
#define MAX_TARGETS (0x23)



typedef enum iSYSResult {
  ERR_OK = 0x0000,
  ERR_FUNCTION_DEPRECATED ,
  ERR_DLL_NOT_FINISHED ,
  ERR_HANDLE_NOT_INITIALIZED ,
  ERR_COMPORT_DOESNT_EXIST ,
  ERR_COMPORT_CANT_INITIALIZE ,
  ERR_COMPORT_ACCESS_DENIED ,
  ERR_COMPORT_BAUDRATE_NOT_VALID ,
  ERR_COMPORT_CANT_OPEN ,
  ERR_COMPORT_CANT_SET_FLOW_CONTROL ,
  ERR_COMPORT_CANT_SET_PARITY ,
  ERR_COMPORT_CANT_SET_STOP_BITS ,
  ERR_COMPORT_CANT_SET_DATA_BITS ,
  ERR_COMPORT_CANT_SET_BAUDRATE ,
  ERR_COMPORT_ALREADY_INITIALIZED ,
  ERR_COMPORT_EQUALS_NULL ,
  ERR_COMPORT_NOT_OPEN ,
  ERR_COMPORT_NOT_READABLE ,
  ERR_COMPORT_NOT_WRITEABLE ,
  ERR_COMPORT_CANT_WRITE ,
  ERR_COMPORT_CANT_READ ,
  ERR_COMMAND_NOT_WRITTEN ,
  ERR_COMMAND_NOT_READ ,
  ERR_COMMAND_NO_DATA_RECEIVED ,
  ERR_COMMAND_NO_VALID_FRAME_FOUND ,
  ERR_COMMAND_RX_FRAME_DAMAGED ,
  ERR_COMMAND_FAILURE ,
  ERR_UNDEFINED_READ ,
  ERR_COMPORT_LESS_DATA_READ ,
  ERR_COMPORT_SYSTEM_INIT_FAILED ,
  ERR_COMPORT_SYSTEM_ALREADY_INITIALIZED ,
  ERR_COMMAND_RX_FRAME_LENGTH ,
  ERR_COMMAND_MAX_DATA_OVERFLOW ,
  ERR_COMMAND_MAX_IQPAIRS_OVERFLOW ,
  ERR_COMMAND_NOT_ACCEPTED ,
  ERR_NULL_POINTER,
  ACK_STARTSTOP_ACQUISITION,
  ACK_SENSOR_WRITE,
  ACK_SENSOR_WRITE_EEPROM,
  READ_CHANNEL,
  READ_POTI
} iSYSResult_t;

typedef struct iSYSResultValue_s {
  iSYSResult_t iSYSResult;
  uint8_t value;
} iSYSResultValue_t;

typedef enum iSYSTargetListError
{
  TARGET_LIST_OK = 0x00,
  TARGET_LIST_FULL = 0x01,
  TARGET_LIST_REFRESHED = 0x02,
  TARGET_LIST_ALREADY_REQUESTED = 0x03,
  TARGET_LIST_ACQUISITION_NOT_STARTED = 0x04
} iSYSTargetListError_t;

typedef struct iSYSTarget {
  float32_t velocity; /* radial velocity in m/s */
  float32_t range; /* range in m */
  float32_t signal; /* signal indicator */
  float32_t angle; /* angle of detected object [°] */
} iSYSTarget_t;

union iSYSTargetListError_u
{
  iSYSTargetListError_t iSYSTargetListError;
  uint32_t dummy;
};

typedef struct iSYSTargetList {
  union iSYSTargetListError_u error;
  uint8_t outputNumber;
  uint16_t nrOfTargets;
  uint32_t clippingFlag;
  iSYSTarget_t targets[MAX_TARGETS];
} iSYSTargetList_t;


/***********************************************************************
  Function: decodes target list frame received from iSYS device.
  Input arguments:
  - Frame array: array with from iSYS received target list frame
  - nrOfElements: number of bytes in the frame array
  - productcode: product code of the connected iSYS (e.g. 6003, 4001, …)
  - bitrate: resolution of the target list in the frame array (16-Bit or 32-Bit)
  - targetList: struct for decoded target list

  Output arguments:
  - targetList: struct with decoded target list

  Return value:
  - ErrorCode

***********************************************************************/

iSYSResult_t decodeTargetFrame(unsigned char *frame_array, uint16_t nrOfElements, uint16_t productcode, uint8_t bitrate, iSYSTargetList_t *targetList) {
  uint16_t ui16_fc;
  uint8_t output_number;
  uint8_t nrOfTargets;
  uint8_t *pData;
  sint16_t tmp;
  uint8_t i;

  if (frame_array[0] == 0x68) /* check SD2 Frame */
  {
    ui16_fc = 6; /* set function code bit for variable length frames */
  }
  else
  {
    ui16_fc = 3; /* set function code bit for fixed length frames */
  }
  output_number = (uint16_t)(frame_array[ui16_fc + 1] & 0x00ff);
  nrOfTargets = (uint16_t)(frame_array[ui16_fc + 2] & 0x00ff);
  pData = &frame_array[ui16_fc + 3];
  if (frame_array[nrOfElements - 1] != 0x16) { /* check end of frame */
    return ERR_COMMAND_NO_VALID_FRAME_FOUND;
  }

  /* check for valid amount of targets */
  if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xff)) {
    return ERR_COMMAND_FAILURE;
  }
  if (nrOfTargets != 0xff) { //0xff  clipping
    for (i = 0; i < MAX_TARGETS ; i++) { //Init Array
      targetList->targets[i].angle = 0;
      targetList->targets[i].range = 0;
      targetList->targets[i].signal = 0;
      targetList->targets[i].velocity = 0;
    }
    targetList->nrOfTargets = nrOfTargets;
    targetList->clippingFlag = 0;
    targetList->outputNumber = output_number;

    if (bitrate == 32) {
      int tmp32;
      for (i = 0; i < nrOfTargets; i++) {
        tmp = (((*pData++) & 0x00ff) << 8);
        tmp |= ((*pData++) & 0x00ff);
        targetList->targets[i].signal = (float)(tmp * 0.01f);
        tmp32 = (((*pData++) & 0x000000ff) << 24);
        tmp32 |= (((*pData++) & 0x000000ff) << 16);
        tmp32 |= (((*pData++) & 0x000000ff) << 8);
        tmp32 |= ((*pData++) & 0x000000ff);
        targetList->targets[i].velocity = (float)tmp32 * 0.001f;
        tmp32 = (((*pData++) & 0x000000ff) << 24);
        tmp32 |= (((*pData++) & 0x000000ff) << 16);
        tmp32 |= (((*pData++) & 0x000000ff) << 8);
        tmp32 |= ((*pData++) & 0x000000ff);
        targetList->targets[i].range = (float)tmp32 * 1E-6f;
        tmp32 = (((*pData++) & 0x000000ff) << 24);
        tmp32 |= (((*pData++) & 0x000000ff) << 16);
        tmp32 |= (((*pData++) & 0x000000ff) << 8);
        tmp32 |= ((*pData++) & 0x000000ff);
        targetList->targets[i].angle = (float)tmp32 * 0.01f;
      }
    }

    if (bitrate == 16) {
      for (i = 0; i < nrOfTargets; i++) {
        targetList->targets[i].signal = (float)((*pData++) & 0x00ff);
        tmp = (((*pData++) & 0x00ff) << 8);
        tmp |= ((*pData++) & 0x00ff);
        targetList->targets[i].velocity = (float)tmp * 0.01f;
        tmp = (((*pData++) & 0x00ff) << 8);
        tmp |= ((*pData++) & 0x00ff);
        if (productcode == 4004 || productcode == 6003) {
          targetList->targets[i].range = (float)tmp * 0.001f;
        }
        else {
          targetList->targets[i].range = (float)tmp * 0.01f;
        }
        tmp = (((*pData++) & 0x00ff) << 8);
        tmp |= ((*pData++) & 0x00ff);
        targetList->targets[i].angle = (float)tmp * 0.01f;
      }
    }
  }
  else {
    targetList->clippingFlag = 1;
  }
  if (nrOfTargets == MAX_TARGETS) {
    targetList->error.iSYSTargetListError = TARGET_LIST_FULL;
  }
  else {
    targetList->error.iSYSTargetListError = TARGET_LIST_OK;
  }
  return ERR_OK;
};

iSYSResultValue_t decodeFrame(unsigned char *frame_array, uint16_t nrOfElements, uint16_t productcode, uint8_t bitrate, iSYSTargetList_t *targetList) {
  iSYSResultValue_t ret;
  ret.iSYSResult = ERR_OK;
  ret.value = 0;


  uint16_t ui16_fc;
  uint8_t function_code;

  uint8_t *pData;
  sint16_t tmp;
  uint8_t i;

  if (frame_array[0] == 0x68) /* check SD2 Frame */
  {
    ui16_fc = 6; /* set function code bit for variable length frames */
  }
  else
  {
    ui16_fc = 3; /* set function code bit for fixed length frames */
  }
  function_code = (uint16_t)(frame_array[ui16_fc] & 0x00ff);

  pData = &frame_array[ui16_fc + 3];
  if (frame_array[nrOfElements - 1] != 0x16) { /* check end of frame */
    ret.iSYSResult = ERR_COMMAND_NO_VALID_FRAME_FOUND;
    return ret;
  }

  if (function_code == 0xD1) { // fc - Start/stop acquisition
    ret.iSYSResult = ACK_STARTSTOP_ACQUISITION;
    return ret;
  }

  if (function_code == 0xD2) { // fc - read sensor settings
    uint8_t channel = 0;
    channel = (frame_array[ui16_fc + 2]);
    Serial.print("channel ");
    Serial.println(channel);

    ret.iSYSResult = READ_CHANNEL;
    ret.value = channel;
    return ret;
  }

  if (function_code == 0xD3) { // fc - write sensor settings
    ret.iSYSResult = ACK_SENSOR_WRITE;
    return ret;
  }

  if (function_code == 0xD4) { // fc - read application settings

    uint8_t amplification = 0;
    amplification = (frame_array[ui16_fc + 2]); // read nf-amplification poti (0-255)

    Serial.print("amplification ");
    Serial.println(amplification);

    ret.iSYSResult = READ_POTI;
    ret.value = amplification;
    return ret;

  }

  if (function_code == 0xD5) { // fc - write application settings
    ret.iSYSResult = ACK_SENSOR_WRITE;
    return ret;
  }


  //ACK_SENSOR_WRITE_EEPROM

  if (function_code == 0xDA) {

    uint8_t output_number;
    uint8_t nrOfTargets;

    output_number = (uint16_t)(frame_array[ui16_fc + 1] & 0x00ff);
    nrOfTargets = (uint16_t)(frame_array[ui16_fc + 2] & 0x00ff);

    /* check for valid amount of targets */
    if ((nrOfTargets > MAX_TARGETS) && (nrOfTargets != 0xff)) {
      ret.iSYSResult = ERR_COMMAND_FAILURE;
      return ret;
    }
    if (nrOfTargets != 0xff) { //0xff  clipping
      for (i = 0; i < MAX_TARGETS ; i++) { //Init Array
        targetList->targets[i].angle = 0;
        targetList->targets[i].range = 0;
        targetList->targets[i].signal = 0;
        targetList->targets[i].velocity = 0;
      }
      targetList->nrOfTargets = nrOfTargets;
      targetList->clippingFlag = 0;
      targetList->outputNumber = output_number;

      if (bitrate == 32) {
        int tmp32;
        for (i = 0; i < nrOfTargets; i++) {
          tmp = (((*pData++) & 0x00ff) << 8);
          tmp |= ((*pData++) & 0x00ff);
          targetList->targets[i].signal = (float)(tmp * 0.01f);
          tmp32 = (((*pData++) & 0x000000ff) << 24);
          tmp32 |= (((*pData++) & 0x000000ff) << 16);
          tmp32 |= (((*pData++) & 0x000000ff) << 8);
          tmp32 |= ((*pData++) & 0x000000ff);
          targetList->targets[i].velocity = (float)tmp32 * 0.001f;
          tmp32 = (((*pData++) & 0x000000ff) << 24);
          tmp32 |= (((*pData++) & 0x000000ff) << 16);
          tmp32 |= (((*pData++) & 0x000000ff) << 8);
          tmp32 |= ((*pData++) & 0x000000ff);
          targetList->targets[i].range = (float)tmp32 * 1E-6f;
          tmp32 = (((*pData++) & 0x000000ff) << 24);
          tmp32 |= (((*pData++) & 0x000000ff) << 16);
          tmp32 |= (((*pData++) & 0x000000ff) << 8);
          tmp32 |= ((*pData++) & 0x000000ff);
          targetList->targets[i].angle = (float)tmp32 * 0.01f;
        }
      }

      if (bitrate == 16) {
        for (i = 0; i < nrOfTargets; i++) {
          targetList->targets[i].signal = (float)((*pData++) & 0x00ff);
          tmp = (((*pData++) & 0x00ff) << 8);
          tmp |= ((*pData++) & 0x00ff);
          targetList->targets[i].velocity = (float)tmp * 0.01f;
          tmp = (((*pData++) & 0x00ff) << 8);
          tmp |= ((*pData++) & 0x00ff);
          if (productcode == 4004 || productcode == 6003) {
            targetList->targets[i].range = (float)tmp * 0.001f;
          }
          else {
            targetList->targets[i].range = (float)tmp * 0.01f;
          }
          tmp = (((*pData++) & 0x00ff) << 8);
          tmp |= ((*pData++) & 0x00ff);
          targetList->targets[i].angle = (float)tmp * 0.01f;
        }
      }
    }
    else {
      targetList->clippingFlag = 1;
    }
    if (nrOfTargets == MAX_TARGETS) {
      targetList->error.iSYSTargetListError = TARGET_LIST_FULL;
    }
    else {
      targetList->error.iSYSTargetListError = TARGET_LIST_OK;
    }
  }
  ret.iSYSResult = ERR_OK;
  return ret;
};

