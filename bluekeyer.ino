// bluetooth, config, discover and audio
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

// double SAMPLE_RATE = 44100;

/* A2DP global state */
enum {
    APP_AV_STATE_IDLE,
    APP_AV_STATE_DISCOVERING,
    APP_AV_STATE_DISCOVERED,
    APP_AV_STATE_UNCONNECTED,
    APP_AV_STATE_CONNECTING,
    APP_AV_STATE_CONNECTED,
    APP_AV_STATE_DISCONNECTING,
};

/* sub states of APP_AV_STATE_CONNECTED */
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

#define BT_APP_HEART_BEAT_EVT                (0xff00)

static esp_bd_addr_t s_peer_bda = {0};
static uint8_t s_peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static int s_a2d_state = APP_AV_STATE_IDLE;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;
static int s_connecting_intv = 0;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static bool get_name_from_eir(uint8_t *eir,
                              uint8_t *bdname,
                              uint8_t *bdname_len)
{
  uint8_t *rmt_bdname = NULL;
  uint8_t rmt_bdname_len = 0;

  if (!eir) {
    return false;
  }

  rmt_bdname = esp_bt_gap_resolve_eir_data(
                   eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
  if (!rmt_bdname) {
    rmt_bdname = esp_bt_gap_resolve_eir_data(
                   eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
  }

  if (rmt_bdname) {
    if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
      rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
    }

    if (bdname) {
      memcpy(bdname, rmt_bdname, rmt_bdname_len);
      bdname[rmt_bdname_len] = '\0';
    }
    if (bdname_len) {
      *bdname_len = rmt_bdname_len;
    }
    return true;
  }

  return false;
}

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;

    // Serial.printf("scanned device: %s\n",
    //               bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            // Serial.printf("--Class of Device: 0x%x\n", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            // Serial.printf("--RSSI: %d\n", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    if (eir) {
        get_name_from_eir(eir, s_peer_bdname, NULL);
        Serial.printf("discovered possible device, address %s, name %s\n",
                      bda2str(param->disc_res.bda, bda_str, 18),
                      s_peer_bdname);
    }

#if 0
    /* search for device with MAJOR service class as "rendering" in COD */
    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        return;
    }
#endif

    /* search for a particular device name in its extended inqury response */
    if (eir) {
        get_name_from_eir(eir, s_peer_bdname, NULL);
        if (strcmp((char *)s_peer_bdname, "Pixel 3") != 0) {
            return;
        }

        Serial.printf("Found a target device, address %s, name %s\n",
                      bda_str, s_peer_bdname);
        s_a2d_state = APP_AV_STATE_DISCOVERED;
        memcpy(s_peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        Serial.printf("Cancel device discovery ...\n");
        esp_bt_gap_cancel_discovery();
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  switch (event) {
  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: /*!< discovery state changed event */
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      if (s_a2d_state == APP_AV_STATE_DISCOVERED) {
        s_a2d_state = APP_AV_STATE_CONNECTING;
        Serial.printf("device discovered\n");
        Serial.printf("a2dp connecting to peer: %s\n", s_peer_bdname);
        esp_a2d_source_connect(s_peer_bda);
      } else {
        // not discovered, continue to discover
        Serial.printf("device discovery failed, continue to discover...\n");
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
      }
    } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        Serial.printf("discovery started\n");
    }
    break;
    
  case ESP_BT_GAP_DISC_RES_EVT: /*!< device discovery result event */
    filter_inquiry_scan_result(param);
    break;
    
  case ESP_BT_GAP_AUTH_CMPL_EVT: /*!< AUTH complete event */
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      Serial.printf("authentication success: %s\n",
                    param->auth_cmpl.device_name);
    } else {
      Serial.printf("authentication failed: %d\n",
                    param->auth_cmpl.stat);
    }
    break;

  case ESP_BT_GAP_RMT_SRVCS_EVT: /*!< get remote services event */
  case ESP_BT_GAP_RMT_SRVC_REC_EVT: /*!< get remote service record event */
  case ESP_BT_GAP_PIN_REQ_EVT: /*!< Legacy Pairing Pin code request */
  case ESP_BT_GAP_CFM_REQ_EVT: /*!< Simple Pairing User Confirmation request. */
  case ESP_BT_GAP_KEY_NOTIF_EVT: /*!< Simple Pairing Passkey Notification */
  case ESP_BT_GAP_KEY_REQ_EVT: /*!< Simple Pairing Passkey request */
  case ESP_BT_GAP_READ_RSSI_DELTA_EVT: /*!< read rssi event */
  default:
    Serial.printf("unknown gap event=%d\n", int(event));
    hang();
  }
}
  
static void avrc_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
      // case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
      // case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
    default:
      Serial.printf("unknown avrc event=%d\n", int(event));
      hang();
    }
}

static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  switch (event) {
  case ESP_A2D_CONNECTION_STATE_EVT: // connection state changed event
    break;
  case ESP_A2D_AUDIO_STATE_EVT: // audio stream transmission state changed event
  case ESP_A2D_AUDIO_CFG_EVT: // audio codec is configured, only used for A2DP SINK
  case ESP_A2D_MEDIA_CTRL_ACK_EVT: // acknowledge event in response to media control commands
  default:
    Serial.printf("unknown a2d_cb event=%d\n", int(event));
    hang();
  }
}

static int32_t a2d_data_cb(uint8_t *buf, int32_t len) {
  // Called to provide PCM data:
  // [in] buf: : buffer to be filled with PCM data stream from higher layer.
  // [in] len: : size(in bytes) of data block to be copied to buf. -1
  //             is an indication to user that data buffer shall be
  //             flushed
  Serial.printf("len=%d\n", len);
  if (len < 0 || buf == NULL) {
    return 0;
  }

  // Fill buffer w/ random data.
  int val = rand() % (1 << 16);
  for (int i = 0; i < (len >> 1); i++) {
    buf[(i << 1)] = val & 0xff;
    buf[(i << 1) + 1] = (val >> 8) & 0xff;
  }

  return len;
}

static void handle_unconnected(uint16_t event, void *param) {
  switch (event) {
    case BT_APP_HEART_BEAT_EVT: {
      uint8_t *p = s_peer_bda;
      Serial.printf("a2dp connecting to peer: %02x:%02x:%02x:%02x:%02x:%02x",
                    p[0], p[1], p[2], p[3], p[4], p[5]);
      esp_a2d_source_connect(s_peer_bda);
      s_a2d_state = APP_AV_STATE_CONNECTING;
      s_connecting_intv = 0;
      break;
    }
    default:
      Serial.printf("unknown handle_unconnected event=%d\n", int(event));
      hang();
  }
}

static void dispatch(uint16_t event, void *param) {
  switch (s_a2d_state) {
  case APP_AV_STATE_IDLE:
  case APP_AV_STATE_DISCOVERING:
  case APP_AV_STATE_DISCOVERED:
    break;
  case APP_AV_STATE_UNCONNECTED:
    handle_unconnected(event, param);
    break;
  case APP_AV_STATE_CONNECTING:
  case APP_AV_STATE_CONNECTED:
  case APP_AV_STATE_DISCONNECTING:
  default:
    Serial.printf("dispatch unhandled state %d\n", s_a2d_state);
    // hang();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("BlueKeyer setup starting\n");
  
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();

  const char *dev_name = "BlueKeyer";
  esp_bt_dev_set_device_name(dev_name);

  // GAP
  esp_bt_gap_register_callback(gap_cb);

  // AVRCP
  esp_avrc_ct_init();
  esp_avrc_ct_register_callback(avrc_cb);

  // A2DP
  esp_a2d_register_callback(a2d_cb);
  esp_a2d_source_register_data_callback(a2d_data_cb);
  esp_a2d_source_init();
  
  // set discoverable and connectable mode, wait to be connected
  esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE); 

  s_a2d_state = APP_AV_STATE_DISCOVERING;
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

  Serial.printf("BlueKeyer setup finished\n");
}

void loop() {
  delay(1000);
  dispatch(BT_APP_HEART_BEAT_EVT, NULL);
}

void hang() {
  // Park the CPU in an infinite loop.
  Serial.printf("BlueKeyer HANGING\n");
  while (true)
    delay(1000);
}
