// bluetooth, config, discover and audio
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#define LOGLVL	4

void logit(int level, const char *fmt, ...) {
  if (level <= LOGLVL) {
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    Serial.println(buffer);
  }
}

#define FATAL(format, ...)                      \
  do {                                          \
    logit(1, format, ##__VA_ARGS__);            \
    hang();                                     \
  } while (false)

#define LOGE(format, ...)  logit(1, format, ##__VA_ARGS__)
#define LOGW(format, ...)  logit(2, format, ##__VA_ARGS__)
#define LOGI(format, ...)  logit(3, format, ##__VA_ARGS__)
#define LOGD(format, ...)  logit(4, format, ##__VA_ARGS__)
#define LOGV(format, ...)  logit(5, format, ##__VA_ARGS__)

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

char const * a2d_state_str(int state) {
  switch (state) {
  case APP_AV_STATE_IDLE: return "APP_AV_STATE_IDLE";
  case APP_AV_STATE_DISCOVERING: return "APP_AV_STATE_DISCOVERING";
  case APP_AV_STATE_DISCOVERED: return "APP_AV_STATE_DISCOVERED";
  case APP_AV_STATE_UNCONNECTED: return "APP_AV_STATE_UNCONNECTED";
  case APP_AV_STATE_CONNECTING: return "APP_AV_STATE_CONNECTING";
  case APP_AV_STATE_CONNECTED: return "APP_AV_STATE_CONNECTED";
  case APP_AV_STATE_DISCONNECTING: return "APP_AV_STATE_DISCONNECTING";
  default: return "UNKNOWN STATE";
  }
}

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

    LOGV("scanned device: %s",
         bda2str(param->disc_res.bda, bda_str, sizeof(bda_str)));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            LOGV("--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            LOGV("--RSSI: %d", rssi);
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
        LOGV("discovered possible device, address %s, name %s",
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

    #define TARGET "TH-D74"
    /* search for a particular device name in its extended inqury response */
    if (eir) {
        get_name_from_eir(eir, s_peer_bdname, NULL);
        if (strcmp((char *)s_peer_bdname, TARGET) != 0) {
            return;
        }

        LOGI("Found a target device, address %s, name %s",
                      bda_str, s_peer_bdname);
        s_a2d_state = APP_AV_STATE_DISCOVERED;
        memcpy(s_peer_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
        LOGI("Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  LOGV("gap_cb starting");
  switch (event) {
  case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: /*!< discovery state changed event */
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      if (s_a2d_state == APP_AV_STATE_DISCOVERED) {
        LOGI("device discovered");
        LOGI("a2dp connecting to peer: %s", s_peer_bdname);
        if (esp_a2d_source_connect(s_peer_bda))
          FATAL("esp_a2d_source_connect failed");
        s_a2d_state = APP_AV_STATE_CONNECTING;
        s_connecting_intv = 0;
      } else {
        // not discovered, continue to discover
        LOGI("device discovery failed, continue to discover...");
        if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0))
          FATAL("esp_bt_gap_start_discovery failed");
      }
    } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        LOGI("discovery started");
    }
    break;
    
  case ESP_BT_GAP_DISC_RES_EVT: /*!< device discovery result event */
    filter_inquiry_scan_result(param);
    break;
    
  case ESP_BT_GAP_AUTH_CMPL_EVT: /*!< AUTH complete event */
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      LOGI("authentication success: %s", param->auth_cmpl.device_name);
      memcpy(s_peer_bda, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      char buf[18];
      LOGI("source connect to %s", bda2str(s_peer_bda, buf, sizeof(buf)));
      if (esp_a2d_source_connect(s_peer_bda))
        FATAL("esp_a2d_source_connect failed");
      s_a2d_state = APP_AV_STATE_CONNECTING;
      s_connecting_intv = 0;
    } else {
      LOGI("authentication failed: %d", param->auth_cmpl.stat);
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
    LOGE("unknown gap event=%d", int(event));
    hang();
  }
  LOGV("gap_cb finished");
}
  
static void avrc_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
  LOGD("avrc_cb starting");
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
      // case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
      // case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
    default:
      LOGE("unknown avrc event=%d", int(event));
      hang();
    }
  LOGD("avrc_cb finished");
}

static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  LOGV("a2d_cb starting");
  dispatch(event, param);
  LOGV("a2d_cb finished");
}

static int32_t a2d_data_cb(uint8_t *buf, int32_t len) {
  // Called to provide PCM data:
  // [in] buf: : buffer to be filled with PCM data stream from higher layer.
  // [in] len: : size(in bytes) of data block to be copied to buf. -1
  //             is an indication to user that data buffer shall be
  //             flushed
  LOGI("a2d_data_cb len=%d", len);
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
  LOGD("handle_unconnected starting");
  switch (event) {
    case BT_APP_HEART_BEAT_EVT: {
      char bda_str[18];
      LOGI("a2dp connecting to peer: %s",
           bda2str(s_peer_bda, bda_str, sizeof(bda_str)));
      if (esp_a2d_source_connect(s_peer_bda))
        FATAL("esp_a2d_source_connect failed");
      s_a2d_state = APP_AV_STATE_CONNECTING;
      s_connecting_intv = 0;
      break;
    }
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        break;
    default:
      LOGE("unknown handle_unconnected event=%d", int(event));
      hang();
  }
  LOGD("handle_unconnected finished");
}

static void handle_connecting(uint16_t event, void *param) {
  LOGV("handle_connecting starting");
  esp_a2d_cb_param_t *a2d = NULL;
  switch (event) {
  case ESP_A2D_CONNECTION_STATE_EVT:
    a2d = (esp_a2d_cb_param_t *)(param);
    switch (a2d->conn_stat.state) {
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED: /*!< connection released  */
      LOGI("handle_connecting saw ESP_A2D_CONNECTION_STATE_DISCONNECTED");
      LOGI("reason = %s", a2d->conn_stat.disc_rsn ? "ABNORMAL" : "NORMAL");
      LOGI("a2dp disconnected");
      s_a2d_state =  APP_AV_STATE_UNCONNECTED;
      break;
    case ESP_A2D_CONNECTION_STATE_CONNECTING: /*!< connecting remote device */
      LOGI("handle_connecting saw ESP_A2D_CONNECTION_STATE_CONNECTING");
      break;
    case ESP_A2D_CONNECTION_STATE_CONNECTED: /*!< connection established */
      LOGI("handle_connecting saw ESP_A2D_CONNECTION_STATE_CONNECTED");
      s_a2d_state =  APP_AV_STATE_CONNECTED;
      s_media_state = APP_AV_MEDIA_STATE_IDLE;
      if (esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_NONE))
        FATAL("esp_bt_gap_set_scan_mode failed");
      break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTING: /*!< disconing remote device */
      LOGI("handle_connecting saw ESP_A2D_CONNECTION_STATE_DISCONNECTING");
      break;
    default:
      LOGI("unknown esp_a2d_connection_state_t %d", a2d->conn_stat.state);
      break;
    }
    break;
  case ESP_A2D_AUDIO_STATE_EVT:
    LOGI("handle_connecting saw ESP_A2D_AUDIO_STATE_EVT");
    break;
  case ESP_A2D_AUDIO_CFG_EVT:
    LOGI("handle_connecting saw ESP_A2D_AUDIO_CFG_EVT");
    break;
  case ESP_A2D_MEDIA_CTRL_ACK_EVT:
    LOGI("handle_connecting saw ESP_A2D_MEDIA_CTRL_ACK_EVT");
    break;
  case BT_APP_HEART_BEAT_EVT:
    LOGI("handle_connecting saw BT_APP_HEART_BEAT_EVT");
    if (++s_connecting_intv >= 10) {
      LOGI("connecting too long, unconnected");
      s_a2d_state = APP_AV_STATE_UNCONNECTED;
      s_connecting_intv = 0;
    }
    break;
  default:
    LOGE("unknown handle_connecting event=%d", int(event));
    hang();
  }
  LOGV("handle_connecting finished");
}

static void dispatch(uint16_t event, void *param) {
  LOGD("dispatch starting in state %s", a2d_state_str(s_a2d_state));
  switch (s_a2d_state) {
  case APP_AV_STATE_IDLE:
  case APP_AV_STATE_DISCOVERING:
  case APP_AV_STATE_DISCOVERED:
    break;
  case APP_AV_STATE_UNCONNECTED:
    handle_unconnected(event, param);
    break;
  case APP_AV_STATE_CONNECTING:
    handle_connecting(event, param);
    break;
  case APP_AV_STATE_CONNECTED:
  case APP_AV_STATE_DISCONNECTING:
  default:
    LOGE("dispatch unhandled state %d", s_a2d_state);
    hang();
  }
  LOGD("dispatch finished in state %s", a2d_state_str(s_a2d_state));
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  LOGI("BlueKeyer setup starting");

  if (!btStart())
    FATAL("btStart failed");
    
  if (esp_bluedroid_init())
    FATAL("esp_bluedroid_init failed");

  if (esp_bluedroid_enable())
    FATAL("esp_bluedroid_enable failed");

  const char *dev_name = "BlueKeyer";
  if (esp_bt_dev_set_device_name(dev_name))
    FATAL("esp_bt_dev_set_device_name %s failed", dev_name);

  // GAP
  if (esp_bt_gap_register_callback(gap_cb))
    FATAL("esp_bt_gap_register_callback failed");

  // AVRCP
  if (esp_avrc_ct_init())
    FATAL("esp_avrc_ct_init failed");
  if (esp_avrc_ct_register_callback(avrc_cb))
    FATAL("esp_avrc_ct_register_callback failed");

  // A2DP
  if (esp_a2d_register_callback(a2d_cb))
    FATAL("esp_a2d_register_callback failed");
  if (esp_a2d_source_register_data_callback(a2d_data_cb))
    FATAL("esp_a2d_source_register_data_callback failed");
  if (esp_a2d_source_init())
    FATAL("esp_a2d_source_init failed");

  // service     major minor
  // 00000000000 00000 000000 00
  // 00100000000 00100 000100 00
  esp_bt_cod_t cod;
  cod.service = ESP_BT_COD_SRVC_AUDIO;	// 0x100	0b100000000
  cod.major = ESP_BT_COD_MAJOR_DEV_AV;	// 4            0b100
  cod.minor = 0b000100;			// Microphone
  if (esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD))
    FATAL("esp_bt_gap_set_cod failed");
  
  // set discoverable and connectable mode, wait to be connected
  if (esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE))
    FATAL("esp_bt_gap_set_scan_mode failed");

  s_a2d_state = APP_AV_STATE_IDLE;
  // s_a2d_state = APP_AV_STATE_DISCOVERING;
  // esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

  LOGI("BlueKeyer setup finished");
}

void loop() {
  delay(10000);
  dispatch(BT_APP_HEART_BEAT_EVT, NULL);
}

void hang() {
  // Park the CPU in an infinite loop.
  LOGE("BLUEKEYER HANGING");
  while (true)
    delay(1000);
}

// B0:2A:43:FC:58:A3
