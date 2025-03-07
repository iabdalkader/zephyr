/*
 * Copyright (c) 2025 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_hosted_wifi.h>
#include <esp_hosted_hal.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include <esp_hosted_proto.pb.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp_hosted, CONFIG_WIFI_LOG_LEVEL);

static esp_hosted_data_t esp_hosted_data = {0};

static esp_hosted_config_t esp_hosted_config = {
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
	.dataready_gpio = GPIO_DT_SPEC_INST_GET(0, dataready_gpios),
	.handshake_gpio = GPIO_DT_SPEC_INST_GET(0, handshake_gpios),
	.spi_bus = SPI_DT_SPEC_INST_GET(0, ESP_HOSTED_SPI_CONFIG | SPI_MODE_CPOL, 10U)
};

K_THREAD_STACK_DEFINE(esp_hosted_event_stack, CONFIG_WIFI_ESP_HOSTED_EVENT_TASK_STACK_SIZE);
static struct k_thread esp_hosted_event_thread;

K_MSGQ_DEFINE(esp_hosted_msgq, sizeof(CtrlMsg), 16, 4);

static uint16_t esp_hosted_checksum(esp_header_t *esp_header) {
    uint16_t checksum = 0;
    esp_header->checksum = 0;
    uint8_t *buf = (uint8_t *)esp_header;
    for (size_t i = 0; i < (esp_header->len + sizeof(esp_header_t)); i++) {
        checksum += buf[i];
    }
    return checksum;
}

#if CONFIG_WIFI_ESP_HOSTED_DEBUG
static void esp_hosted_dump_header(esp_header_t *esp_header) {
    static const char *if_strs[] = { "STA", "AP", "SERIAL", "HCI", "PRIV", "TEST" };
    if (esp_header->if_type > ESP_HOSTED_MAX_IF) {
        return;
    }
    LOG_DBG("esp header: if %s_IF length %d offset %d checksum %d seq %d flags %x",
        if_strs[esp_header->if_type], esp_header->len, esp_header->offset,
        esp_header->checksum, esp_header->seq_num, esp_header->flags);

    if (esp_header->if_type == ESP_HOSTED_SERIAL_IF) {
        tlv_header_t *tlv_header = (tlv_header_t *)(esp_header->payload);
        LOG_DBG("tlv header: ep_type %d ep_length %d ep_value %.8s data_type %d data_length %d",
            tlv_header->ep_type, tlv_header->ep_length,
            tlv_header->ep_value, tlv_header->data_type, tlv_header->data_length);
    }
}
#endif

static int32_t esp_hosted_resp_value(CtrlMsg *ctrl_msg) {
    // Each response struct return value is located at a different offset.
    switch (ctrl_msg->msg_id) {
    case CtrlMsgId_Resp_GetMacAddress:
        return ctrl_msg->resp_get_mac_address.resp;
    case CtrlMsgId_Resp_SetMacAddress:
        return ctrl_msg->resp_set_mac_address.resp;
    case CtrlMsgId_Resp_GetWifiMode:
        return ctrl_msg->resp_get_wifi_mode.resp;
    case CtrlMsgId_Resp_SetWifiMode:
        return ctrl_msg->resp_set_wifi_mode.resp;
    case CtrlMsgId_Resp_GetAPScanList:
        return ctrl_msg->resp_scan_ap_list.resp;
    case CtrlMsgId_Resp_GetAPConfig:
        return ctrl_msg->resp_get_ap_config.resp;
    case CtrlMsgId_Resp_ConnectAP:
        return ctrl_msg->resp_connect_ap.resp;
    case CtrlMsgId_Resp_DisconnectAP:
        return ctrl_msg->resp_disconnect_ap.resp;
    case CtrlMsgId_Resp_GetSoftAPConfig:
        return ctrl_msg->resp_get_softap_config.resp;
    case CtrlMsgId_Resp_SetSoftAPVendorSpecificIE:
        return ctrl_msg->resp_set_softap_vendor_specific_ie.resp;
    case CtrlMsgId_Resp_StartSoftAP:
        return ctrl_msg->resp_start_softap.resp;
    case CtrlMsgId_Resp_GetSoftAPConnectedSTAList:
        return ctrl_msg->resp_softap_connected_stas_list.resp;
    case CtrlMsgId_Resp_StopSoftAP:
        return ctrl_msg->resp_stop_softap.resp;
    case CtrlMsgId_Resp_SetPowerSaveMode:
        return ctrl_msg->resp_set_power_save_mode.resp;
    case CtrlMsgId_Resp_GetPowerSaveMode:
        return ctrl_msg->resp_get_power_save_mode.resp;
    case CtrlMsgId_Resp_OTABegin:
        return ctrl_msg->resp_ota_begin.resp;
    case CtrlMsgId_Resp_OTAWrite:
        return ctrl_msg->resp_ota_write.resp;
    case CtrlMsgId_Resp_OTAEnd:
        return ctrl_msg->resp_ota_end.resp;
    case CtrlMsgId_Resp_SetWifiMaxTxPower:
        return ctrl_msg->resp_set_wifi_max_tx_power.resp;
    case CtrlMsgId_Resp_GetWifiCurrTxPower:
        return ctrl_msg->resp_get_wifi_curr_tx_power.resp;
    case CtrlMsgId_Resp_ConfigHeartbeat:
        return ctrl_msg->resp_config_heartbeat.resp;
    default:
        return -1;
    }
}

// TODO add uint32_t timeout
static int esp_hosted_request(const struct device *dev, CtrlMsgId msg_id, CtrlMsg *ctrl_msg) {
    size_t payload_size = 0;
    esp_hosted_data_t *data = (esp_hosted_data_t *)dev->data;

    // Init control message
    ctrl_msg->msg_type = CtrlMsgType_Req;
    ctrl_msg->msg_id = msg_id;
    ctrl_msg->which_payload = msg_id;

    // Get packed protobuf size
    pb_get_encoded_size(&payload_size, CtrlMsg_fields, ctrl_msg);

    if ((payload_size + sizeof(tlv_header_t)) > ESP_FRAME_MAX_PAYLOAD) {
        LOG_ERR("esp_hosted_request: payload size > max payload %d", msg_id);
        return -1;
    }

    uint8_t buf[ESP_FRAME_MAX_SIZE] = { 0 };
    esp_header_t *esp_header = (esp_header_t *)(buf);
    tlv_header_t *tlv_header = (tlv_header_t *)(esp_header->payload);

    esp_header->if_type = ESP_HOSTED_SERIAL_IF;
    esp_header->if_num = 0;
    esp_header->flags = 0;
    esp_header->len = payload_size + sizeof(tlv_header_t);
    esp_header->offset = sizeof(esp_header_t);
    esp_header->seq_num = data->seq_num++;

    tlv_header->ep_type = TLV_HEADER_TYPE_EP;
    tlv_header->ep_length = 8;
    memcpy(tlv_header->ep_value, TLV_HEADER_EP_RESP, 8);
    tlv_header->data_type = TLV_HEADER_TYPE_DATA;
    tlv_header->data_length = payload_size;

    pb_ostream_t stream = pb_ostream_from_buffer(tlv_header->data, tlv_header->data_length);
    if (!pb_encode(&stream, CtrlMsg_fields, ctrl_msg)) {
        LOG_ERR("failed to encode protobuf");
        return -1;
    }

    // Update frame checksum.
    esp_header->checksum = esp_hosted_checksum(esp_header);

    // Send frame.
    size_t frame_size = (sizeof(esp_header_t) + esp_header->len + 3) & ~3U;

    if (esp_hosted_hal_spi_transfer(dev, buf, NULL, frame_size) != 0) {
        LOG_ERR("esp_hosted_request: request %d failed", msg_id);
        return -1;
    }
    return 0;
}

static int esp_hosted_response(const struct device *dev, CtrlMsgId msg_id, CtrlMsg *ctrl_msg, uint32_t timeout) {
    if (k_msgq_get(&esp_hosted_msgq, ctrl_msg, K_MSEC(timeout))) {
        LOG_ERR("esp_hosted_response: failed to receive response");
        return -1;
    }
    
    // TODO should we peek?
    if (ctrl_msg->msg_id != msg_id) {
        LOG_ERR("esp_hosted_response: expected id %u got id %u", msg_id, ctrl_msg->msg_id);
        return -1;
    }

    // If message type is a response, check the response struct's return value.
    if (ctrl_msg->msg_type == CtrlMsgType_Resp && esp_hosted_resp_value(ctrl_msg) != 0) {
        LOG_ERR("esp_hosted_response response %d failed %d", msg_id, esp_hosted_resp_value(ctrl_msg));
        return -1;
    }

    return 0;
}


static int esp_hosted_ctrl(const struct device *dev, CtrlMsgId req_id, CtrlMsg *ctrl_msg) {
    uint32_t resp_id = (req_id - CtrlMsgId_Req_Base) + CtrlMsgId_Resp_Base;

    if (esp_hosted_request(dev, req_id, ctrl_msg)) {
        return -1;
    }

    if (esp_hosted_response(dev, resp_id, ctrl_msg, ESP_HOSTED_SYNC_TIMEOUT)) {
        return -1;
    }

    return 0;
}

static void esp_hosted_event_task(const struct device *dev, void *p2, void *p3)
{
	esp_hosted_data_t *data = dev->data;

    for (; ; k_msleep(10)) { 
        size_t offset = 0;
        uint8_t buf[ESP_STATE_BUF_SIZE] = {0};
        esp_header_t *esp_header = (esp_header_t *)(buf);

        do {
            esp_header_t *frag_header = (esp_header_t *)(buf + offset);

            if ((ESP_STATE_BUF_SIZE - offset) < ESP_FRAME_MAX_SIZE) {
                // This shouldn't happen, but if it did stop the thread.
                LOG_ERR("spi buffer overflow offset: %d", offset);
                return;
            }

            if (esp_hosted_hal_spi_transfer(dev, NULL, buf + offset, ESP_FRAME_MAX_SIZE) != 0) {
                goto restart;
            }

            if (!ESP_HEADER_IS_VALID(frag_header)) {
                LOG_WRN("invalid or empty frame offset: %d", offset);
                goto restart;
            }

            if (frag_header->checksum != esp_hosted_checksum(frag_header)) {
                LOG_ERR("invalid checksum");
                goto restart;
            }

            if (offset) {
                // Combine fragmented frame
                if ((esp_header->seq_num + 1) != frag_header->seq_num) {
                    LOG_ERR("fragmented frame sequence mismatch");
                    goto restart;
                }
                esp_header->len += frag_header->len;
                esp_header->seq_num = frag_header->seq_num;
                esp_header->flags = frag_header->flags;
                // Append the current fragment's payload to the previous one.
                memcpy(buf + offset, frag_header->payload, frag_header->len);
                LOG_DBG("received fragmented frame, length: %d", esp_header->len);
            }

            offset = sizeof(esp_header_t) + esp_header->len;
        } while (esp_header->flags & ESP_FRAME_FLAGS_FRAGMENT);

        #if ESP_HOSTED_DEBUG
        esp_hosted_dump_header(esp_header);
        #endif

        switch (esp_header->if_type) {
        case ESP_HOSTED_STA_IF:
        case ESP_HOSTED_AP_IF: {
            // Networking traffic
            //uint32_t itf = esp_header->if_type;
            //if (netif_is_link_up(&data->netif[itf])) {
            //    if (esp_hosted_netif_input(&esp_state, itf, esp_header->payload, esp_header->len) != 0) {
            //        LOG_ERR("netif input failed");
            //        return -1;
            //    }
            //    LOG_DBG("eth frame input %d", esp_header->len);
            //}
            LOG_DBG("eth frame input %d", esp_header->len);
            continue;
        }
        case ESP_HOSTED_PRIV_IF: {
            esp_event_t *priv_event = (esp_event_t *)(esp_header->payload);
            if (esp_header->priv_pkt_type == ESP_PACKET_TYPE_EVENT &&
                priv_event->event_type == ESP_PRIV_EVENT_INIT) {
                data->chip_id = priv_event->event_data[2];
                data->spi_clk = priv_event->event_data[5];
                data->chip_flags = priv_event->event_data[8];
                LOG_DBG("chip id %d spi_mhz %d caps 0x%x",
                    data->chip_id, data->spi_clk, data->chip_flags);
            }
            continue;
        }
        case ESP_HOSTED_SERIAL_IF:
            // Requires further processing
            break;
        default:
            LOG_ERR("unexpected interface type %d", esp_header->if_type);
            continue;
        }

        CtrlMsg ctrl_msg = CtrlMsg_init_zero;
        tlv_header_t *tlv_header = (tlv_header_t *)(esp_header->payload);
        pb_istream_t stream = pb_istream_from_buffer(tlv_header->data, tlv_header->data_length);

        if (!pb_decode(&stream, CtrlMsg_fields, &ctrl_msg)) {
            LOG_ERR("failed to decode protobuf");
            continue;
        }

        if (ctrl_msg.msg_type == CtrlMsgType_Event) {
            switch (ctrl_msg.msg_id) {
                case CtrlMsgId_Event_ESPInit:
                    data->flags |= ESP_HOSTED_FLAGS_ACTIVE;
                    break;
                case CtrlMsgId_Event_Heartbeat:
                    data->last_hb_ms = k_uptime_get();
                    continue;
                case CtrlMsgId_Event_StationDisconnectFromAP:
                    //net_if_dormant_on(iface);
                    data->flags &= ~ESP_HOSTED_FLAGS_STA_CONNECTED;
                    continue;
                case CtrlMsgId_Event_StationDisconnectFromESPSoftAP:
                    continue;
                default:
                    LOG_ERR("unexpected event %d", ctrl_msg.msg_id);
                    continue;
            }
        }

        // Responses that should be handled here.
        if (ctrl_msg.msg_type == CtrlMsgType_Resp) {
            switch (ctrl_msg.msg_id) {
            case CtrlMsgId_Resp_ConnectAP:
                if (esp_hosted_resp_value(&ctrl_msg) == 0) {
                    data->flags |= ESP_HOSTED_FLAGS_STA_CONNECTED;
                }
                continue;
            default:
                break;
            }
        }

        // A control message resp/event will be pushed on the stack for further processing.
	    if (k_msgq_put(&esp_hosted_msgq, &ctrl_msg, K_FOREVER)) {
            LOG_ERR("Failed to enqueue message");
            return;
        }
        LOG_DBG("pushed msg_type %u msg_id %u", ctrl_msg.msg_type, ctrl_msg.msg_id);
        restart:
    }
}

static void esp_hosted_mgmt_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	esp_hosted_data_t *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	data->iface = iface;
	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;

    // Set MAC address.
	net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);

    // Configure interface
	ethernet_init(iface);
	net_if_dormant_on(iface);
	net_if_carrier_on(data->iface);
}

static void esp_hosted_mac_to_bytes(const uint8_t *mac_str, size_t mac_len, uint8_t *mac_out) {
    uint8_t byte = 0;
    for (int i = 0; i < mac_len; i++) {
        char c = mac_str[i];
        if (c >= '0' && c <= '9') {
            byte = (byte << 4) | (c - '0');
        } else if (c >= 'a' && c <= 'f') {
            byte = (byte << 4) | (c - 'a' + 10);
        } else if (c >= 'A' && c <= 'F') {
            byte = (byte << 4) | (c - 'A' + 10);
        }
        if (c == ':' || (i + 1) == mac_len) {
            *mac_out++ = byte;
            byte = 0;
        }
    }
}

static int esp_hosted_init(const struct device *dev)
{
	int ret;
	esp_hosted_data_t *data = dev->data;

    // Pins config and SPI init.
    if (esp_hosted_hal_init(dev)) {
		return -EAGAIN;
	}

    // Initialize semaphore.
	ret = k_sem_init(&data->bus_sem, 1, 1);
	if (ret != 0) {
		LOG_ERR("k_sem_init() failed");
		return ret;
	}

	data->tid = k_thread_create(&esp_hosted_event_thread,
            esp_hosted_event_stack, K_THREAD_STACK_SIZEOF(esp_hosted_event_stack),
            (k_thread_entry_t) esp_hosted_event_task, (void *) dev, NULL, NULL,
            CONFIG_WIFI_ESP_HOSTED_EVENT_TASK_PRIORITY, K_INHERIT_PERMS, K_NO_WAIT);
	if (!data->tid) {
		LOG_ERR("ERROR spawning event processing thread");
		return -EAGAIN;
	}
	//k_thread_name_set(data->tid, "esp_hosted_event");

    CtrlMsg ctrl_msg = CtrlMsg_init_zero;
    // Wait for an ESPInit control event.
    if (esp_hosted_response(dev, CtrlMsgId_Event_ESPInit, &ctrl_msg, ESP_HOSTED_SYNC_TIMEOUT)) {
        return -EIO;
    }

    // Set WiFi mode to STA/AP.
    ctrl_msg.req_set_wifi_mode.mode = Ctrl_WifiMode_APSTA;
    if (esp_hosted_ctrl(dev, CtrlMsgId_Req_SetWifiMode, &ctrl_msg)) {
        return -EIO;
    }
	return 0;
}

static int esp_hosted_set_config(const struct device *dev,
			    enum ethernet_config_type type,
			    const struct ethernet_config *config) {
    return -EIO;
}

static int esp_hosted_mgmt_connect(const struct device *dev, struct wifi_connect_req_params *params)
{
	return -EAGAIN;
}

static int esp_hosted_mgmt_disconnect(const struct device *dev)
{
    return 0;
}

static int esp_hosted_mgmt_ap_enable(const struct device *dev, struct wifi_connect_req_params *params)
{
    return -EAGAIN;
}

static int esp_hosted_mgmt_ap_disable(const struct device *dev)
{
    return -EAGAIN;
}

static int esp_hosted_mgmt_send(const struct device *dev, struct net_pkt *pkt)
{
    return -EIO;
}

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int esp_hosted_mgmt_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
	esp_hosted_data_t *data = dev->data;

	//stats->bytes.received = data->stats.bytes.received;
	//stats->bytes.sent = data->stats.bytes.sent;
	//stats->pkts.rx = data->stats.pkts.rx;
	//stats->pkts.tx = data->stats.pkts.tx;
	//stats->errors.rx = data->stats.errors.rx;
	//stats->errors.tx = data->stats.errors.tx;
	//stats->broadcast.rx = data->stats.broadcast.rx;
	//stats->broadcast.tx = data->stats.broadcast.tx;
	//stats->multicast.rx = data->stats.multicast.rx;
	//stats->multicast.tx = data->stats.multicast.tx;
	//stats->sta_mgmt.beacons_rx = data->stats.sta_mgmt.beacons_rx;
	//stats->sta_mgmt.beacons_miss = data->stats.sta_mgmt.beacons_miss;

	return 0;
}
#endif

static int esp_hosted_mgmt_scan(const struct device *dev,
        struct wifi_scan_params *params, scan_result_cb_t cb)
{
	esp_hosted_data_t *data = dev->data;
    CtrlMsg ctrl_msg = CtrlMsg_init_zero;

    if (esp_hosted_ctrl(dev, CtrlMsgId_Req_GetAPScanList, &ctrl_msg) != 0) {
        return -ETIMEDOUT;
    }

    size_t ap_count = ctrl_msg.resp_scan_ap_list.count;
    ScanResult *ap_list = ctrl_msg.resp_scan_ap_list.entries;

    for (size_t i = 0; i < ap_count; i++) {
        struct wifi_scan_result result = { 0 };

		result.rssi = ap_list[i].rssi;
		result.channel = ap_list[i].chnl;

        switch (ap_list[i].sec_prot) {
        case Ctrl_WifiSecProt_Open:
			result.security = WIFI_SECURITY_TYPE_NONE;
            break;
        case Ctrl_WifiSecProt_WEP:
			result.security = WIFI_SECURITY_TYPE_WEP;
            break;
        case Ctrl_WifiSecProt_WPA_PSK:
			result.security = WIFI_SECURITY_TYPE_WPA_PSK;
            break;
        case Ctrl_WifiSecProt_WPA2_PSK:
			result.security = WIFI_SECURITY_TYPE_PSK;
            break;
        case Ctrl_WifiSecProt_WPA2_ENTERPRISE:
			result.security = WIFI_SECURITY_TYPE_EAP;
            break;
        case Ctrl_WifiSecProt_WPA3_PSK:
			result.security = WIFI_SECURITY_TYPE_SAE;
            break;
        case Ctrl_WifiSecProt_WPA2_WPA3_PSK:
        case Ctrl_WifiSecProt_WPA_WPA2_PSK:
            result.security = WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL;
            break;
        default:
			result.security = WIFI_SECURITY_TYPE_UNKNOWN;
        }

        if (ap_list[i].ssid.size) {
            result.ssid_length = MIN(ap_list[i].ssid.size, WIFI_SSID_MAX_LEN);
            memcpy(result.ssid, ap_list[i].ssid.bytes, result.ssid_length);
        }

        if (ap_list[i].bssid.size) {
            result.mac_length = WIFI_MAC_ADDR_LEN;
            esp_hosted_mac_to_bytes(ap_list[i].bssid.bytes, ap_list[i].bssid.size, result.mac);
        }

        cb(data->iface, 0, &result);
        k_yield();
    }

    // End of scan
    cb(data->iface, 0, NULL);

    // Entries are dynamically allocated.
    //pb_release(CtrlMsg_fields, &ctrl_msg);
    pb_release(CtrlMsg_Resp_ScanResult_fields, &ctrl_msg.resp_scan_ap_list);
    return 0;
}

static enum ethernet_hw_caps esp_hosted_mgmt_caps(const struct device *dev)
{
	return ETHERNET_HW_FILTERING;
}

static const struct wifi_mgmt_ops esp_hosted_mgmt = {
	.scan 		  = esp_hosted_mgmt_scan,
	.connect 	  = esp_hosted_mgmt_connect,
	.disconnect   = esp_hosted_mgmt_disconnect,
	.ap_enable    = esp_hosted_mgmt_ap_enable,
	.ap_disable   = esp_hosted_mgmt_ap_disable,
#if defined(CONFIG_NET_STATISTICS_WIFI)
	.get_stats	    = esp_hosted_mgmt_wifi_stats,
#endif
};

static const struct net_wifi_mgmt_offload esp_hosted_api = {
	.wifi_iface.iface_api.init = esp_hosted_mgmt_init,
	.wifi_iface.send = esp_hosted_mgmt_send,
	.wifi_iface.get_capabilities = esp_hosted_mgmt_caps,
	.wifi_iface.set_config = esp_hosted_set_config,
	.wifi_mgmt_api = &esp_hosted_mgmt,
};

NET_DEVICE_DT_INST_DEFINE(0,
		esp_hosted_init, NULL,
		&esp_hosted_data, &esp_hosted_config,
		CONFIG_WIFI_INIT_PRIORITY, &esp_hosted_api, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
