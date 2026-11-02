/** @file comm_bridge.c
 * @brief Subsystem 1 side of the comm_bridge.h contract (interface
 * I1.5): the outgoing queue the comm task fills, and the hooks the
 * kernel's WiFi task calls (comm_bridge_tx_pop/rx_command/mqtt_state).
 *
 * Every kernel call here polls (TMO_POL), so the WiFi task can never be
 * blocked by the application, and the comm task never by the network.
 */

#include "comm_bridge.h"
#include "s1_private.h"

#define TOPIC_MASK  0x03U /* tx record header: topic in bits 0..1 */
#define RETAIN_BIT  0x04U /* tx record header: retain in bit 2    */

volatile uint8_t g_s1_mqtt_state = (uint8_t)COMM_MQTT_IDLE;

/* Written only by the WiFi task, read by anyone (16-bit store is
 * atomic on the RP2040). */
static volatile uint16_t g_rx_dropped = 0U;

/** @brief Queue one frame for the WiFi task; false = dropped, counted.
 *
 * Called by the comm task only (its record buffer is not re-entrant).
 */
bool
comm_bridge_tx_enqueue (comm_bridge_topic_t topic, bool b_retain,
                        const uint8_t * p_payload, uint32_t len)
{
    static uint8_t record[TX_RECORD_MAX];
    uint32_t       index;
    bool           b_queued = false;

    if ((NULL != p_payload) && (len <= COMM_PAYLOAD_MAX))
    {
        /* Topic is 0..3 and the flag one bit, so the header fits. */
        record[0] = (uint8_t)(((uint32_t)topic & TOPIC_MASK) |
                              (b_retain ? RETAIN_BIT : 0U));
        for (index = 0U; index < len; index++)
        {
            record[TX_RECORD_HDR + index] = p_payload[index];
        }

        /* At most TX_RECORD_MAX (33) bytes: fits the kernel's INT. */
        b_queued = (E_OK == tk_snd_mbf(gh_s1_tx_mbf, record,
                                       (INT)(TX_RECORD_HDR + len), TMO_POL));
    }

    if (b_queued)
    {
        g_s1_stats.tx_queued++;
    }
    else
    {
        g_s1_stats.tx_dropped++; /* network slower than we produce */
    }

    return (b_queued);
}

/** @brief WiFi task: take the next queued frame (see comm_bridge.h). */
int32_t
comm_bridge_tx_pop (uint8_t * p_topic_id, uint8_t * p_retain,
                    uint8_t * p_payload, uint16_t payload_max)
{
    static uint8_t record[TX_RECORD_MAX]; /* WiFi task only */
    int32_t        length   = -1;
    INT            received = E_TMOUT;
    int32_t        index;

    if (gh_s1_tx_mbf > 0) /* 0 until subsystem1_init() has run */
    {
        received = tk_rcv_mbf(gh_s1_tx_mbf, record, TMO_POL);
    }

    if (received >= (INT)TX_RECORD_HDR) /* E_TMOUT: nothing queued */
    {
        length = (int32_t)received - (int32_t)TX_RECORD_HDR;
        if (length > (int32_t)payload_max)
        {
            length = (int32_t)payload_max;
        }

        *p_topic_id = (uint8_t)(record[0] & TOPIC_MASK);
        *p_retain   = (0U != (record[0] & RETAIN_BIT)) ? 1U : 0U;
        for (index = 0; index < length; index++)
        {
            p_payload[index] = record[(int32_t)TX_RECORD_HDR + index];
        }
    }

    return (length);
}

/** @brief WiFi task: one complete message arrived on <prefix>/cmd. */
void
comm_bridge_rx_command (const uint8_t * p_data, uint16_t len)
{
    static const uint8_t empty = 0U;
    const uint8_t *      p_msg = p_data;
    uint32_t             size  = len;

    if (gh_s1_rx_mbf > 0)
    {
        /* Over-long frames are cut to RX_RECORD_MAX bytes: still one
         * byte too long, so the decoder rejects them as wrong length. */
        if (size > RX_RECORD_MAX)
        {
            size = RX_RECORD_MAX;
        }

        /* An empty message still gets an ack (a 1-byte zero frame is
         * rejected as the wrong length). */
        if ((0U == size) || (NULL == p_msg))
        {
            size  = 1U;
            p_msg = &empty;
        }

        /* size is 1..RX_RECORD_MAX: fits the kernel's INT. */
        if (E_OK == tk_snd_mbf(gh_s1_rx_mbf, p_msg, (INT)size, TMO_POL))
        {
            (void)tk_set_flg(gh_s1_comm_flg, FLG_COMM_RX);
        }
        else
        {
            g_rx_dropped++;
        }
    }
}

/** @brief WiFi task: the MQTT client changed state. */
void
comm_bridge_mqtt_state (int32_t state)
{
    /* comm_bridge_mqtt_t values are 0..5, so they fit in a byte. */
    g_s1_mqtt_state = (uint8_t)state;

    if (gh_s1_comm_flg > 0)
    {
        (void)tk_set_flg(gh_s1_comm_flg, FLG_COMM_LINK);
    }
}

/** @brief Commands lost because the receive queue was full. */
uint16_t
comm_bridge_rx_dropped (void)
{
    return (g_rx_dropped);
}

/*** end of file ***/

