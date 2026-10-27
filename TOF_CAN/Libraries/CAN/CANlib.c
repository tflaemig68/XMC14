/**
 * @file CANlib.c
 * @brief CAN-Senden und -Empfangen mit der im DAVE-Projekt enthaltenen Infineon-XMCLib.
 */
#include "CANlib.h"
#include "xmc_can.h"
#include "xmc_gpio.h"

/** @brief Hardwareobjekt fuer die parametrierte Empfangs-ID. */
static XMC_CAN_MO_t rx_object;
/** @brief Eigenes Sendeobjekt, damit Empfang und Senden unabhaengig arbeiten. */
static XMC_CAN_MO_t tx_object;
/** @brief Gibt Hardwarezugriffe erst nach erfolgreicher Initialisierung frei. */
static bool initialized = false;
/** @brief Beobachtete Meldungen ueber ueberschriebene Empfangsdaten. */
static uint32_t overrun_count = 0U;

bool CANLIB_Init(void)
{
    const XMC_GPIO_CONFIG_t rx_config = { .mode = XMC_GPIO_MODE_INPUT_PULL_UP };
    const XMC_GPIO_CONFIG_t tx_config = {
        .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9,
        .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH
    };
    XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t timing = {
        .can_frequency = 0U,
        .baudrate = CANLIB_BITRATE,
        .sample_point = 8000U,
        .sjw = 1U
    };

    initialized = false;
    overrun_count = 0U;
    /* Einzelzuweisungen vermeiden Warnungen der alten GCC-Version bei den
     * anonymen Structs/Unions des XMCLib-Handles. */
    rx_object = (XMC_CAN_MO_t){0};
    rx_object.can_mo_ptr = CAN_MO0;
    rx_object.can_identifier = CANLIB_RX_ID;
#if CANLIB_ID_BITS == 11U
    rx_object.can_id_mode = XMC_CAN_FRAME_TYPE_STANDARD_11BITS;
#else
    rx_object.can_id_mode = XMC_CAN_FRAME_TYPE_EXTENDED_29BITS;
#endif
    rx_object.can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2;
    rx_object.can_id_mask = CANLIB_ID_MASK;
    rx_object.can_ide_mask = 1U;
    rx_object.can_data_length = 8U;
    rx_object.can_mo_type = XMC_CAN_MO_TYPE_RECMSGOBJ;
    /* Der Rueckgabewert ist der wirklich eingestellte CAN-Modultakt.
     * Er ist die Grundlage fuer die Berechnung der 500-kbit/s-Bitzeit. */
    timing.can_frequency = XMC_CAN_InitEx(CAN, XMC_CAN_CANCLKSRC_MCLK, 8000000UL);
    XMC_CAN_NODE_SetInitBit(CAN_NODE0);
    if (XMC_CAN_NODE_NominalBitTimeConfigureEx(CAN_NODE0, &timing)
        != XMC_CAN_STATUS_SUCCESS) {
        return false;
    }
    XMC_GPIO_Init(XMC_GPIO_PORT1, 1U, &rx_config);
    XMC_GPIO_Init(XMC_GPIO_PORT1, 0U, &tx_config);
    XMC_CAN_NODE_EnableConfigurationChange(CAN_NODE0);
    XMC_CAN_NODE_SetReceiveInput(CAN_NODE0, CAN_NODE0_RXD_P1_1);
    XMC_CAN_NODE_DisableLoopBack(CAN_NODE0);
    XMC_CAN_NODE_ReSetAnalyzerMode(CAN_NODE0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_NODE0);

    /* Alle ID-Bits und das IDE-Bit muessen passen: Eine Extended-Botschaft
     * mit gleichem Zahlenwert wird im 11-Bit-Modus nicht angenommen. */
    XMC_CAN_AllocateMOtoNodeList(CAN, 0U, 0U);
    while (!XMC_CAN_IsPanelControlReady(CAN)) { }
    XMC_CAN_MO_Config(&rx_object);
    XMC_CAN_MO_ResetStatus(&rx_object,
        XMC_CAN_MO_RESET_STATUS_NEW_DATA | XMC_CAN_MO_RESET_STATUS_RX_PENDING |
        XMC_CAN_MO_RESET_STATUS_MESSAGE_LOST);
    /* MO1 ist ausschliesslich zum Senden da. MO0 bleibt der Empfaenger.
     * Die gemeinsamen ID-Einstellungen gelten fuer beide Objekte. */
    tx_object = rx_object;
    tx_object.can_mo_ptr = CAN_MO1;
    tx_object.can_identifier = CANLIB_TX_ID;
    tx_object.can_mo_type = XMC_CAN_MO_TYPE_TRANSMSGOBJ;
    XMC_CAN_AllocateMOtoNodeList(CAN, 0U, 1U);
    while (!XMC_CAN_IsPanelControlReady(CAN)) { }
    XMC_CAN_MO_Config(&tx_object);
    XMC_CAN_MO_ResetStatus(&tx_object,
        XMC_CAN_MO_RESET_STATUS_TX_PENDING | XMC_CAN_MO_RESET_STATUS_TX_REQUEST);
    XMC_CAN_NODE_ResetInitBit(CAN_NODE0);
    initialized = true;
    return true;
}

bool CANLIB_Receive(CANLIB_Message_t *message)
{
    uint32_t status;
    if (!initialized || message == NULL || CANLIB_GetState() == CANLIB_STATE_BUS_OFF) {
        return false;
    }
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        CANLIB_Message_t snapshot = {0};
        uint32_t arbitration;
        uint32_t low;
        uint32_t high;
        status = XMC_CAN_MO_GetStatus(&rx_object);
        if (status & XMC_CAN_MO_STATUS_MESSAGE_LOST) {
            ++overrun_count;
            XMC_CAN_MO_ResetStatus(&rx_object, XMC_CAN_MO_RESET_STATUS_MESSAGE_LOST);
        }
        if (!(status & XMC_CAN_MO_STATUS_NEW_DATA) ||
            (status & XMC_CAN_MO_STATUS_RX_UPDATING)) {
            return false;
        }

        /* NEWDAT vor dem Lesen quittieren. Wird es waehrend der Kopie erneut
         * gesetzt, gehoeren die Register moeglicherweise zu verschiedenen Frames.
         * Dann die Kopie verwerfen und beim naechsten Versuch erneut lesen. */
        XMC_CAN_MO_ResetStatus(&rx_object,
            XMC_CAN_MO_RESET_STATUS_NEW_DATA | XMC_CAN_MO_RESET_STATUS_RX_PENDING);
        arbitration = rx_object.can_mo_ptr->MOAR;
        snapshot.dlc = (uint8_t)((rx_object.can_mo_ptr->MOFCR & CAN_MO_MOFCR_DLC_Msk)
                                 >> CAN_MO_MOFCR_DLC_Pos);
        low = rx_object.can_mo_ptr->MODATAL;
        high = rx_object.can_mo_ptr->MODATAH;
        status = XMC_CAN_MO_GetStatus(&rx_object);
        if (status & (XMC_CAN_MO_STATUS_RX_UPDATING | XMC_CAN_MO_STATUS_NEW_DATA)) {
            continue;
        }
        snapshot.id_bits = (arbitration & CAN_MO_MOAR_IDE_Msk) ? 29U : 11U;
        snapshot.id = (snapshot.id_bits == 11U)
            ? (arbitration & XMC_CAN_MO_MOAR_STDID_Msk) >> XMC_CAN_MO_MOAR_STDID_Pos
            : (arbitration & CAN_MO_MOAR_ID_Msk);
        /* Zusaetzliche Softwarepruefung neben dem Hardwarefilter. */
        if (snapshot.id != CANLIB_RX_ID || snapshot.id_bits != CANLIB_ID_BITS) {
            return false;
        }
        snapshot.length = snapshot.dlc > 8U ? 8U : snapshot.dlc;
        for (uint8_t i = 0U; i < snapshot.length; ++i) {
            snapshot.data[i] = (uint8_t)((i < 4U ? low : high) >> (8U * (i % 4U)));
        }
        *message = snapshot;
        return true;
    }
    return false;
}

CANLIB_State_t CANLIB_GetState(void)
{
    uint32_t status;
    if (!initialized) { return CANLIB_STATE_NOT_READY; }
    status = XMC_CAN_NODE_GetStatus(CAN_NODE0);
    if (status & XMC_CAN_NODE_STATUS_BUS_OFF) { return CANLIB_STATE_BUS_OFF; }
    if (status & XMC_CAN_NODE_STATUS_ERROR_WARNING_STATUS) { return CANLIB_STATE_WARNING; }
    return CANLIB_STATE_ACTIVE;
}

uint32_t CANLIB_GetOverrunCount(void)
{
    return overrun_count;
}

CANLIB_TxResult_t CANLIB_Send(const uint8_t *data, uint8_t length)
{
    if (!initialized || data == NULL || length > 8U ||
        CANLIB_GetState() == CANLIB_STATE_BUS_OFF) {
        return CANLIB_TX_ERROR;
    }
    /* Unbedingt VOR UpdateData pruefen: Ein laufender Auftrag darf nicht
     * deaktiviert oder mit teilweise neuen Nutzdaten ueberschrieben werden. */
    if (XMC_CAN_MO_GetStatus(&tx_object) & XMC_CAN_MO_STATUS_TX_REQUEST) {
        return CANLIB_TX_BUSY;
    }
    tx_object.can_data_length = length;
    tx_object.can_data[0] = 0U;
    tx_object.can_data[1] = 0U;
    for (uint8_t i = 0U; i < length; ++i) {
        /* Vier Bytes bilden jeweils ein 32-Bit-Hardwareregister.
         * Explizite Verschiebungen vermeiden Alignment- und Cast-Probleme. */
        tx_object.can_data[i / 4U] |= (uint32_t)data[i] << (8U * (i % 4U));
    }
    XMC_CAN_MO_ResetStatus(&tx_object, XMC_CAN_MO_RESET_STATUS_TX_PENDING);
    if (XMC_CAN_MO_UpdateData(&tx_object) != XMC_CAN_STATUS_SUCCESS) {
        return CANLIB_TX_ERROR;
    }
    return XMC_CAN_MO_Transmit(&tx_object) == XMC_CAN_STATUS_SUCCESS
        ? CANLIB_TX_ACCEPTED : CANLIB_TX_ERROR;
}
