/**
 * @file CANlib.c
 * @brief 32-Gruppen-CAN-Empfang mit der im DAVE-Projekt enthaltenen Infineon-XMCLib.
 */
#include "CANlib.h"
#include "xmc_can.h"
#include "xmc_gpio.h"

/** @brief Hardwareobjekt fuer die parametrierte Empfangs-ID. */
#define RX_OBJECTS 17U
static XMC_CAN_MO_t rx_objects[RX_OBJECTS];
/** @brief MO17 sendet zentrale Sollwerte auf ID 0x100. */
static XMC_CAN_MO_t tx_object;
/** @brief Naechster fair abgefragter Gruppenindex. */
static uint8_t next_rx;
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
    XMC_CAN_MO_t rx_object;
    next_rx = 0U;
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

    /* Rand-IDs einzeln, dazwischen 15 disjunkte Zweierfilter.
     * Gruppen 2/3 bis 30/31 teilen jeweils einen Empfangsspeicher.
     * Alle IDs bleiben gleichzeitig empfangsbereit, MO17 bleibt zum Senden frei. */
    for (uint8_t i = 0U; i < RX_OBJECTS; ++i) {
        rx_objects[i] = rx_object;
        rx_objects[i].can_mo_ptr = &CAN_MO->MO[i];
        rx_objects[i].can_identifier = i==0U ? CANLIB_RX_ID :
            i==16U ? CANLIB_RX_ID+31U : CANLIB_RX_ID+2U*i-1U;
        if (i>0U && i<16U) { rx_objects[i].can_id_mask=CANLIB_ID_MASK & ~1UL; }
        XMC_CAN_AllocateMOtoNodeList(CAN, 0U, i);
        while (!XMC_CAN_IsPanelControlReady(CAN)) { }
        XMC_CAN_MO_Config(&rx_objects[i]);
        XMC_CAN_MO_ResetStatus(&rx_objects[i],
            XMC_CAN_MO_RESET_STATUS_NEW_DATA | XMC_CAN_MO_RESET_STATUS_RX_PENDING |
            XMC_CAN_MO_RESET_STATUS_MESSAGE_LOST);
    }
    tx_object=rx_object;
    tx_object.can_mo_ptr=&CAN_MO->MO[17];
    tx_object.can_identifier=0x100UL;
    tx_object.can_mo_type=XMC_CAN_MO_TYPE_TRANSMSGOBJ;
    XMC_CAN_AllocateMOtoNodeList(CAN,0U,17U);
    while (!XMC_CAN_IsPanelControlReady(CAN)) { }
    XMC_CAN_MO_Config(&tx_object);
    XMC_CAN_MO_ResetStatus(&tx_object,XMC_CAN_MO_RESET_STATUS_TX_PENDING | XMC_CAN_MO_RESET_STATUS_TX_REQUEST);
    XMC_CAN_NODE_ResetInitBit(CAN_NODE0);
    initialized = true;
    return true;
}

static bool receive_object(XMC_CAN_MO_t *object, CANLIB_Message_t *message)
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
        status = XMC_CAN_MO_GetStatus(object);
        if (status & XMC_CAN_MO_STATUS_MESSAGE_LOST) {
            ++overrun_count;
            XMC_CAN_MO_ResetStatus(object, XMC_CAN_MO_RESET_STATUS_MESSAGE_LOST);
        }
        if (!(status & XMC_CAN_MO_STATUS_NEW_DATA) ||
            (status & XMC_CAN_MO_STATUS_RX_UPDATING)) {
            return false;
        }

        /* NEWDAT vor dem Lesen quittieren. Wird es waehrend der Kopie erneut
         * gesetzt, gehoeren die Register moeglicherweise zu verschiedenen Frames.
         * Dann die Kopie verwerfen und beim naechsten Versuch erneut lesen. */
        XMC_CAN_MO_ResetStatus(object,
            XMC_CAN_MO_RESET_STATUS_NEW_DATA | XMC_CAN_MO_RESET_STATUS_RX_PENDING);
        arbitration = object->can_mo_ptr->MOAR;
        snapshot.dlc = (uint8_t)((object->can_mo_ptr->MOFCR & CAN_MO_MOFCR_DLC_Msk)
                                 >> CAN_MO_MOFCR_DLC_Pos);
        low = object->can_mo_ptr->MODATAL;
        high = object->can_mo_ptr->MODATAH;
        status = XMC_CAN_MO_GetStatus(object);
        if (status & (XMC_CAN_MO_STATUS_RX_UPDATING | XMC_CAN_MO_STATUS_NEW_DATA)) {
            continue;
        }
        snapshot.id_bits = (arbitration & CAN_MO_MOAR_IDE_Msk) ? 29U : 11U;
        snapshot.id = (snapshot.id_bits == 11U)
            ? (arbitration & XMC_CAN_MO_MOAR_STDID_Msk) >> XMC_CAN_MO_MOAR_STDID_Pos
            : (arbitration & CAN_MO_MOAR_ID_Msk);
        /* Zusaetzliche Softwarepruefung neben dem Hardwarefilter. */
        if (snapshot.id < CANLIB_RX_ID || snapshot.id >= CANLIB_RX_ID+32U || snapshot.id_bits != CANLIB_ID_BITS) {
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

bool CANLIB_Receive(CANLIB_Message_t *message)
{
    if (!initialized || message == NULL) { return false; }
    for (uint8_t scanned = 0U; scanned < RX_OBJECTS; ++scanned) {
        uint8_t index = next_rx;
        next_rx = (uint8_t)((next_rx + 1U) % RX_OBJECTS);
        if (receive_object(&rx_objects[index], message)) { return true; }
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
    if (!initialized || data==NULL || length>8U || CANLIB_GetState()==CANLIB_STATE_BUS_OFF) { return CANLIB_TX_ERROR; }
    if (XMC_CAN_MO_GetStatus(&tx_object) & XMC_CAN_MO_STATUS_TX_REQUEST) { return CANLIB_TX_BUSY; }
    tx_object.can_data_length=length;
    tx_object.can_data[0]=0U; tx_object.can_data[1]=0U;
    for (uint8_t i=0U;i<length;++i) { tx_object.can_data[i/4U]|=(uint32_t)data[i]<<(8U*(i%4U)); }
    XMC_CAN_MO_ResetStatus(&tx_object,XMC_CAN_MO_RESET_STATUS_TX_PENDING);
    if (XMC_CAN_MO_UpdateData(&tx_object)!=XMC_CAN_STATUS_SUCCESS) { return CANLIB_TX_ERROR; }
    return XMC_CAN_MO_Transmit(&tx_object)==XMC_CAN_STATUS_SUCCESS ? CANLIB_TX_ACCEPTED : CANLIB_TX_ERROR;
}
