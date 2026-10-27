/**
 * @file tof_dave_port.inc
 * @brief Uebersetzt TOF-Registerzugriffe in DAVE-I2C-Befehle.
 *
 * @details Diese Datei wird in i2cTOF.c eingebunden, nicht separat kompiliert.
 * static beschraenkt die Hilfsfunktionen auf diese Uebersetzungseinheit.
 * Das I2C_MASTER_t-Handle enthaelt unter anderem den Hardwarekanal und
 * Laufzeitinformationen. Die Portierung setzt DIRECT-Betrieb mit FIFO voraus.
 * Ein FIFO ist ein Puffer: Das zuerst abgelegte Byte wird zuerst entnommen.
 *
 * I2C-Wartephasen haben 20 ms Zeitlimit, Sensorbereitschaft 1000 ms.
 *
 * @pre DAVE und der Millisekundentimer laufen; Aufruf nur aus dem Hauptprogramm.
 * @note Die vorhandenen DAVE-Bytefunktionen werden mit begrenztem Warten
 * kombiniert. Ein Sensor verwendet die 7-Bit-Adresse 0x29; auf dem Bus
 * reserviert DAVE zusaetzlich das niederwertigste Bit fuer Lesen/Schreiben.
 */

extern volatile uint32_t SysTickCounter;
/** @brief Rueckgabecode: 0 bedeutet Erfolg, 1 bedeutet Fehler. */
typedef enum { I2C_OK = 0, I2C_ERROR = 1 } I2C_RETURN_CODE_t;
/* Maximale Wartezeit einer I2C-Phase in Millisekunden. */
#define TOF_I2C_TIMEOUT_MS 20U
/* Maximale Wartezeit auf Sensorbereitschaft in Millisekunden. */
#define TOF_SENSOR_TIMEOUT_MS 1000U
/* Bitmaske der als Fehler gewerteten I2C-Hardwaremeldungen. */
#define TOF_I2C_ERRORS (XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED | \
                        XMC_I2C_CH_STATUS_FLAG_ARBITRATION_LOST | \
                        XMC_I2C_CH_STATUS_FLAG_ERROR)

/**
 * @brief Wartet begrenzt auf mindestens eines der angeforderten Statusbits.
 * @param[in] bus Gueltiger Zeiger auf die DAVE-I2C-APP.
 * @param[in] flags Bitmaske der erwarteten Hardwareereignisse.
 * @retval I2C_OK Ein angefordertes Ereignis ist eingetreten und wurde geloescht.
 * @retval I2C_ERROR NACK, Busfehler, Arbitration-Verlust oder Zeitlimit.
 * @details Polling bedeutet wiederholtes Nachsehen. Ein Fehler hat hier Vorrang
 * vor einer Erfolgsmeldung. Das Loeschen quittiert die gelesenen Statusbits.
 */
static I2C_RETURN_CODE_t tof_wait(I2C_MASTER_t *bus, uint32_t flags)
{
    uint32_t start = SysTickCounter;
    for (;;) {
        uint32_t status = XMC_I2C_CH_GetStatusFlag(bus->channel);
        if (status & TOF_I2C_ERRORS) { return I2C_ERROR; }
        if (status & flags) {
            XMC_I2C_CH_ClearStatusFlag(bus->channel, flags);
            return I2C_OK;
        }
        if ((uint32_t)(SysTickCounter - start) >= TOF_I2C_TIMEOUT_MS) {
            return I2C_ERROR;
        }
    }
}

/**
 * @brief Fuehrt einen vollstaendigen Register-Lese- oder Schreibzugriff aus.
 * @param[in,out] bus DAVE-Handle mit Hardwarekanal und Buszustand.
 * @param[in] address Unverschobene 7-Bit-Adresse des Sensors.
 * @param[in] reg Adresse des ersten Sensorregisters.
 * @param[in,out] data Puffer: beim Schreiben Quelle, beim Lesen Ziel.
 * @param[in] count Anzahl Datenbytes; mindestens 1 und hoechstens Puffergroesse.
 * @param[in] read true zum Lesen, false zum Schreiben.
 * @retval I2C_OK Alle Transferphasen einschliesslich STOP erfolgreich.
 * @retval I2C_ERROR Ungueltige Argumente, belegter Bus oder Transferfehler.
 * @details Auch beim Lesen wird zuerst die Registeradresse geschrieben.
 * Ein Repeated Start wechselt danach in Leserichtung, ohne den Bus freizugeben.
 * Nach dem letzten Lesebyte sendet der Master absichtlich NACK; dies ist das
 * normale Ende des Empfangs und kein vom Sensor empfangener Fehler.
 * @note Zeitlimits gelten pro Wartephase, nicht fuer den gesamten Aufruf.
 * Der Fehlerpfad versucht STOP; er kann eine elektrisch festgehaltene Leitung
 * nicht reparieren. Die separate OLED-Bibliothek verwendet diesen Port nicht.
 */
static I2C_RETURN_CODE_t tof_transfer(I2C_MASTER_t *bus, uint16_t address,
                                    uint8_t reg, uint8_t *data,
                                    uint32_t count, bool read)
{
    I2C_RETURN_CODE_t result;
    uint32_t nack_event;
    uint32_t start;
    if (!bus || !data || !count || address > 0x7fU ||
        bus->runtime->bus_acquired || bus->runtime->tx_busy || bus->runtime->rx_busy) {
        return I2C_ERROR;
    }
    /* Nur den NACK-Interrupt maskieren, damit er das Fehlerbit nicht vor uns
     * loescht. Den Timer weiterlaufen lassen: Er wird fuer Zeitlimits gebraucht. */
    nack_event = bus->channel->PCR_IICMode & XMC_I2C_CH_EVENT_NACK;
    XMC_I2C_CH_DisableEvent(bus->channel, XMC_I2C_CH_EVENT_NACK);
    start = SysTickCounter;
    while (!XMC_USIC_CH_TXFIFO_IsEmpty(bus->channel) ||
           XMC_USIC_CH_GetTransmitBufferStatus(bus->channel) == XMC_USIC_CH_TBUF_STATUS_BUSY) {
        if ((uint32_t)(SysTickCounter - start) >= TOF_I2C_TIMEOUT_MS) {
            result = I2C_ERROR;
            goto recover;
        }
    }
    XMC_USIC_CH_RXFIFO_Flush(bus->channel);
    XMC_I2C_CH_ClearStatusFlag(bus->channel, 0xffffffffU);
    bus->runtime->bus_acquired = true;
    /* << 1 schafft Platz fuer das Richtungsbit: aus 0x29 wird 0x52. */
    I2C_MASTER_SendStart(bus, (uint32_t)address << 1, XMC_I2C_CH_CMD_WRITE);
    result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
    if (result != I2C_OK) { goto recover; }
    I2C_MASTER_TransmitByte(bus, reg);
    result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
    if (result != I2C_OK) { goto recover; }
    if (read) {
        I2C_MASTER_SendRepeatedStart(bus, (uint32_t)address << 1, XMC_I2C_CH_CMD_READ);
        result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
        if (result != I2C_OK) { goto recover; }
    }
    for (uint32_t i = 0U; i < count; ++i) {
        if (read) {
            if (i + 1U == count) { I2C_MASTER_ReceiveNACK(bus); }
            else { I2C_MASTER_ReceiveACK(bus); }
            result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION |
                                   XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
            if (result != I2C_OK) { goto recover; }
            data[i] = I2C_MASTER_GetReceivedByte(bus);
        } else {
            I2C_MASTER_TransmitByte(bus, data[i]);
            result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
            if (result != I2C_OK) { goto recover; }
        }
    }
    I2C_MASTER_SendStop(bus);
    result = tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_STOP_CONDITION_RECEIVED);
    if (result == I2C_OK) { goto done; }
recover:
    /* goto recover buendelt die Aufraeumarbeit fuer alle Fehlerstellen.
     * Alte Sendebefehle verwerfen und STOP mit begrenzter Wartezeit versuchen. */
    XMC_USIC_CH_TXFIFO_Flush(bus->channel);
    XMC_I2C_CH_ClearStatusFlag(bus->channel, 0xffffffffU);
    I2C_MASTER_SendStop(bus);
    (void)tof_wait(bus, XMC_I2C_CH_STATUS_FLAG_STOP_CONDITION_RECEIVED);
    XMC_USIC_CH_RXFIFO_Flush(bus->channel);
done:
    /* Sowohl Erfolg als auch Fehler geben den Software-Buszustand wieder frei. */
    bus->runtime->bus_acquired = false;
    XMC_I2C_CH_ClearStatusFlag(bus->channel, 0xffffffffU);
    if (nack_event) { XMC_I2C_CH_EnableEvent(bus->channel, XMC_I2C_CH_EVENT_NACK); }
    return result;
}

/**
 * @brief Schreibt genau ein Byte in ein Sensorregister.
 * @param[in,out] bus DAVE-I2C-Handle.
 * @param[in] addr Unverschobene 7-Bit-Adresse.
 * @param[in] reg Zielregister.
 * @param[in] value Zu schreibender Bytewert.
 * @return I2C_OK bei Erfolg, sonst I2C_ERROR.
 */
static I2C_RETURN_CODE_t i2cSendByteToSlaveReg(I2C_MASTER_t *bus, uint16_t addr,
                                             uint8_t reg, uint8_t value)
{
    return tof_transfer(bus, addr, reg, &value, 1U, false);
}

/**
 * @brief Liest mehrere aufeinanderfolgende Sensorregister.
 * @param[in,out] bus DAVE-I2C-Handle.
 * @param[in] addr Unverschobene 7-Bit-Adresse.
 * @param[in] reg Erstes Register.
 * @param[out] data Zielpuffer mit mindestens count Bytes; nur bei Erfolg auswerten.
 * @param[in] count Anzahl zu lesender Bytes, groesser als 0.
 * @return I2C_OK bei Erfolg, sonst I2C_ERROR.
 */
static I2C_RETURN_CODE_t i2cBurstRegRead(I2C_MASTER_t *bus, uint16_t addr,
                                       uint8_t reg, uint8_t *data, uint32_t count)
{
    return tof_transfer(bus, addr, reg, data, count, true);
}

/**
 * @brief Liest ein Byte und legt es beim Aufrufer ab.
 * @param[in,out] bus DAVE-I2C-Handle.
 * @param[in] addr Unverschobene 7-Bit-Adresse.
 * @param[in] reg Quellregister.
 * @param[out] value Adresse eines Byte-Speicherplatzes; nur bei Erfolg auswerten.
 * @return I2C_OK bei Erfolg, sonst I2C_ERROR.
 */
static I2C_RETURN_CODE_t i2cReadByteFromSlaveReg(I2C_MASTER_t *bus, uint16_t addr,
                                              uint8_t reg, uint8_t *value)
{
    return i2cBurstRegRead(bus, addr, reg, value, 1U);
}

/**
 * @brief Schreibt einen 16-Bit-Wert als zwei Bytes, hoeheres Byte zuerst.
 * @param[in,out] bus DAVE-I2C-Handle.
 * @param[in] addr Unverschobene 7-Bit-Adresse.
 * @param[in] reg Adresse des hoeherwertigen Registers.
 * @param[in] value Zu uebertragender 16-Bit-Wert.
 * @return I2C_OK bei Erfolg, sonst I2C_ERROR.
 * @details Beispiel: 0x1234 wird in der Reihenfolge 0x12, 0x34 gesendet.
 * Dieses Format heisst Big Endian.
 */
static I2C_RETURN_CODE_t tof_write16(I2C_MASTER_t *bus, uint16_t addr,
                                    uint8_t reg, uint16_t value)
{
    /* >> 8 liefert das obere Byte; der uint8_t-Cast behaelt nur 8 Bits. */
    uint8_t bytes[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    return tof_transfer(bus, addr, reg, bytes, 2U, false);
}

/**
 * @brief Setzt zwei empfangene Bytes zu einem 16-Bit-Wert zusammen.
 * @param[in,out] bus DAVE-I2C-Handle.
 * @param[in] addr Unverschobene 7-Bit-Adresse.
 * @param[in] reg Adresse des hoeherwertigen Registers.
 * @param[out] value Gueltiger Ergebniszeiger; wird nur bei Erfolg beschrieben.
 * @return I2C_OK bei Erfolg, sonst I2C_ERROR.
 */
static I2C_RETURN_CODE_t tof_read16(I2C_MASTER_t *bus, uint16_t addr,
                                   uint8_t reg, uint16_t *value)
{
    uint8_t bytes[2];
    I2C_RETURN_CODE_t status = i2cBurstRegRead(bus, addr, reg, bytes, 2U);
    /* *value schreibt an die Adresse des Aufrufers; | verbindet die Bits. */
    if (status == I2C_OK) { *value = ((uint16_t)bytes[0] << 8) | bytes[1]; }
    return status;
}

/**
 * @brief Wartet blockierend anhand des Millisekundenzaehlers.
 * @param[in] milliseconds Wartezeit in Millisekunden.
 * @pre Timer und Interrupts muessen waehrend des Wartens aktiv bleiben.
 * @note Andere Aufgaben des Hauptprogramms werden in dieser Zeit nicht bearbeitet.
 */
static void delayms(uint16_t milliseconds)
{
    uint32_t start = SysTickCounter;
    while ((uint32_t)(SysTickCounter - start) < milliseconds) { }
}
