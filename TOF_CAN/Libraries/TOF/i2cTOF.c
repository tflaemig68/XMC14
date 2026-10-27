/**
 * @file i2cTOF.c
 * @brief VL53L0X-Entfernungssensor: Implementierung fuer DAVE 4.
 * @author Linus Blanke, Christoph Lederbogen, Andreas Ladner, Philipp Roehlke
 * @author tflaemig68 (Erweiterungen der Vorlage)
 *
 * @details Ein TOF-Sensor ermittelt die Entfernung ueber die Laufzeit von Licht.
 * Die .h-Datei beschreibt die aufrufbaren Funktionen; die .c-Datei enthaelt
 * deren Anweisungen. I2C uebertraegt Registeradressen und Daten zwischen
 * Mikrocontroller (Master) und Sensor (Slave). Ein Register ist ein kleiner
 * Speicherplatz im Sensor, der einen Messwert oder eine Einstellung enthaelt.
 *
 * @par Voraussetzungen
 * DAVE_Init() und der Millisekundentimer muessen laufen. Sensorzeiger und
 * Ausgabepuffer muessen gueltig sein. Buszugriffe nur aus dem Hauptprogramm;
 * die Bibliothek verwendet gemeinsame interne Zustandsvariablen und ist
 * nicht fuer gleichzeitige Aufrufe mehrerer Sensoren ausgelegt.
 *
 * @see https://www.st.com/en/embedded-software/stsw-img005.html
 */
// standard includes
//#include <stdbool.h>
//#include <stdlib.h>

// includes from mcal and cmsis
//#include <stm32f4xx.h>
//#include <mcalI2C.h>

// inits from Balancer library
#include "i2cTOF.h"
#include "tof_dave_port.inc"
//#include <ST7735.h>


// Variables for I2C and TOF sensor address
/** @brief Adresse des zuletzt fuer einen Zugriff ausgewaehlten Sensors. */
TOF_ADDR_t TOF_address_used = TOF_ADDR_NONE; // TOF sensor address, initially set to NONE (-1)
/** @brief Gemeinsamer Buszeiger; parallele Bibliotheksaufrufe sind nicht vorgesehen. */
I2C_MASTER_t *TOF_i2c;  // Zeiger auf die DAVE-I2C-APP, normalerweise &I2C_MASTER_0

// TOF stop variable for register store value (used to store certain register values temporarily)
/** @brief Vom Sensor gelesener interner Wert fuer spaetere Startsequenzen. */
static uint8_t TOF_stop_variable = 0;

// Flag for continuous mode (Indicates if the sensor is in continuous measurement mode)
/** @brief Merkt sich den Dauerbetrieb fuer den einzigen verwendeten Sensor. */
bool TOF_continuous_mode = false;

// Math macros for sensor configuration and period decoding

// Calculate macro period based on VCSEL period (pulses per clock)
#define calcMacroPeriod(vcsel_period_pclks) (((uint32_t)(2304) * (vcsel_period_pclks) * 1655 + 500) / 1000)

// Decode VCSEL period based on register value
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)


//--------------------- SENSOR FUNCTIONS ------------------------

// Function to initialize the TOF sensor
// Initializes the sensor with given address, I2C interface, ranging profile, and measurement range.
/**
 * @brief Legt die Anfangswerte einer Sensorstruktur im RAM an.
 *
 * @details Es findet noch kein I2C-Zugriff statt. Der Zeiger verweist auf die Struktur
 * des Aufrufers; Aenderungen an sensor->feld bleiben daher nach dem Aufruf erhalten.
 *
 * @param[in,out] sensor Adresse der Sensorstruktur, die beschrieben wird.
 * @param[in] i2c_tof DAVE-I2C-Handle, in diesem Projekt &I2C_MASTER_0.
 * @param[in] TOF_address_used Unverschobene 7-Bit-Sensoradresse, normalerweise 0x29.
 * @param[in] Ranging_Profiles_t Gewuenschtes Profil aus der Aufzaehlung Ranging_Profiles_t.
 * @param[in] measuredRange Anfangswert fuer das Rohdistanzfeld, normalerweise 0.
 */
void initTOFSensorData(TOFSensor_t* sensor, I2C_MASTER_t *i2c_tof, uint16_t TOF_address_used, uint16_t Ranging_Profiles_t, uint16_t measuredRange)
{
	/* Die gesamte Struktur zuerst nullen, auch Flags und reservierte Zeiger. */
	*sensor = (TOFSensor_t){0};
	sensor->i2c_tof = i2c_tof;                    // DAVE-Handle speichern; die Hardware wurde bereits von DAVE_Init() eingerichtet
	sensor->TOF_address_used = TOF_address_used;  // Set the TOF sensor address
    sensor->Ranging_Profiles_t = Ranging_Profiles_t;  // Set the ranging profile (measurement mode)
    sensor->measuredRange = measuredRange;        // Set the maximum measurable range
    sensor->distanceFromTOF = 0;                  // Initialize the measured distance to zero
    sensor->enableTOFSensor = false;              // Default is to keep the sensor disabled
}

// Function to configure the TOF sensor
// Configures the sensor's ranging profile and whether the sensor should be enabled or disabled.
/**
 * @brief Speichert das Messprofil und den Freigabewunsch.
 *
 * @details Bei enable=true wird das Profil sofort in den Sensor geschrieben. Diese
 * void-Funktion liefert keinen Erfolgscode; main() verwendet deshalb
 * TOF_set_ranging_profile(). enable=false allein stoppt keine bereits laufende Messung.
 *
 * @param[in,out] sensor Adresse der Sensorstruktur, die beschrieben wird.
 * @param[in] Ranging_Profiles_t Gewuenschtes Profil aus der Aufzaehlung Ranging_Profiles_t.
 * @param[in] enable true zum Anwenden des Profils, false nur zum Zuruecksetzen des
 * Freigabefeldes.
 */
void configTOFSensor(TOFSensor_t* sensor, uint16_t Ranging_Profiles_t, bool enable) {
    sensor->Ranging_Profiles_t = Ranging_Profiles_t;  // Set the new ranging profile
    sensor->enableTOFSensor = enable;                  // Enable or disable the TOF sensor
    if(sensor->enableTOFSensor == true)
    {
    	TOF_set_ranging_profile(sensor);
    }
}


//---------------------INTERNAL FUNCTIONS---------------------


/**
 * @brief Konfiguriert die Meldung fuer einen neuen Messwert.
 *
 * @details Das Sensorregister meldet, wann Daten bereitstehen. Die Anwendung fragt
 * dieses Register ueber I2C ab; sie benoetigt dafuer keinen angeschlossenen
 * Sensor-Interruptpin.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_configure_interrupt(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	/* Interrupt on new sample ready */
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    if (i2c_return != I2C_OK)
    {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high[1];
    i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high, 1);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    gpio_hv_mux_active_high[0] &= ~0x10;
    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH,  gpio_hv_mux_active_high[0]);
    if (i2c_return != I2C_OK)
	{
		return false;
	}

    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	if (i2c_return != I2C_OK)
	{
		return false;
	}

    return true;
}


/**
 * @brief Prueft die Sensoridentitaet an der eingestellten I2C-Adresse.
 *
 * @details Liest die Modellkennung und vergleicht sie mit dem erwarteten VL53L0X-Wert.
 * Die Funktion sucht nicht den gesamten I2C-Bus ab.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_init_address(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	//activate GPIO if not already done
	//gpioActivate();

	//set i2c clock speed
	//i2cSetClkSpd(TOF_i2c, I2C_CLOCK_400);

	//check correct device by reading the ID
	uint8_t device_id[1];
	I2C_RETURN_CODE_t i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_IDENTIFICATION_MODEL_ID, device_id, 1);

	if (i2c_return != I2C_OK)
	{
		// returns false, if i2cBurstRegRead was not successful
		return false;
	}

	//returns true, if correct TOF sensor is connected, otherwise false
	return (device_id[0] == TOF_VL53L0X_EXPECTED_DEVICE_ID);
}


/**
 * @brief Setzt die grundlegenden Schnittstellen- und Versorgungseinstellungen.
 *
 * @details Die interne Stoppvariable wird aus dem Sensor gelesen und fuer spaetere
 * Messstarts gespeichert.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_data_init(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = false;

	/* Set 2v8 mode */
	uint8_t vhv_config_scl_sda = 0;
	success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	vhv_config_scl_sda |= 0x01;

	success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda);
	if (success != I2C_OK)
	{
		return false;
	}

	/* Set I2C standard mode */
	success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_I2C_MODE, 0x00);
	/* |= sammelt Fehlerbits: Ein frueherer Fehler bleibt auch bei spaeterem Erfolg erhalten. */
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	success |= i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, &TOF_stop_variable);
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);

	if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


/**
 * @brief Liest die werkseitigen Informationen ueber die Lichtempfaenger.
 *
 * @details SPADs sind die kleinen lichtempfindlichen Elemente des Sensors. NVM
 * bezeichnet seinen nichtfluechtigen Speicher. Die Bereitschaftsabfrage besitzt ein
 * Zeitlimit.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[out] count Adresse fuer die Anzahl der vom Werk vorgesehenen SPADs.
 * @param[out] type_is_aperture Adresse fuer die Kennzeichnung der SPAD-Bauart.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_get_spad_info_from_nvm(TOFSensor_t* TOFSENS, uint8_t * count, bool * type_is_aperture)
{
    uint32_t poll_start = SysTickCounter;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t tmp;

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00) != I2C_OK) { return false; }

	uint8_t data = 0;

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x06) != I2C_OK) { return false; }
	if (i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, &data) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, data | 0x04) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x07) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_NON, 0x01) != I2C_OK) { return false; }
	/// TF_July25 deleted by comment-> i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x01);

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01) != I2C_OK) { return false; }

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x94, 0x6b) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x00) != I2C_OK) { return false; }

	data = 0;
	I2C_RETURN_CODE_t i2c_return;

	do
		{
        if ((uint32_t)(SysTickCounter - poll_start) >= TOF_SENSOR_TIMEOUT_MS) { return false; }
			i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_INTERNAL_CONFIG_0x83, &data, 1);
		} while (i2c_return == I2C_OK && data == 0x00);
    if (i2c_return != I2C_OK) { return false; }

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, 0x01) != I2C_OK) { return false; }
	if (i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, 0x92, &tmp) != I2C_OK) { return false; }

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	data = 0;

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_NON, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x06) != I2C_OK) { return false; }
	if (i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, &data) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x83, data  & ~0x04) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01) != I2C_OK) { return false; }

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00) != I2C_OK) { return false; }

	return true;
}


/**
 * @brief Waehlt die verwendeten Lichtempfaenger anhand der Werksdaten aus.
 *
 * @details Die sechs Bytes der SPAD-Maske enthalten 48 einzelne Freigabebits. Ein
 * gesetztes Bit aktiviert das zugehoerige Empfangselement.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_spads_from_nvm(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t spad_count;
	bool spad_type_is_aperture;
	if (!TOF_get_spad_info_from_nvm(TOFSENS, &spad_count, &spad_type_is_aperture))
	{
		return false;
	}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	if (i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6) != I2C_OK) { return false; }

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4) != I2C_OK) { return false; }

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
		// This bit is lower than the first one that should be enabled, or
		// (reference_spad_count) bits have already been enabled, so zero this bit
		ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
		spads_enabled++;
		}
	}

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 1, ref_spad_map[1]) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 2, ref_spad_map[2]) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 3, ref_spad_map[3]) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 4, ref_spad_map[4]) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 5, ref_spad_map[5]) != I2C_OK) { return false; }

	return true;
}


/**
 * @brief Schreibt die vorgegebene Folge von Sensor-Grundeinstellungen.
 *
 * @details Die Reihenfolge und die Registerbankwechsel gehoeren zusammen. Einzelne
 * Zahlen sollten nicht ohne Kenntnis des Sensorprotokolls geaendert werden.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_load_default_tuning_settings(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_RANGE_CONFIG, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x10, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x11, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x24, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x25, TOF_DEFAULT_0xFF);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x75, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, TOF_DEFAULT_0x2C);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_ALGO_PHASECAL, TOF_DEFAULT_0x20);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_ALGO_PHASECAL, TOF_DEFAULT_0x09);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x54, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x31, TOF_DEFAULT_0x04);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, TOF_DEFAULT_0x03);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_BIN, TOF_DEFAULT_0x83);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, TOF_DEFAULT_0x25);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_CONTROL, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_MIN_SNR, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, TOF_DEFAULT_0x06);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, TOF_DEFAULT_0x96);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x08);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x30);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x65, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x66, TOF_DEFAULT_0xA0);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x22, TOF_DEFAULT_0x32);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x14);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x49, TOF_DEFAULT_0xFF);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4A, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x7A, TOF_DEFAULT_0x0A);  //!< TF_July25 data changed form 0xA0 to 0x0A
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x7B, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x78, TOF_DEFAULT_0x21);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x23, TOF_DEFAULT_0x34);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x42, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0xFF);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x45, TOF_DEFAULT_0x26);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, TOF_DEFAULT_0x05);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_HISTOGRAM_BIN, TOF_DEFAULT_0x40);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_THRESH_LOW, TOF_DEFAULT_0x06);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x20, TOF_DEFAULT_0x1A);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x43, TOF_DEFAULT_0x40);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x34, TOF_DEFAULT_0x03);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x35, TOF_DEFAULT_0x44);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x31, TOF_DEFAULT_0x04);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4B, TOF_DEFAULT_0x09);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4C, TOF_DEFAULT_0x05);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x4D, TOF_DEFAULT_0x04);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x45, TOF_DEFAULT_0x20);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, TOF_DEFAULT_0x08);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, TOF_DEFAULT_0x28);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_SNR, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, TOF_DEFAULT_0x04);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, TOF_DEFAULT_0xFE);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x76, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x77, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x0D, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, TOF_DEFAULT_0xF8);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_CONFIG_0x8E, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, TOF_DEFAULT_0x01);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, TOF_DEFAULT_0x00);
    success |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, TOF_DEFAULT_0x00);

    if (success != I2C_OK)
	{
		return false;
	}

	return true;
}


/**
 * @brief Aktiviert die angegebenen Schritte einer Entfernungsmessung.
 *
 * @details Eine Bitmaske fasst mehrere Ein/Aus-Einstellungen in einem Byte zusammen.
 * Jede TOF_RANGE_SEQUENCE_STEP-Konstante steht fuer ein bestimmtes Bit.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] sequence_step Bitmaske der freizugebenden Messschritte.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_sequence_steps_enabled(TOFSensor_t* TOFSENS, uint8_t sequence_step)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	bool result = false;

	I2C_RETURN_CODE_t success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);

	if(success == I2C_OK)
	{
		result = true;
	}

	return result;
}


/**
 * @brief Fuehrt einen einzelnen Referenzabgleich durch.
 *
 * @details Der Sensor wird gestartet und sein Bereitschaftsregister bis zum Ergebnis
 * oder Zeitlimit abgefragt. Danach wird die Ergebnismeldung quittiert.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] calib_type Art des Abgleichs: VHV oder Phase.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_perform_single_ref_calibration(TOFSensor_t* TOFSENS, TOF_calibration_type_t calib_type)
{
    uint32_t poll_start = SysTickCounter;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t success;

    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case TOF_CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case TOF_CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
    if (success != I2C_OK)
    {
        return false;
    }
//ToDo Abfrage ob die jeweiligen Initialisierungen richtig gesetzt wurden.
    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, sysrange_start);
    if (success != I2C_OK)
    {
        return false;
    }

    /* Wait for interrupt */


    uint8_t interrupt_status = 0;
    do {
        if ((uint32_t)(SysTickCounter - poll_start) >= TOF_SENSOR_TIMEOUT_MS) { return false; }		//Funktion in welcher der MCAL Fehler auftritt //ToDo i2cReadByteFromSlaveReg durch i2cBurstRegRead ersetzen

        //success = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
        success = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_RESULT_INTERRUPT_STATUS, &interrupt_status, 1);
    } while (success == I2C_OK && ((interrupt_status & 0x07) == 0));

    //} while (success == I2C_OK && ((interrupt_status & 0x07) == 0));
    if (success != I2C_OK)
    {
        return false;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (success != I2C_OK)
    {
        return false;
    }

    success = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
    if (success != I2C_OK)
    {
        return false;
    }

    return true;
}


/**
 * @brief Fuehrt beide Referenzabgleiche nacheinander aus.
 *
 * @details Zuerst erfolgt der VHV-Abgleich, anschliessend der Phasenabgleich. Zum
 * Schluss werden die normalen Messschritte wieder aktiviert.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV)) {
		return false;
	}

	if (!TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_PHASE)) {		//Funktion in welcher der MCAL Fehler auftritt
		return false;
	}


	/* Restore sequence steps enabled */

	if (!TOF_set_sequence_steps_enabled(TOFSENS, TOF_RANGE_SEQUENCE_STEP_DSS + TOF_RANGE_SEQUENCE_STEP_PRE_RANGE + TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

	return true;
}


/**
 * @brief Initialisiert Sensorregister, Lichtempfaenger und Kalibrierung.
 *
 * @details Bricht ab, sobald ein aufgerufener Initialisierungsschritt fehlschlaegt. Die
 * Identitaetspruefung erfolgt getrennt in TOF_init_address().
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_init_device(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if (!TOF_data_init(TOFSENS))
	{
		return false;
	}

	if (!TOF_set_spads_from_nvm(TOFSENS)) {
		return false;
	}

	if (!TOF_load_default_tuning_settings(TOFSENS))
	{
		return false;
	}

	if (!TOF_configure_interrupt(TOFSENS))
	{
		return false;
	}

	if (!TOF_set_sequence_steps_enabled(TOFSENS,
			TOF_RANGE_SEQUENCE_STEP_DSS +
			TOF_RANGE_SEQUENCE_STEP_PRE_RANGE +
			TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
		return false;
	}

    if (!TOF_perform_ref_calibration(TOFSENS))			//Funktion in welcher der MCAL Fehler auftritt
    {
        return false;
    }

	return true;
}


/**
 * @brief Wartet auf einen Messwert und liest die Entfernung.
 *
 * @details Zwei aufeinanderfolgende Bytes ergeben die Entfernung in Millimetern.
 * measuredRange speichert den Rohwert; range erhaelt bei 8190/8191 den Ersatzwert
 * TOF_VL53L0X_OUT_OF_RANGE. Die Bereitschaftsabfrage hat ein Zeitlimit.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[out] range Gueltiger Zeiger auf den Ergebnisplatz; nur bei Erfolg auswerten.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_getMeasurement(TOFSensor_t* TOFSENS, uint16_t *range)
{
    uint32_t poll_start = SysTickCounter;
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;
	uint8_t interrupt_status[1];
	do
	{
        if ((uint32_t)(SysTickCounter - poll_start) >= TOF_SENSOR_TIMEOUT_MS) { return false; }
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, interrupt_status, 1);
	} while (i2c_return == I2C_OK && ((interrupt_status[0] & 0x07) == 0));
	if (i2c_return != I2C_OK)
	{
		return false;
	}

	uint8_t readBuffer[2];
	i2c_return = i2cBurstRegRead(
			TOF_i2c, TOF_address_used,
			TOF_REG_RESULT_RANGE_STATUS + 10,
			readBuffer, 2);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}
	*range = (readBuffer[0] << 8) + readBuffer[1];

	/* Den Zahlenwert speichern, nicht die Adresse des lokalen Bytepuffers. */
	TOFSENS->measuredRange = *range;
	i2c_return = i2cSendByteToSlaveReg(
			TOF_i2c, TOF_address_used,
			TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	/* 8190 or 8191 may be returned when obstacle is out of range. */
	if (*range == 8190 || *range == 8191)
	{
		*range = TOF_VL53L0X_OUT_OF_RANGE;
	}

	return true;
}


//--------------------- EXTERNAL FUNCTIONS ---------------------


/**
 * @brief Prueft und initialisiert den VL53L0X.
 *
 * @details Typische Reihenfolge: initTOFSensorData(), TOF_init(),
 * TOF_set_ranging_profile(), TOF_start_continuous(). Diese Funktion startet noch keine
 * Dauermessung.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_init(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	// Init i2c address and check connectivity
	if (!TOF_init_address(TOFSENS))
	{
		return false;
	}

	//device initialization
	if (!TOF_init_device(TOFSENS))		//Funktion in welcher der MCAL Fehler auftritt
	{
		return false;
	}

	// return true, if everything was fine
	return true;
}


/**
 * @brief Startet wiederholte Messungen ohne weitere Startbefehle.
 *
 * @details Ranging_Profile_time bestimmt den Abstand in Millisekunden. Bei 0 startet
 * jeweils sofort die naechste Messung. Die Sensor-Taktkalibrierung wird bei
 * zeitgesteuertem Betrieb beruecksichtigt.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_start_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint32_t period_ms = TOFSENS->Ranging_Profile_time;
	I2C_RETURN_CODE_t i2c_return;

	TOF_address_used = TOFSENS->TOF_address_used;

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00) != I2C_OK) { return false; }

	if (period_ms != 0)
	{
		// continuous timed mode

		uint8_t readBuffer[2];

		i2c_return = i2cBurstRegRead(
				TOF_i2c, TOF_address_used,
				TOF_REG_OSC_CALIBRATE_VAL,
				readBuffer, 2);
		if (i2c_return != I2C_OK)
		{
			// returns false, if i2c communication was not successful
			return false;
		}

		uint16_t osc_calibrate_val = (readBuffer[0]<<8) + readBuffer[1];

		if (osc_calibrate_val != 0)
		{
			period_ms *= osc_calibrate_val;
		}

		uint8_t bytes[4];

		bytes[0] = (period_ms >> 24) & 0xFF;
		bytes[1] = (period_ms >> 16) & 0xFF;
		bytes[2] = (period_ms >> 8) & 0xFF;
		bytes[3] = period_ms & 0xFF;

		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD, bytes[0]) != I2C_OK) { return false; }
		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 1, bytes[1]) != I2C_OK) { return false; }
		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 2, bytes[2]) != I2C_OK) { return false; }
		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD + 3, bytes[3]) != I2C_OK) { return false; }

		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x04) != I2C_OK) { return false; } // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
		// continuous back-to-back mode
		if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x02) != I2C_OK) { return false; } // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}

	TOF_continuous_mode = true;

	return true;
}


/**
 * @brief Beendet den kontinuierlichen Messbetrieb.
 *
 * @details Sendet die Stoppsequenz und setzt die gemeinsame Zustandsvariable
 * TOF_continuous_mode zurueck.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_stop_continuous(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if(!TOF_continuous_mode)
	{
		return false;
	}

	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01) != I2C_OK) { return false; } // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, 0x00) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01) != I2C_OK) { return false; }
	if (i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00) != I2C_OK) { return false; }

	TOF_continuous_mode = false;

	return true;
}


/**
 * @brief Liest eine Entfernung im laufenden Dauerbetrieb.
 *
 * @details Das Ergebnis steht danach in distanceFromTOF. Intern kann die Funktion auf
 * einen neuen Messwert warten; sie ist kein sofort zurueckkehrender Interrupt-Handler.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_read_continuous_distance(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	if(!TOF_continuous_mode)
	{
		return false;
	}

	if(!TOF_getMeasurement(TOFSENS, &TOFSENS->distanceFromTOF))
	{
		return false;
	}

	return true;
}


/**
 * @brief Startet genau eine Messung und wartet auf das Ergebnis.
 *
 * @details Anders als im Dauerbetrieb muss fuer jeden Messwert ein neuer Startbefehl
 * gesendet werden. Die Entfernung wird in distanceFromTOF abgelegt.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_read_single_distance(TOFSensor_t* TOFSENS)
{
    uint32_t poll_start = SysTickCounter;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	if (i2c_return != I2C_OK) {
		return false;
	}


	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	uint8_t sysrange_start[1];
	do
	{
        if ((uint32_t)(SysTickCounter - poll_start) >= TOF_SENSOR_TIMEOUT_MS) { return false; }
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, sysrange_start, 1);
	} while (i2c_return == I2C_OK && (sysrange_start[0] & 0x01));
	if (i2c_return != I2C_OK)
	{
		return false;
	}
	TOF_getMeasurement(TOFSENS, &TOFSENS->distanceFromTOF);


	return true;
}


/**
 * @brief Startet die erste Einzelmessung fuer den abfragenden Taskbetrieb.
 *
 * @details Zusammen mit TOF_read_distance_task() kann das Hauptprogramm die Bereitschaft
 * wiederholt pruefen, statt durchgehend auf das Sensorergebnis zu warten.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_start_up_task(TOFSensor_t* TOFSENS)
{
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;	//TOF Adress from Struct TOF
	TOF_i2c = TOFSENS->i2c_tof;		//I2C Adress from Struct TOF

	//start new measurement without reading
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		return false;
	}
	return true;
}


/**
 * @brief Prueft einmal die Messbereitschaft und verarbeitet vorhandene Daten.
 *
 * @details Ist noch kein Ergebnis vorhanden, wird lediglich TOF_measuringage erhoeht.
 * Bei einem Ergebnis werden distanceFromTOF und measuredRange aktualisiert und die
 * naechste Einzelmessung gestartet. true bedeutet daher nicht zwingend, dass ein neuer
 * Messwert vorliegt.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_read_distance_task(TOFSensor_t* TOFSENS)
{
	I2C_RETURN_CODE_t i2c_return;
	TOF_address_used = TOFSENS->TOF_address_used;	//TOF Adress from Struct TOF
	TOF_i2c = TOFSENS->i2c_tof;		//I2C Adress from Struct TOF
	uint8_t interrupt_status[1];	//Variable for the Register Content
	uint16_t taskdistance;			//Variable to Store the read distance

	//check the ReadyData Flag
	i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_RESULT_INTERRUPT_STATUS, interrupt_status, 1);
	if (i2c_return != I2C_OK)
	{
		return false;
	}

	//readydata Flag high: read measurement result
	if(i2c_return == I2C_OK && ((interrupt_status[0] & 0x07) != 0))
	{
		uint8_t readBuffer[2];
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used, TOF_REG_RESULT_RANGE_STATUS + 10, readBuffer, 2);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		taskdistance = (readBuffer[0] << 8) + readBuffer[1];

		TOFSENS->measuredRange = taskdistance;

		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		if (taskdistance == 8190 || taskdistance == 8191)
		{
			taskdistance = TOF_VL53L0X_OUT_OF_RANGE;
		}


		TOFSENS->distanceFromTOF = taskdistance;


		//Successfull Measurement start new one
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
		if (i2c_return != I2C_OK)
		{
			return false;
		}

		TOFSENS->TOF_measuringage = 0; 		//reset measuring age
	}


	else		//readydata Flag LOW !
	{
		TOFSENS->TOF_measuringage ++;

	}
	return true;

}


/**
 * @brief Aendert die I2C-Adresse des Sensors und der Sensorstruktur.
 *
 * @details Die neue Adresse muss als gueltige 7-Bit-Adresse uebergeben werden. Sie darf
 * nicht bereits um ein Bit verschoben sein.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] new_Addr Neue, gueltige 7-Bit-Adresse; kein R/W-Bit enthalten.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_address(TOFSensor_t* TOFSENS, uint8_t new_Addr)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

    I2C_RETURN_CODE_t i2c_return;

    uint8_t newaddr = new_Addr;

    // Send the new address to the device
    i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, 0x8A, newaddr & 0x7F);

    if (i2c_return != I2C_OK) {
        return false; // Return false if the operation fails
    }
    TOFSENS->TOF_address_used = newaddr;
    return true;
}


/**
 * @brief Wartet eine vorgegebene Zeit und fuehrt eine Einzelmessung aus.
 *
 * @details Die Wartezeit blockiert das Hauptprogramm. Fuer regelmaessige Aufgaben
 * verwendet main() deshalb Zeitstempel statt dieser Funktion.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] time Zusaetzliche Wartezeit vor der Messung in Millisekunden.
 * @param[out] range Gueltiger Zeiger auf den Ergebnisplatz; nur bei Erfolg auswerten.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_read_distance_timed(TOFSensor_t* TOFSENS, uint16_t time, uint16_t *range)
{
    uint32_t poll_start = SysTickCounter;
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	delayms(time);

	I2C_RETURN_CODE_t i2c_return;

	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_1, TOF_stop_variable);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_INTERNAL_TUNING_2, 0x00);
	i2c_return |= i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
	if (i2c_return != I2C_OK) {
		return false;
	}


	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSRANGE_START, 0x01);
	if (i2c_return != I2C_OK)
	{
		// returns false, if i2c communication was not successful
		return false;
	}

	uint8_t sysrange_start[1];
	do
	{
        if ((uint32_t)(SysTickCounter - poll_start) >= TOF_SENSOR_TIMEOUT_MS) { return false; }
		i2c_return = i2cBurstRegRead(TOF_i2c, TOF_address_used,	TOF_REG_SYSRANGE_START,	sysrange_start, 1);
	}

	while (i2c_return == I2C_OK && (sysrange_start[0] & 0x01));
		if (i2c_return != I2C_OK)
		{
			return false;
		}

	TOF_getMeasurement(TOFSENS, range);

	return true;
}


/**
 * @brief Uebertraegt das in der Struktur gewaehlte Messprofil.
 *
 * @details Ein Messprofil legt Zeit- und gegebenenfalls Reichweiteneinstellungen fest.
 * Die Anwendung verwendet TOF_DEFAULT_MODE_D; weitere Profile der Vorlage sind nicht auf
 * Hardware verifiziert.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_ranging_profile(TOFSensor_t* TOFSENS)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	bool value = false;
	bool prevalue = false;
    switch (TOFSENS->Ranging_Profiles_t) {
    case TOF_DEFAULT_MODE_D:
    	if(TOF_set_measurement_timing_budget(TOFSENS, 30000) == true)
    	{
    		TOFSENS->Ranging_Profiles_t = TOF_DEFAULT_MODE_D;
    		TOFSENS->Ranging_Profile_time = 30;
    		value = true;
    		break;
    	}
    	else
    	{
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
        	value = TOF_RANGINGPROFILE_ERROR;
    		return false;
    		break;
    	}

    case TOF_HIGH_SPEED_MODE_S:
        if(TOF_set_measurement_timing_budget(TOFSENS, 20000) == true)
        {
        	TOFSENS->Ranging_Profiles_t = TOF_HIGH_SPEED_MODE_S;
        	TOFSENS->Ranging_Profile_time = 20;
        	value = true;
        	break;
        }
        else
        {
        	value = TOF_RANGINGPROFILE_ERROR;
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;

        	break;
        }

    case TOF_HIGH_ACCURACY_MODE_A:
        if(TOF_set_measurement_timing_budget(TOFSENS, 200000) == true)
        {
        	TOFSENS->Ranging_Profiles_t = TOF_HIGH_ACCURACY_MODE_A;
        	TOFSENS->Ranging_Profile_time = 200;
        	value = true;
        	break;
        }
        else
        {
        	value = TOF_RANGINGPROFILE_ERROR;
        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
        	break;
        }

    case TOF_LONG_RANGE_MODE_R:
    	if(TOF_set_measurement_timing_budget(TOFSENS, 33) == true)
    	        {
    	        	value = true;
    	        	break;
    	        }
    	        else
    	        {
    	        	value = TOF_RANGINGPROFILE_ERROR;
    	        	TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    	        	break;
    	        }

        if(TOF_set_signal_rate_limit(TOFSENS, 0.1) == true)
    	{
    		prevalue = true;
    		value = true;

    	}
    	else
    	{
    		prevalue = false;
        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    	if(TOF_set_vcsel_pulse_period(TOFSENS, VcselPeriodPreRange, 18) == true && prevalue == true)
    	{
    		prevalue = true;
    	}
    	else
    	{

        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    	if(TOF_set_vcsel_pulse_period(TOFSENS, VcselPeriodFinalRange, 14) == true && prevalue == true)
    	{
        	TOFSENS->Ranging_Profiles_t = TOF_LONG_RANGE_MODE_R;
        	TOFSENS->Ranging_Profile_time = 33000;

    		break;
    	}

    	else
    	{
    		prevalue = false;
        	value = TOF_RANGINGPROFILE_ERROR;
    		TOFSENS->Ranging_Profiles_t = TOF_RANGINGPROFILE_ERROR;
    		break;
    	}

    default:
        // Handle an invalid profile case
        return value;
    }

    return value;
}


/**
 * @brief Aendert die Pulsperiode der internen Laserquelle.
 *
 * @details VCSEL bezeichnet die Laserquelle des Sensors. Die Funktion passt
 * zusammenhaengende Zeitregister an und fuehrt einen Phasenabgleich aus. Diese
 * Einstellung ist fuer fortgeschrittene Anpassungen vorgesehen.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] type Auswahl zwischen Vormessung und abschliessender Messung.
 * @param[in] period_pclks Laser-Pulsperiode in PCLK-Takten; vormessen: 12/14/16/18,
 * final: 8/10/12/14.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type, uint8_t period_pclks)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
	I2C_RETURN_CODE_t i2c_return;

	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	// Get the current sequence step enables and timeouts from the sensor
	if (!TOF_get_sequence_step_enables(TOFSENS, &enables)) { return false; }
	if (!TOF_get_sequence_step_timeouts(TOFSENS, &enables, &timeouts)) { return false; }

	// Apply specific settings for the requested VCSEL period
	if (type == VcselPeriodPreRange)
	{
		// Set phase check limits based on the requested period
		switch (period_pclks)
		{
			case 12:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 16:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			case 18:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				if (i2c_return != I2C_OK) {
						return false;
					}
				break;
			default:
				return false;  // Invalid VCSEL period for pre-range
		}
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

		// Apply new VCSEL period for pre-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

		// Update timeouts for pre-range
		uint16_t new_pre_range_timeout_mclks = timeout_microseconds_to_mclks(timeouts.pre_range_us, period_pclks);
		new_pre_range_timeout_mclks = encode_timeOut(new_pre_range_timeout_mclks);
		i2c_return = tof_write16(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_mclks);

		// Update MSRC timeout
		uint16_t new_msrc_timeout_mclks = timeout_microseconds_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
	}
	else if (type == VcselPeriodFinalRange)
	{
		// Set phase check limits for final-range VCSEL period
		switch (period_pclks)
		{
			case 8:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 10:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 12:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			case 14:
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
				i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if (i2c_return != I2C_OK) {
					return false;
				}
				break;
			default:
				return false;  // Invalid VCSEL period for final-range
		}

		// Apply new VCSEL period for final-range
		i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if (i2c_return != I2C_OK) {
			return false;
		}
		// Update timeouts for final-range
		uint16_t new_final_range_timeout_mclks = timeout_microseconds_to_mclks(timeouts.final_range_us, period_pclks);
		if (enables.pre_range)
		{
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}
		new_final_range_timeout_mclks = encode_timeOut(new_final_range_timeout_mclks);
		i2c_return = tof_write16(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_mclks);
		if (i2c_return != I2C_OK) {
			return false;
		}
	}
	else
	{
		return false;  // Invalid type
	}

	// Re-apply the timing budget
	//setMeasurementTimingBudget(measurement_timing_budget_us);

	// Perform phase calibration if needed

	uint8_t sequence_config;
	i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config);
		if (i2c_return != I2C_OK) {
			return false; // Return false if the I2C read fails
		}
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (i2c_return != I2C_OK) {
		return false;
	}
	TOF_perform_single_ref_calibration(TOFSENS, TOF_CALIBRATION_TYPE_VHV);
	i2c_return = i2cSendByteToSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);
	if (i2c_return != I2C_OK) {
		return false;
	}
	return true;
}


/**
 * @brief Setzt die Mindeststaerke des auswertbaren Empfangssignals.
 *
 * @details Der Sensor erwartet einen 16-Bit-Festkommawert im Format Q9.7. Die
 * Multiplikation mit 128 wandelt den Dezimalwert in dieses Registerformat um.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] signalRateLimit Signalgrenze in Millionen Impulsen pro Sekunde (MCPS), 0
 * bis 511.99.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_signal_rate_limit(TOFSensor_t* TOFSENS, float signalRateLimit)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

	float limitMCPS = signalRateLimit;
	  if (limitMCPS < 0 || limitMCPS > 511.99) {
		  return false;
	  }

	  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	  i2c_return = tof_write16(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limitMCPS * (1 << 7));
	  if (i2c_return != I2C_OK) {
	          return false; // Return false if the I2C read fails
	      }
	  return true;

}


/**
 * @brief Liest die Freigabebits der einzelnen Messschritte.
 *
 * @details Bitverschiebung und UND mit 1 isolieren jeweils ein einzelnes Bit des
 * Konfigurationsregisters.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[out] enables Adresse der Struktur fuer die Freigabebits.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_get_sequence_step_enables(TOFSensor_t* TOFSENS, SequenceStepEnables *enables)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

    I2C_RETURN_CODE_t i2c_return;
    uint8_t sequence_config;

    // Read the byte from the TOF_REG_SYSTEM_SEQUENCE_CONFIG register
    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    // Check if the I2C read was successful
    if (i2c_return != I2C_OK)
    {
        return false;  // Return false if there was an error
    }

    // Extract bit values using shifts and masks
    enables->tcc         = (sequence_config >> 4) & 0x1;  // Extract the TCC bit
    enables->dss         = (sequence_config >> 3) & 0x1;  // Extract the DSS bit
    enables->msrc        = (sequence_config >> 2) & 0x1;  // Extract the MSRC bit
    enables->pre_range   = (sequence_config >> 6) & 0x1;  // Extract the PRE_RANGE bit
    enables->final_range = (sequence_config >> 7) & 0x1;  // Extract the FINAL_RANGE bit

    return true;  // Return true if everything succeeded
}


/**
 * @brief Liest und berechnet die Zeitwerte der Messschritte.
 *
 * @details Sensorregister verwenden interne Takte. Die Ergebnisstruktur enthaelt sowohl
 * diese Taktzahlen als auch die umgerechneten Mikrosekunden.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] enables Bereits gelesene Freigabebits; bestimmen die Zeitberechnung.
 * @param[out] timeouts Adresse der Struktur fuer die gelesenen und berechneten Zeitwerte.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_get_sequence_step_timeouts(TOFSensor_t* TOFSENS, SequenceStepEnables *enables, SequenceStepTimeouts *timeouts)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	uint8_t data;
	I2C_RETURN_CODE_t i2c_return;

    timeouts->pre_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodPreRange);
    if (timeouts->pre_range_vcsel_period_pclks == 0U || timeouts->pre_range_vcsel_period_pclks == 255U) { return false; }

    i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, &data);
    timeouts->msrc_dss_tcc_mclks = data;

	if (i2c_return != I2C_OK)
	{
		return false;
	}
    timeouts->msrc_dss_tcc_mclks += 1;
    timeouts->msrc_dss_tcc_us = timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    i2c_return = tof_read16(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &timeouts->pre_range_mclks);
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->pre_range_mclks = decode_timeout(timeouts->pre_range_mclks);
    timeouts->pre_range_us = timeout_mclks_to_microseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = TOF_get_vcsel_pulse_period(TOFSENS, VcselPeriodFinalRange);
    if (timeouts->final_range_vcsel_period_pclks == 0U || timeouts->final_range_vcsel_period_pclks == 255U) { return false; }


    i2c_return = tof_read16(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &timeouts->final_range_mclks);
    if (i2c_return != I2C_OK)
    	{
    		return false;
    	}
    timeouts->final_range_mclks = decode_timeout(timeouts->final_range_mclks);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeout_mclks_to_microseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

    return true;
}


/**
 * @brief Verteilt das verfuegbare Zeitbudget auf die Messschritte.
 *
 * @details Zuerst werden feste und bereits konfigurierte Zeiten abgezogen. Die
 * verbleibende Zeit wird als Timeout fuer den letzten Messschritt gespeichert. 30000
 * Mikrosekunden entsprechen 30 Millisekunden.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] budget_us Gewuenschtes Gesamtbudget in Mikrosekunden.
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
bool TOF_set_measurement_timing_budget(TOFSensor_t* TOFSENS, uint32_t budget_us)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t StartOverhead     = 1910;
    uint16_t EndOverhead       = 960;
    uint16_t MsrcOverhead      = 660;
    uint16_t TccOverhead       = 590;
    uint16_t DssOverhead       = 690;
    uint16_t PreRangeOverhead  = 660;
    uint16_t FinalRangeOverhead = 550;

    /* Overhead ist Zeit fuer interne Vorbereitung und Abschluss der Messung. */
    uint32_t used_budget_us = StartOverhead + EndOverhead;

    // Get sequence step enables and timeouts
    if (!TOF_get_sequence_step_enables(TOFSENS, &enables)) { return false; }
    if (!TOF_get_sequence_step_timeouts(TOFSENS, &enables, &timeouts)) { return false; }

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us)
        {
            // Requested timeout too big
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // Convert the final range timeout to MCLks
        uint32_t final_range_timeout_mclks =
        timeout_microseconds_to_mclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        // Write the final range timeout to the register

        final_range_timeout_mclks = encode_timeOut(final_range_timeout_mclks);
        i2c_return = tof_write16(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, final_range_timeout_mclks);
        if (i2c_return != I2C_OK){
            		return false;
            	}
        // Store the timing budget for internal reuse
        //uint32_t measurement_timing_budget_us = budget_us;
    }

    return true;
}


/**
 * @brief Liest die Laser-Pulsperiode und dekodiert den Registerwert.
 *
 * @details Die Umrechnung lautet (Registerwert + 1) * 2. Das Ergebnis ist eine Anzahl
 * interner Takte, keine Zeit in Millisekunden.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @param[in] type Auswahl zwischen Vormessung und abschliessender Messung.
 * @return Pulsperiode in PCLK-Takten; 0 bei I2C-Fehler, 255 bei ungueltiger Auswahl.
 */
uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type)
{
	TOF_address_used = TOFSENS->TOF_address_used;
	TOF_i2c = TOFSENS->i2c_tof;

	I2C_RETURN_CODE_t i2c_return;

    uint8_t vcsel_period = 255;  // Default to 255 (error case)

    if (type == VcselPeriodPreRange)
    {
        // Read the pre-range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
        vcsel_period = decodeVcselPeriod(vcsel_period);
    }
    else if (type == VcselPeriodFinalRange)
    {
        // Read the final range VCSEL period register and decode
        i2c_return = i2cReadByteFromSlaveReg(TOF_i2c, TOF_address_used, TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &vcsel_period);
    	if (i2c_return != I2C_OK){
    		return false;
    	}
    	vcsel_period = decodeVcselPeriod(vcsel_period);
    }

    return vcsel_period;
}


//--------------------- ADDITIONAL NON TOF FUNCTIONS ---------------------


/**
 * @brief Kodiert eine Taktanzahl fuer ein 16-Bit-Timeoutregister.
 *
 * @details Der Sensor speichert Mantisse und Exponent statt der Taktzahl direkt. Beim
 * Verschieben koennen niederwertige Bits entfallen; die Darstellung ist daher gerundet.
 *
 * @param[in] final_range_timeout_mclks Zu kodierende Anzahl interner Makrotakte.
 * @return Kodierter 16-Bit-Wert; bei Eingabe 0 ebenfalls 0.
 */
uint16_t encode_timeOut(uint16_t final_range_timeout_mclks)
{
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (final_range_timeout_mclks > 0){
	ls_byte = final_range_timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0){
	      ls_byte >>= 1;
	      ms_byte++;
	    }

	    return (ms_byte << 8) | (ls_byte & 0xFF);
	  }
	  else { return 0; }
}


/**
 * @brief Dekodiert den Sensor-Timeout in eine Taktanzahl.
 *
 * @details Das untere Byte wird um die im oberen Byte angegebene Bitzahl nach links
 * verschoben; anschliessend wird 1 addiert. Nur gueltige Sensor-Timeoutwerte verwenden.
 *
 * @param[in] reg_val Kodierter 16-Bit-Timeoutwert aus dem Sensor.
 * @return Dekodierte Anzahl der Makrotakte als 16-Bit-Wert.
 */
uint16_t decode_timeout(uint16_t reg_val)
{
    // Sensorformat: (unteres Byte * 2 hoch oberes Byte) + 1.
    // reg_val is a 16-bit value; the MSByte (Most Significant Byte) is the upper 8 bits
    // and the LSByte (Least Significant Byte) is the lower 8 bits.

    uint8_t msb = (reg_val >> 8) & 0xFF;  // Extract the most significant byte
    uint8_t lsb = reg_val & 0xFF;         // Extract the least significant byte

    // Calculate the timeout as per the formula
    uint16_t timeout = (lsb << msb) + 1;

    return timeout;
}


/**
 * @brief Rechnet Sensortakte in Mikrosekunden um.
 *
 * @details Die Dauer eines Makrotaktes haengt von der Laser-Pulsperiode ab. Der Faktor
 * 1000 wandelt Nanosekunden in Mikrosekunden um.
 *
 * @param[in] timeout_period_mclks Anzahl interner Makrotakte.
 * @param[in] vcsel_period_pclks Gueltige Laser-Pulsperiode in PCLK-Takten, groesser als 0.
 * @return Berechnete Zeitspanne in Mikrosekunden.
 */
uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert the timeout period in MCLKs to microseconds

    return (uint32_t)((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}


/**
 * @brief Rechnet Mikrosekunden in interne Sensortakte um.
 *
 * @details Die ganzzahlige Rechnung rundet auf eine passende Anzahl Makrotakte.
 * vcsel_period_pclks muss groesser als 0 sein, damit keine Division durch 0 erfolgt.
 *
 * @param[in] timeout_period_us Zeitspanne in Mikrosekunden.
 * @param[in] vcsel_period_pclks Gueltige Laser-Pulsperiode in PCLK-Takten, groesser als 0.
 * @return Berechnete Anzahl der Makrotakte.
 */
uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    // Calculate the macro period in nanoseconds
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    // Convert timeout from microseconds to MCLKs
    // The formula uses rounding by adding (macro_period_ns / 2) before dividing
    uint32_t return_value = (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    return return_value;
}
