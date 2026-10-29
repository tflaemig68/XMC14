/**
 * @file i2cTOF.h
 * @brief VL53L0X-Entfernungssensor: Schnittstelle und Datentypen.
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
#ifndef I2CTOF_H
#define I2CTOF_H

// include standard libraries
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "DAVE.h"

// Register defines for communication with the TOF sensor (according to the API)
#define TOF_REG_SYSRANGE_START                               (0x00)  ///< Trigger start of range measurement
#define TOF_REG_SYSTEM_SEQUENCE_CONFIG                       (0x01)  ///< Sequence configuration register
#define TOF_REG_SYSTEM_INTERMEASUREMENT_PERIOD               (0x04)  ///< Inter-measurement period for the system
#define TOF_REG_SYSTEM_RANGE_CONFIG							 (0x09)	 ///< System range config register
#define TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                 (0x0A)  ///< System interrupt config GPIO register
#define TOF_REG_SYSTEM_INTERRUPT_CLEAR                       (0x0B)  ///< System interrupt clear register
#define TOF_REG_SYSTEM_THRESH_HIGH							 (0x0C)	 ///< high tresh register
#define TOF_REG_INTERNAL_CONFIG_0x0D						 (0x0D)	 ///< Not officially documented
#define TOF_REG_SYSTEM_THRESH_LOW							 (0x0E)	 ///< low tresh register
#define TOF_REG_INTERNAL_CONFIG_0x10						 (0x10)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x11						 (0x11)	 ///< Not officially documented
#define TOF_REG_RESULT_INTERRUPT_STATUS                      (0x13)  ///< Interrupt status for results
#define TOF_REG_RESULT_RANGE_STATUS                          (0x14)  ///< Range status result register
#define TOF_REG_INTERNAL_CONFIG_0x20						 (0x20)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x22						 (0x22)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x23						 (0x23)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x24						 (0x24)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x25						 (0x25)	 ///< Not officially documented
#define TOF_REG_PRE_RANGE_CONFIG_MIN_SNR					 (0x27)	 ///< Minimum pre range config reg
#define TOF_REG_ALGO_PHASECAL								 (0x30)	 ///< Algo phasecal register
#define TOF_REG_INTERNAL_CONFIG_0x31						 (0x31)	 ///< Not officially documented
#define TOF_REG_GLOBAL_CONFIG_VCSEL_WIDTH                    (0x32)  ///< VCSEL width configuration for global settings
#define TOF_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT		 (0x33)	 ///< Initial phase select register
#define TOF_REG_INTERNAL_CONFIG_0x34						 (0x34)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x35						 (0x35)	 ///< Not officially documented
#define TOF_REG_SYSTEM_HISTOGRAM_BIN						 (0x40)	 ///< System histogram register 1
#define TOF_REG_INTERNAL_CONFIG_0x42						 (0x42)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x43						 (0x43)	 ///< Not officially documented
#define TOF_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  (0x44)  ///< Final range config for count rate return limit
#define TOF_REG_INTERNAL_CONFIG_0x45						 (0x45)	 ///< Not officially documented
#define TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP                   (0x46)  ///< MSRC timeout macrop register
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW           (0x47)  ///< Final range valid phase low register
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH          (0x48)  ///< Final range valid phase high register
#define TOF_REG_INTERNAL_CONFIG_0x49						 (0x49)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4A						 (0x4A)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4B						 (0x4B)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4C						 (0x4C)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x4D						 (0x4D)	 ///< Not officially documented
#define TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          (0x4E)  ///< Dynamic SPAD requested reference count register
#define TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET             (0x4F)  ///< Dynamic SPAD reference enable start offset register
#define TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD                (0x50)  ///< Pre-range VCSEL period register
#define TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI           (0x51)  ///< High byte for pre-range timeout
#define TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO			 (0x52)	 ///< Low byte for pre-range timeout
#define TOF_REG_INTERNAL_CONFIG_0x54						 (0x54)	 ///< Not officially documented
#define TOF_REG_HISTOGRAM_CONFIG_READOUT_CTRL				 (0x55)	 ///< Readout CTRL register
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW             (0x56)  ///< Pre-range valid phase low register
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH            (0x57)  ///< Pre-range valid phase high register
#define TOF_REG_MSRC_CONFIG_CONTROL                          (0x60)  ///< MSRC (Minimum Signal Rate Check) control register
#define TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI			 (0x61)	 ///< High sigma tresh register
#define TOF_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO			 (0x62)	 ///< Low sigma tresh register
#define TOF_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT			 (0x64)	 ///< Minimum count rate register
#define TOF_REG_INTERNAL_CONFIG_0x65						 (0x65)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x66						 (0x66)	 ///< Not officially documented
#define TOF_REG_FINAL_RANGE_CONFIG_MIN_SNR					 (0x67)	 ///< Final-range minimum SNR register
#define TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD              (0x70)  ///< Final-range VCSEL period register
#define TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         (0x71)  ///< High byte for final range timeout
#define TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO         (0x72)  ///< LOW byte for final range timeout
#define TOF_REG_INTERNAL_CONFIG_0x75						 (0x75)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x76						 (0x76)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x77						 (0x77)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x78						 (0x78)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x7A						 (0x7A)	 ///< Not officially documented
#define TOF_REG_INTERNAL_CONFIG_0x7B						 (0x7B)	 ///< Not officially documented
#define TOF_REG_POWER_MANAGEMENT_GO1_POWER_FORCE             (0x80)  ///< Power management register
#define TOF_REG_SYSTEM_HISTOGRAM_NON						 (0x81)	 ///< System Histogram register 2
#define TOF_REG_INTERNAL_CONFIG_0x83						 (0x83)	 ///< Not officially documented
#define TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH                      (0x84)  ///< GPIO HV MUX active high configuration register
#define TOF_REG_I2C_MODE                                     (0x88)  ///< NOT officially documented
#define TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV             (0x89)  ///< VHV (Voltage High Voltage) configuration register
#define TOF_REG_SLAVE_DEVICE_ADDRESS                         (0x8A)  ///< Slave device address register
#define TOF_REG_INTERNAL_CONFIG_0x8E						 (0x8E)	 ///< Not officially documented
#define TOF_REG_INTERNAL_TUNING_1                            (0x91)  ///< Internal tuning register 1
#define TOF_REG_INTERNAL_CONFIG_0x94						 (0x94)	 ///< Not officially documented
#define TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0             (0xB0)  ///< Global SPAD enables reference register
#define TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT            (0xB6)  ///< Global config for reference enable start selection
#define TOF_REG_IDENTIFICATION_MODEL_ID                      (0xC0)  ///< Get Device ID (Model ID)
#define TOF_REG_OSC_CALIBRATE_VAL                            (0xF8)  ///< Oscillator calibration value register
#define TOF_REG_INTERNAL_TUNING_2                            (0xFF)  // Internal tuning register 2


//Configuration Constants
// Range Out-of-Range Value
#define TOF_VL53L0X_OUT_OF_RANGE                             (8190)  // Value indicating out-of-range result

// Range Sequence Step Registers (Step identifiers for different ranges)
#define TOF_RANGE_SEQUENCE_STEP_TCC                          (0x10)  ///< Target Center Check step (TCC)
#define TOF_RANGE_SEQUENCE_STEP_MSRC                         (0x04)  ///< Minimum Signal Rate Check (MSRC)
#define TOF_RANGE_SEQUENCE_STEP_DSS                          (0x28)  ///< Dynamic SPAD selection (DSS)
#define TOF_RANGE_SEQUENCE_STEP_PRE_RANGE                    (0x40)  ///< Pre-range step
#define TOF_RANGE_SEQUENCE_STEP_FINAL_RANGE                  (0x80)  // Final range step

// Identification Registers
#define TOF_VL53L0X_EXPECTED_DEVICE_ID                       (0xEE)  ///< Expected device ID for VL53L0X
#define TOF_VL53L0X_DEFAULT_ADDRESS                          (0x29)  // Default I2C address for VL53L0X sensor

// Helper macro for encoding VCSEL period
#define encodeVcselPeriod(period_pclks)                      (((period_pclks) >> 1) - 1)  // Encodes VCSEL period in PCLKs


// Enum defining implemented TOF Sensors and their addresses (currently only VL53LOX)
typedef enum
{
	TOF_ADDR_NONE       = -1,  ///< No address
	TOF_ADDR_VL53LOX    = 0x29, ///< VL53LOX sensor address
	TOF_REGULAR_ADDRESS = 0x54	///< Unbenutzter Altwert der Vorlage; fuer diesen Sensor TOF_ADDR_VL53LOX verwenden.
} TOF_ADDR_t;

// Enum for calibration phases
typedef enum
{
	TOF_CALIBRATION_TYPE_VHV   = 0,  ///< VHV calibration phase
	TOF_CALIBRATION_TYPE_PHASE = 1   ///< Phase calibration phase
} TOF_calibration_type_t;

// Enum defining Ranging Profiles for the TOF sensor
typedef enum
{
	TOF_DEFAULT_MODE_D         = 1, ///< Default mode
	TOF_HIGH_SPEED_MODE_S      = 2, ///< High speed mode
	TOF_HIGH_ACCURACY_MODE_A   = 3, ///< High accuracy mode
	TOF_LONG_RANGE_MODE_R      = 4, ///< Long range mode
	TOF_RANGINGPROFILE_ERROR   = 5  ///< Error state
} Ranging_Profiles_t;

// Enum for VCSEL Period Types (Pre-range and Final-range)
typedef enum
{
	VcselPeriodPreRange  = 0, ///< Pre-range VCSEL period
	VcselPeriodFinalRange = 1  ///< Final-range VCSEL period
} vcselPeriodType;

// Enum for Default tuning setting
typedef enum
{
	TOF_DEFAULT_0x00	= 0x00,
	TOF_DEFAULT_0x01	= 0x01,
	TOF_DEFAULT_0x03	= 0x03,
	TOF_DEFAULT_0x04	= 0x04,
	TOF_DEFAULT_0x05	= 0x05,
	TOF_DEFAULT_0x06	= 0x06,
	TOF_DEFAULT_0x08	= 0x08,
	TOF_DEFAULT_0x09	= 0x09,
	TOF_DEFAULT_0x0A	= 0x0A,
	TOF_DEFAULT_0x14	= 0x14,
	TOF_DEFAULT_0x1A	= 0x1A,
	TOF_DEFAULT_0x20	= 0x20,
	TOF_DEFAULT_0x21	= 0x21,
	TOF_DEFAULT_0x25	= 0x25,
	TOF_DEFAULT_0x26	= 0x26,
	TOF_DEFAULT_0x28	= 0x28,
	TOF_DEFAULT_0x2C	= 0x2C,
	TOF_DEFAULT_0x30	= 0x30,
	TOF_DEFAULT_0x32	= 0x32,
	TOF_DEFAULT_0x34	= 0x34,
	TOF_DEFAULT_0x40	= 0x40,
	TOF_DEFAULT_0x44	= 0x44,
	TOF_DEFAULT_0x83	= 0x83,
	TOF_DEFAULT_0x96	= 0x96,
	TOF_DEFAULT_0xA0	= 0xA0,
	TOF_DEFAULT_0xF8	= 0xF8,
	TOF_DEFAULT_0xFE	= 0xFE,
	TOF_DEFAULT_0xFF	= 0xFF

} defaulttuningsetting;


// Struct for SequenceStepTimeouts, containing timing values for different steps in the sensor's sequence
typedef struct
{
	uint8_t pre_range_vcsel_period_pclks;  ///< Pre-range VCSEL period in PCLKs
	uint16_t msrc_dss_tcc_mclks;           ///< MSRC/DSS/TCC time in MCLKs
	uint32_t msrc_dss_tcc_us;              ///< MSRC/DSS/TCC time in microseconds
	uint16_t pre_range_mclks;              ///< Pre-range time in MCLKs
	uint32_t pre_range_us;                 ///< Pre-range time in microseconds
	uint8_t final_range_vcsel_period_pclks; ///< Final-range VCSEL period in PCLKs
	uint16_t final_range_mclks;            ///< Final-range time in MCLKs
	uint32_t final_range_us;               ///< Final-range time in microseconds
} SequenceStepTimeouts;

// Struct for SequenceStepEnables, containing flags for enabling/disabling different sequence steps
typedef struct
{
	uint8_t tcc;         ///< TCC step enabled (1 for enabled, 0 for disabled)
	uint8_t dss;         ///< DSS step enabled (1 for enabled, 0 for disabled)
	uint8_t msrc;        ///< MSRC step enabled (1 for enabled, 0 for disabled)
	uint8_t pre_range;   ///< Pre-range step enabled (1 for enabled, 0 for disabled)
	uint8_t final_range; ///< Final range step enabled (1 for enabled, 0 for disabled)
} SequenceStepEnables;


// --------------------- SENSOR STRUCTURE AND FUNCTIONS ------------------------


/** @brief Kurzname fuer struct TOFSensor, auch vor deren Definition verwendbar. */
typedef struct TOFSensor TOFSensor_t;

/**
 * @brief Buendelt Bus, Einstellungen und Ergebnis eines Sensors.
 * @details Eine Struktur fasst zusammengehoerige Variablen zusammen.
 * Mit &TOF_Sensor_1 wird ihre Adresse uebergeben; sensor->distanceFromTOF
 * greift ueber einen Zeiger auf ein einzelnes Feld zu.
 */
struct TOFSensor {
	I2C_MASTER_t *i2c_tof; ///< DAVE-Handle: verbindet die Sensorstruktur mit dem verwendeten I2C-Bus.
	uint16_t TOF_address_used; ///< 7-Bit-Adresse ohne R/W-Bit, beim VL53L0X anfangs 0x29.
	uint16_t Ranging_Profiles_t; ///< Ausgewaehltes Messprofil; der Feldname entspricht dem Namen des Enum-Typs.
	uint16_t distanceFromTOF; ///< Letzte Entfernung in mm oder TOF_VL53L0X_OUT_OF_RANGE.
	uint32_t measuredRange; ///< Numerischer Rohmesswert vor der Behandlung von Reichweitenfehlern.
	bool enableTOFSensor; ///< Freigabewunsch der Anwendung; kein automatischer Hardware-Ein/Aus-Schalter.
	uint32_t Ranging_Profile_time; ///< Messabstand in ms; 0 bedeutet Messungen direkt nacheinander.
	uint8_t TOF_readyFlag; ///< Reserviertes Bereitschaftsfeld der Vorlage; aktuell nicht ausgewertet.
	uint8_t TOF_measuringage; 		   // Age of the measured distance

	// Function pointers for initializing, configuring, and retrieving measurements from the TOF sensor
	void (*initialize)(TOFSensor_t*, I2C_MASTER_t*, uint16_t, uint16_t, uint16_t); ///< Reservierter Funktionszeiger, nach Initialisierung NULL; nicht ungeprueft aufrufen.
	void (*configure)(TOFSensor_t*, uint16_t, bool); ///< Reservierter Funktionszeiger, nach Initialisierung NULL; direkte API verwenden.
	uint16_t (*getMeasurement)(TOFSensor_t*); ///< Reservierter Funktionszeiger, nach Initialisierung NULL; direkte API verwenden.
};

// Function prototypes for initializing and configuring the TOF sensor
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
extern void initTOFSensorData(TOFSensor_t* sensor, I2C_MASTER_t *i2c_tof, uint16_t TOF_address_used, uint16_t Ranging_Profiles_t, uint16_t measuredRange);
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
extern void configTOFSensor(TOFSensor_t* sensor, uint16_t Ranging_Profiles_t, bool enable);


//--------------------- INTERNAL FUNCTIONS ---------------------


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
extern bool TOF_configure_interrupt(TOFSensor_t* TOFSENS);


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
extern bool TOF_init_address(TOFSensor_t* TOFSENS);


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
extern bool TOF_data_init(TOFSensor_t* TOFSENS);


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
extern bool TOF_get_spad_info_from_nvm(TOFSensor_t* TOFSENS, uint8_t * count, bool * type_is_aperture);


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
extern bool TOF_set_spads_from_nvm(TOFSensor_t* TOFSENS);


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
extern bool TOF_load_default_tuning_settings(TOFSensor_t* TOFSENS);


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
extern bool TOF_set_sequence_steps_enabled(TOFSensor_t* TOFSENS, uint8_t sequence_step);


              
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
extern bool TOF_perform_single_ref_calibration(TOFSensor_t* TOFSENS, TOF_calibration_type_t calib_type);


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
extern bool TOF_perform_ref_calibration(TOFSensor_t* TOFSENS);


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
extern bool TOF_init_device(TOFSensor_t* TOFSENS);


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
extern bool TOF_getMeasurement(TOFSensor_t* TOFSENS, uint16_t *range);


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
extern bool TOF_init(TOFSensor_t* TOFSENS);


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
extern bool TOF_start_continuous(TOFSensor_t* TOFSENS);


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
extern bool TOF_stop_continuous(TOFSensor_t* TOFSENS);


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
extern bool TOF_read_continuous_distance(TOFSensor_t* TOFSENS);


/**
 * @brief Startet genau eine Messung und wartet auf das Ergebnis.
 *
 * @details Anders als im Dauerbetrieb muss fuer jeden Messwert ein neuer Startbefehl
 * gesendet werden. Die Entfernung wird in distanceFromTOF abgelegt.
 *
 * @par Parametername im Header
 * TOFSensor bezeichnet den Sensorzeiger (in der Implementierung TOFSENS).
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
extern bool TOF_read_single_distance(TOFSensor_t* TOFSensor);


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
extern bool TOF_start_up_task(TOFSensor_t* TOFSENS);


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
extern bool TOF_read_distance_task(TOFSensor_t* TOFSENS);


/**
 * @brief Aendert die I2C-Adresse des Sensors und der Sensorstruktur.
 *
 * @details Die neue Adresse muss als gueltige 7-Bit-Adresse uebergeben werden. Sie darf
 * nicht bereits um ein Bit verschoben sein.
 *
 * @param[in,out] TOFSENS Gueltiger Zeiger auf die zuvor angelegte Sensorstruktur.
 * @par Parametername im Header
 * newAddr bezeichnet die neue 7-Bit-Adresse (in der Implementierung new_Addr).
 * @retval true Funktion erfolgreich abgeschlossen.
 * @retval false Fehler bei Kommunikation, Voraussetzung oder Konfiguration.
 */
extern bool TOF_set_address(TOFSensor_t* TOFSENS, uint8_t newAddr);


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
extern bool TOF_read_distance_timed(TOFSensor_t* TOFSENS, uint16_t time, uint16_t *range);


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
extern bool TOF_set_ranging_profile(TOFSensor_t* TOFSENS);


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
extern bool TOF_set_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type, uint8_t period_pclks);


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
extern bool TOF_set_signal_rate_limit(TOFSensor_t* TOFSENS, float signalRateLimit);


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
extern bool TOF_get_sequence_step_enables(TOFSensor_t* TOFSENS, SequenceStepEnables *enables);


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
extern bool TOF_get_sequence_step_timeouts(TOFSensor_t* TOFSENS, SequenceStepEnables *enables, SequenceStepTimeouts *timeouts);


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
extern uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type);


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
extern bool TOF_set_measurement_timing_budget(TOFSensor_t* TOFSENS, uint32_t budget_us);


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
extern uint8_t TOF_get_vcsel_pulse_period(TOFSensor_t* TOFSENS, vcselPeriodType type);


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
extern uint16_t encode_timeOut(uint16_t final_range_timeout_mclks);


/**
 * @brief Dekodiert den Sensor-Timeout in eine Taktanzahl.
 *
 * @details Das untere Byte wird um die im oberen Byte angegebene Bitzahl nach links
 * verschoben; anschliessend wird 1 addiert. Nur gueltige Sensor-Timeoutwerte verwenden.
 *
 * @param[in] reg_val Kodierter 16-Bit-Timeoutwert aus dem Sensor.
 * @return Dekodierte Anzahl der Makrotakte als 16-Bit-Wert.
 */
extern uint16_t decode_timeout(uint16_t reg_val);


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
extern uint32_t timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);


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
extern uint32_t timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

#endif /* SENSORTOF_H */
