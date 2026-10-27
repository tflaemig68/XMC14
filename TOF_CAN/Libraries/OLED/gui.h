
/***************************************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
****************************************************************************************************/	
#ifndef __GUI_H__
#define __GUI_H__

#define	u8 unsigned char
#define	u16 unsigned int
#define	u32 unsigned long

void GUI_Fill(u8 sx,u8 sy,u8 ex,u8 ey,u8 color);
void GUI_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size,u8 mode);
void GUI_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 Size,u8 mode);
void GUI_ShowString(u8 x,u8 y,char *chr,u8 Char_Size,u8 mode);
void GUI_ShowFont16(u8 x,u8 y,char *s,u8 mode);
void GUI_ShowFont24(u8 x,u8 y,char *s,u8 mode);
void GUI_ShowFont32(u8 x,u8 y,char *s,u8 mode);
void GUI_ShowCHinese(u8 x,u8 y,u8 hsize,char *str,u8 mode);
void GUI_DrawBMP(u8 x,u8 y,u8 width, u8 height, char *BMP);

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Zustand eines einkanaligen OLED-Oszilloskops ohne Bildpuffer.
 * @details Ein Aufruf von GUI_ScopePlot() entspricht einem Zeitschritt/einer
 * Spalte. Die Zeitbasis bestimmt der Aufrufer. Nach width Werten beginnt der
 * Strahl wieder links; alte Spalten bleiben bis zum Ueberschreiben sichtbar.
 * Nur mit GUI_ScopeInit() initialisieren, Felder danach nicht veraendern.
 */
typedef struct {
    uint8_t x;          /**< Linke Pixelspalte. */
    uint8_t first_page; /**< Erste OLED-Seite (eine Seite umfasst 8 Pixel). */
    uint8_t pages;      /**< Hoehe in ganzen OLED-Seiten. */
    uint8_t width;      /**< Anzahl Messwerte pro Durchlauf, mindestens 2. */
    uint8_t next;       /**< Schreibposition relativ zum linken Rand. */
    uint16_t maximum;  /**< Oberer Y-Skalenwert; untere Grenze ist 0. */
    bool ready;        /**< true nach erfolgreicher Initialisierung. */
} GUI_Scope_t;

/**
 * @brief Loescht den Plotbereich und zeichnet ein einfaches Skalenraster.
 * @param[out] scope Gueltiger Zustandspuffer, darf nicht NULL sein.
 * @param[in] x Linke Pixelspalte.
 * @param[in] first_page Oberste Seite, keine Pixelkoordinate.
 * @param[in] width Breite in Pixeln (2 bis zur verbleibenden Displaybreite).
 * @param[in] pages Hoehe in Seiten (1 bis zur verbleibenden Displayhoehe).
 * @param[in] maximum Oberer Messwert der Y-Skala, muss groesser 0 sein.
 * @return true bei Erfolg; false bei ungueltigem Bereich, ohne Displayzugriff.
 * @note Belegt den Bereich exklusiv. Keine Textausgabe dort waehrend des Plots.
 * Nur im Hauptprogramm aufrufen, nachdem OLED_Init() ausgefuehrt wurde.
 */
bool GUI_ScopeInit(GUI_Scope_t *scope, uint8_t x, uint8_t first_page,
                   uint8_t width, uint8_t pages, uint16_t maximum);

/**
 * @brief Zeichnet genau einen Y-Wert und rueckt die Zeitachse eine Spalte weiter.
 * @param[in,out] scope Mit GUI_ScopeInit() initialisierter Zustand.
 * @param[in] value Messwert; oberhalb maximum wird am oberen Rand begrenzt.
 * @details 0 liegt unten, maximum oben. Ein Punkt je Messwert, keine Interpolation.
 * Die naechste Spalte wird als gestrichelte Schreibmarkierung geloescht.
 * Aufwand O(pages), unabhaengig von der Breite: zwei Datenbytes pro Seite,
 * keine Gleitkommazahlen, kein Vollbildtransfer, kein 1024-Byte-Bildpuffer.
 * Am rechten Rand werden die zwei getrennten Spalten einzeln adressiert.
 * NULL oder ein Zustand mit ready=false bewirkt keinen Displayzugriff.
 * @note Der OLED-Treiber arbeitet synchron; I2C-Fehler/Timeouts behandelt diese
 * Funktion nicht. Regelmaessige Aufrufe ergeben eine gleichmaessige Zeitachse.
 */
void GUI_ScopePlot(GUI_Scope_t *scope, uint16_t value);

#endif

