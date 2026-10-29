/* OLED-RAM-Ersatz: prueft echte GUI-Funktionen ohne Hardware. */
#include "Libraries/OLED/gui.h"
#include <stddef.h>
uint8_t screen[8][128];
static unsigned x_pos, page_pos, bytes, calls, invalid;
void OLED_Set_Pos(unsigned char x, unsigned char page)
{
    x_pos = x; page_pos = page;
    if (x >= 128 || page >= 8) { ++invalid; }
}
void OLED_WriteDataS(unsigned char *data, unsigned length)
{
    ++calls; bytes += length;
    for (unsigned i = 0; i < length; ++i) {
        if (page_pos >= 8 || x_pos >= 128) { ++invalid; }
        else { screen[page_pos][x_pos] = data[i]; }
        ++x_pos;
    }
}
#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
int scope_test(void)
{
    GUI_Scope_t scope = {0};
    GUI_ScopePlot(NULL, 10); GUI_ScopePlot(&scope, 10);
    CHECK(calls == 0);
    CHECK(!GUI_ScopeInit(NULL, 0, 1, 128, 6, 300));
    CHECK(!GUI_ScopeInit(&scope, 1, 1, 128, 6, 300));
    CHECK(!GUI_ScopeInit(&scope, 0, 7, 128, 2, 300));
    CHECK(!GUI_ScopeInit(&scope, 0, 1, 1, 6, 300));
    CHECK(!GUI_ScopeInit(&scope, 0, 1, 128, 0, 300));
    CHECK(!GUI_ScopeInit(&scope, 0, 1, 128, 6, 0));
    CHECK(calls == 0);
    CHECK(GUI_ScopeInit(&scope, 0, 1, 128, 6, 300));
    CHECK(bytes == 768 && calls == 6);
    bytes = calls = 0;
    GUI_ScopePlot(&scope, 0); /* Spalte 0 ist die Y-Achse. */
    CHECK(bytes == 12 && calls == 6 && scope.next == 1);
    GUI_ScopePlot(&scope, 300);
    CHECK(screen[1][1] == 1 && screen[6][1] == 128);
    CHECK(screen[2][1] == 0); /* Alte Cursorpixel entfernt. */
    GUI_ScopePlot(&scope, 65535);
    CHECK(screen[1][2] == 1); /* Begrenzung auf maximum. */
    GUI_ScopePlot(&scope, 150);
    CHECK(screen[4][3] == 1); /* Mitte: lokales Y=24. */
    for (unsigned i = 4; i < 127; ++i) { GUI_ScopePlot(&scope, 75); }
    bytes = calls = 0;
    GUI_ScopePlot(&scope, 150);
    CHECK(scope.next == 0 && bytes == 12 && calls == 12);
    GUI_ScopePlot(&scope, 100); CHECK(scope.next == 1);
    CHECK(screen[0][10] == 0 && screen[7][10] == 0); /* Textbereich geschuetzt. */
    CHECK(GUI_ScopeInit(&scope, 10, 2, 2, 1, 65535));
    GUI_ScopePlot(&scope, 0); GUI_ScopePlot(&scope, 65535);
    CHECK(scope.next == 0 && (screen[2][11] & 1));
    CHECK(!GUI_ScopeInit(&scope, 0, 0, 2, 1, 0));
    bytes = 0; GUI_ScopePlot(&scope, 100); CHECK(bytes == 0);
    /* Current layout: 16-pixel title and bottom caption must survive plotting. */
    for (unsigned x=0; x<128; ++x) {
        screen[0][x]=0xA5; screen[1][x]=0x5A; screen[7][x]=0xCC;
    }
    bytes=calls=0;
    CHECK(GUI_ScopeInit(&scope,0,2,128,5,300));
    CHECK(bytes==640 && calls==5);
    for (unsigned i=0; i<260; ++i) { GUI_ScopePlot(&scope,(uint16_t)(i*3)); }
    for (unsigned x=0; x<128; ++x) {
        CHECK(screen[0][x]==0xA5 && screen[1][x]==0x5A && screen[7][x]==0xCC);
    }
    CHECK(invalid == 0);
    return 0;
}
