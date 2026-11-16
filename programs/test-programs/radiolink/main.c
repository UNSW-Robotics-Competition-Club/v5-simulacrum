/*
 * Minimal de-risk: can a tester brain bring up the radio link to a second brain
 * over generic serial? Poses as a type-8 radio on port 1; reports counts on the
 * user serial port.
 */
#include "v5_sdk.h"

#define PORT_INDEX 0
#define TICK_MS 15

unsigned int strlen(const char *s) { unsigned int n = 0; while (s[n]) n++; return n; }
void *memset(void *d, int c, unsigned int n)
{ unsigned char *p = d; while (n--) *p++ = (unsigned char)c; return d; }
void *memcpy(void *d, const void *s, unsigned int n)
{ unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }

static uint16_t crc16(const uint8_t *d, int n)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

static void wait_ms(uint32_t ms)
{
    uint32_t end = vexSystemTimeGet() + ms;
    while (vexSystemTimeGet() < end) vexTasksRun();
}

static void say_counts(int total, int a85, int a80, int a13)
{
    char b[80]; int p = 0; static const char H[] = "0123456789";
    const char *t = "rx= a85= a80= 13= ";
    int vals[4] = {total, a85, a80, a13};
    const char *labs[4] = {"rx=", "a85=", "a80=", "13="};
    (void)t;
    for (int k = 0; k < 4; k++) {
        const char *L = labs[k];
        for (int i = 0; L[i]; i++) b[p++] = L[i];
        int v = vals[k] < 0 ? 0 : vals[k], d = 1;
        while (d * 10 <= v) d *= 10;
        while (d >= 1) { b[p++] = H[(v / d) % 10]; d /= 10; }
        b[p++] = ' ';
    }
    b[p++] = '\n';
    vexSerialWriteBuffer(1, (uint8_t *)b, p);
}

static const uint8_t INFO[14]  = {0x0A, 0x30, 0, 0, 0x08, 0, 0, 0x40,
                                  0x78, 0x56, 0x34, 0x12, 0x01, 0};
static const uint8_t READY[6]  = {0x13, 0x20, 0x78, 0x56, 0x34, 0x12};
static const uint8_t LINK[30]  = {0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0x40, 0x01, 0x30, 0, 0, 0, 0, 0x05, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t INPUT[30] = {0x85, 0, 0, 0, 0x05, 0, 0, 0, 0, 0, 0x64, 0xBE, 0, 0, 0,
                                  0x01, 0x7F, 0x7F, 0x7F, 0x7F, 0, 0, 0x7F, 0x7F, 0x7F, 0x7F, 0, 0, 0, 0};

static void send_frame(V5_DeviceT dev, const uint8_t *body, int bl, int len)
{
    uint8_t f[32];
    memset(f, 0, 32);
    memcpy(f, body, bl);
    uint16_t c = crc16(f, len - 2);
    f[len - 2] = c >> 8;
    f[len - 1] = c & 0xFF;
    vexDeviceGenericSerialTransmit(dev, f, len);
}

void user_main(void)
{
    V5_DeviceT dev = vexDeviceGetByIndex(PORT_INDEX);
    uint8_t rx[512];
    int total = 0, a85 = 0, a80 = 0, a13 = 0;

    vexDisplayErase();
    vexDisplayString(1, "radiolink port 1");
    vexDeviceGenericSerialEnable(dev, 0);
    vexDeviceGenericSerialBaudrate(dev, 115200);
    wait_ms(50);
    vexDeviceGenericSerialFlush(dev);

    for (int i = 0; ; i++) {
        if (i < 60)            send_frame(dev, INFO,  14, 16);
        else if (i < 66)       send_frame(dev, READY,  6, 16);
        else if (i % 4 == 0)   send_frame(dev, LINK,  30, 32);
        else                   send_frame(dev, INPUT, 30, 32);
        wait_ms(TICK_MS);

        int n = vexDeviceGenericSerialReceive(dev, rx, sizeof rx);
        if (n > 0) {
            total += n;
            for (int j = 0; j + 16 <= n; j++) {
                int L = (j + 32 <= n && crc16(rx + j, 30) ==
                         (uint16_t)((rx[j+30] << 8) | rx[j+31])) ? 32 :
                        (crc16(rx + j, 14) ==
                         (uint16_t)((rx[j+14] << 8) | rx[j+15])) ? 16 : 0;
                if (!L) continue;
                if (rx[j] == 0x02 && rx[j+1] == 0x85) a85++;
                else if (rx[j] == 0x02 && rx[j+1] == 0x80) a80++;
                else if (rx[j] == 0x02 && rx[j+1] == 0x13) a13++;
                j += L - 1;
            }
        }
        if (i % 40 == 39) {
            vexDisplayString(3, "rx=%d a85=%d a80=%d 13=%d", total, a85, a80, a13);
            say_counts(total, a85, a80, a13);
        }
    }
}
