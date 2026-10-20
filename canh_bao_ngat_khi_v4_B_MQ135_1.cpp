#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <mcp_can.h>
#include <util/delay.h>
#include <math.h>

#define PIN_PIR      3
#define PIN_MQ       A0
#define PIN_BUZ      5
#define BUZ_ACTIVE_LOW 1
#define PIN_CS_SD    4
#define PIN_CS_CAN   10
#define PIN_INT_CAN  2
#define DHT_HIGH()   ((PIND & (1 << PD7)) ? 1 : 0)

#define CAN_CRYSTAL  MCP_8MHZ
#define CAN_TX_ID    0x001

enum { EMPTY_CAR = 0, SAFE, WARNING, DANGER, EMERGENCY };
enum { TH_NONE = 0, TH1_VENT, TH2_EMPTY, TH3_SEALED };
enum { WHY_NONE = 0, WHY_MOTION, WHY_BREATH, WHY_STILL };

static const char *levelName(uint8_t lv)
{
    switch (lv) {
        case SAFE:      return "SAFE";
        case WARNING:   return "WARNING";
        case DANGER:    return "DANGER";
        case EMERGENCY: return "EMERGENCY";
        default:        return "EMPTY_CAR";
    }
}

static const char *thName(uint8_t th)
{
    switch (th) {
        case TH1_VENT:   return "TH1";
        case TH2_EMPTY:  return "TH2";
        case TH3_SEALED: return "TH3";
        default:         return "---";
    }
}

static const char *whyName(uint8_t w)
{
    switch (w) {
        case WHY_MOTION: return "MOTION";
        case WHY_BREATH: return "BREATH";
        case WHY_STILL:  return "STILL";
        default:         return "NONE";
    }
}

static Print &Log = Serial;

#define N_POINTS        30
#define SD_WRITE_TRIES  3

struct Point {
    uint16_t t;
    int16_t  dht10;
    uint16_t co2;
    uint8_t  level;
    uint8_t  th;
    uint8_t  pir;
    uint8_t  score;
};
static Point   pts[N_POINTS];
static uint8_t nPts = 0;
static bool    sdDone = false;
static char    dataName[13] = "DATA00.TXT";

static void printX10(Print &p, int16_t v)
{
    if (v < 0) { p.print('-'); v = -v; }
    p.print(v / 10);
    p.print('.');
    p.print(v % 10);
}

static void printPoints(Print &p)
{
    p.println(F("t_s,DHT_C,CO2_ppm,TH,score,level,PIR"));
    for (uint8_t i = 0; i < nPts; i++) {
        p.print(pts[i].t);            p.print(',');
        printX10(p, pts[i].dht10);    p.print(',');
        p.print(pts[i].co2);          p.print(',');
        p.print(thName(pts[i].th));   p.print(',');
        p.print(pts[i].score);        p.print(',');
        p.print(levelName(pts[i].level)); p.print(',');
        p.println(pts[i].pir);
    }
}

static bool sdWriteAllOnce(void)
{
    if (!SD.begin(PIN_CS_SD)) return false;
    for (uint8_t i = 0; i < 100; i++) {
        dataName[4] = '0' + i / 10;
        dataName[5] = '0' + i % 10;
        if (!SD.exists(dataName)) break;
    }
    File f = SD.open(dataName, FILE_WRITE);
    if (!f) return false;
    f.clearWriteError();
    printPoints(f);
    f.flush();
    bool ok = !f.getWriteError();
    f.close();
    return ok;
}

static void sdSaveAll(void)
{
    Log.print(F("[SD  ] Du "));
    Log.print(nPts);
    Log.println(F(" diem, ghi vao the..."));
    for (uint8_t k = 1; k <= SD_WRITE_TRIES; k++) {
        if (sdWriteAllOnce()) {
            Log.print(F("[SD  ] Ghi OK vao "));
            Log.println(dataName);
            SD.end();
            sdDone = true;
            return;
        }
        Log.print(F("[ERR ] SD ghi lan "));
        Log.print(k);
        Log.print(F(" fail (ma loi the 0x"));
        Log.print(SD.cardErrorCode(), HEX);
        Log.println(')');
        SD.end();
        delay(300);
    }
    Log.println(F("[ERR ] SD khong ghi duoc: rut USB cam lai roi chay lai. So lieu van co o bang duoi."));
    sdDone = true;
}

static uint8_t dhtWait(uint8_t level, uint8_t us)
{
    while (DHT_HIGH() != level) {
        if (us-- == 0) return 0;
        _delay_us(1);
    }
    return 1;
}

static bool dhtReadTemp(float *t)
{
    uint8_t d[5] = {0, 0, 0, 0, 0};

    DDRD  |=  (1 << PD7);
    PORTD &= ~(1 << PD7);
    _delay_ms(2);
    DDRD  &= ~(1 << PD7);

    cli();
    if (!dhtWait(0, 100) || !dhtWait(1, 100) || !dhtWait(0, 100)) { sei(); return false; }
    for (uint8_t i = 0; i < 40; i++) {
        if (!dhtWait(1, 80)) { sei(); return false; }
        _delay_us(35);
        if (DHT_HIGH()) {
            d[i / 8] |= (uint8_t)(1 << (7 - (i % 8)));
            if (!dhtWait(0, 80)) { sei(); return false; }
        }
    }
    sei();

    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return false;

    int16_t t10 = (int16_t)(((uint16_t)(d[2] & 0x7F) << 8) | d[3]);
    if (d[2] & 0x80) t10 = -t10;
    *t = t10 / 10.0f;
    return true;
}

#define C0       400.0f

#define EMPTY_N        6
#define EMPTY_DT_MIN  -10
#define EMPTY_DT_MAX   3
#define EMPTY_DC_MAX   100.0f
#define EMPTY_NEED_LOW_CO2 1
#define C_EMPTY        600.0f

#define T_VENT        -3
#define VENT_N         4

#define C_SEAL_IN      600.0f
#define C_SEAL_OUT     500.0f

struct B1Out { float C; float Td; uint8_t P; uint8_t th; uint8_t why; float dC; float dT; };

static float   Td_last = 28.0f;
static int16_t Tprev10 = 280;
static float   Cprev   = C0;
static bool    havePrev = false;
static bool    occ = false;
static uint8_t why = WHY_NONE;
static uint8_t emptyCnt = 0, ventCnt = 0;
static bool    ventLatch = false;
static uint8_t thState = TH_NONE;

#define MQ_RL        22000.0f
#define MQ_WARMUP_S  20
#define MQ_C_MIN     400.0f
#define MQ_C_MAX     20000.0f
static float mq_Ro = 1.0f;

static float mqRs(void)
{
    int adc = analogRead(PIN_MQ);
    if (adc < 1) adc = 1;
    return MQ_RL * (1023 - adc) / adc;
}

static void mqCalibrate(void)
{
    for (uint8_t s = MQ_WARMUP_S; s > 0; s--) {
        Log.print(F("[INFO] MQ135 warm-up ")); Log.print(s); Log.println(F(" s"));
        delay(1000);
    }
    float sum = 0;
    for (uint8_t i = 0; i < 16; i++) { sum += mqRs(); delay(20); }
    mq_Ro = (sum / 16) / powf(400.0f / 116.602f, 1.0f / -2.769f);
    Log.print(F("[INFO] MQ135 Ro=")); Log.println(mq_Ro, 0);
}

static float mqReadCO2(void)
{
    float C = 116.602f * powf(mqRs() / mq_Ro, -2.769f);
    if (C < MQ_C_MIN) C = MQ_C_MIN;
    if (C > MQ_C_MAX) C = MQ_C_MAX;
    return C;
}

static B1Out b1_step(uint8_t P)
{
    B1Out out;

    float t;
    if (dhtReadTemp(&t)) Td_last = t;
    else Log.println(F("[ERR ] DHT22 NaN, giu T cu"));
    out.Td = Td_last;
    int16_t T10  = (int16_t)lroundf(Td_last * 10.0f);
    int16_t dT10 = havePrev ? (int16_t)(T10 - Tprev10) : 0;

    out.C = mqReadCO2();
    float dC = havePrev ? out.C - Cprev : 0.0f;
    out.P  = P;
    out.dC = dC;
    out.dT = dT10 / 10.0f;

    bool quiet = (P == 0) && (dT10 >= EMPTY_DT_MIN) && (dT10 <= EMPTY_DT_MAX) && (dC <= EMPTY_DC_MAX);
    emptyCnt = quiet ? (emptyCnt < 255 ? emptyCnt + 1 : 255) : 0;
    bool lowCO2 = !EMPTY_NEED_LOW_CO2 || out.C < C_EMPTY;

    if (P)                                       { occ = true;  why = WHY_MOTION; }
    else if (occ && emptyCnt >= EMPTY_N && lowCO2) { occ = false; why = WHY_NONE; }
    else if (occ && dT10 > EMPTY_DT_MAX)         { why = WHY_BREATH; }
    else if (occ)                                { why = WHY_STILL; }
    else                                         { why = WHY_NONE; }
    out.why = why;

    ventCnt = (dT10 <= T_VENT) ? (ventCnt < 255 ? ventCnt + 1 : 255) : 0;
    if (ventCnt >= VENT_N)      ventLatch = true;
    if (dT10 > EMPTY_DT_MAX)    ventLatch = false;

    uint8_t th;
    if (!occ)                       { th = TH2_EMPTY; ventLatch = false; }
    else if (ventLatch)             th = TH1_VENT;
    else if (thState == TH3_SEALED) th = (out.C >= C_SEAL_OUT) ? TH3_SEALED : TH1_VENT;
    else                            th = (out.C >= C_SEAL_IN)  ? TH3_SEALED : TH1_VENT;
    thState = th;
    out.th = th;

    Tprev10 = T10;
    Cprev = out.C;
    havePrev = true;
    return out;
}

#define PIR_WIN    5
#define H          1.0f
#define C_W        1500.0f
#define C_D        5000.0f
#define C_E        10000.0f
#define D_1        29.5f
#define D_2        30.2f
#define P_C1       3
#define P_C2       6
#define P_C3       9
#define P_T1       1
#define P_T2       2
#define P_STILL    2
#define P_RISE     1
#define DWELL      3

struct B2Out { uint8_t level; uint8_t score; uint8_t pc, pt, ps, pr; float Ceff; };

static uint8_t pirWin[PIR_WIN];
static uint8_t pirIdx = 0;
static uint8_t posCnt = 0, downCnt = 0;
static uint8_t level = EMPTY_CAR;

static void b2_reset(void)
{
    for (uint8_t i = 0; i < PIR_WIN; i++) pirWin[i] = 1;
    pirIdx = 0;
    posCnt = 0;
    downCnt = 0;
}

static uint8_t scoreToLevel(uint8_t s)
{
    if (s >= 9) return EMERGENCY;
    if (s >= 6) return DANGER;
    if (s >= 3) return WARNING;
    return SAFE;
}

static B2Out b2_step(float C, float Td, float dC, float dT, uint8_t P)
{
    B2Out out;

    pirWin[pirIdx] = P;
    pirIdx = (pirIdx + 1) % PIR_WIN;
    uint8_t S_W = 0;
    for (uint8_t i = 0; i < PIR_WIN; i++) S_W += pirWin[i];

    posCnt = (dT >= 0.05f && dC > 0.0f) ? posCnt + 1 : 0;
    bool rise = (posCnt >= 2);
    float Ceff = rise ? C + dC * H : C;
    float Teff = rise ? Td + dT * H : Td;
    out.Ceff = Ceff;

    uint8_t pc = 0, pt = 0, ps = 0, pr = 0;
    if (Ceff >= C_E)      pc = P_C3;
    else if (Ceff >= C_D) pc = P_C2;
    else if (Ceff >= C_W) pc = P_C1;
    if (Teff >= D_2)      pt = P_T2;
    else if (Teff >= D_1) pt = P_T1;
    if (S_W == 0 && (pc > 0 || pt > 0)) ps = P_STILL;
    if (rise) pr = P_RISE;
    uint8_t score = pc + pt + ps + pr;
    uint8_t raw = scoreToLevel(score);
    out.pc = pc; out.pt = pt; out.ps = ps; out.pr = pr; out.score = score;

    uint8_t cur = (level > SAFE) ? level : (uint8_t)SAFE;
    if (raw > cur) {
        level = cur + 1;
        downCnt = 0;
    } else if (raw < cur) {
        downCnt++;
        if (downCnt >= DWELL) { level = cur - 1; downCnt = 0; }
        else level = cur;
    } else {
        level = cur;
        downCnt = 0;
    }

    out.level = level;
    return out;
}

static MCP_CAN CAN0(PIN_CS_CAN);
static bool    canOk = false;

static void buzzerSet(bool on)
{
    digitalWrite(PIN_BUZ, (on != BUZ_ACTIVE_LOW) ? HIGH : LOW);
}

static void buzzerUpdate(uint8_t lv)
{
    static uint8_t       lastLv = EMPTY_CAR;
    static unsigned long lvStart = 0;
    unsigned long now = millis();
    if (lv != lastLv) { lastLv = lv; lvStart = now; }
    unsigned long m = now - lvStart;

    bool on;
    switch (lv) {
        case WARNING:   on = (m % 1500 < 200); break;
        case DANGER:    on = (m % 300  < 150); break;
        case EMERGENCY: on = true;             break;
        default:        on = false;            break;
    }
    buzzerSet(on);
}

static void canInit(void)
{
    if (CAN0.begin(MCP_ANY, CAN_125KBPS, CAN_CRYSTAL) != CAN_OK) {
        Log.println(F("[ERR ] CAN init fail (CS=D10, kiem tra 8/16 MHz)"));
        return;
    }
    CAN0.setMode(MCP_NORMAL);
    CAN0.enOneShotTX();
    canOk = true;
    Log.println(F("[INFO] CAN OK, 125 kbps, NORMAL, ID 0x001"));
}

static void canSend(const B1Out &in, const B2Out &r)
{
    static uint8_t lastStatus = 0xFF;
    if (!canOk) return;

    long c  = constrain(lroundf(in.C), 0L, 65535L);
    long t  = constrain(lroundf(in.Td), 0L, 255L);
    long dc = constrain(lroundf(in.dC), -32768L, 32767L);

    uint8_t d[8];
    d[0] = (uint8_t)(c >> 8);
    d[1] = (uint8_t)c;
    d[2] = (uint8_t)t;
    d[3] = in.P;
    d[4] = r.level;
    d[5] = (uint8_t)((uint16_t)dc >> 8);
    d[6] = (uint8_t)dc;
    d[7] = (uint8_t)(((in.th & 0x03) << 6) | ((in.why & 0x03) << 4) | (r.score & 0x0F));

    byte rc = CAN0.sendMsgBuf(CAN_TX_ID, 0, 8, d);
    uint8_t status = (rc != CAN_OK) ? 2 : (CAN0.errorCountTX() == 0 ? 0 : 1);
    if (status == lastStatus) return;
    lastStatus = status;
    switch (status) {
        case 0:  Log.println(F("[CAN ] Dang gui ID 0x001 moi 2 s, co ACK")); break;
        case 1:  Log.println(F("[CAN ] Dang phat ID 0x001 moi 2 s, khong co ACK (binh thuong khi chua co node nhan)")); break;
        default: Log.print(F("[ERR ] CAN gui fail (rc=")); Log.print(rc); Log.println(')'); break;
    }
}

static void recordPoint(uint32_t tSec, const B1Out &in, const B2Out &r)
{
    if (nPts >= N_POINTS) return;
    Point &p = pts[nPts++];
    p.t     = (uint16_t)tSec;
    p.dht10 = (int16_t)lroundf(in.Td * 10.0f);
    p.co2   = (uint16_t)constrain(lroundf(in.C), 0L, 65535L);
    p.level = r.level;
    p.th    = in.th;
    p.pir   = in.P;
    p.score = r.score;
}

static void dataLog(uint32_t tSec, const B1Out &in, const B2Out &r)
{
    Log.print(F("[DATA] t="));   Log.print(tSec);
    Log.print(F(" DHT="));       Log.print(in.Td, 1);
    Log.print(F("  MOTION="));   Log.print(in.P);
    Log.print(F(" CO2="));       Log.print(lroundf(in.C));
    Log.print(' ');              Log.print(thName(in.th));
    Log.print('/');              Log.print(whyName(in.why));
    Log.print(F(" dC="));        Log.print(lroundf(in.dC));
    Log.print(F(" dT="));        Log.print(in.dT, 2);
    Log.print(F(" S="));         Log.print(r.score);
    Log.print('(');              Log.print(r.pc); Log.print('+'); Log.print(r.pt);
    Log.print('+');              Log.print(r.ps); Log.print('+'); Log.print(r.pr); Log.print(')');
    Log.print(F(" -> "));        Log.println(levelName(r.level));
}

static unsigned long nextTick = 0;
static uint32_t      tick = 0;
static uint8_t       pirLatch = 0;

void setup()
{

    Serial.begin(9600);
    delay(100);
    Log.println();
    Log.println();

    buzzerSet(false);
    pinMode(PIN_BUZ, OUTPUT);
    buzzerSet(false);
    pinMode(PIN_PIR, INPUT);
    pinMode(PIN_INT_CAN, INPUT_PULLUP);
    pinMode(PIN_CS_SD, OUTPUT);  digitalWrite(PIN_CS_SD, HIGH);
    pinMode(PIN_CS_CAN, OUTPUT); digitalWrite(PIN_CS_CAN, HIGH);
    SPI.begin();

    DDRD  &= ~(1 << PD7);
    PORTD &= ~(1 << PD7);
    delay(2000);

    Log.println(F("=== KHOI DONG (v4 - BAN B: CO2 tu MQ135) ==="));
    Log.print(F("[INFO] Thu "));
    Log.print(N_POINTS);
    Log.println(F(" diem, moi 2 s, du thi ghi 1 lan vao the SD (DATAxx.TXT)"));

    canInit();

    mqCalibrate();

    b2_reset();

    Log.println(F("[INFO] Nguon CO2: MQ135 (doc doc lap voi DHT22)"));
    nextTick = millis();
}

void loop()
{
    if ((long)(millis() - nextTick) >= 0) {
        nextTick += 500;

        if (digitalRead(PIN_PIR)) pirLatch = 1;
        tick++;

        if (tick % 4 == 0) {
            uint8_t prev = level;
            B1Out in = b1_step(pirLatch);
            pirLatch = 0;

            B2Out r = {0, 0, 0, 0, 0, 0, in.C};
            if (in.th == TH3_SEALED) {
                r = b2_step(in.C, in.Td, in.dC, in.dT, in.P);
            } else {
                b2_reset();
                level = (in.th == TH2_EMPTY) ? EMPTY_CAR : SAFE;
                r.level = level;
            }

            uint32_t tSec = tick / 2;
            recordPoint(tSec, in, r);
            dataLog(tSec, in, r);
            if (r.level != prev) {
                Log.print(F("[STATE] "));
                Log.print(levelName(prev));
                Log.print(F(" -> "));
                Log.println(levelName(r.level));
            }
            canSend(in, r);

            if (!sdDone && nPts >= N_POINTS) {
                buzzerSet(false);
                sdSaveAll();
                Log.println(F("----- BANG DIEM (copy vao Excel de ve do thi) -----"));
                printPoints(Log);
                Log.println(F("--------------------------------------------------"));
            }
        }
    }

    buzzerUpdate(level);
}
