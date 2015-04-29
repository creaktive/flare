#ifndef FLARM_DECODE

#define FLARM_DECODE

#define FLARM_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b }
#define FLARM_KEY2 0x045d9f3b
#define FLARM_KEY3 0x87b562f4

typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int magic:8;
    /********************/
    int vs:10;
    unsigned int _unk0:6;
    unsigned int gps:12;
    unsigned int type:4;
    /********************/
    int lat:19;
    unsigned int alt:13;
    /********************/
    int lon:20;
    unsigned int _unk1:10;
    unsigned int vsmult:2;
    /********************/
    int8_t ns[4];
    int8_t ew[4];
    /********************/
} flarm_packet;

#endif
