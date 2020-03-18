#define PORTD_12 0x00000100       // D8
#define PORTD_11 0x00000080       // D7
#define PORTD_10 0x20000000       // C29
#define PORTD_9  0x00200000       // C21
#define PORTD_8  0x00400000       // C22
#define PORTD_7  0x00800000       // C23
#define PORTD_6  0x01000000       // C24
#define PORTD_5  0x02000000       // C25

#define PORTD_D_MALL 0x00000180
#define PORTD_C_MALL 0x23E00000
#define PORTD_MALL   0x23E00180

#define ALL_PORTM_HIGH ({REG_PIOC_SODR = PORTD_C_MALL; REG_PIOD_SODR = PORTD_D_MALL; })
#define ALL_PORTM_LOW  ({REG_PIOC_CODR = PORTD_C_MALL; REG_PIOD_CODR = PORTD_D_MALL; })

//  port_10_status = c_status & (c_status << 29);
//  port_9_status  = c_status & (c_status << 21);
//  port_8_status  = c_status & (c_status << 22);
//  port_7_status  = c_status & (c_status << 23);
//  port_6_status  = c_status & (c_status << 24);
//  port_5_status  = c_status & (c_status << 25);
