//beacon uorb cmd
#define BEACON_UORB_CMD_SET_FRE			0X01
#define BEACON_UORB_CMD_READ_FRE		0X81
#define BEACON_UORB_CMD_SET_LNB_CLK		0X02
#define BEACON_UORB_CMD_READ_LNB_CLK	0X82
#define BEACON_UORB_CMD_SET_LNB_POWER	0X03
#define BEACON_UORB_CMD_READ_LNB_POWER	0X83
#define BEACON_UORB_CMD_SET_AUTO_BACK	0X04
#define BEACON_UORB_CMD_READ_AUTO_BACK	0X84
#define BEACON_UORB_CMD_READ_SIGNAL		0X85
#define BEACON_UORB_CMD_SET_RESET		0X06
#define BEACON_UORB_CMD_SET_SAVE		0X07
#define BEACON_UORB_CMD_SET_POWER_VOL	0X08
#define BEACON_UORB_CMD_READ_POWER_VOL	0X88
#define BEACON_UORB_CMD_SET_22KHZ		0X09
#define BEACON_UORB_CMD_READ_22KHZ		0X89

#define BEACON_UORB_CMD_SET_AUTO		0X0A
#define BEACON_UORB_CMD_READ_AUTO		0X8A

#define BEACON_UORB_CMD_AUTO_DATA		0X0B
#define BEACON_UORB_CMD_LOAD_BEACON		0X0C

//BEACON return SET result
#define BEACON_UORB_CMD_STATE_0K		0X43
#define BEACON_UORB_CMD_STATE_FAIL		0X53


//LNB POWER VOL DEFINE
#define BEACON_UORB_LNB_POWER_13_4		1
#define BEACON_UORB_LNB_POWER_18_2		2
#define BEACON_UORB_LNB_POWER_14_6		3
#define BEACON_UORB_LNB_POWER_19_4		4
#define BEACON_UORB_LNB_POWER_OFF		0


//beacon uart cmd type
#define BEACON_UART_CMD_SET			0x00
#define BEACON_UART_CMD_READ		0x01
//beacon uart return set result
#define BEACON_UART_CMD_STATE_SETOK		0x43
#define BEACON_UART_CMD_STATE_SETFAULT	0x53

//beacon uart cmd
#define BEACON_UART_TYPE_FRE		0X10
#define BEACON_UART_TYPE_LNB_CLOCK	0X11
#define BEACON_UART_TYPE_LNB_VOL	0X12
#define BEACON_UART_TYPE_AUTO_BACK	0X14
#define BEACON_UART_TYPE_SIGNAL		0X13
#define BEACON_UART_TYPE_RESET		0X15
#define BEACON_UART_TYPE_SAVE		0X16
#define BEACON_UART_TYPE_AUTO_READ	0X17//need?
#define BEACON_UART_TYPE_POWER_VOL	0X18
#define BEACON_UART_TYPE_22KHZ		0X19

//beacon device define
#define BEACON_TEST_APP	1
#define BEACON_DEVICE		"/dev/ttyS3"

//beacon frequency define
#define BEACON_MAX_FRE	2200000000//Hz
#define BEACON_MIN_FRE 	950000000 //Hz

#define BEACON_SWITCH	18950000//kHz
#define SATE_MIN_FRE	17700000//kHz
#define SATE_MAX_FRE	20200000//khz


/* beacon private data structure */
struct beacon_s
{
	
	uint32_t		frequency;		/*beacon receive frequency*/		
	uint32_t		autoback_time;//dds1 sent data at timers
	uint32_t		power_voltage_proportion;//dds1 sent data at timers
	int16_t			back_power;
	int16_t			back_voltage;
	uint8_t			LNB_power_voltage;
	bool			LNB_switch_clock;//
	bool			LNB_switch_power;//
	bool			back_lock;
	bool			back_effective;
	bool			switch_atuoback;
	bool			switch_22KHZ;

	char 			reserve[3];

	int32_t			uorb_cmd;//beacon cmd no
	int32_t 		uorb_cmd_type;//beacon cmd type
	int32_t			uart_fd;//beacon device communication serial port file fd
};


int beacon_init(struct beacon_s *beacon);
int beacon_read(int read_type,int cmd,struct beacon_s *beacon,int file_fd);










