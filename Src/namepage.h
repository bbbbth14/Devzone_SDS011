#define LCD_PAGE_INDOOR (0)
#define LCD_PAGE_OUTDOOR (1)
#define LCD_FIELD_OF_PAGE (6)
#define LCD_VAR_LEVEL_NAME (0)
#define LCD_VAR_MAX_NAME (1)
#define LCD_VAR_MIN_NAME (2)
#define LCD_VAR_TIME_NAME (3)
#define LCD_VAR_DATE_NAME (4)
#define LCD_VAR_STATE_NAME (5)
#define LCD_VAR_PROCESS_NAME (6)
#define PROC_SYNC_TIME (1)
#define PROC_SEND_DATA_TO_THINGSPEAK (2)
#define PROC_RECV_DATA_OUTDOOR_DEVICE (3)
#define PROC_SEND_COMMAND_MODE_AUTO (4)
#define PROC_SEND_COMMAND_MODE_MANUAL (5)
#define PROC_QUERY_STATUS (6)
#define PROC_WRITE_CONFIG (7)
#define CNT0 (0)
#define CNT1 (1)
#define CNT2 (2)
#define CNT3 (3)
#define CNT4 (4)
#define CNT5 (5)
#define CNT6 (6)
#define CNT7 (7)
#define CNT8 (8)
#define CNT9 (9)

const char *LCD_PAGE_NAME[] = {"indoor", "outdoor", "wifisetup", "editbox", "allsetting"};
const char *LCD_VAR_NAME[] = {"level", "max", "min", "time", "date", "state", "process"};
const char *MONTH[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                       "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
                      };
const char *WEEKDAY[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};



