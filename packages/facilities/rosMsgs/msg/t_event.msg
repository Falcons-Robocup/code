# event type enum
uint8 TYPE_INFO    = 0
uint8 TYPE_WARNING = 1
uint8 TYPE_ERROR   = 2

int32   eventId     # internal counter, maintained by robotControl, used on receiver side to filter unique events
uint8   robotId     # robot number
string  fileName    # originating source file
string  funcName    # originating function name
int32   lineNumber  # originating line number
float64 timeStamp   # originating timestamp
uint8   eventType   # info, warning or error
string  eventString # string to show in eventLog

