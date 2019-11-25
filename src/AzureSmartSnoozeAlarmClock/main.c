
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <hw/sample_hardware.h>

#include "azure_iot_utilities.h"
#include "build_options.h"
#include "epoll_timerfd_utilities.h"
#include "sd1306.h"


#define INVALID_DATE_TIME 1262304000 //unix time for 1/1/2010, used to know when got time from NTP server 
#define SECONDS_IN_DAY 86400
#define SNOOZE_LENGTH 20
#define STORAGE_HOUR_SIZE 1
#define STORAGE_MINUTE_SIZE 1
#define STORAGE_OFFSET_SIZE 4
#define STORAGE_TIME_ZONE_SIZE 3
#define NTP_SYNC_RETRIES 10

// Azure IoT Hub/Central defines.
#define SCOPEID_LENGTH 20
char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application and DPS set in app_manifest.json, CmdArgs

void buttonTimerEventHandler(EventData* eventData);
void buzzerTimerEventHandler(EventData* eventData);
void terminationHandler(int signalNumber);
enum buttonName {ButtonA, ButtonB, ButtonC, ButtonSet};
enum runningState{ Normal, DisplayAlarm, SetSettings, SetTimeZone, SetAlarmHour, SetAlarmMinute, SoundAlarm, Snooze };
typedef struct AlarmTime {
	uint8_t hour;
	uint8_t minute;
	uint16_t offsetSeconds;
	time_t currentAlarmTime;
	bool active;
} AlarmTime;

int buttonAFd = -1;
int buttonBFd = -1;
int buttonCFd = -1;
int buttonSetFd = -1;
int buttonPollTimerFd = -1;
int buzzerFd = -1;
int buzzerPollTimerFd = -1;
int epollFd = -1;
char timezone[STORAGE_TIME_ZONE_SIZE + 1] = { 0 };
volatile sig_atomic_t terminationRequired = false;
enum runningState currentState = Normal;
GPIO_Value_Type buttonAState;
GPIO_Value_Type buttonBState;
GPIO_Value_Type buttonCState;
GPIO_Value_Type buttonSetState;
EventData buttonEventData = { .eventHandler = &buttonTimerEventHandler };
EventData buzzerEventData = { .eventHandler = &buzzerTimerEventHandler };
AlarmTime alarmTime = { .hour = 0, .minute = 0, .offsetSeconds = 0, .currentAlarmTime = 0, .active = false};
struct timespec soundAlarmStarted;
struct timespec snoozeTime;


int setup() {
	initializeTerminationHandler();

	if (loadSettings() < 0) {
		Log_Debug("Error: Could not load settings from storage.\n");
	}

	if (initializeIOPorts() < 0) {
		Log_Debug("Error: Could not initialize IO ports.\n");
		return -1;
	}

	if (initializeClock() < 0) {
		Log_Debug("Error: Could not initialize clock.\n");
	}

	if (initializeButtonEpollTimer() < 0) {
		Log_Debug("Error: Could not initialize button EPoll timer.\n");
		return -1;
	}
	// not sure why setTimeZone is not working in initializeClock() so doing here again.
	// seems to work when placed here
	setTimeZone();
	setCurrentAlarm();

	return 0;
}

void initializeTerminationHandler() {
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = terminationHandler;
}

int initializeIOPorts() {
	if (initializeI2C() < 0) {
		Log_Debug("Error: Could not initialize I2C.\n");
	}

	if (initializeGPIOs() < 0) {
		Log_Debug("Error: Could not initialize GPIOs.\n");
		return -1;
	}

	return 0;
}

int initializeGPIOs() {
	if ((buttonAFd = openAsInput(AVNET_MT3620_SK_GPIO42)) < 0) {
		Log_Debug("Error: Could not open button A GPIO.\n");
		return -1;
	}
	if ((buttonBFd = openAsInput(AVNET_MT3620_SK_GPIO43)) < 0) {
		Log_Debug("Error: Could not open button B GPIO.\n");
		return -1;
	}
	if ((buttonCFd = openAsInput(AVNET_MT3620_SK_USER_BUTTON_A)) < 0) {
		Log_Debug("Error: Could not open button C GPIO.\n");
		return -1;
	}
	if ((buttonSetFd = openAsInput(AVNET_MT3620_SK_USER_BUTTON_B)) < 0) {
		Log_Debug("Error: Could not open button SET GPIO.\n");
		return -1;
	}
	if ((buzzerFd = openAsOutput(AVNET_MT3620_SK_GPIO0)) < 0) {
		Log_Debug("Error: Could not open buzzer GPIO.\n");
		return -1;
	}

	return 0;
}

int initializeButtonEpollTimer() {
	if ((epollFd = CreateEpollFd()) < 0) {
		Log_Debug("Error: Could not create Epoll file descriptor.\n");
		return -1;
	}
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	if ((buttonPollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonEventData, EPOLLIN)) < 0) {
		return -1;
	}

	struct timespec buzzerInterval = { 0, 100000000 };
	if ((buzzerPollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buzzerInterval, &buzzerEventData, EPOLLIN)) < 0) {
		return -1;
	}
	return 0;
}

int loadSettings() {
	int storageFd = Storage_OpenMutableFile();
	if (storageFd < 0) {
		Log_Debug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	
	if (read(storageFd, timezone, STORAGE_TIME_ZONE_SIZE) < 0) {
		Log_Debug("ERROR: An error occurred while reading file:  %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (read(storageFd, &alarmTime.hour, STORAGE_HOUR_SIZE) < 0) {
		Log_Debug("Error: Could not read hour from storage: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (read(storageFd, &alarmTime.minute, STORAGE_MINUTE_SIZE) < 0) {
		Log_Debug("Error: Could not read hour from storage: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (read(storageFd, &alarmTime.offsetSeconds, STORAGE_OFFSET_SIZE) < 0) {
		Log_Debug("Error: Could not read hour from storage: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	close(storageFd);

	// 8th bit of hour is active setting
	alarmTime.active = alarmTime.hour & 0b10000000;
	alarmTime.hour = alarmTime.hour & 0b01111111;

	// load a default if no time zone existed
	if (strlen(timezone) != STORAGE_TIME_ZONE_SIZE) {
		strcpy(timezone, "+00");
	}

	return 0;
}

int saveSettings() {
	int storageFd = Storage_OpenMutableFile();
	if (storageFd < 0) {
		Log_Debug("ERROR: Could not open mutable file:  %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	if (write(storageFd, timezone, STORAGE_TIME_ZONE_SIZE) < 0) {
		Log_Debug("Error: Could not write zimezone:  %s (%d).\n", strerror(errno), errno);
	}
	uint8_t activeHour = alarmTime.hour | (alarmTime.active << 7);
	if (write(storageFd, &activeHour, STORAGE_HOUR_SIZE) < 0) {
		Log_Debug("Error: Could not write hour from storage: %s (%d).\n", strerror(errno), errno);
	}
	if (write(storageFd, &alarmTime.minute, STORAGE_MINUTE_SIZE) < 0) {
		Log_Debug("Error: Could not write hour from storage: %s (%d).\n", strerror(errno), errno);
	}
	if (write(storageFd, &alarmTime.offsetSeconds, STORAGE_OFFSET_SIZE) < 0) {
		Log_Debug("Error: Could not write hour from storage: %s (%d).\n", strerror(errno), errno);
	}
	close(storageFd);

	// send message to cloud that saved the alarm time
#if (defined(IOT_CENTRAL_APPLICATION))
	// Allocate memory for a message to Azure
	char* pjsonBuffer = (char*)malloc(128);
	if (pjsonBuffer == NULL) {
		Log_Debug("ERROR: not enough memory to send telemetry");
	}
	// construct the message
	snprintf(pjsonBuffer, 128, "{\"alarmTimeSet\":\"%02d:%02d\"}", alarmTime.hour, alarmTime.minute);

	Log_Debug("\n[Info] Sending info: %s\n", pjsonBuffer);
	AzureIoT_SendMessage(pjsonBuffer);
	free(pjsonBuffer);
#endif 

	return 0;
}

int initializeI2C() {
	if (initI2c() < 0) {
		Log_Debug("Error: Could not initialize I2C.\n");
		return -1;
	}

	if (initializeDisplay() < 0) {
		Log_Debug("Error: Could not initialize display.\n");
		return -1;
	}

	return 0;
}

int initializeDisplay() {
	if (sd1306_init() != 0) {
		Log_Debug("Error: could not init sd1306 %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	clear_oled_buffer();
	sd1306_draw_string(0, 0, "Starting", 2, white_pixel);
	sd1306_refresh();
	return 0;
}

int initializeClock() {
	bool isTimeSyncEnabled = false;
	if (Networking_TimeSync_GetEnabled(&isTimeSyncEnabled) != 0) {
		Log_Debug("Error: Networking_TimeSync_GetEnabled failed %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	if (!isTimeSyncEnabled) {
		Log_Debug("Error: NTP is required.\n");
		return -1;
	}
	struct timespec currentTime = { .tv_sec = 0 };
	const struct timespec sleepTime = { 2, 0 };
	int syncRetries = NTP_SYNC_RETRIES;
	// need to loop till we sync up with NTP server
	while (syncRetries-- > 0) {
		if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
			Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
		if (currentTime.tv_sec > INVALID_DATE_TIME) {
			break;
		}
		Log_Debug("Info: Not yet sync'd with time server, %d tries left", syncRetries);
		nanosleep(&sleepTime, NULL);
	}

	clear_oled_buffer();
	sd1306_draw_string(0, 0, "Setting", 2, white_pixel);
	sd1306_draw_string(0, 20, "TimeZone", 2, white_pixel);
	sd1306_refresh();

	if (setTimeZone() < 0) {
		Log_Debug("Error: Could not set time zone.\n");
		return -1;
	}

	return 0;
}

int setTimeZone() {
	Log_Debug("Info: Setting time zone to %s\n", timezone);
	if ((setenv("TZ", timezone, 1)) == -1) {
		Log_Debug("ERROR: setenv failed with error code: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	tzset();
	return 0;
}

int openAsInput(int gpioId) {
	int fd = GPIO_OpenAsInput(gpioId);

	if (fd < 0) {
		Log_Debug("Error opening GPIO as input %s (%d).\n", strerror(errno), errno);
	}

	return fd;
}

int openAsOutput(int gpioId) {
	int fd = GPIO_OpenAsOutput(gpioId, GPIO_OutputMode_PushPull, GPIO_Value_Low);

	if (fd < 0) {
		Log_Debug("Error opening GPIO as output %s (%d).\n", strerror(errno), errno);
	}

	return fd;
}

void setCurrentAlarm() {
	struct timespec currentTime = { .tv_sec = 0 };
	const struct timespec sleepTime = { 2, 0 };
	int syncRetries = NTP_SYNC_RETRIES;
	// need to loop till we sync up with NTP server, thought this would be done in initializeClock but doesn't alway happen
	while (syncRetries-- > 0) {
		if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
			Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
		if (currentTime.tv_sec > INVALID_DATE_TIME) {
			break;
		}
		Log_Debug("Info: Not yet sync'd with time server, %d tries left", syncRetries);
		nanosleep(&sleepTime, NULL);
	}


	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// set alarm to the current day at hour and time from settings
	struct tm local = *localtime(&currentTime.tv_sec);
#ifdef DEBUG_ALARM_TIME
	// for debugging setting off the alarm soon after startup
	if (soundAlarmStarted.tv_sec == 0) {
		alarmTime.currentAlarmTime = currentTime.tv_sec + 10;
	}
	else {
		local.tm_hour = alarmTime.hour;
		local.tm_min = alarmTime.minute;
		local.tm_sec = 0;
		alarmTime.currentAlarmTime = mktime(&local) - alarmTime.offsetSeconds;
	}
#else
	local.tm_hour = alarmTime.hour;
	local.tm_min = alarmTime.minute;
	local.tm_sec = 0;
	alarmTime.currentAlarmTime = mktime(&local) - alarmTime.offsetSeconds;
#endif // DEBUG_ALARM_TIME
	
	// check to see if we already passed the alarm and if so, set alarm to next day
	if (alarmTime.currentAlarmTime < currentTime.tv_sec) {
		alarmTime.currentAlarmTime += SECONDS_IN_DAY;
	}

#ifdef DEBUG
	local = *localtime(&alarmTime.currentAlarmTime);
	Log_Debug("Alarm set to %02d:%02d %d/%d/%d\n", alarmTime.hour, alarmTime.minute, local.tm_mon + 1, local.tm_mday, local.tm_year + 1900);
	Log_Debug("Alarm with offset (%d seconds) is %02d:%02d:%02d %d/%d/%d\n", alarmTime.offsetSeconds, local.tm_hour, local.tm_min, local.tm_sec, local.tm_mon + 1, local.tm_mday, local.tm_year + 1900);
#endif // DEBUG
}

void terminationHandler(int signalNumber) {
	terminationRequired = true;
}

void buttonTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	processButtonA();
	processButtonB();
	processButtonC();
	processButtonSet();
}

void buzzerTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(buzzerPollTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	GPIO_Value_Type buzzerState;
	if (GPIO_GetValue(buzzerFd, &buzzerState) != 0) {
		Log_Debug("Error: Could not get buzzer state. %s (%d)", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (currentState == SoundAlarm) {
		if (buzzerState == GPIO_Value_High) {
			GPIO_SetValue(buzzerFd, GPIO_Value_Low);
		}
		else {
			GPIO_SetValue(buzzerFd, GPIO_Value_High);
		}
	}
	else if (currentState == Snooze) {
		if (buzzerState == GPIO_Value_High) {
			GPIO_SetValue(buzzerFd, GPIO_Value_Low);
		}
		struct timespec currentTime;
		if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
			Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
		if (currentTime.tv_sec > snoozeTime.tv_sec) {
			currentState = SoundAlarm;
		}
	}
	else if (buzzerState == GPIO_Value_High) {
		GPIO_SetValue(buzzerFd, GPIO_Value_Low);
	}
}

void processButtonA() {
	GPIO_Value_Type newButtonState;
	if (GPIO_GetValue(buttonAFd, &newButtonState) != 0) {
		Log_Debug("Error: Could not get Button A state. %s (%d)", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newButtonState != buttonAState) {
		if (currentState == Normal) {
			if (newButtonState == GPIO_Value_Low) {
				currentState = DisplayAlarm;
			}
		}
		else if (currentState == DisplayAlarm) {
			if (newButtonState == GPIO_Value_High) {
				saveSettings();
				currentState = Normal;
			}
		}
		else if (currentState == SetSettings) {
			if (newButtonState == GPIO_Value_Low) {
				currentState = SetAlarmHour;
			}
		}
		else if (currentState == SetAlarmHour) {
			if (newButtonState == GPIO_Value_Low) {
				alarmTime.hour = alarmTime.hour - 1;
				if (alarmTime.hour > 23) {
					alarmTime.hour = 23;
				}
			}
		}
		else if (currentState == SetAlarmMinute) {
			if (newButtonState == GPIO_Value_Low) {
				alarmTime.minute = alarmTime.minute - 1;
				if (alarmTime.hour > 59) {
					alarmTime.hour = 59;
				}
			}
		}
		else if (currentState == SetTimeZone) {
			if (newButtonState == GPIO_Value_Low) {
				timezone[0] = (timezone[0] == '+' ? '-' : '+');
			}
		}
		else if (currentState == SoundAlarm || currentState == Snooze) {
			processAlarmButtonPress(ButtonA);
		}

		buttonAState = newButtonState;
	}
}
void processButtonB() {
	GPIO_Value_Type newButtonState;
	if (GPIO_GetValue(buttonBFd, &newButtonState) != 0) {
		Log_Debug("Error: Could not get Button A state. %s (%d)", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newButtonState != buttonBState) {
		if (currentState == SetAlarmHour) {
			if (newButtonState == GPIO_Value_Low) {
				alarmTime.hour = (alarmTime.hour + 1) % 24;
			}
		}
		else if (currentState == SetAlarmMinute) {
			if (newButtonState == GPIO_Value_Low) {
				alarmTime.minute = (alarmTime.minute + 1) % 60;
			}
		}
		else if (currentState == SetTimeZone) {
			if (newButtonState == GPIO_Value_Low) {
				char* hourString = &timezone[1];
				int hours = atoi(hourString);
				hours = hours - 1;
				if (hours < 0) {
					hours = 23;
				}
				sprintf(hourString, "%02d", hours);
			}
		}

		buttonBState = newButtonState;
	}
}
void processButtonC() {
	GPIO_Value_Type newButtonState;
	if (GPIO_GetValue(buttonCFd, &newButtonState) != 0) {
		Log_Debug("Error: Could not get Button A state. %s (%d)", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newButtonState != buttonCState) {
		if (currentState == SetSettings) {
			if (newButtonState == GPIO_Value_Low) {
				currentState = SetTimeZone;
			}
		}
		else if (currentState == DisplayAlarm) {
			if (newButtonState == GPIO_Value_Low) {
				alarmTime.active = !alarmTime.active;
			}
		}
		else if (currentState == SetTimeZone) {
			if (newButtonState == GPIO_Value_Low) {
				char* hourString = &timezone[1];
				int hours = atoi(hourString);
				hours = (hours + 1) % 24;
				sprintf(hourString, "%02d", hours);
			}
		}

		buttonCState = newButtonState;
	}
}
void processButtonSet() {
	GPIO_Value_Type newButtonState;
	if (GPIO_GetValue(buttonSetFd, &newButtonState) != 0) {
		Log_Debug("Error: Could not get Button A state. %s (%d)", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newButtonState != buttonSetState) {
		if (currentState == Normal) {
			if (newButtonState == GPIO_Value_Low) {
				currentState = SetSettings;
			}
		}
		else if (currentState == SetAlarmHour) {
			if (newButtonState == GPIO_Value_High) {
				currentState = SetAlarmMinute;
			}
		}
		else if (currentState == SetAlarmMinute) {
			if (newButtonState == GPIO_Value_High) {
				alarmTime.offsetSeconds = 0;
				saveSettings();
				setCurrentAlarm();
				currentState = Normal;
			}
		} 
		else if (currentState == SetTimeZone) {
			if (newButtonState == GPIO_Value_High) {
				if (setTimeZone() < 0) {
					terminationRequired = true;
					return;
				}
				saveSettings();
				currentState = Normal;
			}
		}
		else if (currentState == SoundAlarm) {
			processAlarmButtonPress(ButtonSet);
		}

		buttonSetState = newButtonState;
	}
}

void processAlarmButtonPress(enum buttonName button) {
	if (button == ButtonA) {
		endSoundAlarm();
	}
	else if (button == ButtonSet) {
		currentState = Snooze;
		if (clock_gettime(CLOCK_REALTIME, &snoozeTime) == -1) {
			Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
		snoozeTime.tv_sec += SNOOZE_LENGTH;
	}
}

void endSoundAlarm() {
	currentState = Normal;
	GPIO_SetValue(buzzerFd, GPIO_Value_Low);
	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}
	uint16_t elapsedTime = currentTime.tv_sec - soundAlarmStarted.tv_sec;
	alarmTime.offsetSeconds = (elapsedTime + alarmTime.offsetSeconds) / 2;
	Log_Debug("Info: %d seconds to turn off alarm. New offset is %d.\n", elapsedTime, alarmTime.offsetSeconds);
	setCurrentAlarm();
#if (defined(IOT_CENTRAL_APPLICATION))
	// Allocate memory for a message to Azure
	char* pjsonBuffer = (char*)malloc(128);
	if (pjsonBuffer == NULL) {
		Log_Debug("ERROR: not enough memory to send telemetry");
	}
	// construct the message
	snprintf(pjsonBuffer, 128, "{\"snoozeTime\":\"%f\"}", elapsedTime/60.0);

	Log_Debug("\n[Info] Sending info: %s\n", pjsonBuffer);
	AzureIoT_SendMessage(pjsonBuffer);
	free(pjsonBuffer);
#endif 
}

void debugTime() {
	struct timespec currentTime;
	struct tm local;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}
	else {
		char displayTimeBuffer[26];
		if (!asctime_r((gmtime(&currentTime.tv_sec)), (char* restrict) & displayTimeBuffer)) {
			Log_Debug("Error: asctime_r failed with error code: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
		local = *localtime(&currentTime.tv_sec);
		Log_Debug("UTC: %s", displayTimeBuffer);
		Log_Debug("local: %02d:%02d\n", local.tm_hour, local.tm_min);
	}
}



void displayAlarm() {
	char buffer[10];
	char* active = (alarmTime.active ? "On" : "Off");
	clear_oled_buffer();
	sprintf(buffer, "Alarm %s", active);
	sd1306_draw_string(0, 0, buffer, 2, white_pixel);
	sprintf(buffer, "%02d:%02d", alarmTime.hour, alarmTime.minute);
	sd1306_draw_string(0, 25, buffer, 3, white_pixel);
	sd1306_refresh();
}

void displayTime() {
	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}
	else {
		clear_oled_buffer();
		struct tm local = *localtime(&currentTime.tv_sec);
		// check to make sure sync'd up with an NTP server since the default date would be 1/1/1900
		if (currentTime.tv_sec < INVALID_DATE_TIME) {
			sd1306_draw_string(0, 0, "Syncing", 3, white_pixel);
		}
		else {
			char buffer[11] = { 0 };
			sprintf(buffer, "%02d:%02d", local.tm_hour, local.tm_min);
			sd1306_draw_string(0, 0, buffer, 4, white_pixel);
			sprintf(buffer, "%d/%d/%d", local.tm_mon + 1, local.tm_mday, local.tm_year + 1900);
			sd1306_draw_string(0, 35, buffer, 2, white_pixel);
		}
		sd1306_refresh();
	}

}

void displaySoundAlarm() {
	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}
	else {
		char buffer[11] = { 0 };
		struct tm local = *localtime(&currentTime.tv_sec);
		clear_oled_buffer();
		sprintf(buffer, "%02d:%02d", local.tm_hour, local.tm_min);
		sd1306_draw_string(0, 0, buffer, 4, white_pixel);
		sprintf(buffer, "%d/%d/%d", local.tm_mon + 1, local.tm_mday, local.tm_year + 1900);
		sd1306_draw_string(0, 35, buffer, 2, white_pixel);
		sd1306_draw_string(0, 50, "buzzz", 2, white_pixel);
		sd1306_refresh();
	}
}

void displaySetSettings() {
	clear_oled_buffer();
	sd1306_draw_string(0, 0, "A:Alarm", 3, white_pixel);
	sd1306_draw_string(0, 22, "C:TZ", 3, white_pixel);
	sd1306_refresh();
}

void displaySetAlarmHour() {
	char hour[3] = { 0 };
	sprintf(hour, "%d", alarmTime.hour);
	clear_oled_buffer();
	sd1306_draw_string(0, 0, "Hour", 3, white_pixel);
	sd1306_draw_string(0, 22, hour, 3, white_pixel);
	sd1306_refresh();
}

void displaySetAlarmMinute() {
	char buffer[3] = { 0 };
	sprintf(buffer, "%d", alarmTime.minute);
	clear_oled_buffer();
	sd1306_draw_string(0, 0, "Minute", 3, white_pixel);
	sd1306_draw_string(0, 22, buffer, 3, white_pixel);
	sd1306_refresh();
}

void displaySetTimeZone() {
	clear_oled_buffer();
	sd1306_draw_string(0, 0, "TZ", 3, white_pixel);
	sd1306_draw_string(0, 22, timezone, 3, white_pixel);
	sd1306_refresh();
}

void checkAlarm() {
	struct timespec currentTime;
	if (clock_gettime(CLOCK_REALTIME, &currentTime) == -1) {
		Log_Debug("Error: clock_getTime failed with error code: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (alarmTime.currentAlarmTime < currentTime.tv_sec) {
		soundAlarmStarted = currentTime;
		if(alarmTime.active){
			currentState = SoundAlarm;
		}
		alarmTime.currentAlarmTime += SECONDS_IN_DAY;
	}
}

void closePeripheralsAndHandlers(void)
{
	Log_Debug("Closing file descriptors.\n");

	closeI2c();
	CloseFdAndPrintError(buttonAFd, "Button A");
	CloseFdAndPrintError(buttonBFd, "Button B");
	CloseFdAndPrintError(buttonCFd, "Button C");
	CloseFdAndPrintError(buttonSetFd, "Button Set");
	CloseFdAndPrintError(buttonPollTimerFd, "Button Poll Timer");
	CloseFdAndPrintError(buzzerFd, "Buzzer");
	CloseFdAndPrintError(epollFd, "epoll");
}

int main(int argc, char* argv[]) {
	if (setup() < 0) {
		Log_Debug("Error: Setup failed.\n");
		terminationRequired = true;
	}

#if (defined(IOT_CENTRAL_APPLICATION))
	if (argc == 2) {
		Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
		strncpy(scopeId, argv[1], SCOPEID_LENGTH);
	}
	else {
		Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
		return -1;
	}
#endif 

#ifdef DEBUG
	debugTime();
#endif // DEBUG

	while (!terminationRequired) {

#if (defined(IOT_CENTRAL_APPLICATION))
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			Log_Debug("ERROR: Failed to set up IoT Hub client\n");
		}
#endif 

		if (WaitForEventAndCallHandler(epollFd) != 0) {
			terminationRequired = true;
		}

		if (currentState == Normal) {
			displayTime();
			checkAlarm();
		}
		else if (currentState == SoundAlarm || currentState == Snooze) {
			displaySoundAlarm();
		}
		else if (currentState == DisplayAlarm) {
			displayAlarm();
		}
		else if (currentState == SetSettings) {
			displaySetSettings();
		}
		else if (currentState == SetAlarmHour) {
			displaySetAlarmHour();
		}
		else if (currentState == SetAlarmMinute) {
			displaySetAlarmMinute();
		}
		else if (currentState == SetTimeZone) {
			displaySetTimeZone();
		}

#if (defined(IOT_CENTRAL_APPLICATION))
		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		AzureIoT_DoPeriodicTasks();
#endif
	}
	Log_Debug("Info: Application exiting.\n");
	closePeripheralsAndHandlers();
	return 0;
}