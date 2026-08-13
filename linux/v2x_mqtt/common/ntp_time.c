#define _GNU_SOURCE
#include "ntp_time.h"
#include <stdio.h>
#include <string.h>
#include <sys/timex.h>
#include <time.h>

#define NTP_TS_MODULO_MS 4096ULL
#define NTP_TS_MASK 0x0FFFU

uint64_t ntp_time_now_ms(void) { struct timespec ts; return clock_gettime(CLOCK_REALTIME, &ts) == 0 ? (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL) : 0; }
uint64_t ntp_time_sync_epoch_ms(void) { uint64_t now = ntp_time_now_ms(); return now >= NTP_SYNC_EPOCH_UNIX_MS ? now - NTP_SYNC_EPOCH_UNIX_MS : 0; }
NtpSyncStatus ntp_time_get_sync_status(void) { struct timex tx = {0}; int ret = adjtimex(&tx); return ret < 0 ? NTP_SYNC_NONE : ((tx.status & STA_UNSYNC) != 0 || ret == TIME_ERROR ? NTP_SYNC_RTC_ONLY : NTP_SYNC_OK); }
uint64_t ntp_time_reconstruct(uint16_t ts, uint64_t now) { uint64_t value = (now & ~(NTP_TS_MODULO_MS - 1ULL)) | (uint64_t)(ts & NTP_TS_MASK); return value > now ? value - NTP_TS_MODULO_MS : value; }
bool ntp_time_format_iso8601_utc(uint64_t timestamp, char out[NTP_TIME_ISO8601_UTC_STRLEN + 1]) { if (!out) return false; uint64_t unix_ms = NTP_SYNC_EPOCH_UNIX_MS + timestamp; time_t seconds = (time_t)(unix_ms / 1000ULL); struct tm utc; if (!gmtime_r(&seconds, &utc)) return false; return snprintf(out, NTP_TIME_ISO8601_UTC_STRLEN + 1, "%04d-%02d-%02dT%02d:%02d:%02d.%03lluZ", utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec, (unsigned long long)(unix_ms % 1000ULL)) == (int)NTP_TIME_ISO8601_UTC_STRLEN; }
bool ntp_time_parse_iso8601_utc(const char* s, uint64_t* out) { if (!s || !out || strlen(s) != NTP_TIME_ISO8601_UTC_STRLEN) return false; int y, mo, d, h, mi, se, ms; if (sscanf(s, "%4d-%2d-%2dT%2d:%2d:%2d.%3dZ", &y,&mo,&d,&h,&mi,&se,&ms) != 7 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || se > 59 || ms > 999) return false; struct tm parsed = {0}; parsed.tm_year=y-1900; parsed.tm_mon=mo-1; parsed.tm_mday=d; parsed.tm_hour=h; parsed.tm_min=mi; parsed.tm_sec=se; time_t seconds=timegm(&parsed); struct tm verified; if (seconds < 0 || !gmtime_r(&seconds,&verified) || verified.tm_year != parsed.tm_year || verified.tm_mon != parsed.tm_mon || verified.tm_mday != parsed.tm_mday || verified.tm_hour != parsed.tm_hour || verified.tm_min != parsed.tm_min || verified.tm_sec != parsed.tm_sec) return false; uint64_t unix_ms=(uint64_t)seconds*1000ULL+(uint64_t)ms; if(unix_ms<NTP_SYNC_EPOCH_UNIX_MS) return false; *out=unix_ms-NTP_SYNC_EPOCH_UNIX_MS; return true; }
