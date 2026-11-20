#pragma once
// Calendar helpers for the guest's wall clock (RTC, BIOS tick count, DOS
// date/time).  Pure integer arithmetic on the proleptic Gregorian calendar,
// independent of the host's time zone database; the host clock is read only
// by hostLocalTime(), once at start-up outside --deterministic runs.
#include <cstdint>
#include <ctime>
#include <optional>
#include <string_view>

namespace fador::hw {

struct CivilDateTime {
  int year = 1980;
  int month = 1; // 1..12
  int day = 1;   // 1..31
  int hour = 0;
  int minute = 0;
  int second = 0;

  friend bool operator==(const CivilDateTime &, const CivilDateTime &) = default;
};

/// Days since 1970-01-01 (H. Hinnant's days_from_civil).
constexpr int64_t daysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int64_t yoe = y - era * 400;
  const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

/// Inverse of daysFromCivil.
constexpr void civilFromDays(int64_t z, int &y, int &m, int &d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int64_t doe = z - era * 146097;
  const int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int64_t mp = (5 * doy + 2) / 153;
  d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
  m = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
  y = static_cast<int>(yoe + era * 400 + (m <= 2));
}

/// Day of the week, 0 = Sunday (1970-01-01 was a Thursday).
constexpr int weekdayFromDays(int64_t days) {
  return static_cast<int>(days >= -4 ? (days + 4) % 7 : (days + 5) % 7 + 6);
}

constexpr int daysInMonth(int y, int m) {
  constexpr int kDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0))
    return 29;
  return (m >= 1 && m <= 12) ? kDays[m - 1] : 0;
}

constexpr bool isValidDate(int y, int m, int d) {
  return m >= 1 && m <= 12 && d >= 1 && d <= daysInMonth(y, m);
}

constexpr bool isValidTime(int h, int m, int s) {
  return h >= 0 && h <= 23 && m >= 0 && m <= 59 && s >= 0 && s <= 59;
}

/// The date/time a --deterministic run starts at unless --start-time is
/// given: a fixed, plausible DOS-era moment, so programs that print the
/// date or time produce identical output on every run.
inline constexpr CivilDateTime kDeterministicStartTime{1994, 6, 1, 12, 0, 0};

/// PIT channel 0 at its BIOS default (divisor 65536) ticks 0x1800B0 times
/// per day: the BIOS tick count at 0040:006C wraps there.
inline constexpr uint32_t kBiosTicksPerDay = 0x1800B0;

/// BIOS tick count for a time of day (what the BIOS POST derives from the
/// RTC): seconds * 0x1800B0 / 86400, rounded up so that the time read back
/// from the tick count is not earlier than the RTC's.
constexpr uint32_t biosTicksAt(int hour, int minute, int second) {
  const uint64_t s = static_cast<uint64_t>(hour) * 3600 +
                     static_cast<uint64_t>(minute) * 60 +
                     static_cast<uint64_t>(second);
  return static_cast<uint32_t>((s * kBiosTicksPerDay + 86399) / 86400);
}

/// The host's local date and time (start-up only).
inline CivilDateTime hostLocalTime() {
  const std::time_t t = std::time(nullptr);
  std::tm lt{};
#ifdef _WIN32
  localtime_s(&lt, &t);
#else
  localtime_r(&t, &lt);
#endif
  return {lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
          lt.tm_hour,        lt.tm_min,     lt.tm_sec};
}

/// Parses YYYY-MM-DDTHH:MM:SS (also with a space instead of T, or a date
/// alone meaning midnight).  Years 1980..2099 (what DOS and the RTC hold).
inline std::optional<CivilDateTime> parseCivilDateTime(std::string_view s) {
  auto num = [&s](size_t pos, size_t len, int &out) {
    if (pos + len > s.size())
      return false;
    int v = 0;
    for (size_t i = pos; i < pos + len; ++i) {
      if (s[i] < '0' || s[i] > '9')
        return false;
      v = v * 10 + (s[i] - '0');
    }
    out = v;
    return true;
  };
  CivilDateTime dt;
  if (!num(0, 4, dt.year) || s.size() < 10 || s[4] != '-' ||
      !num(5, 2, dt.month) || s[7] != '-' || !num(8, 2, dt.day))
    return std::nullopt;
  if (s.size() > 10) {
    if ((s[10] != 'T' && s[10] != ' ') || s.size() != 19 ||
        !num(11, 2, dt.hour) || s[13] != ':' || !num(14, 2, dt.minute) ||
        s[16] != ':' || !num(17, 2, dt.second))
      return std::nullopt;
  }
  if (dt.year < 1980 || dt.year > 2099 ||
      !isValidDate(dt.year, dt.month, dt.day) ||
      !isValidTime(dt.hour, dt.minute, dt.second))
    return std::nullopt;
  return dt;
}

} // namespace fador::hw
