#include "IsoImage.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <sstream>

namespace fador::hw::cdrom {

namespace {

namespace fs = std::filesystem;

uint32_t readLE32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t readLE16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

std::string upperASCII(std::string_view s) {
  std::string out(s);
  for (auto &ch : out)
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  return out;
}

std::string trimRight(std::string s) {
  while (!s.empty() && (s.back() == ' ' || s.back() == '\0'))
    s.pop_back();
  return s;
}

std::string fieldString(const uint8_t *p, size_t len) {
  return trimRight(std::string(reinterpret_cast<const char *>(p), len));
}

// "FILE.EXT;1" -> "FILE.EXT", "DIR." -> "DIR".
std::string isoNameToDos(const uint8_t *p, size_t len) {
  if (len == 1 && p[0] == 0x00)
    return ".";
  if (len == 1 && p[0] == 0x01)
    return "..";
  std::string name(reinterpret_cast<const char *>(p), len);
  if (auto semi = name.find(';'); semi != std::string::npos)
    name.resize(semi);
  if (!name.empty() && name.back() == '.')
    name.pop_back();
  return upperASCII(name);
}

bool parseDirRecord(const uint8_t *rec, size_t avail, IsoDirEntry &out) {
  if (avail < 34)
    return false;
  const uint8_t len = rec[0];
  const uint8_t nameLen = rec[32];
  if (len < 34 || len > avail || 33u + nameLen > len)
    return false;
  out.extentLba = readLE32(rec + 2);
  out.size = readLE32(rec + 10);
  std::copy_n(rec + 18, 7, out.recordingTime.begin());
  out.flags = rec[25];
  out.name = isoNameToDos(rec + 33, nameLen);
  return true;
}

std::vector<std::string> splitPath(std::string_view path) {
  std::vector<std::string> parts;
  std::string cur;
  for (char ch : path) {
    if (ch == '\\' || ch == '/') {
      if (!cur.empty())
        parts.push_back(upperASCII(cur));
      cur.clear();
    } else {
      cur += ch;
    }
  }
  if (!cur.empty())
    parts.push_back(upperASCII(cur));
  return parts;
}

bool iequalsExt(const std::string &path, std::initializer_list<const char *> exts) {
  std::string ext = upperASCII(fs::path(path).extension().string());
  for (const char *e : exts)
    if (ext == e)
      return true;
  return false;
}

// "mm:ss:ff" -> frames
std::optional<uint32_t> parseMSF(const std::string &s) {
  unsigned m = 0, sec = 0, f = 0;
  char c1 = 0, c2 = 0;
  std::istringstream is(s);
  if (!(is >> m >> c1 >> sec >> c2 >> f) || c1 != ':' || c2 != ':')
    return std::nullopt;
  return (m * 60u + sec) * 75u + f;
}

} // namespace

// ---------------------------------------------------------------------------
// IsoDirEntry
// ---------------------------------------------------------------------------

uint8_t IsoDirEntry::dosAttributes() const {
  uint8_t attr = 0x01; // read-only
  if (isDirectory())
    attr = 0x10 | 0x01;
  if (isHidden())
    attr |= 0x02;
  return attr;
}

uint16_t IsoDirEntry::dosTime() const {
  const unsigned hour = std::min<unsigned>(recordingTime[3], 23);
  const unsigned min = std::min<unsigned>(recordingTime[4], 59);
  const unsigned sec = std::min<unsigned>(recordingTime[5], 59);
  return static_cast<uint16_t>((hour << 11) | (min << 5) | (sec / 2));
}

uint16_t IsoDirEntry::dosDate() const {
  int year = 1900 + recordingTime[0];
  year = std::clamp(year, 1980, 2107);
  unsigned month = std::clamp<unsigned>(recordingTime[1], 1, 12);
  unsigned day = std::clamp<unsigned>(recordingTime[2], 1, 31);
  return static_cast<uint16_t>(((year - 1980) << 9) | (month << 5) | day);
}

// ---------------------------------------------------------------------------
// IsoImage
// ---------------------------------------------------------------------------

bool IsoImage::hasImageExtension(const std::string &path) {
  return iequalsExt(path, {".ISO", ".CUE", ".BIN", ".IMG"});
}

std::unique_ptr<IsoImage> IsoImage::open(const std::string &path,
                                         std::string *error) {
  auto setError = [&](const std::string &msg) {
    if (error)
      *error = msg;
  };

  std::unique_ptr<IsoImage> img(new IsoImage());
  img->m_path = path;

  bool layoutFromCue = false;
  if (iequalsExt(path, {".CUE"})) {
    if (!img->parseCue(path, error))
      return nullptr;
    layoutFromCue = true;
  } else {
    img->m_dataFile = path;
  }

  img->m_file.open(img->m_dataFile, std::ios::binary);
  if (!img->m_file.is_open()) {
    setError("cannot open '" + img->m_dataFile + "'");
    return nullptr;
  }
  std::error_code ec;
  img->m_fileSize = fs::file_size(img->m_dataFile, ec);
  if (ec) {
    setError("cannot get size of '" + img->m_dataFile + "'");
    return nullptr;
  }

  // Validate the layout claimed by the cue sheet; otherwise (or if that
  // fails) probe the known sector layouts.
  bool layoutOk = false;
  if (layoutFromCue) {
    std::array<uint8_t, kSectorSize> pvd{};
    layoutOk = img->readRawAt(16, pvd) && pvd[0] == 0x01 &&
               std::memcmp(pvd.data() + 1, "CD001", 5) == 0;
  }
  if (!layoutOk && !img->detectLayout()) {
    setError("no ISO 9660 volume descriptor found");
    return nullptr;
  }

  if (!img->parseVolumeDescriptors(error))
    return nullptr;

  if (img->m_tracks.empty()) {
    img->m_discSectors =
        static_cast<uint32_t>(img->m_fileSize / img->m_rawSectorSize);
    img->m_tracks.push_back(IsoTrack{1, false, 0});
  }
  return img;
}

bool IsoImage::parseCue(const std::string &cuePath, std::string *error) {
  std::ifstream cue(cuePath);
  if (!cue.is_open()) {
    if (error)
      *error = "cannot open cue sheet '" + cuePath + "'";
    return false;
  }
  const fs::path baseDir = fs::path(cuePath).parent_path();

  struct CueFile {
    std::string path;
    uint32_t rawSize = 2352;
    uint32_t dataOffset = 16;
    bool sizeKnown = false;
    uint32_t startLba = 0;
  };
  std::vector<CueFile> files;
  struct PendingTrack {
    uint8_t number;
    bool audio;
    size_t fileIndex;
    uint32_t index1 = 0;
  };
  std::vector<PendingTrack> tracks;

  std::string line;
  while (std::getline(cue, line)) {
    std::istringstream is(line);
    std::string keyword;
    is >> keyword;
    keyword = upperASCII(keyword);
    if (keyword == "FILE") {
      std::string rest;
      std::getline(is, rest);
      // Name may be quoted; the file type follows the name.
      std::string name;
      auto q1 = rest.find('"');
      auto q2 = (q1 == std::string::npos) ? q1 : rest.find('"', q1 + 1);
      if (q1 != std::string::npos && q2 != std::string::npos) {
        name = rest.substr(q1 + 1, q2 - q1 - 1);
      } else {
        std::istringstream rs(rest);
        rs >> name;
      }
      fs::path p = fs::path(name);
      if (p.is_relative())
        p = baseDir / p;
      files.push_back(CueFile{p.string()});
    } else if (keyword == "TRACK" && !files.empty()) {
      unsigned number = 0;
      std::string mode;
      is >> number >> mode;
      mode = upperASCII(mode);
      CueFile &f = files.back();
      const bool audio = (mode == "AUDIO");
      if (!f.sizeKnown) {
        if (mode == "MODE1/2048") {
          f.rawSize = 2048;
          f.dataOffset = 0;
        } else if (mode == "MODE1/2352") {
          f.rawSize = 2352;
          f.dataOffset = 16;
        } else if (mode == "MODE2/2352") {
          f.rawSize = 2352;
          f.dataOffset = 24;
        } else if (mode == "MODE2/2336") {
          f.rawSize = 2336;
          f.dataOffset = 8;
        } else {
          f.rawSize = 2352;
          f.dataOffset = 0;
        }
        f.sizeKnown = true;
      }
      tracks.push_back(PendingTrack{static_cast<uint8_t>(number), audio,
                                    files.size() - 1});
    } else if (keyword == "INDEX" && !tracks.empty()) {
      unsigned idx = 0;
      std::string msf;
      is >> idx >> msf;
      if (idx == 1) {
        if (auto frames = parseMSF(msf))
          tracks.back().index1 = *frames;
      }
    }
  }

  if (files.empty() || tracks.empty()) {
    if (error)
      *error = "cue sheet has no FILE/TRACK entries";
    return false;
  }

  // Lay the files out back to back on the disc.
  uint32_t lba = 0;
  for (auto &f : files) {
    f.startLba = lba;
    std::error_code ec;
    const auto size = fs::file_size(f.path, ec);
    if (ec) {
      if (error)
        *error = "cannot open '" + f.path + "'";
      return false;
    }
    lba += static_cast<uint32_t>(size / f.rawSize);
  }
  m_discSectors = lba;

  for (const auto &t : tracks)
    m_tracks.push_back(
        IsoTrack{t.number, t.audio, files[t.fileIndex].startLba + t.index1});

  const auto firstData = std::find_if(tracks.begin(), tracks.end(),
                                      [](const PendingTrack &t) { return !t.audio; });
  if (firstData == tracks.end()) {
    if (error)
      *error = "cue sheet has no data track";
    return false;
  }
  const CueFile &df = files[firstData->fileIndex];
  m_dataFile = df.path;
  m_rawSectorSize = df.rawSize;
  m_dataOffset = df.dataOffset;
  m_dataTrackLba = firstData->index1; // relative to the start of its file
  return true;
}

bool IsoImage::detectLayout() {
  struct Layout {
    uint32_t raw;
    uint32_t offset;
  };
  constexpr Layout layouts[] = {{2048, 0}, {2352, 16}, {2352, 24}, {2336, 8}};
  for (const auto &l : layouts) {
    m_rawSectorSize = l.raw;
    m_dataOffset = l.offset;
    m_dataTrackLba = 0;
    std::array<uint8_t, kSectorSize> pvd{};
    if (readRawAt(16, pvd) && pvd[0] == 0x01 &&
        std::memcmp(pvd.data() + 1, "CD001", 5) == 0)
      return true;
  }
  return false;
}

bool IsoImage::readRawAt(uint32_t lba, std::span<uint8_t, kSectorSize> out) const {
  const uint64_t pos =
      (static_cast<uint64_t>(m_dataTrackLba) + lba) * m_rawSectorSize +
      m_dataOffset;
  if (pos + kSectorSize > m_fileSize)
    return false;
  m_file.clear();
  m_file.seekg(static_cast<std::streamoff>(pos));
  m_file.read(reinterpret_cast<char *>(out.data()), kSectorSize);
  return m_file.gcount() == static_cast<std::streamsize>(kSectorSize);
}

bool IsoImage::parseVolumeDescriptors(std::string *error) {
  std::array<uint8_t, kSectorSize> sec{};
  bool foundPrimary = false;
  for (uint32_t lba = 16; lba < 16 + 64; ++lba) {
    if (!readRawAt(lba, sec))
      break;
    if (std::memcmp(sec.data() + 1, "CD001", 5) != 0)
      break;
    const uint8_t type = sec[0];
    if (type == 0xFF)
      break;
    if (type == 0x01 && !foundPrimary) {
      foundPrimary = true;
      m_volumeLabel = fieldString(sec.data() + 40, 32);
      m_volumeSectors = readLE32(sec.data() + 80);
      const uint16_t blockSize = readLE16(sec.data() + 128);
      if (blockSize != kSectorSize) {
        if (error)
          *error = "unsupported logical block size";
        return false;
      }
      m_copyrightFile = fieldString(sec.data() + 702, 37);
      m_abstractFile = fieldString(sec.data() + 739, 37);
      m_bibliographicFile = fieldString(sec.data() + 776, 37);
      if (!parseDirRecord(sec.data() + 156, 34, m_root) || !m_root.isDirectory()) {
        if (error)
          *error = "invalid root directory record";
        return false;
      }
      m_root.name.clear();
    }
  }
  if (!foundPrimary && error)
    *error = "no primary volume descriptor";
  return foundPrimary;
}

bool IsoImage::readSector(uint32_t lba, std::span<uint8_t, kSectorSize> out) const {
  return readRawAt(lba, out);
}

bool IsoImage::readSectors(uint32_t lba, uint32_t count,
                           std::span<uint8_t> out) const {
  if (out.size() < static_cast<size_t>(count) * kSectorSize)
    return false;
  for (uint32_t i = 0; i < count; ++i) {
    if (!readRawAt(lba + i, std::span<uint8_t, kSectorSize>(
                                out.data() + static_cast<size_t>(i) * kSectorSize,
                                kSectorSize)))
      return false;
  }
  return true;
}

const std::vector<IsoDirEntry> *IsoImage::directoryFor(const IsoDirEntry &dir) const {
  if (auto it = m_dirCache.find(dir.extentLba); it != m_dirCache.end())
    return &it->second;

  std::vector<IsoDirEntry> entries;
  const uint32_t sectors = (dir.size + kSectorSize - 1) / kSectorSize;
  std::array<uint8_t, kSectorSize> sec{};
  for (uint32_t s = 0; s < sectors; ++s) {
    if (!readRawAt(dir.extentLba + s, sec))
      break;
    size_t off = 0;
    while (off < kSectorSize) {
      const uint8_t len = sec[off];
      if (len == 0)
        break; // rest of the sector is padding
      IsoDirEntry e;
      if (!parseDirRecord(sec.data() + off, kSectorSize - off, e))
        break;
      off += len;
      if (e.name == "." || e.name == ".." || e.name.empty())
        continue;
      if (e.flags & 0x04) // associated file
        continue;
      // Multi-extent files: the following records continue the same file.
      if (!entries.empty() && (entries.back().flags & 0x80) &&
          entries.back().name == e.name) {
        entries.back().size += e.size;
        entries.back().flags = e.flags;
        continue;
      }
      entries.push_back(std::move(e));
    }
  }
  auto [it, inserted] = m_dirCache.emplace(dir.extentLba, std::move(entries));
  (void)inserted;
  return &it->second;
}

std::optional<IsoDirEntry> IsoImage::lookup(std::string_view path) const {
  IsoDirEntry current = m_root;
  for (const auto &part : splitPath(path)) {
    if (part == ".")
      continue;
    if (!current.isDirectory())
      return std::nullopt;
    const auto *entries = directoryFor(current);
    auto it = std::find_if(entries->begin(), entries->end(),
                           [&](const IsoDirEntry &e) { return e.name == part; });
    if (it == entries->end())
      return std::nullopt;
    current = *it;
  }
  return current;
}

std::optional<std::vector<IsoDirEntry>>
IsoImage::listDirectory(std::string_view path) const {
  auto dir = lookup(path);
  if (!dir || !dir->isDirectory())
    return std::nullopt;
  return *directoryFor(*dir);
}

size_t IsoImage::readFile(const IsoDirEntry &entry, uint64_t offset,
                          std::span<uint8_t> out) const {
  if (offset >= entry.size || out.empty())
    return 0;
  size_t toRead = static_cast<size_t>(
      std::min<uint64_t>(out.size(), entry.size - offset));
  size_t done = 0;
  std::array<uint8_t, kSectorSize> sec{};
  while (done < toRead) {
    const uint64_t pos = offset + done;
    const uint32_t lba = entry.extentLba + static_cast<uint32_t>(pos / kSectorSize);
    const size_t inSector = static_cast<size_t>(pos % kSectorSize);
    const size_t chunk = std::min(toRead - done, kSectorSize - inSector);
    if (!readRawAt(lba, sec))
      break;
    std::memcpy(out.data() + done, sec.data() + inSector, chunk);
    done += chunk;
  }
  return done;
}

} // namespace fador::hw::cdrom
