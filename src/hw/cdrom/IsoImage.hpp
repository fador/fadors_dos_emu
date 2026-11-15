#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace fador::hw::cdrom {

// One entry of an ISO 9660 directory, with the name already converted to the
// form DOS programs see through MSCDEX: upper case, ";1" version suffix and a
// trailing '.' removed.
struct IsoDirEntry {
  std::string name;
  uint32_t extentLba = 0;
  uint32_t size = 0;
  uint8_t flags = 0; // ISO 9660 file flags (bit 0 hidden, bit 1 directory)
  std::array<uint8_t, 7> recordingTime{}; // years since 1900, month, day,
                                          // hour, minute, second, GMT offset

  bool isDirectory() const { return (flags & 0x02) != 0; }
  bool isHidden() const { return (flags & 0x01) != 0; }

  // DOS attribute byte: files on a CD are read-only.
  uint8_t dosAttributes() const;
  uint16_t dosTime() const;
  uint16_t dosDate() const;
};

// A track of the disc.  Plain .iso images have a single data track; .cue
// sheets can describe audio tracks as well.
struct IsoTrack {
  uint8_t number = 1;
  bool audio = false;
  uint32_t startLba = 0; // absolute disc LBA (sector 0 = MSF 00:02:00)
};

// Read-only ISO 9660 file system on a CD-ROM image.  Supported containers:
//   - .iso / cooked images with 2048-byte sectors,
//   - raw 2352-byte (MODE1 and MODE2/XA form 1) and 2336-byte images,
//   - .cue sheets whose first track is a data track (BIN file).
// The layout of raw images is detected from the volume descriptor position.
class IsoImage {
public:
  static constexpr uint32_t kSectorSize = 2048;

  // Opens and validates an image; returns nullptr when the file is not a
  // readable ISO 9660 image.  `error` (optional) receives a reason.
  static std::unique_ptr<IsoImage> open(const std::string &path,
                                        std::string *error = nullptr);

  // True when `path` has an extension handled by open() (.iso, .cue, .bin,
  // .img).  Used by the drive manager to route mounts.
  static bool hasImageExtension(const std::string &path);

  const std::string &path() const { return m_path; }
  std::string volumeLabel() const { return m_volumeLabel; }
  // Number of 2048-byte sectors in the ISO 9660 volume (PVD space size).
  uint32_t volumeSectors() const { return m_volumeSectors; }
  // Number of sectors in the image file(s), i.e. the lead-out LBA.
  uint32_t discSectors() const { return m_discSectors; }
  const std::vector<IsoTrack> &tracks() const { return m_tracks; }

  // Copyright / abstract / bibliographic file identifiers from the PVD
  // (trailing blanks removed).
  std::string copyrightFile() const { return m_copyrightFile; }
  std::string abstractFile() const { return m_abstractFile; }
  std::string bibliographicFile() const { return m_bibliographicFile; }

  // Reads one cooked 2048-byte sector of the data track.
  bool readSector(uint32_t lba, std::span<uint8_t, kSectorSize> out) const;
  // Reads `count` consecutive cooked sectors into `out` (count * 2048 bytes).
  bool readSectors(uint32_t lba, uint32_t count, std::span<uint8_t> out) const;

  // Path lookup.  Components may be separated by '\\' or '/'; matching is
  // case-insensitive.  The empty path (or "\\") is the root directory.
  std::optional<IsoDirEntry> lookup(std::string_view path) const;
  // Entries of a directory (without "." and ".."), or nullopt when `path`
  // is not a directory.
  std::optional<std::vector<IsoDirEntry>> listDirectory(std::string_view path) const;

  // Reads file data at `offset`; returns the number of bytes copied (short
  // at end of file).
  size_t readFile(const IsoDirEntry &entry, uint64_t offset,
                  std::span<uint8_t> out) const;

  // The raw ISO 9660 directory record of the root directory (34 bytes).
  const IsoDirEntry &rootEntry() const { return m_root; }

private:
  IsoImage() = default;

  bool readRawAt(uint32_t lba, std::span<uint8_t, kSectorSize> out) const;
  bool detectLayout();
  bool parseVolumeDescriptors(std::string *error);
  bool parseCue(const std::string &cuePath, std::string *error);
  const std::vector<IsoDirEntry> *directoryFor(const IsoDirEntry &dir) const;

  std::string m_path;
  std::string m_dataFile;
  mutable std::ifstream m_file;
  uint64_t m_fileSize = 0;
  uint32_t m_rawSectorSize = kSectorSize; // bytes per sector in the file
  uint32_t m_dataOffset = 0;              // offset of user data in a sector
  uint32_t m_dataTrackLba = 0;            // disc LBA where the file starts

  std::string m_volumeLabel;
  std::string m_copyrightFile;
  std::string m_abstractFile;
  std::string m_bibliographicFile;
  uint32_t m_volumeSectors = 0;
  uint32_t m_discSectors = 0;
  IsoDirEntry m_root;
  std::vector<IsoTrack> m_tracks;

  // Directory cache keyed by extent LBA.
  mutable std::unordered_map<uint32_t, std::vector<IsoDirEntry>> m_dirCache;
};

} // namespace fador::hw::cdrom
