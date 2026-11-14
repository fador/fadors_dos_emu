#include "DriveManager.hpp"
#include "zip_reader.hpp"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace fador::hw {

namespace {
// True when a relative path climbs above its base (portable across
// wchar_t (Windows) and char (POSIX) native path encodings).
bool escapesRoot(const std::filesystem::path& rel) {
  return !rel.empty() && *rel.begin() == std::filesystem::path("..");
}

bool iequals(const std::string& a, const std::string& b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::toupper(static_cast<unsigned char>(a[i])) !=
        std::toupper(static_cast<unsigned char>(b[i])))
      return false;
  }
  return true;
}

bool hasZipExtension(const std::filesystem::path& path) {
  auto u8ext = path.extension().u8string();
  std::string ext(reinterpret_cast<const char*>(u8ext.data()), u8ext.size());
  return iequals(ext, ".zip");
}

constexpr size_t DOS_LABEL_MAX_LEN = 11;

std::string makeZipVolumeLabel(const std::filesystem::path& zipPath) {
  auto u8stem = zipPath.stem().u8string();
  std::string stem(reinterpret_cast<const char*>(u8stem.data()), u8stem.size());
  std::string label;
  label.reserve(DOS_LABEL_MAX_LEN);
  for (unsigned char ch : stem) {
    if (std::isalnum(ch) || ch == '_' || ch == '-') {
      label.push_back(static_cast<char>(std::toupper(ch)));
      if (label.size() == DOS_LABEL_MAX_LEN) {
        break;
      }
    }
  }
  return label.empty() ? "MS-DOS_622" : label;
}

std::filesystem::path createUniqueTempZipDir() {
  namespace fs = std::filesystem;
  static std::atomic<uint64_t> counter{0};
  std::error_code ec;
  fs::path baseTemp = fs::temp_directory_path(ec);
  if (ec) {
    return {};
  }
  const auto ticks = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  for (int attempt = 0; attempt < 32; ++attempt) {
    const uint64_t id = counter.fetch_add(1, std::memory_order_relaxed);
    fs::path candidate =
        baseTemp / ("fadors_zip_" + std::to_string(ticks) + "_" + std::to_string(id));
    if (!fs::exists(candidate, ec) && fs::create_directories(candidate, ec) && !ec) {
      return candidate;
    }
  }
  return {};
}

std::filesystem::path resolveExtractedMountRoot(const std::filesystem::path& tempDir) {
  namespace fs = std::filesystem;
  std::error_code ec;
  size_t entryCount = 0;
  fs::path singleDir;
  for (fs::directory_iterator it(tempDir, ec), end; !ec && it != end; it.increment(ec)) {
    ++entryCount;
    if (entryCount > 1) {
      return tempDir;
    }
    if (it->is_directory(ec) && !ec) {
      singleDir = it->path();
    } else {
      return tempDir;
    }
  }
  if (entryCount == 1 && !singleDir.empty()) {
    return singleDir;
  }
  return tempDir;
}
} // namespace

std::string DriveManager::matchHostCase(const std::string& base,
                                        const std::string& relative) {
  namespace fs = std::filesystem;
  fs::path root(base);
  fs::path current = root;
  int depth = 0;
  std::string component;
  bool missing = false; // once a component is missing, stop scanning
  auto flush = [&]() {
    if (component.empty()) {
      return;
    }
    if (component == ".") {
      component.clear();
      return;
    }
    if (component == "..") {
      // Never walk above the mount/base directory.
      if (depth > 0) {
        current = current.parent_path();
        --depth;
      }
      component.clear();
      return;
    }
    std::error_code ec;
    if (!missing) {
      fs::path exact = current / component;
      if (!fs::exists(exact, ec)) {
        bool found = false;
        if (fs::is_directory(current, ec)) {
          for (fs::directory_iterator it(current, ec), end; !ec && it != end;
               it.increment(ec)) {
            std::string name;
            try {
              // Use UTF-8 conversion to avoid CP_ACP failures on Windows hosts.
              auto u8 = it->path().filename().u8string();
              name.assign(reinterpret_cast<const char*>(u8.data()), u8.size());
            } catch (...) {
              continue;
            }
            if (iequals(name, component)) {
              current /= it->path().filename();
              found = true;
              break;
            }
          }
        }
        if (!found) {
          missing = true;
          current /= component;
        }
      } else {
        current = std::move(exact);
      }
    } else {
      current /= component;
    }
    ++depth;
    component.clear();
  };
  for (char ch : relative) {
    if (ch == '/' || ch == '\\') {
      flush();
    } else {
      component += ch;
    }
  }
  flush();
  return current.string();
}

DriveManager::~DriveManager() {
  std::error_code ec;
  for (auto& tempDir : m_tempZipDirs) {
    if (!tempDir.empty()) {
      std::filesystem::remove_all(tempDir, ec);
      tempDir.clear();
    }
  }
}

bool DriveManager::mount(char letter, const std::string& hostPath, const std::string& label) {
  auto idx = letterToIndex(letter);
  if (!m_mounts[idx].empty()) {
    return false;
  }

  std::error_code ec;
  auto canonical = std::filesystem::canonical(hostPath, ec);
  if (ec) {
    return false;
  }

  if (std::filesystem::is_regular_file(canonical, ec) && hasZipExtension(canonical)) {
    std::filesystem::path tempDir = createUniqueTempZipDir();
    if (tempDir.empty()) {
      return false;
    }
    if (!fzip::extract_to_directory(canonical.string(), tempDir)) {
      std::filesystem::remove_all(tempDir, ec);
      return false;
    }
    std::filesystem::path mountRoot = resolveExtractedMountRoot(tempDir);
    auto canonicalRoot = std::filesystem::canonical(mountRoot, ec);
    if (ec || !std::filesystem::is_directory(canonicalRoot, ec)) {
      std::filesystem::remove_all(tempDir, ec);
      return false;
    }
    m_mounts[idx] = canonicalRoot.string();
    m_tempZipDirs[idx] = tempDir.string();
    m_currentDir[idx].clear();
    m_volumeLabels[idx] = label.empty() ? makeZipVolumeLabel(canonical) : label;
    return true;
  }

  if (!std::filesystem::is_directory(canonical, ec)) {
    return false;
  }

  m_mounts[idx] = canonical.string();
  m_currentDir[idx].clear();
  m_volumeLabels[idx] = label.empty() ? "MS-DOS_622" : label;
  return true;
}

bool DriveManager::unmount(char letter) {
  auto idx = letterToIndex(letter);
  if (idx == 25 || m_mounts[idx].empty()) {
    return false;
  }

  if (!m_tempZipDirs[idx].empty()) {
    std::error_code ec;
    std::filesystem::remove_all(m_tempZipDirs[idx], ec);
    m_tempZipDirs[idx].clear();
  }
  m_mounts[idx].clear();
  m_currentDir[idx].clear();
  m_volumeLabels[idx].clear();
  return true;
}

bool DriveManager::isMounted(char letter) const {
  return !m_mounts[letterToIndex(letter)].empty();
}

std::string DriveManager::getHostPath(char letter) const {
  return m_mounts[letterToIndex(letter)];
}

void DriveManager::setCurrentDrive(uint8_t driveIndex) {
  m_currentDrive = driveIndex;
}

void DriveManager::setCurrentDrive(char letter) {
  m_currentDrive = letterToIndex(letter);
}

uint8_t DriveManager::getCurrentDriveIndex() const {
  return m_currentDrive;
}

char DriveManager::getCurrentDriveLetter() const {
  return indexToLetter(m_currentDrive);
}

std::string DriveManager::getCurrentDir(uint8_t driveIndex) const {
  return m_currentDir[driveIndex];
}

std::string DriveManager::getCurrentDir(char letter) const {
  return m_currentDir[letterToIndex(letter)];
}

bool DriveManager::setCurrentDir(uint8_t driveIndex, const std::string& dosDir) {
  if (m_mounts[driveIndex].empty())
    return false;

  auto normalized = normalizeDosPath(dosDir);
  if (normalized.empty() || normalized == "\\") {
    m_currentDir[driveIndex].clear();
    return true;
  }

  if (normalized.front() == '\\')
    normalized.erase(normalized.begin());

  auto hostDir = matchHostCase(m_mounts[driveIndex], normalized);
  std::error_code ec;
  auto abs = std::filesystem::absolute(hostDir, ec);
  if (ec)
    return false;

  auto canonical = std::filesystem::canonical(abs, ec);
  if (ec || !std::filesystem::is_directory(canonical, ec))
    return false;

  auto mountRoot = std::filesystem::path(m_mounts[driveIndex]);
  auto rel = std::filesystem::relative(canonical, mountRoot, ec);
  if (ec || rel.empty() || escapesRoot(rel))
    return false;

  m_currentDir[driveIndex] = hostToDosStyle(rel.string());
  return true;
}

bool DriveManager::setCurrentDir(char letter, const std::string& dosDir) {
  return setCurrentDir(letterToIndex(letter), dosDir);
}

std::string DriveManager::getFullDosCwd() const {
  auto letter = indexToLetter(m_currentDrive);
  auto& dir = m_currentDir[m_currentDrive];
  if (dir.empty())
    return std::string(1, letter) + ":\\";
  return std::string(1, letter) + ":\\" + dir;
}

int DriveManager::getMountedCount() const {
  return static_cast<int>(std::count_if(m_mounts.begin(), m_mounts.end(),
    [](const std::string& s) { return !s.empty(); }));
}

std::string DriveManager::resolvePath(const std::string& dosPath) const {
  auto [driveLetter, remainder] = parseDrive(dosPath);
  auto idx = letterToIndex(driveLetter);

  if (m_mounts[idx].empty())
    return {};

  auto normalized = normalizeDosPath(remainder);
  if (normalized.empty())
    normalized = "\\";

  if (normalized.front() != '\\') {
    auto& cwd = m_currentDir[idx];
    if (!cwd.empty())
      normalized = "\\" + cwd + "\\" + normalized;
    else
      normalized = "\\" + normalized;
  }

  normalized = collapsePath(normalized);

  std::filesystem::path mountRoot(m_mounts[idx]);
  std::filesystem::path combined(
      matchHostCase(m_mounts[idx], normalized.substr(1)));

  std::error_code ec;
  auto abs = std::filesystem::absolute(combined, ec);
  if (ec)
    return {};

  auto canonical = std::filesystem::canonical(abs, ec);
  if (!ec) {
    auto rel = std::filesystem::relative(canonical, mountRoot, ec);
    if (!ec && !rel.empty() && !escapesRoot(rel))
      return canonical.string();
    return {};
  }

  auto normAbs = std::filesystem::weakly_canonical(abs, ec);
  if (ec)
    normAbs = abs;

  auto rel = std::filesystem::relative(normAbs, mountRoot, ec);
  if (ec || rel.empty() || escapesRoot(rel))
    return {};

  return normAbs.string();
}

std::string DriveManager::normalizeDosPath(const std::string& path) {
  std::string result;
  result.reserve(path.size());

  for (auto ch : path) {
    if (ch == '/')
      result += '\\';
    else
      result += static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }

  return result;
}

std::string DriveManager::getVolumeLabel(char letter) const {
  return m_volumeLabels[letterToIndex(letter)];
}

void DriveManager::setVolumeLabel(char letter, const std::string& label) {
  m_volumeLabels[letterToIndex(letter)] = label;
}

std::pair<char, std::string> DriveManager::parseDrive(const std::string& dosPath) const {
  if (dosPath.size() >= 2 && std::isalpha(static_cast<unsigned char>(dosPath[0])) && dosPath[1] == ':') {
    return { static_cast<char>(std::toupper(static_cast<unsigned char>(dosPath[0]))), dosPath.substr(2) };
  }
  return { indexToLetter(m_currentDrive), dosPath };
}

std::string DriveManager::collapsePath(const std::string& path) {
  std::vector<std::string> components;
  std::string current;

  for (auto ch : path) {
    if (ch == '\\' || ch == '/') {
      if (!current.empty()) {
        if (current == "..") {
          if (!components.empty() && components.back() != "..")
            components.pop_back();
        } else if (current != ".") {
          components.push_back(std::move(current));
        }
        current.clear();
      }
    } else {
      current += ch;
    }
  }

  if (!current.empty()) {
    if (current == "..") {
      if (!components.empty() && components.back() != "..")
        components.pop_back();
    } else if (current != ".") {
      components.push_back(std::move(current));
    }
  }

  std::string result = "\\";
  for (size_t i = 0; i < components.size(); ++i) {
    if (i > 0)
      result += '\\';
    result += components[i];
  }

  return result;
}

std::string DriveManager::hostToDosStyle(const std::string& path) {
  std::string result;
  result.reserve(path.size());
  for (auto ch : path) {
    if (ch == '/')
      result += '\\';
    else
      result += static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
  return result;
}

} // namespace fador::hw
