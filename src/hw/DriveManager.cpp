#include "DriveManager.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>

namespace fador::hw {

bool DriveManager::mount(char letter, const std::string& hostPath, const std::string& label) {
  auto idx = letterToIndex(letter);
  if (!m_mounts[idx].empty())
    return false;

  std::error_code ec;
  auto canonical = std::filesystem::canonical(hostPath, ec);
  if (ec || !std::filesystem::is_directory(canonical, ec))
    return false;

  m_mounts[idx] = canonical.string();
  m_currentDir[idx].clear();
  m_volumeLabels[idx] = label.empty() ? "MS-DOS_622" : label;
  return true;
}

bool DriveManager::unmount(char letter) {
  auto idx = letterToIndex(letter);
  if (idx == 25 || m_mounts[idx].empty())
    return false;

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

  auto hostDir = m_mounts[driveIndex] + "\\" + normalized;
  std::error_code ec;
  auto abs = std::filesystem::absolute(hostDir, ec);
  if (ec)
    return false;

  auto canonical = std::filesystem::canonical(abs, ec);
  if (ec || !std::filesystem::is_directory(canonical, ec))
    return false;

  auto mountRoot = std::filesystem::path(m_mounts[driveIndex]);
  auto rel = std::filesystem::relative(canonical, mountRoot, ec);
  if (ec || rel.empty() || rel.native().starts_with(L".."))
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
  std::filesystem::path dosPart = std::filesystem::path(normalized.substr(1));
  std::filesystem::path combined = mountRoot / dosPart;

  std::error_code ec;
  auto abs = std::filesystem::absolute(combined, ec);
  if (ec)
    return {};

  auto canonical = std::filesystem::canonical(abs, ec);
  if (!ec) {
    auto rel = std::filesystem::relative(canonical, mountRoot, ec);
    if (!ec && !rel.empty() && !rel.native().starts_with(L".."))
      return canonical.string();
    return {};
  }

  auto normAbs = std::filesystem::weakly_canonical(abs, ec);
  if (ec)
    normAbs = abs;

  auto rel = std::filesystem::relative(normAbs, mountRoot, ec);
  if (ec || rel.empty() || rel.native().starts_with(L".."))
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
