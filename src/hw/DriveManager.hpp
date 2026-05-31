#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>

namespace fador::hw {

class DriveManager {
public:
  DriveManager() = default;
  ~DriveManager() = default;

  bool mount(char letter, const std::string& hostPath, const std::string& label = "");
  bool unmount(char letter);
  bool isMounted(char letter) const;
  std::string getHostPath(char letter) const;

  void setCurrentDrive(uint8_t driveIndex);
  void setCurrentDrive(char letter);
  uint8_t getCurrentDriveIndex() const;
  char getCurrentDriveLetter() const;

  std::string getCurrentDir(uint8_t driveIndex) const;
  std::string getCurrentDir(char letter) const;
  bool setCurrentDir(uint8_t driveIndex, const std::string& dosDir);
  bool setCurrentDir(char letter, const std::string& dosDir);

  std::string getFullDosCwd() const;
  int getMountedCount() const;

  std::string resolvePath(const std::string& dosPath) const;

  static std::string normalizeDosPath(const std::string& path);

  std::string getVolumeLabel(char letter) const;
  void setVolumeLabel(char letter, const std::string& label);

private:
  std::pair<char, std::string> parseDrive(const std::string& dosPath) const;
  static std::string collapsePath(const std::string& path);
  static std::string hostToDosStyle(const std::string& path);

  static uint8_t letterToIndex(char letter) {
    return static_cast<uint8_t>(std::toupper(static_cast<unsigned char>(letter)) - 'A');
  }

  static constexpr char indexToLetter(uint8_t index) {
    return static_cast<char>('A' + index);
  }

  std::array<std::string, 26> m_mounts{};
  std::array<std::string, 26> m_currentDir{};
  std::array<std::string, 26> m_volumeLabels{};
  uint8_t m_currentDrive = 25;
};

} // namespace fador::hw
