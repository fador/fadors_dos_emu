#pragma once

#include "AdLib.hpp"
#include "MidiInstruments.hpp"
#include <array>
#include <cstdint>

namespace fador::hw::audio {

struct MidiChannelState {
  uint8_t program = 0;
  uint8_t volume = 127;
  uint8_t expression = 127;
  uint8_t pan = 64;
  int16_t pitchBend = 0;
  bool sustain = false;
};

struct OPL2Voice {
  int midiChannel = -1;
  uint8_t note = 0;
  int oplChannel = -1;
  bool active = false;
  uint16_t fnum = 0;
  uint8_t block = 0;
};

class MidiSynth {
public:
  MidiSynth(AdLib &adlib, float sampleRate = 44100.0f);
  ~MidiSynth() = default;

  void handleEvent(uint8_t channel, uint8_t statusType, uint8_t data1,
                   uint8_t data2);
  void allNotesOff();
  void reset();

private:
  void noteOn(uint8_t channel, uint8_t note, uint8_t velocity);
  void noteOff(uint8_t channel, uint8_t note);
  void programChange(uint8_t channel, uint8_t program);
  void controlChange(uint8_t channel, uint8_t controller, uint8_t value);
  void pitchBend(uint8_t channel, uint8_t lsb, uint8_t msb);

  void loadInstrument(int oplChan, uint8_t program);
  void setFreq(int oplChan, uint8_t note, int16_t pitchBend);
  void setFreq(int oplChan, uint16_t fnum, uint8_t block);
  void calcFnumBlock(uint8_t note, int16_t pitchBend, uint16_t &fnum,
                     uint8_t &block);
  int allocateVoice(uint8_t channel, uint8_t note);
  int findVoice(uint8_t channel, uint8_t note);
  void releaseVoice(int voiceIdx);

  AdLib &m_adlib;
  float m_sampleRate;

  std::array<MidiChannelState, 16> m_channels;
  std::array<OPL2Voice, 9> m_voices;
};

} // namespace fador::hw::audio
