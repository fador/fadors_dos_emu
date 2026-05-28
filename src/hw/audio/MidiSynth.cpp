#include "MidiSynth.hpp"
#include "../../utils/Logger.hpp"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

MidiSynth::MidiSynth(AdLib &adlib, float sampleRate)
    : m_adlib(adlib), m_sampleRate(sampleRate) {
  for (int i = 0; i < 9; i++) {
    m_voices[i].oplChannel = i;
  }
  LOG_INFO("MidiSynth initialized, sample rate: ", sampleRate);
}

void MidiSynth::handleEvent(uint8_t channel, uint8_t statusType, uint8_t data1,
                            uint8_t data2) {
  if (channel >= 16)
    return;

  switch (statusType) {
  case 0x80: // Note off
    noteOff(channel, data1);
    break;
  case 0x90: // Note on
    noteOn(channel, data1, data2);
    break;
  case 0xB0: // Control change
    controlChange(channel, data1, data2);
    break;
  case 0xC0: // Program change
    programChange(channel, data1);
    break;
  case 0xE0: // Pitch bend
    pitchBend(channel, data1, data2);
    break;
  default:
    break;
  }
}

void MidiSynth::noteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  if (velocity == 0) {
    noteOff(channel, note);
    return;
  }

  int voiceIdx = allocateVoice(channel, note);
  if (voiceIdx < 0)
    return;

  OPL2Voice &voice = m_voices[voiceIdx];
  int oplChan = voice.oplChannel;
  uint8_t program = m_channels[channel].program;
  const auto *inst = getInstrument(program);

  calcFnumBlock(note, m_channels[channel].pitchBend, voice.fnum, voice.block);

  loadInstrument(oplChan, program);

  uint8_t velocityTL =
      static_cast<uint8_t>(std::min(63, (127 - velocity) / 2));

  uint8_t op2tl = inst->op2_flags40 & 0x3F;
  op2tl = static_cast<uint8_t>(std::min(63, op2tl + velocityTL));
  uint8_t op1tl = inst->op1_flags40 & 0x3F;
  op1tl = static_cast<uint8_t>(std::min(63, op1tl + velocityTL));

  uint8_t reg40_op1 =
      static_cast<uint8_t>(0x40 + AdLib::opOffsets[oplChan]);
  uint8_t reg40_op2 =
      static_cast<uint8_t>(0x40 + AdLib::opOffsets[oplChan + 9]);
  m_adlib.writeRegister(reg40_op1, (inst->op1_flags40 & 0xC0) | op1tl);
  m_adlib.writeRegister(reg40_op2, (inst->op2_flags40 & 0xC0) | op2tl);

  uint8_t regA = 0xA0 + static_cast<uint8_t>(oplChan);
  uint8_t regB = 0xB0 + static_cast<uint8_t>(oplChan);
  m_adlib.writeRegister(regA, static_cast<uint8_t>(voice.fnum & 0xFF));

  uint8_t bval = static_cast<uint8_t>(((voice.fnum >> 8) & 0x03) | (voice.block << 2));
  bval |= 0x20;
  m_adlib.writeRegister(regB, bval);

  voice.midiChannel = channel;
  voice.note = note;
  voice.active = true;

  LOG_DEBUG("[MIDI] Note on: ch=", (int)channel, " note=", (int)note,
            " vel=", (int)velocity, " prog=", (int)program,
            " oplCh=", oplChan, " fnum=", voice.fnum, " block=", (int)voice.block);
}

void MidiSynth::noteOff(uint8_t channel, uint8_t note) {
  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel &&
        m_voices[i].note == note) {
      if (m_channels[channel].sustain) {
        return;
      }
      releaseVoice(i);
      return;
    }
  }

  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel) {
      if (m_channels[channel].sustain) {
        return;
      }
      releaseVoice(i);
      return;
    }
  }
}

void MidiSynth::releaseVoice(int voiceIdx) {
  OPL2Voice &voice = m_voices[voiceIdx];
  if (!voice.active)
    return;

  int oplChan = voice.oplChannel;

  int off1 = AdLib::opOffsets[oplChan];
  int off2 = AdLib::opOffsets[oplChan + 9];

  m_adlib.writeRegister(0x40 + off1, 0x3F);
  m_adlib.writeRegister(0x40 + off2, 0x3F);

  uint8_t regB = 0xB0 + static_cast<uint8_t>(oplChan);
  uint8_t bval = static_cast<uint8_t>(((voice.fnum >> 8) & 0x03) | (voice.block << 2));
  m_adlib.writeRegister(regB, bval);

  voice.active = false;
  voice.midiChannel = -1;
  voice.note = 0;
}

void MidiSynth::programChange(uint8_t channel, uint8_t program) {
  if (program >= 128)
    return;
  m_channels[channel].program = program;
  LOG_DEBUG("[MIDI] Program change: ch=", (int)channel, " prog=", (int)program);
}

void MidiSynth::controlChange(uint8_t channel, uint8_t controller,
                              uint8_t value) {
  switch (controller) {
  case 7: // Volume
    m_channels[channel].volume = value;
    break;
  case 10: // Pan
    m_channels[channel].pan = value;
    break;
  case 11: // Expression
    m_channels[channel].expression = value;
    break;
  case 64: // Sustain
    m_channels[channel].sustain = (value >= 64);
    if (!m_channels[channel].sustain) {
      for (int i = 0; i < 9; i++) {
        if (m_voices[i].active &&
            m_voices[i].midiChannel == channel) {
          auto it = std::find_if(m_voices.begin(), m_voices.end(),
                                 [channel, note = m_voices[i].note](
                                     const OPL2Voice &v) {
                                   return v.active &&
                                          v.midiChannel == channel &&
                                          v.note != note;
                                 });
          if (it == m_voices.end()) {
            releaseVoice(i);
          }
        }
      }
    }
    break;
  case 120: // All sound off
    for (int i = 0; i < 9; i++) {
      if (m_voices[i].active && m_voices[i].midiChannel == channel) {
        releaseVoice(i);
      }
    }
    break;
  case 121: // Reset all controllers
    m_channels[channel].volume = 127;
    m_channels[channel].expression = 127;
    m_channels[channel].pan = 64;
    m_channels[channel].pitchBend = 0;
    m_channels[channel].sustain = false;
    break;
  case 123: // All notes off
    for (int i = 0; i < 9; i++) {
      if (m_voices[i].active && m_voices[i].midiChannel == channel) {
        releaseVoice(i);
      }
    }
    break;
  default:
    break;
  }
}

void MidiSynth::pitchBend(uint8_t channel, uint8_t lsb, uint8_t msb) {
  int16_t value = static_cast<int16_t>((msb << 7) | lsb) - 8192;
  m_channels[channel].pitchBend = value;

  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel) {
      calcFnumBlock(m_voices[i].note, value, m_voices[i].fnum, m_voices[i].block);
      setFreq(m_voices[i].oplChannel, m_voices[i].fnum, m_voices[i].block);
    }
  }
}

void MidiSynth::calcFnumBlock(uint8_t note, int16_t pitchBend,
                              uint16_t &fnum, uint8_t &block) {
  float semitones = static_cast<float>(note) - 69.0f;
  semitones += static_cast<float>(pitchBend) / 8192.0f * 2.0f;

  float freq = 440.0f * std::pow(2.0f, semitones / 12.0f);

  static constexpr float OPL2_CLOCK = 49716.0f;

  block = 0;
  fnum = static_cast<uint16_t>(freq * 1048576.0f / OPL2_CLOCK);
  while (fnum > 1023 && block < 7) {
    block++;
    fnum = static_cast<uint16_t>(freq * 1048576.0f / (OPL2_CLOCK * (1 << block)));
  }
  if (fnum > 1023)
    fnum = 1023;
}

void MidiSynth::setFreq(int oplChan, uint8_t note, int16_t pitchBend) {
  uint16_t fnum;
  uint8_t block;
  calcFnumBlock(note, pitchBend, fnum, block);
  setFreq(oplChan, fnum, block);
}

void MidiSynth::setFreq(int oplChan, uint16_t fnum, uint8_t block) {
  uint8_t regA = 0xA0 + static_cast<uint8_t>(oplChan);
  uint8_t regB = 0xB0 + static_cast<uint8_t>(oplChan);

  m_adlib.writeRegister(regA, static_cast<uint8_t>(fnum & 0xFF));

  uint8_t bval = static_cast<uint8_t>(((fnum >> 8) & 0x03) | (block << 2));
  bval |= 0x20;
  m_adlib.writeRegister(regB, bval);
}

void MidiSynth::loadInstrument(int oplChan, uint8_t program) {
  const auto *inst = getInstrument(program);

  int off1 = AdLib::opOffsets[oplChan];
  int off2 = AdLib::opOffsets[oplChan + 9];

  m_adlib.writeRegister(0x20 + off1, inst->op1_flags20);
  m_adlib.writeRegister(0x40 + off1, inst->op1_flags40);
  m_adlib.writeRegister(0x60 + off1, inst->op1_flags60);
  m_adlib.writeRegister(0x80 + off1, inst->op1_flags80);
  if (inst->op1_waveform != 0) {
    m_adlib.writeRegister(0xE0 + off1, inst->op1_waveform);
  }

  m_adlib.writeRegister(0x20 + off2, inst->op2_flags20);
  m_adlib.writeRegister(0x40 + off2, inst->op2_flags40);
  m_adlib.writeRegister(0x60 + off2, inst->op2_flags60);
  m_adlib.writeRegister(0x80 + off2, inst->op2_flags80);
  if (inst->op2_waveform != 0) {
    m_adlib.writeRegister(0xE0 + off2, inst->op2_waveform);
  }

  uint8_t regC = 0xC0 + static_cast<uint8_t>(oplChan);
  uint8_t cval =
      static_cast<uint8_t>((inst->feedback << 1) | (inst->connection & 0x01));
  m_adlib.writeRegister(regC, cval);

  m_adlib.writeRegister(0x01, 0x20);
}

int MidiSynth::allocateVoice(uint8_t channel, uint8_t note) {
  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel &&
        m_voices[i].note == note) {
      releaseVoice(i);
      return i;
    }
  }

  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel) {
      int oldNote = m_voices[i].note;
      releaseVoice(i);
      LOG_DEBUG("[MIDI] Voice stolen: ch=", (int)channel,
                " oldNote=", oldNote, " newNote=", (int)note,
                " oplCh=", i);
      return i;
    }
  }

  for (int i = 0; i < 9; i++) {
    if (!m_voices[i].active) {
      return i;
    }
  }

  // Steal the voice with the oldest note
  int victim = 0;
  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active) {
      victim = i;
      break;
    }
  }
  LOG_DEBUG("[MIDI] Voice stolen (all busy): ch=", (int)channel,
            " note=", (int)note, " victimOplCh=", victim);
  releaseVoice(victim);
  return victim;
}

int MidiSynth::findVoice(uint8_t channel, uint8_t note) {
  for (int i = 0; i < 9; i++) {
    if (m_voices[i].active && m_voices[i].midiChannel == channel &&
        m_voices[i].note == note) {
      return i;
    }
  }
  return -1;
}

void MidiSynth::allNotesOff() {
  for (int i = 0; i < 9; i++) {
    releaseVoice(i);
  }
}

void MidiSynth::reset() {
  allNotesOff();
  for (auto &ch : m_channels) {
    ch = MidiChannelState{};
  }
  for (auto &voice : m_voices) {
    voice = OPL2Voice{};
    voice.oplChannel = static_cast<int>(&voice - &m_voices[0]);
  }
}

} // namespace fador::hw::audio
