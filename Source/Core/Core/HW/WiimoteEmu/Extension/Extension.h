// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/HW/WiimoteEmu/Extension/Extension.h"

#include <array>
#include <string>

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Core/HW/WiimoteEmu/Encryption.h"
#include "Core/HW/WiimoteEmu/I2CBus.h"
#include "InputCommon/ControllerEmu/ControllerEmu.h"

namespace WiimoteEmu
{
class Extension : public ControllerEmu::EmulatedController, public I2CSlave
{
public:
  explicit Extension(const char* name);
  Extension(const char* config_name, const char* display_name);

  std::string GetName() const override;
  std::string GetDisplayName() const override;

  // Used by the wiimote to detect extension changes.
  // The normal extensions short this pin so it's always connected,
  // but M+ does some tricks with it during activation.
  virtual bool ReadDeviceDetectPin() const = 0;

  virtual bool IsButtonPressed() const = 0;
  virtual void Reset() = 0;
  virtual void DoState(PointerWrap& p) = 0;
  virtual void Update() = 0;

private:
  const char* const m_config_name;
  const char* const m_display_name;
};

class None : public Extension
{
public:
  explicit None();

private:
  bool ReadDeviceDetectPin() const override;
  void Update() override;
  bool IsButtonPressed() const override;
  void Reset() override;
  void DoState(PointerWrap& p) override;

  int BusRead(u8 slave_addr, u8 addr, int count, u8* data_out) override;
  int BusWrite(u8 slave_addr, u8 addr, int count, const u8* data_in) override;
};

// This class provides the encryption and initialization behavior of most extensions.
class EncryptedExtension : public Extension
{
public:
  static constexpr u8 I2C_ADDR = 0x52;
  static constexpr int CONTROLLER_DATA_BYTES = 21;

  using Extension::Extension;

  // TODO: This is public for TAS reasons.
  // TODO: TAS handles encryption poorly.
  EncryptionKey ext_key;

protected:
  static constexpr int CALIBRATION_CHECKSUM_BYTES = 2;

#pragma pack(push, 1)
  struct Register
  {
    // 21 bytes of possible extension data
    u8 controller_data[CONTROLLER_DATA_BYTES];

    u8 unknown2[11];

    // address 0x20
    std::array<u8, 0x10> calibration;
    u8 unknown3[0x10];

    // address 0x40
    std::array<u8, 0x10> encryption_key_data;
    u8 unknown4[0xA0];

    // address 0xF0
    u8 encryption;
    u8 unknown5[0x9];

    // address 0xFA
    std::array<u8, 6> identifier;
  };
#pragma pack(pop)

  static_assert(0x100 == sizeof(Register));

  Register m_reg = {};

  void Reset() override;
  void DoState(PointerWrap& p) override;

  virtual void UpdateEncryptionKey() = 0;

public:
  bool m_is_key_dirty = true;

  static constexpr u8 ENCRYPTION_ENABLED = 0xaa;

  bool ReadDeviceDetectPin() const override;

  int BusRead(u8 slave_addr, u8 addr, int count, u8* data_out) override;
  int BusWrite(u8 slave_addr, u8 addr, int count, const u8* data_in) override;
};

class Extension1stParty : public EncryptedExtension
{
protected:
  using EncryptedExtension::EncryptedExtension;

private:
  void UpdateEncryptionKey() final override;
};

class Extension3rdParty : public EncryptedExtension
{
protected:
  using EncryptedExtension::EncryptedExtension;

private:
  void UpdateEncryptionKey() final override;
};

}  // namespace WiimoteEmu
