// PianoDiskIdentity.h: what diskutil, Disk Utility and System Information show for sheng's own disks (NativeUfs's root windows,
// NativeRamBlock's Data disk), ported from Pad 8 Pro (lenzhang/UniXNU-xiaomipad8pro PianoDiskIdentity.h).
#pragma once

#include <IOKit/IOService.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSString.h>
#include <pexpert/pexpert.h>

namespace pianodisk {

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#define PIANO_DISK_CONST __attribute__((section("__DATA_CONST,__const")))
#else
#define PIANO_DISK_CONST
#endif

static const char kVendor[] PIANO_DISK_CONST = "Xiaomi";
static const char kInterconnect[] PIANO_DISK_CONST = "UFS";   // no constant of Apple's names UFS; diskutil prints the string as the Protocol

inline bool enabled()
{
    uint32_t value = 1;
    PE_parse_boot_argn("piano-disk-identity", &value, sizeof value);
    return value != 0;
}

inline bool putString(OSDictionary *dict, const char *key, const char *value)
{
    OSString *string = OSString::withCString(value);
    const bool ok = string && dict->setObject(key, string);
    if (string) string->release();
    return ok;
}

// On the block device (the IOBlockStorageDevice) before registerService, so the IOMedia IOBlockStorageDriver makes finds them.
inline bool publish(IOService *device, const char *product, const char *revision)
{
    if (!device || !product || !enabled()) return false;
    static const char kProtoInterconnect[] PIANO_DISK_CONST = "Physical Interconnect";
    static const char kProtoLocation[] PIANO_DISK_CONST = "Physical Interconnect Location";
    static const char kInternal[] PIANO_DISK_CONST = "Internal";
    static const char kVendorName[] PIANO_DISK_CONST = "Vendor Name";
    static const char kProductName[] PIANO_DISK_CONST = "Product Name";
    static const char kRevLevel[] PIANO_DISK_CONST = "Product Revision Level";
    static const char kDefaultRev[] PIANO_DISK_CONST = "1.0";
    static const char kMediumType[] PIANO_DISK_CONST = "Medium Type";
    static const char kSolidState[] PIANO_DISK_CONST = "Solid State";
    static const char kProtocolKey[] PIANO_DISK_CONST = "Protocol Characteristics";
    static const char kDeviceKey[] PIANO_DISK_CONST = "Device Characteristics";
    OSDictionary *protocol = OSDictionary::withCapacity(2), *characteristics = OSDictionary::withCapacity(4);
    bool ok = protocol && characteristics &&
              putString(protocol, kProtoInterconnect, kInterconnect) &&
              putString(protocol, kProtoLocation, kInternal) &&
              putString(characteristics, kVendorName, kVendor) &&
              putString(characteristics, kProductName, product) &&
              putString(characteristics, kRevLevel, revision ? revision : kDefaultRev) &&
              putString(characteristics, kMediumType, kSolidState);
    if (ok) ok = device->setProperty(kProtocolKey, protocol) && device->setProperty(kDeviceKey, characteristics);
    OSSafeReleaseNULL(protocol);
    OSSafeReleaseNULL(characteristics);
    return ok;
}

#undef PIANO_DISK_CONST

}  // namespace pianodisk
