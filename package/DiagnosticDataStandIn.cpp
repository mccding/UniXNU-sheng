// SPDX-License-Identifier: BSD-2-Clause-Patent
// Ported from Pad 8 Pro (lenzhang/UniXNU-xiaomipad8pro DiagnosticDataStandIn.cpp).
// Provides AppleDiagnosticDataAccessReadOnly with erased 24-byte AppleDiagnosticDataSysCfg so libMobileGestalt does not stall 5 s.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSSymbol.h>

class AppleDiagnosticDataAccess : public IOService {
    OSDeclareDefaultStructors(AppleDiagnosticDataAccess)
};
OSDefineMetaClassAndStructors(AppleDiagnosticDataAccess, IOService)

class AppleDiagnosticDataAccessReadOnly : public AppleDiagnosticDataAccess {
    OSDeclareDefaultStructors(AppleDiagnosticDataAccessReadOnly)

public:
    bool start(IOService *provider) override;
};
OSDefineMetaClassAndStructors(AppleDiagnosticDataAccessReadOnly, AppleDiagnosticDataAccess)

namespace {
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#define DIAG_CONST __attribute__((section("__DATA_CONST,__const")))
#else
#define DIAG_CONST
#endif
static const unsigned char kErased[24] DIAG_CONST = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static const char kDiagKey[] DIAG_CONST = "AppleDiagnosticDataSysCfg";
static const char kDiagLogOk[] DIAG_CONST = "DiagnosticDataStandIn: AppleDiagnosticDataAccessReadOnly registered (AppleDiagnosticDataSysCfg %u bytes, erased)\n";
static const char kDiagLogNoProp[] DIAG_CONST = "DiagnosticDataStandIn: AppleDiagnosticDataAccessReadOnly registered without AppleDiagnosticDataSysCfg\n";
}

bool AppleDiagnosticDataAccessReadOnly::start(IOService *provider)
{
    if (!AppleDiagnosticDataAccess::start(provider)) return false;
    const OSSymbol *key = OSSymbol::withCStringNoCopy(kDiagKey);
    OSData *erased = key ? OSData::withBytes(kErased, sizeof kErased) : nullptr;
    bool published = false;
    if (erased) {
        published = setProperty(key, erased);
        erased->release();
    }
    if (key) key->release();
    registerService();
    if (published) {
        IOLog(kDiagLogOk, (unsigned)sizeof kErased);
    } else {
        IOLog(kDiagLogNoProp);
    }
    return true;
}
