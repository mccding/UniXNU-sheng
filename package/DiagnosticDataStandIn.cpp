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
constexpr unsigned char kErased[24] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                       0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
}

bool AppleDiagnosticDataAccessReadOnly::start(IOService *provider)
{
    if (!AppleDiagnosticDataAccess::start(provider)) return false;
    const OSSymbol *key = OSSymbol::withCStringNoCopy("AppleDiagnosticDataSysCfg");
    OSData *erased = key ? OSData::withBytes(kErased, sizeof kErased) : nullptr;
    bool published = false;
    if (erased) {
        published = setProperty(key, erased);
        erased->release();
    }
    if (key) key->release();
    registerService();
    if (published) {
        IOLog("DiagnosticDataStandIn: AppleDiagnosticDataAccessReadOnly registered (AppleDiagnosticDataSysCfg %u bytes, erased)\n",
              (unsigned)sizeof kErased);
    } else {
        IOLog("DiagnosticDataStandIn: AppleDiagnosticDataAccessReadOnly registered without AppleDiagnosticDataSysCfg\n");
    }
    return true;
}
