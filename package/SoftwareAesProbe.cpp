// SPDX-License-Identifier: BSD-2-Clause-Patent
// macOS: registers the software AES-CBC provider (UniXNU drivers/SoftwareAES, NativeAES) and the Pad 8 Pro
// AppleDiagnosticDataAccessReadOnly stand-in (DiagnosticDataStandIn.cpp).
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/OSKextLib.h>
#include <kern/thread.h>

extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_aes_static_init(void) __asm__("__GLOBAL__sub_I_NativeAES.cpp");
extern "C" void diagnostic_data_static_init(void) __asm__("__GLOBAL__sub_I_DiagnosticDataStandIn.cpp");
// 0 not run, 1 started (NativeAES registered itself), negative: the step that failed.
extern "C" volatile int sheng_software_aes_state = 0;

namespace {

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#define AES_PROBE_CONST __attribute__((section("__DATA_CONST,__const")))
#else
#define AES_PROBE_CONST
#endif
static const char kAesFailLog[] AES_PROBE_CONST = "SoftwareAES: not started (%d)\n";
static const char kAesOkLog[] AES_PROBE_CONST = "SoftwareAES: IOAESAccelerator registered\n";
static const char kDiagPostFailLog[] AES_PROBE_CONST = "DiagnosticDataStandIn: postModLoad failed\n";
static const char kDiagSkipLog[] AES_PROBE_CONST = "DiagnosticDataStandIn: start skipped\n";

void fail(int step)
{
    sheng_software_aes_state = -step;
    IOLog(kAesFailLog, -step);
    thread_terminate(current_thread());
}

void registerDiagnosticStandIn(IOService *parent)
{
    if (!parent) return;
    void *diagHandle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!diagHandle) return;
    diagnostic_data_static_init();
    if (OSMetaClass::postModLoad(diagHandle)) {
        IOLog(kDiagPostFailLog);
        return;
    }
    OSObject *obj = OSMetaClass::allocClassWithName("AppleDiagnosticDataAccessReadOnly");
    IOService *diag = OSDynamicCast(IOService, obj);
    const bool attached = diag && diag->init() && diag->attach(parent);
    if (!attached || !diag->start(parent)) {
        if (attached) diag->detach(parent);
        IOLog(kDiagSkipLog);
    }
    OSSafeReleaseNULL(obj);
}

void worker(void *, wait_result_t)
{
    const OSReturn loaded = OSKextLoadKextWithIdentifier("com.apple.iokit.IOCryptoAcceleratorFamily");
    if (loaded != kOSReturnSuccess && loaded != kOSKextReturnStartStopError) fail(1);
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) fail(2);
    native_aes_static_init();
    if (OSMetaClass::postModLoad(handle)) fail(3);
    OSObject *object = OSMetaClass::allocClassWithName("NativeAES");
    IOService *aes = OSDynamicCast(IOService, object);
    IOService *parent = IOService::getResourceService();
    if (!aes || !parent || !aes->init()) {OSSafeReleaseNULL(object); fail(4);}
    if (!aes->attach(parent)) {aes->release(); fail(5);}
    if (!aes->start(parent)) {aes->detach(parent); aes->release(); fail(6);}
    aes->release();
    sheng_software_aes_state = 1;
    IOLog(kAesOkLog);
    registerDiagnosticStandIn(parent);
    thread_terminate(current_thread());
}

}  // namespace

extern "C" void sheng_software_aes_probe(void)
{
    thread_t thread = nullptr;
    if (kernel_thread_start(worker, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else sheng_software_aes_state = -7;
}
