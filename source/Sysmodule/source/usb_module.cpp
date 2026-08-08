#include <switch.h>
#include "usb_module.h"
#include "controller_handler.h"
#include "Controllers.h"
#include "Controllers/ShanWanPantherLordController.h"

#include "SwitchUSBDevice.h"
#include "SwitchUSBLock.h"
#include "logger.h"
#include <string.h>

#define MS_TO_NS(x) (x * 1000000ul)

namespace syscon::usb
{
    namespace
    {
        constexpr size_t MaxUsbHsInterfacesSize = 8;
        constexpr size_t MaxUsbEvents = 3; // MaxUsbEvents is limited by usbHsCreateInterfaceAvailableEvent, we can have only up to 3 events

        // Thread that waits on generic usb event
        void UsbEventThreadFunc(void *arg);
        // Thread that waits on any disconnected usb devices
        void UsbInterfaceChangeThreadFunc(void *arg);

        alignas(0x1000) u8 usb_event_thread_stack[0x4000];
        alignas(0x1000) u8 usb_interface_change_thread_stack[0x4000];

        Thread g_usb_event_thread;
        Thread g_usb_interface_change_thread;

        bool is_usb_event_thread_running = false;
        bool is_usb_interface_change_thread_running = false;
        bool g_auto_add_controller = false;

        Event g_usbEvent[MaxUsbEvents] = {};
        Waiter g_usbWaiters[MaxUsbEvents] = {};
        size_t g_usbEventCount = 0;

        s32 QueryAcquiredInterfaces(UsbHsInterface *interfaces, size_t interfaces_maxsize);
        s32 QueryAvailableInterfacesByClass(UsbHsInterface *interfaces, size_t interfaces_maxsize, u8 iclass);
        s32 QueryAvailableInterfacesByClassSubClassProtocol(UsbHsInterface *interfaces, size_t interfaces_maxsize, u8 iclass, u8 isubclass, u8 iprotocol);

        Result AddEvent(UsbHsInterfaceFilter *filter, const std::string &name);

// --- DIAG TEMPORAIRE : sondes usb:qdb + usb:obsv ---
        // usb:qdb::HasQuirk (cmd 1) -> le firmware a-t-il une entree de quirk pour 0810:0000 ?
        void ProbeQuirkDb()
        {
            Result r = smInitialize();
            if (R_FAILED(r))
            {
                syscon::logger::LogInfo("[DIAG] sm init failed rc=0x%x", r);
                return;
            }

            Service qdb = {};
            r = smGetService(&qdb, "usb:qdb");
            if (R_FAILED(r))
            {
                syscon::logger::LogInfo("[DIAG] usb:qdb service open failed rc=0x%x", r);
                smExit();
                return;
            }

            const struct
            {
                u16 vid;
                u16 pid;
                u16 bcdDevice;
            } in = { SHANWAN_VID, SHANWAN_PID, 0x0000 };

            static const char *quirk_names[] = {
                "HidGamepadWhitelist",
                "ApplicationBlacklist",
                "NoClearHaltOnEpInit",
            };

            for (const char *name : quirk_names)
            {
                u8 out = 0;
                Result rc = serviceDispatchInOut(&qdb, 1, in, out,
                    .buffer_attrs = { SfBufferAttr_In | SfBufferAttr_HipcMapAlias },
                    .buffers = { { name, strlen(name) } });
                syscon::logger::LogInfo("[DIAG] usb:qdb HasQuirk(%s) rc=0x%x result=%u (0=absent, 1=present)", name, rc, out);
            }

            serviceClose(&qdb);
            smExit();
        }

        // usb:obsv::GetFlattenedTopology (cmd 1) -> dongle present au niveau port/hub ?
        void ProbeTopology()
        {
            Result r = smInitialize();
            if (R_FAILED(r))
            {
                syscon::logger::LogInfo("[DIAG] sm init failed rc=0x%x", r);
                return;
            }

            Service svc = {};
            r = smGetService(&svc, "usb:obsv");
            if (R_FAILED(r))
            {
                syscon::logger::LogInfo("[DIAG] usb:obsv service open failed rc=0x%x", r);
                smExit();
                return;
            }

            alignas(0x1000) static u8 topo_buf[0x2000] = {};
            static bool topo_dumped = false;

            Result rc = serviceDispatch(&svc, 1,
                .buffer_attrs = { SfBufferAttr_HipcMapAlias | SfBufferAttr_Out },
                .buffers = { { topo_buf, sizeof(topo_buf) } });

            if (R_FAILED(rc))
            {
                syscon::logger::LogInfo("[DIAG] usb:obsv GetFlattenedTopology rc=0x%x", rc);
            }

            serviceClose(&svc);
            smExit();

            // Recherche de signatures : 0810-0001 (LE) / 0f0d-00c1 (LE)
            size_t shanwan_hit = 0, hori_hit = 0, vid0810_hit = 0;
            for (size_t i = 0; i + 3 < sizeof(topo_buf); i++)
            {
                if (topo_buf[i + 0] == 0x10 && topo_buf[i + 1] == 0x08 && topo_buf[i + 2] == 0x01 && topo_buf[i + 3] == 0x00)
                    shanwan_hit++;
                if (topo_buf[i + 0] == 0x0D && topo_buf[i + 1] == 0x0F && topo_buf[i + 2] == 0xC1 && topo_buf[i + 3] == 0x00)
                    hori_hit++;
            }
            for (size_t i = 0; i + 1 < sizeof(topo_buf); i++)
            {
                if (topo_buf[i + 0] == 0x10 && topo_buf[i + 1] == 0x08)
                    vid0810_hit++;
            }

            syscon::logger::LogInfo("[DIAG] Topology: shanwan(0810-0001)=%zu hori(0f0d-00c1)=%zu vid0810raw=%zu", shanwan_hit, hori_hit, vid0810_hit);

            if (!topo_dumped)
            {
                topo_dumped = true;
                for (size_t i = 0; i < sizeof(topo_buf); i += 0x10)
                {
                    syscon::logger::LogInfo("[DIAG] topo[%04zx]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                        i,
                        topo_buf[i + 0], topo_buf[i + 1], topo_buf[i + 2], topo_buf[i + 3],
                        topo_buf[i + 4], topo_buf[i + 5], topo_buf[i + 6], topo_buf[i + 7],
                        topo_buf[i + 8], topo_buf[i + 9], topo_buf[i + 10], topo_buf[i + 11],
                        topo_buf[i + 12], topo_buf[i + 13], topo_buf[i + 14], topo_buf[i + 15]);
                }
            }
        }
        // Probe hub 1a40:0101: cherche Alternate Setting 1 (MTT) et log tout
        // Appelee une fois (cadencee par topo_iter, comme les autres sondes)
        void ProbeHubAltSettings()
        {
            constexpr u16 TARGET_VID = 0x1a40;
            constexpr u16 TARGET_PID = 0x0101;

            UsbHsInterfaceFilter filter = {};
            filter.Flags = UsbHsInterfaceFilterFlags_idVendor | UsbHsInterfaceFilterFlags_idProduct;
            filter.idVendor = TARGET_VID;
            filter.idProduct = TARGET_PID;

            UsbHsInterface hubs[4] = {};
            s32 count = 0;

            Result r = usbHsQueryAllInterfaces(&filter, hubs, sizeof(hubs), &count);
            if (R_FAILED(r) || count == 0)
            {
                syscon::logger::LogInfo("[DIAG] Hub %04x:%04x not found via QueryAllInterfaces (rc=0x%x count=%d)", TARGET_VID, TARGET_PID, r, count);
                return;
            }

            syscon::logger::LogInfo("[DIAG] Hub %04x:%04x found count=%d", TARGET_VID, TARGET_PID, count);

            for (s32 i = 0; i < count; i++)
            {
                syscon::logger::LogInfo("[DIAG]  Hub[%d]: interface=0x%02x class=0x%02x sub=0x%02x proto=0x%02x",
                    i,
                    hubs[i].inf.interface_desc.bInterfaceNumber,
                    hubs[i].inf.interface_desc.bInterfaceClass,
                    hubs[i].inf.interface_desc.bInterfaceSubClass,
                    hubs[i].inf.interface_desc.bInterfaceProtocol);

                // Acquerir l'interface
                UsbHsClientIfSession sess = {};
                Result rc = usbHsAcquireUsbIf(&sess, &hubs[i]);
                if (R_FAILED(rc))
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: AcquireUsbIf failed rc=0x%x", i, rc);
                    continue;
                }

                // GetInterface (etat actuel)
                UsbHsInterfaceInfo cur_inf = {};
                rc = usbHsIfGetInterface(&sess, &cur_inf);
                if (R_SUCCEEDED(rc))
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: GetInterface alt=%d class=0x%02x proto=0x%02x",
                        i, cur_inf.interface_desc.bAlternateSetting,
                        cur_inf.interface_desc.bInterfaceClass,
                        cur_inf.interface_desc.bInterfaceProtocol);
                }
                else
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: GetInterface failed rc=0x%x", i, rc);
                }

                // GetAlternateInterface(0)
                UsbHsInterfaceInfo alt0 = {};
                rc = usbHsIfGetAlternateInterface(&sess, &alt0, 0);
                if (R_SUCCEEDED(rc))
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: AltSetting0 alt=%d class=0x%02x sub=0x%02x proto=0x%02x numEp=%d",
                        i, alt0.interface_desc.bAlternateSetting,
                        alt0.interface_desc.bInterfaceClass,
                        alt0.interface_desc.bInterfaceSubClass,
                        alt0.interface_desc.bInterfaceProtocol,
                        alt0.interface_desc.bNumEndpoints);
                    // hexdump partiel
                    const u8 *d = (const u8*)&alt0;
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: alt0[0..31] = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                        i,
                        d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11],d[12],d[13],d[14],d[15],
                        d[16],d[17],d[18],d[19],d[20],d[21],d[22],d[23],d[24],d[25],d[26],d[27],d[28],d[29],d[30],d[31]);
                }
                else
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: AltSetting0 failed rc=0x%x", i, rc);
                }

                // GetAlternateInterface(1) -> MTT ?
                UsbHsInterfaceInfo alt1 = {};
                rc = usbHsIfGetAlternateInterface(&sess, &alt1, 1);
                if (R_SUCCEEDED(rc))
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: ** AltSetting1 EXISTS ** alt=%d class=0x%02x sub=0x%02x proto=0x%02x numEp=%d",
                        i, alt1.interface_desc.bAlternateSetting,
                        alt1.interface_desc.bInterfaceClass,
                        alt1.interface_desc.bInterfaceSubClass,
                        alt1.interface_desc.bInterfaceProtocol,
                        alt1.interface_desc.bNumEndpoints);
                    const u8 *d = (const u8*)&alt1;
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: alt1[0..31] = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                        i,
                        d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7],d[8],d[9],d[10],d[11],d[12],d[13],d[14],d[15],
                        d[16],d[17],d[18],d[19],d[20],d[21],d[22],d[23],d[24],d[25],d[26],d[27],d[28],d[29],d[30],d[31]);
                }
                else
                {
                    syscon::logger::LogInfo("[DIAG]  Hub[%d]: AltSetting1 NOT available (rc=0x%x) -> MTT improbable", i, rc);
                }

                usbHsIfClose(&sess);
            }
        }
        // --- FIN DIAG ---

        void UsbEventThreadFunc(void *arg)
        {
            UsbHsInterface interfaces[MaxUsbHsInterfacesSize] = {};
            u64 timeoutNs = MS_TO_NS(1);
            (void)arg;

            do
            {

                /*
                    Weird issue with this function, This function will never return in some cases (XBOX Serie for example #14)
                    So, we need to use the timeout=1ms to avoid being stuck in this function the first call, and then we can use the UINT64_MAX
                */
                s32 idx_out = 0;
                Result rc = waitObjects(&idx_out, g_usbWaiters, g_usbEventCount, timeoutNs);
                if (R_SUCCEEDED(rc) || R_VALUE(rc) == KERNELRESULT(TimedOut))
                {
                    syscon::logger::LogDebug("USB event poll: idx_out=%d events=%zu rc=0x%x", idx_out, g_usbEventCount, rc);

                    // --- DIAG TEMPORAIRE : cadence pour ne pas polluer le log ---
                    {
                        static bool quirk_probed = false;
                        if (!quirk_probed)
                        {
                            ProbeQuirkDb();
                            ProbeTopology(); // baseline (rien branche)
                            quirk_probed = true;
                        }

                        static u64 topo_iter = 0;
                        topo_iter++;
                        if ((topo_iter & 0x0F) == 0) // ~15 poll => ~8s avec timeout 1s en absence de device
                        {
                            ProbeTopology();
                        }
                        if ((topo_iter & 0x01) == 0) // ~1s
                        {
                            ProbeHubAltSettings();
                        }
                    }
                    // --- FIN DIAG(source cadence) ---

                    /*
                        For unknown reason we have to keep this lock in order to lock the usb stacks during the controller initialization
                        If we don't do that, we will have some issue with the USB stack when we have multiple controllers connected at boot time
                        (Example: Not being able to setLed to the device - On XBOX360 wired controller)
                    */

                    SwitchUSBLock usbLock;

                    // --- DEBUG TEMPORAIRE : diagnostic de toutes les interfaces USB visibles par usbHs ---
                    // A retirer une fois le diagnostic termine.
                    {
                        const auto dump_interface = [](const char *tag, int i, const UsbHsInterface &iface)
                        {
                            syscon::logger::LogInfo("[DEBUG] %s[%d]: infID=%d class=0x%02x sub=0x%02x proto=0x%02x ep=%d VID=0x%04x PID=0x%04x devclass=0x%02x bcd=0x%04x",
                                tag,
                                i,
                                iface.inf.ID,
                                iface.inf.interface_desc.bInterfaceClass,
                                iface.inf.interface_desc.bInterfaceSubClass,
                                iface.inf.interface_desc.bInterfaceProtocol,
                                iface.inf.interface_desc.bNumEndpoints,
                                iface.device_desc.idVendor,
                                iface.device_desc.idProduct,
                                iface.device_desc.bDeviceClass,
                                iface.device_desc.bcdDevice);
                        };

                        static UsbHsInterface acquired_interfaces[MaxUsbHsInterfacesSize] = {};
                        s32 acquired_count = QueryAcquiredInterfaces(acquired_interfaces, sizeof(acquired_interfaces));
                        syscon::logger::LogInfo("[DEBUG] Acquired interfaces: %d", acquired_count);
                        for (s32 i = 0; i < acquired_count; i++)
                            dump_interface("Acquired", i, acquired_interfaces[i]);

                        static UsbHsInterface avail_debug[MaxUsbHsInterfacesSize] = {};
                        s32 avail_debug_count = 0;

                        // Filtre vide = toutes les interfaces disponibles (pas seulement le VID 0810)
                        UsbHsInterfaceFilter filterAll{
                            .Flags = 0,
                        };
                        memset(avail_debug, 0, sizeof(avail_debug));
                        Result rc = usbHsQueryAvailableInterfaces(&filterAll, avail_debug, sizeof(avail_debug), &avail_debug_count);
                        syscon::logger::LogInfo("[DEBUG] usbHsQueryAvailableInterfaces(ALL) rc=0x%x (module=%d desc=%d) count=%d", rc, R_MODULE(rc), R_DESCRIPTION(rc), avail_debug_count);
                        for (s32 i = 0; i < avail_debug_count; i++)
                            dump_interface("Available", i, avail_debug[i]);

                        // usbHsQueryAllInterfaces : interfaces disponibles + acquises (que device soient en use ou non)
                        static UsbHsInterface all_debug[MaxUsbHsInterfacesSize] = {};
                        s32 all_debug_count = 0;
                        memset(all_debug, 0, sizeof(all_debug));
                        Result rcAll = usbHsQueryAllInterfaces(&filterAll, all_debug, sizeof(all_debug), &all_debug_count);
                        syscon::logger::LogInfo("[DEBUG] usbHsQueryAllInterfaces(ALL) rc=0x%x count=%d", rcAll, all_debug_count);
                        for (s32 i = 0; i < all_debug_count; i++)
                        {
                            const UsbHsInterface &iface = all_debug[i];
                            syscon::logger::LogInfo("[DEBUG] All[%d]: infID=%d VID=0x%04x PID=0x%04x bcd=0x%04x class=0x%02x sub=0x%02x proto=0x%02x classif=0x%02x subif=0x%02x protoif=0x%02x",
                                i,
                                iface.inf.ID,
                                iface.device_desc.idVendor,
                                iface.device_desc.idProduct,
                                iface.device_desc.bcdDevice,
                                iface.device_desc.bDeviceClass,
                                iface.device_desc.bDeviceSubClass,
                                iface.device_desc.bDeviceProtocol,
                                iface.inf.interface_desc.bInterfaceClass,
                                iface.inf.interface_desc.bInterfaceSubClass,
                                iface.inf.interface_desc.bInterfaceProtocol);
                        }
                    }
                    // --- FIN DEBUG ---

                    // --- ShanWan PantherLord (VID 0810 PID 0001) : detection dediee ---
                    // (HID standard ne le detecte pas : QueryAvailableInterfacesByClass
                    // retourne 0 pour ce device)
                    if (!controllers::IsAtControllerLimit())
                    {
                        s32 shanwan_count = 0;
                        memset(interfaces, 0, sizeof(interfaces));

                        UsbHsInterfaceFilter filterShanwan{
                            .Flags = UsbHsInterfaceFilterFlags_idVendor | UsbHsInterfaceFilterFlags_idProduct,
                            .idVendor = SHANWAN_VID,
                            .idProduct = SHANWAN_PID,
                        };

                        if (R_SUCCEEDED(usbHsQueryAvailableInterfaces(&filterShanwan, interfaces, sizeof(interfaces), &shanwan_count))
                            && shanwan_count > 0)
                        {
                            timeoutNs = MS_TO_NS(1);

                            syscon::logger::LogInfo("Trying to initialize ShanWan PantherLord: [%04x-%04x] ...",
                                                    interfaces[0].device_desc.idVendor, interfaces[0].device_desc.idProduct);

                            ControllerConfig config;
                            ::syscon::config::LoadControllerConfig(CONFIG_FULLPATH, &config,
                                interfaces[0].device_desc.idVendor, interfaces[0].device_desc.idProduct,
                                g_auto_add_controller, "");

                            controllers::Insert(std::make_unique<ShanWanPantherLordController>(
                                std::make_unique<SwitchUSBDevice>(interfaces, 1),
                                config,
                                std::make_unique<syscon::logger::Logger>()));

                            continue;
                        }
                    }
                    // --- fin ShanWan PantherLord ---

                    s32 total_interfaces_hid = 0, total_interfaces_xbox360 = 0, total_interfaces_xboxone = 0, total_interfaces_xbox360w = 0, total_interfaces_xbox = 0;

                    if (
                        (total_interfaces_xbox360 = QueryAvailableInterfacesByClassSubClassProtocol(interfaces, sizeof(interfaces), USB_CLASS_VENDOR_SPEC, 0x5D, 0x01)) > 0 ||  // XBOX360 Wired
                        (total_interfaces_xbox360w = QueryAvailableInterfacesByClassSubClassProtocol(interfaces, sizeof(interfaces), USB_CLASS_VENDOR_SPEC, 0x5D, 0x81)) > 0 || // XBOX360 Wireless
                        (total_interfaces_xboxone = QueryAvailableInterfacesByClassSubClassProtocol(interfaces, sizeof(interfaces), USB_CLASS_VENDOR_SPEC, 0x47, 0xD0)) > 0 ||  // XBOX ONE
                        (total_interfaces_xbox = QueryAvailableInterfacesByClassSubClassProtocol(interfaces, sizeof(interfaces), 0x58, 0x42, 0x00)) > 0 ||                      // XBOX Original
                        (total_interfaces_hid = QueryAvailableInterfacesByClass(interfaces, sizeof(interfaces), USB_CLASS_HID)) > 0                                             // Generic HID
                    )
                    {
                        timeoutNs = MS_TO_NS(1); // Everytime we find a controller we reset the timeout to loop again on next controllers

                        s32 total_entries = total_interfaces_hid + total_interfaces_xbox360 + total_interfaces_xboxone + total_interfaces_xbox360w + total_interfaces_xbox;
                        if (controllers::IsAtControllerLimit())
                        {
                            syscon::logger::LogError("Reach controller limit - Can't add anymore controller !");
                            continue;
                        }

                        UsbHsInterface *interface = &interfaces[0];

                        syscon::logger::LogInfo("Trying to initialize USB device: [%04x-%04x] (Class: 0x%02X, SubClass: 0x%02X, Protocol: 0x%02X, bcd: 0x%04X)...",
                                                interface->device_desc.idVendor,
                                                interface->device_desc.idProduct,
                                                interface->device_desc.bDeviceClass,
                                                interface->device_desc.bDeviceSubClass,
                                                interface->device_desc.bDeviceProtocol,
                                                interface->device_desc.bcdDevice);

                        std::string default_profile = "";
                        if (total_interfaces_xbox360 > 0)
                            default_profile = "xbox360";
                        else if (total_interfaces_xbox360w > 0)
                            default_profile = "xbox360w";
                        else if (total_interfaces_xboxone > 0)
                            default_profile = "xboxone";
                        else if (total_interfaces_xbox > 0)
                            default_profile = "xbox";

                        ControllerConfig config;
                        ::syscon::config::LoadControllerConfig(CONFIG_FULLPATH, &config, interface->device_desc.idVendor, interface->device_desc.idProduct, g_auto_add_controller, default_profile);

                        if (config.driver == "dualshock3")
                        {
                            syscon::logger::LogInfo("Initializing Dualshock 3 controller (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<Dualshock3Controller>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "xbox360w")
                        {
                            syscon::logger::LogInfo("Initializing Xbox 360 Wireless controller (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<Xbox360WirelessController>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "xbox360")
                        {
                            syscon::logger::LogInfo("Initializing Xbox 360 controller (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<Xbox360Controller>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "xboxone")
                        {
                            /* One XboxOne controller will expose 2 interfaces, thus we have to take all of them */
                            syscon::logger::LogInfo("Initializing Xbox One controller (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<XboxOneController>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "xbox")
                        {
                            syscon::logger::LogInfo("Initializing Xbox 1st gen (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<XboxController>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "switch")
                        {
                            syscon::logger::LogInfo("Initializing Switch (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<SwitchController>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "wii")
                        {
                            syscon::logger::LogInfo("Initializing Wii (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<WiiController>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else if (config.driver == "steam2026")
                        {
                            syscon::logger::LogInfo("Initializing Steam Controller 2026 (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<SteamController2026>(std::make_unique<SwitchUSBDevice>(interfaces, total_entries), config, std::make_unique<syscon::logger::Logger>()));
                        }
                        else
                        {
                            /* For now if Generic controller expose more than 1 interface, we will create as many GenericHIDController as we have interfaces */
                            syscon::logger::LogInfo("Initializing Generic controller (Interface count: %d) ...", total_entries);
                            controllers::Insert(std::make_unique<GenericHIDController>(std::make_unique<SwitchUSBDevice>(interfaces, 1), config, std::make_unique<syscon::logger::Logger>()));
                        }
                    }
                    else
                    {
                        syscon::logger::LogDebug("No HID or XBOX interfaces found !");
                        timeoutNs = MS_TO_NS(1000); // DIAG: garder le poll actif pour logger les changements USB (au lieu de UINT64_MAX qui bloque le log)
                    }
                }
            } while (is_usb_event_thread_running);
        }

        void UsbInterfaceChangeThreadFunc(void *arg)
        {
            (void)arg;
            UsbHsInterface interfaces[MaxUsbHsInterfacesSize] = {};

            do
            {
                if (R_SUCCEEDED(eventWait(usbHsGetInterfaceStateChangeEvent(), UINT64_MAX)))
                {
                    eventClear(usbHsGetInterfaceStateChangeEvent());

                    syscon::logger::LogDebug("USBInterface state was changed !");

                    s32 total_entries = QueryAcquiredInterfaces(interfaces, sizeof(interfaces));

                    syscon::logger::LogDebug("USBInterface %d interfaces acquired !", total_entries);

                    std::vector<s32> interfaceIDsPlugged;
                    for (int i = 0; i < total_entries; i++)
                        interfaceIDsPlugged.push_back(interfaces[i].inf.ID);

                    controllers::RemoveAllNonPlugged(interfaceIDsPlugged);
                }

            } while (is_usb_interface_change_thread_running);
        }

        s32 QueryAcquiredInterfaces(UsbHsInterface *interfaces, size_t interfaces_maxsize)
        {
            SwitchUSBLock usbLock;
            s32 out_entries = 0;

            if (R_SUCCEEDED(usbHsQueryAcquiredInterfaces(interfaces, interfaces_maxsize, &out_entries)))
                return out_entries;

            return 0;
        }

        s32 QueryAvailableInterfacesByClassSubClassProtocol(UsbHsInterface *interfaces, size_t interfaces_maxsize, u8 iclass, u8 isubclass, u8 iprotocol)
        {
            SwitchUSBLock usbLock;

            UsbHsInterfaceFilter filter{
                .Flags = UsbHsInterfaceFilterFlags_bInterfaceClass | UsbHsInterfaceFilterFlags_bInterfaceSubClass | UsbHsInterfaceFilterFlags_bInterfaceProtocol,
                .bInterfaceClass = iclass,
                .bInterfaceSubClass = isubclass,
                .bInterfaceProtocol = iprotocol,
            };

            s32 out_entries = 0;
            memset(interfaces, 0, interfaces_maxsize);

            if (R_SUCCEEDED(usbHsQueryAvailableInterfaces(&filter, interfaces, interfaces_maxsize, &out_entries)))
                return out_entries;

            return 0;
        }

        s32 QueryAvailableInterfacesByClass(UsbHsInterface *interfaces, size_t interfaces_maxsize, u8 iclass)
        {
            SwitchUSBLock usbLock;

            UsbHsInterfaceFilter filter{
                .Flags = UsbHsInterfaceFilterFlags_bInterfaceClass,
                .bInterfaceClass = iclass};

            s32 out_entries = 0;
            memset(interfaces, 0, interfaces_maxsize);

            if (R_SUCCEEDED(usbHsQueryAvailableInterfaces(&filter, interfaces, interfaces_maxsize, &out_entries)))
                return out_entries;

            return 0;
        }

        inline Result AddEvent(UsbHsInterfaceFilter *filter, const std::string &name)
        {
            SwitchUSBLock usbLock;

            if (g_usbEventCount >= MaxUsbEvents)
            {
                static bool isMaxEventLogged = false;
                if (!isMaxEventLogged)
                {
                    syscon::logger::LogError("Unable to add future events ! (Max USB events reached !)");
                    isMaxEventLogged = true;
                }
                return CONTROLLER_STATUS_OUT_OF_MEMORY;
            }

            syscon::logger::LogDebug("Adding event with filter: %s (%d/%d)...", name.c_str(), g_usbEventCount + 1, MaxUsbEvents);
            Result ret = usbHsCreateInterfaceAvailableEvent(&g_usbEvent[g_usbEventCount], true, g_usbEventCount, filter);
            g_usbWaiters[g_usbEventCount] = waiterForEvent(&g_usbEvent[g_usbEventCount]);

            g_usbEventCount++;
            return ret;
        }

    } // namespace

    int Initialize(syscon::config::DiscoveryMode discovery_mode, std::vector<syscon::config::ControllerVidPid> &discovery_vidpid, bool auto_add_controller)
    {
        g_auto_add_controller = auto_add_controller;

        syscon::logger::LogInfo("USB configuration: Discovery mode(%d), Auto add controller(%s)", discovery_mode, auto_add_controller ? "true" : "false");

        if (discovery_mode == syscon::config::DiscoveryMode::HID_AND_XBOX || discovery_mode == syscon::config::DiscoveryMode::VIDPID_AND_XBOX)
        {
            // Filter use to detect XBOX controllers
            UsbHsInterfaceFilter filterAllDevices1{
                .Flags = UsbHsInterfaceFilterFlags_bcdDevice_Min,
                .bcdDevice_Min = 0x0000,
            };
            AddEvent(&filterAllDevices1, "XBOX");
        }

        if (discovery_mode == syscon::config::DiscoveryMode::HID_AND_XBOX)
        {
            // Filter used to detect Generic HID controllers
            // Cause issue with Native Switch Controllers
            UsbHsInterfaceFilter filterAllDevices2{
                .Flags = UsbHsInterfaceFilterFlags_bInterfaceClass | UsbHsInterfaceFilterFlags_bcdDevice_Min,
                .bcdDevice_Min = 0x0000,
                .bInterfaceClass = USB_CLASS_HID,
            };
            AddEvent(&filterAllDevices2, "USB_CLASS_HID");
        }

        if (discovery_mode == syscon::config::DiscoveryMode::VIDPID || discovery_mode == syscon::config::DiscoveryMode::VIDPID_AND_XBOX)
        {
            // Filter known VID/PID
            for (syscon::config::ControllerVidPid &vidpid : discovery_vidpid)
            {
                UsbHsInterfaceFilter filterKnownDevice{
                    .Flags = UsbHsInterfaceFilterFlags_idVendor,
                    .idVendor = vidpid.vid,
                };

                if (vidpid.pid != 0)
                {
                    filterKnownDevice.Flags |= UsbHsInterfaceFilterFlags_idProduct;
                    filterKnownDevice.idProduct = vidpid.pid;
                }

                AddEvent(&filterKnownDevice, std::string(vidpid));
            }
        }

        is_usb_event_thread_running = true;
        Result rc = threadCreate(&g_usb_event_thread, &UsbEventThreadFunc, nullptr, usb_event_thread_stack, sizeof(usb_event_thread_stack), 0x3A, -2);
        if (R_FAILED(rc))
            return rc;
        rc = threadStart(&g_usb_event_thread);
        if (R_FAILED(rc))
            return rc;

        is_usb_interface_change_thread_running = true;
        rc = threadCreate(&g_usb_interface_change_thread, &UsbInterfaceChangeThreadFunc, nullptr, usb_interface_change_thread_stack, sizeof(usb_interface_change_thread_stack), 0x2C, -2);
        if (R_FAILED(rc))
            return rc;
        rc = threadStart(&g_usb_interface_change_thread);
        if (R_FAILED(rc))
            return rc;

        return 0;
    }

    void Exit()
    {
        is_usb_event_thread_running = false;
        is_usb_interface_change_thread_running = false;

        svcCancelSynchronization(g_usb_event_thread.handle);
        threadWaitForExit(&g_usb_event_thread);
        threadClose(&g_usb_event_thread);

        svcCancelSynchronization(g_usb_interface_change_thread.handle);
        threadWaitForExit(&g_usb_interface_change_thread);
        threadClose(&g_usb_interface_change_thread);

        for (size_t i = 0; i < g_usbEventCount; i++)
        {
            SwitchUSBLock usbLock;
            usbHsDestroyInterfaceAvailableEvent(&g_usbEvent[i], i);
        }

        controllers::Clear();
    }

} // namespace syscon::usb