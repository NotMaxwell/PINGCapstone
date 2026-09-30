// Standalone demo for the NEO-M9N GPS: runs the application's GPS task on
// its own and prints each navigation solution. Build it with the gps-demo
// preset.
//
// The task logs the receiver's module and firmware, switches it to UBX
// output at 10 Hz, and prints nothing further until a solution arrives. The
// configuration goes to the receiver's RAM only, so a power cycle puts it
// back to factory settings.

#include <chrono>
#include <cstdio>

#include "demos/demo.hpp"
#include "sensors.hpp"

extern "C" void app_main()
{
    using namespace ping;
    demo::run(sensors::start_gps, readings::gps, std::chrono::milliseconds{0}, [](const GpsFix& fix) {
        if (const auto position = fix.position()) {
            std::printf("%.7f, %.7f +/-%.1f m, %u satellites\n", position->latitude_deg,
                        position->longitude_deg, static_cast<double>(fix.horizontal_accuracy_m),
                        fix.satellites);
        } else {
            std::printf("no fix (%s, %u satellites)\n", to_string(fix.fix_type), fix.satellites);
        }
    });
}
