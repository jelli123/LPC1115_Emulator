#pragma once
//
// Serielles Nutzerinterface auf Core 0. Liest USB-CDC, parst Kommandos,
// streamt Intel-Hex-Uploads in den Firmware-Slot, gibt Status/Stats aus.
//

namespace cli {
void init();
void run();   // niemals zurückkehren
} // namespace cli