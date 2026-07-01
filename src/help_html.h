#pragma once
//
// HTML-Kurzanleitung, die als HELP.HTM auf das USB-MSC-Laufwerk gelegt wird
// (siehe usb_msc.cpp build_info_files). Der Inhalt liegt bewusst in einer
// eigenen Quelldatei (help_html.cpp) als reiner HTML-Raw-String, damit er
// unabhaengig von der MSC-Logik direkt editiert werden kann.
//
// HELP_HTML     : NUL-terminierter HTML-Text (Raw-String).
// HELP_HTML_LEN : Laenge in Bytes ohne die abschliessende NUL.
//

extern const char     HELP_HTML[];
extern const unsigned  HELP_HTML_LEN;
