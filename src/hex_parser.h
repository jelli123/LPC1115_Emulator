#pragma once
//
// Stream-basierter Intel-HEX-Parser.
// Robust gegen: zu lange Zeilen, ungültige Hex-Zeichen, falsche Checksummen,
// Adress-Sprünge in Lücken (rejected → Aufrufer entscheidet),
// Records außerhalb des Firmware-Slots.
//

#include <cstdint>
#include <cstddef>

namespace hex {

enum class Result : uint8_t {
    Ok,
    InProgress,
    BadFormat,
    BadChecksum,
    OutOfRange,
    Overflow,
    EndOfFile
};

class Parser {
public:
    using WriteFn = bool (*)(uint32_t address, const uint8_t* data, std::size_t len);

    explicit Parser(WriteFn writer, uint32_t base_address, uint32_t max_size)
        : writer_(writer), base_(base_address), max_size_(max_size) {}

    // Liefert Bytes/Zeichen ein. Unicode-/CR-/LF-tolerant.
    Result feed(char c);

    uint32_t bytes_written() const { return bytes_written_; }
    bool     finished()      const { return finished_; }

private:
    Result handle_line();
    static int hex_nibble(char c);

    WriteFn  writer_;
    uint32_t base_;
    uint32_t max_size_;

    char     line_[1 + 2 + 4 + 2 + 255 * 2 + 2 + 4]; // : LL AAAA TT DD..DD CC + Slack
    uint16_t line_len_ = 0;
    bool     in_line_  = false;

    uint32_t ext_linear_addr_ = 0;
    uint32_t bytes_written_   = 0;
    bool     finished_        = false;
};

} // namespace hex
