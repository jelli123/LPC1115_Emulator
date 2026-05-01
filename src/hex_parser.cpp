#include "hex_parser.h"

#include <cstring>

namespace hex {

int Parser::hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

Result Parser::feed(char c) {
    if (finished_) return Result::EndOfFile;

    if (c == '\r') return Result::InProgress;
    if (c == '\n') {
        if (!in_line_) return Result::InProgress;
        in_line_ = false;
        Result r = handle_line();
        line_len_ = 0;
        return r;
    }
    if (c == ':') {
        in_line_ = true;
        line_len_ = 0;
        return Result::InProgress;
    }
    if (!in_line_) return Result::InProgress;
    if (line_len_ >= sizeof line_) {
        in_line_ = false;
        line_len_ = 0;
        return Result::Overflow;
    }
    line_[line_len_++] = c;
    return Result::InProgress;
}

Result Parser::handle_line() {
    // Mindestens LL(2)+AAAA(4)+TT(2)+CC(2) = 10 Hex-Zeichen
    if (line_len_ < 10 || (line_len_ % 2) != 0) return Result::BadFormat;

    uint8_t bytes[1 + 2 + 1 + 255 + 1];
    uint16_t n = line_len_ / 2;
    if (n > sizeof bytes) return Result::BadFormat;

    uint8_t sum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        int hi = hex_nibble(line_[i * 2]);
        int lo = hex_nibble(line_[i * 2 + 1]);
        if (hi < 0 || lo < 0) return Result::BadFormat;
        bytes[i] = static_cast<uint8_t>((hi << 4) | lo);
        sum = static_cast<uint8_t>(sum + bytes[i]);
    }
    if (sum != 0) return Result::BadChecksum;

    uint8_t  ll = bytes[0];
    uint16_t aa = static_cast<uint16_t>((bytes[1] << 8) | bytes[2]);
    uint8_t  tt = bytes[3];
    if (4u + ll + 1u != n) return Result::BadFormat;
    const uint8_t* data = &bytes[4];

    switch (tt) {
        case 0x00: { // Data
            uint32_t addr = ext_linear_addr_ + aa;
            if (addr < base_) return Result::OutOfRange;
            uint32_t off = addr - base_;
            if (off + ll > max_size_) return Result::OutOfRange;
            if (!writer_(off, data, ll)) return Result::OutOfRange;
            uint32_t end = off + ll;
            if (end > bytes_written_) bytes_written_ = end;
            return Result::Ok;
        }
        case 0x01: // EOF
            finished_ = true;
            return Result::EndOfFile;
        case 0x02: { // Extended Segment Address
            if (ll != 2) return Result::BadFormat;
            ext_linear_addr_ = static_cast<uint32_t>((data[0] << 8 | data[1]) << 4);
            return Result::Ok;
        }
        case 0x04: { // Extended Linear Address
            if (ll != 2) return Result::BadFormat;
            ext_linear_addr_ = static_cast<uint32_t>(data[0] << 24 | data[1] << 16);
            return Result::Ok;
        }
        case 0x03: // Start Segment Address (ignoriert)
        case 0x05: // Start Linear  Address (ignoriert)
            return Result::Ok;
        default:
            return Result::BadFormat;
    }
}

} // namespace hex
