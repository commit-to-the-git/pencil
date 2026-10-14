// zipwriter.hpp - minimal STORE-only ZIP archive writer for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.6: replaces the old `std::system("zip -j ... *.png")` call which
//   (a) required the `zip` binary on PATH (Linux/macOS yes, Windows NO),
//   (b) used Unix shell semantics ("cd ... && zip ..."),
//   (c) failed silently if `zip` wasn't installed.
//
// this writer uses the STORE method (no recompression) because PNG
// frames are already DEFLATE-compressed by stb_image_write. storing
// them uncompressed in the zip is fast and produces archives that
// 7-zip / WinRAR / Explorer / unzip / `tar -xf` all open correctly.
//
// implements a curated subset of PKZIP 2.0 (APPNOTE 6.3.x):
//   * local file header (sig 0x04034b50)
//   * data (raw bytes, STORE method)
//   * central directory file header (sig 0x02014b50)
//   * end of central directory record (sig 0x06054b50)
// CRC32 is computed inline (no zlib dependency).

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace pencil {

// compute CRC32 (IEEE 802.3 polynomial 0xEDB88320) for a buffer.
// we use the standard 256-entry table computed lazily on first call.
inline uint32_t crc32_compute(const void* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        init = true;
    }
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// write a STORE-method zip archive to `out_path` containing the files
// listed in `entries` (each is {filename_in_archive, raw_bytes}).
//
// filenames should use forward slashes; they are stored verbatim. empty
// entries are skipped. returns true on success, false + sets err on
// failure (cannot open output, write error).
inline bool write_zip_archive(const std::string& out_path,
                              const std::vector<std::pair<std::string, std::vector<uint8_t>>>& entries,
                              std::string& err) {
    std::ofstream f(out_path, std::ios::binary);
    if (!f) {
        err = "cannot open output zip: " + out_path;
        return false;
    }

    // track central directory entries so we can emit them at the end.
    struct CDEntry {
        std::string name;
        uint32_t crc;
        uint32_t size;
        uint32_t local_offset;
    };
    std::vector<CDEntry> cd;

    for (const auto& [name, data] : entries) {
        if (name.empty()) continue;
        uint32_t crc = crc32_compute(data.data(), data.size());
        uint32_t size = (uint32_t)data.size();
        uint32_t local_offset = (uint32_t)f.tellp();

        // ---- local file header (30 bytes + name) ----
        uint8_t lfh[30];
        // signature 0x04034b50
        lfh[0]=0x50; lfh[1]=0x4b; lfh[2]=0x03; lfh[3]=0x04;
        // version needed to extract (20 = 2.0)
        lfh[4]=20; lfh[5]=0;
        // general purpose bit flag (0)
        lfh[6]=0; lfh[7]=0;
        // compression method (0 = STORE)
        lfh[8]=0; lfh[9]=0;
        // last mod file time (0)
        lfh[10]=0; lfh[11]=0;
        // last mod file date (0)
        lfh[12]=0; lfh[13]=0;
        // CRC32
        lfh[14]=(uint8_t)(crc & 0xFF);
        lfh[15]=(uint8_t)((crc >> 8) & 0xFF);
        lfh[16]=(uint8_t)((crc >> 16) & 0xFF);
        lfh[17]=(uint8_t)((crc >> 24) & 0xFF);
        // compressed size (== uncompressed for STORE)
        lfh[18]=(uint8_t)(size & 0xFF);
        lfh[19]=(uint8_t)((size >> 8) & 0xFF);
        lfh[20]=(uint8_t)((size >> 16) & 0xFF);
        lfh[21]=(uint8_t)((size >> 24) & 0xFF);
        // uncompressed size
        lfh[22]=(uint8_t)(size & 0xFF);
        lfh[23]=(uint8_t)((size >> 8) & 0xFF);
        lfh[24]=(uint8_t)((size >> 16) & 0xFF);
        lfh[25]=(uint8_t)((size >> 24) & 0xFF);
        // filename length
        uint16_t name_len = (uint16_t)name.size();
        lfh[26]=(uint8_t)(name_len & 0xFF);
        lfh[27]=(uint8_t)((name_len >> 8) & 0xFF);
        // extra field length (0)
        lfh[28]=0; lfh[29]=0;

        f.write((const char*)lfh, 30);
        f.write(name.data(), name_len);
        if (!data.empty()) {
            f.write((const char*)data.data(), data.size());
        }

        cd.push_back({name, crc, size, local_offset});
    }

    uint32_t cd_offset = (uint32_t)f.tellp();
    uint32_t cd_size = 0;

    // ---- central directory file headers (46 bytes + name each) ----
    for (const auto& e : cd) {
        uint8_t cdh[46];
        // signature 0x02014b50
        cdh[0]=0x50; cdh[1]=0x4b; cdh[2]=0x01; cdh[3]=0x02;
        // version made by (20)
        cdh[4]=20; cdh[5]=0;
        // version needed (20)
        cdh[6]=20; cdh[7]=0;
        // flags (0)
        cdh[8]=0; cdh[9]=0;
        // compression (0 = STORE)
        cdh[10]=0; cdh[11]=0;
        // mod time (0)
        cdh[12]=0; cdh[13]=0;
        // mod date (0)
        cdh[14]=0; cdh[15]=0;
        // CRC32
        cdh[16]=(uint8_t)(e.crc & 0xFF);
        cdh[17]=(uint8_t)((e.crc >> 8) & 0xFF);
        cdh[18]=(uint8_t)((e.crc >> 16) & 0xFF);
        cdh[19]=(uint8_t)((e.crc >> 24) & 0xFF);
        // compressed size
        cdh[20]=(uint8_t)(e.size & 0xFF);
        cdh[21]=(uint8_t)((e.size >> 8) & 0xFF);
        cdh[22]=(uint8_t)((e.size >> 16) & 0xFF);
        cdh[23]=(uint8_t)((e.size >> 24) & 0xFF);
        // uncompressed size
        cdh[24]=(uint8_t)(e.size & 0xFF);
        cdh[25]=(uint8_t)((e.size >> 8) & 0xFF);
        cdh[26]=(uint8_t)((e.size >> 16) & 0xFF);
        cdh[27]=(uint8_t)((e.size >> 24) & 0xFF);
        // filename length
        uint16_t name_len = (uint16_t)e.name.size();
        cdh[28]=(uint8_t)(name_len & 0xFF);
        cdh[29]=(uint8_t)((name_len >> 8) & 0xFF);
        // extra field length (0)
        cdh[30]=0; cdh[31]=0;
        // comment length (0)
        cdh[32]=0; cdh[33]=0;
        // disk number start (0)
        cdh[34]=0; cdh[35]=0;
        // internal attrs (0)
        cdh[36]=0; cdh[37]=0;
        // external attrs (0)
        cdh[38]=0; cdh[39]=0; cdh[40]=0; cdh[41]=0;
        // local header offset
        cdh[42]=(uint8_t)(e.local_offset & 0xFF);
        cdh[43]=(uint8_t)((e.local_offset >> 8) & 0xFF);
        cdh[44]=(uint8_t)((e.local_offset >> 16) & 0xFF);
        cdh[45]=(uint8_t)((e.local_offset >> 24) & 0xFF);

        f.write((const char*)cdh, 46);
        f.write(e.name.data(), name_len);
        cd_size += 46 + name_len;
    }

    // ---- end of central directory record (22 bytes) ----
    uint8_t eocd[22];
    // signature 0x06054b50
    eocd[0]=0x50; eocd[1]=0x4b; eocd[2]=0x05; eocd[3]=0x06;
    // disk number (0)
    eocd[4]=0; eocd[5]=0;
    // disk where CD starts (0)
    eocd[6]=0; eocd[7]=0;
    // number of CD entries on this disk
    uint16_t n = (uint16_t)cd.size();
    eocd[8]=(uint8_t)(n & 0xFF);
    eocd[9]=(uint8_t)((n >> 8) & 0xFF);
    // total CD entries
    eocd[10]=(uint8_t)(n & 0xFF);
    eocd[11]=(uint8_t)((n >> 8) & 0xFF);
    // CD size
    eocd[12]=(uint8_t)(cd_size & 0xFF);
    eocd[13]=(uint8_t)((cd_size >> 8) & 0xFF);
    eocd[14]=(uint8_t)((cd_size >> 16) & 0xFF);
    eocd[15]=(uint8_t)((cd_size >> 24) & 0xFF);
    // CD offset
    eocd[16]=(uint8_t)(cd_offset & 0xFF);
    eocd[17]=(uint8_t)((cd_offset >> 8) & 0xFF);
    eocd[18]=(uint8_t)((cd_offset >> 16) & 0xFF);
    eocd[19]=(uint8_t)((cd_offset >> 24) & 0xFF);
    // comment length (0)
    eocd[20]=0; eocd[21]=0;

    f.write((const char*)eocd, 22);
    f.close();

    if (!f) {
        err = "write error during zip: " + out_path;
        return false;
    }
    return true;
}

} // namespace pencil
