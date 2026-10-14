// portable.hpp - cross-platform OS helpers for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.6: centralizes every platform-specific call so the rest of the
// codebase doesn't have to sprinkle #ifdef _WIN32 everywhere. before
// this, the source had:
//   * #include <unistd.h>           (Linux/macOS only -- mingw ships a
//                                    shim, but MSVC doesn't)
//   * #include <sys/stat.h>          (POSIX, partially shimmed by mingw)
//   * getpid()                      (POSIX; Windows uses _getpid)
//   * "/tmp/pencil_anim_..."        (Linux only -- Windows has no /tmp)
//   * std::system("cd ... && zip")  (Unix shell, no `zip` on Windows)
//   * std::system("ffmpeg ...")     (works if ffmpeg on PATH; quotes
//                                    need to be platform-aware)
//
// All of these are now funneled through this header so callers stay
// platform-agnostic.

#pragma once

#include <string>

#ifdef _WIN32
// windows-specific headers. process.h gives _getpid; windows.h gives
// GetTempPathA; direct.h gives _mkdir/_getcwd/_chdir; io.h gives _access.
#include <winsock2.h>    // must come before <windows.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <direct.h>
#include <io.h>
#include <sys/stat.h>    // mingw provides this; used by std::filesystem on some setups

#define PENCIL_GETPID()        _getpid()
#define PENCIL_MKDIR_ONE(p)    _mkdir(p)
#define PENCIL_CHDIR(p)        _chdir(p)
#define PENCIL_GETCWD(buf, sz) _getcwd((buf), (sz))
#define PENCIL_ACCESS(p)       _access((p), 0)
typedef int socklen_t;
#pragma comment(lib, "ws2_32.lib")
#define PENCIL_CLOSESOCK(s) closesocket(s)
#define PENCIL_PATHSEP '\\'
#else
// posix
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#define PENCIL_GETPID()        getpid()
#define PENCIL_MKDIR_ONE(p)   ::mkdir(p, 0777)
#define PENCIL_CHDIR(p)       ::chdir(p)
#define PENCIL_GETCWD(buf, sz) getcwd((buf), (sz))
#define PENCIL_ACCESS(p)      access((p), F_OK)
#define PENCIL_CLOSESOCK(s)  close(s)
#define PENCIL_PATHSEP '/'
#endif

namespace pencil {

// return a writable temp directory for this platform.
//   * Windows: %TEMP% / %TMP% / %USERPROFILE%\AppData\Local\Temp
//     (via GetTempPathA, which handles all the env lookups)
//   * POSIX: $TMPDIR if set, else "/tmp"
// the returned string never has a trailing slash.
inline std::string portable_temp_dir() {
#ifdef _WIN32
    char buf[1024];
    DWORD len = GetTempPathA((DWORD)sizeof(buf), buf);
    if (len > 0 && len < sizeof(buf)) {
        // strip trailing backslash
        if (buf[len - 1] == '\\' || buf[len - 1] == '/') buf[len - 1] = 0;
        return std::string(buf);
    }
    return "."; // fall back to current dir
#else
    const char* t = std::getenv("TMPDIR");
    if (t && *t) return std::string(t);
    return std::string("/tmp");
#endif
}

// make a directory and all parents (like mkdir -p). uses native mkdir
// one level at a time. ignores EEXIST.
inline bool portable_mkdir_p(const std::string& path) {
    if (path.empty()) return false;
    std::string acc;
    size_t i = 0;
    if (path.size() >= 2 && path[1] == ':') {
        // windows drive prefix like "C:"
        acc = path.substr(0, 2);
        i = 2;
        if (path[i] == '/' || path[i] == '\\') {
            acc.push_back(path[i]);
            i++;
        }
    } else if (path[0] == '/' || path[0] == '\\') {
        acc.push_back(path[0]);
        i = 1;
    }
    while (i <= path.size()) {
        if (i == path.size() || path[i] == '/' || path[i] == '\\') {
            if (!acc.empty()) {
                PENCIL_MKDIR_ONE(acc.c_str()); // ignore error; if it fails
                                               // for non-EEXIST reasons,
                                               // the open() below will
                                               // report it.
            }
        }
        if (i < path.size()) acc.push_back(path[i]);
        i++;
    }
    return true;
}

} // namespace pencil
