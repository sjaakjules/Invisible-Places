#!/bin/sh

set -eu

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <macOS CMake configure preset> [additional CMake arguments...]" >&2
    exit 2
fi

preset="$1"
shift

case "$preset" in
    macos-*) ;;
    *)
        echo "Error: '$preset' is not a macOS configure preset." >&2
        exit 2
        ;;
esac

if [ "$(uname -s)" != "Darwin" ]; then
    echo "Error: scripts/configure_macos.sh can only configure macOS builds." >&2
    exit 2
fi

macos_sdk=""
if command -v xcrun >/dev/null 2>&1; then
    macos_sdk="$(xcrun --sdk macosx --show-sdk-path 2>/dev/null || true)"
fi
if [ -z "$macos_sdk" ] || [ ! -d "$macos_sdk" ]; then
    echo "Error: xcrun could not locate an installed macOS SDK." >&2
    echo "Install or repair Xcode Command Line Tools, then retry." >&2
    exit 1
fi

selected_compiler=""
selected_compiler_flags=""
selected_linker_flags=""
checked_compilers=""

resolve_compiler() {
    candidate="$1"
    case "$candidate" in
        */*)
            if [ -x "$candidate" ]; then
                printf '%s\n' "$candidate"
            fi
            ;;
        *)
            command -v "$candidate" 2>/dev/null || true
            ;;
    esac
}

probe_source() {
    printf '%s\n' \
        '#include <array>' \
        '#include <charconv>' \
        '#include <filesystem>' \
        '#include <stop_token>' \
        '#include <thread>' \
        'int main() { float value = 0.0F; const char* text = "1"; std::jthread worker([] {}); std::stop_source source; return std::from_chars(text, text + 1, value).ec == std::errc{} && source.stop_possible() ? 0 : 1; }'
}

compiler_supports_project_headers() {
    candidate="$(resolve_compiler "$1")"
    if [ -z "$candidate" ]; then
        return 1
    fi

    case "$checked_compilers" in
        *"|$candidate|"*) return 1 ;;
    esac
    checked_compilers="${checked_compilers}|${candidate}|"

    # Probe Objective-C++ as well as C++ because the app enables both languages
    # on macOS. Floating from_chars, jthread and stop_token cover the newer
    # libc++ surface used by the app, not merely the presence of header files.
    if probe_source \
        | "$candidate" -isysroot "$macos_sdk" -x objective-c++ -std=c++20 -fsyntax-only - >/dev/null 2>&1; then
        selected_compiler="$candidate"
        return 0
    fi

    # Homebrew LLVM uses its own newer libc++ headers with Apple's system
    # runtime. On supported current macOS versions those headers may require
    # disabling conservative vendor availability annotations, as the working
    # debug toolchain already does. Only select this mode when the full probe
    # proves that it is sufficient.
    if probe_source \
        | "$candidate" -D_LIBCPP_DISABLE_AVAILABILITY -isysroot "$macos_sdk" \
            -x objective-c++ -std=c++20 -fsyntax-only - >/dev/null 2>&1; then
        compiler_prefix="$(CDPATH= cd -- "$(dirname -- "$candidate")/.." && pwd)"
        compiler_cxx_runtime="$compiler_prefix/lib/c++"
        compiler_unwind_runtime="$compiler_prefix/lib/unwind"
        if [ ! -f "$compiler_cxx_runtime/libc++.dylib" ] \
            || [ ! -f "$compiler_cxx_runtime/libc++abi.dylib" ] \
            || [ ! -f "$compiler_unwind_runtime/libunwind.dylib" ]; then
            return 1
        fi

        selected_compiler="$candidate"
        selected_compiler_flags="-D_LIBCPP_DISABLE_AVAILABILITY"
        selected_linker_flags="-L$compiler_cxx_runtime -Wl,-rpath,$compiler_cxx_runtime -Wl,-rpath,$compiler_unwind_runtime"
        return 0
    fi

    return 1
}

select_required_compiler() {
    requested_name="$1"
    requested_value="$2"
    if compiler_supports_project_headers "$requested_value"; then
        return 0
    fi

    echo "Error: $requested_name='$requested_value' cannot compile the required macOS C++/Objective-C++ headers." >&2
    echo "Unset it to allow automatic Apple Clang/Homebrew LLVM discovery, or point it at a working clang++." >&2
    exit 1
}

# Honour an intentional override first. CXX remains supported for standard
# toolchain workflows; the project-specific name avoids changing a whole shell.
if [ -n "${INVISIBLE_PLACES_CXX:-}" ]; then
    select_required_compiler "INVISIBLE_PLACES_CXX" "$INVISIBLE_PLACES_CXX"
elif [ -n "${CXX:-}" ]; then
    select_required_compiler "CXX" "$CXX"
else
    # A complete Xcode/Command Line Tools install is the most portable choice.
    xcrun_compiler=""
    if command -v xcrun >/dev/null 2>&1; then
        xcrun_compiler="$(xcrun --find clang++ 2>/dev/null || true)"
    fi
    if [ -n "$xcrun_compiler" ]; then
        compiler_supports_project_headers "$xcrun_compiler" || true
    fi

    # GUI-launched VS Code may not inherit Homebrew's bin directory. Locate the
    # installation from either architecture's standard prefix, then accept both
    # the rolling llvm formula and installed versioned llvm@N formulae.
    if [ -z "$selected_compiler" ]; then
        brew_command="$(command -v brew 2>/dev/null || true)"
        if [ -z "$brew_command" ] && [ -x /opt/homebrew/bin/brew ]; then
            brew_command=/opt/homebrew/bin/brew
        elif [ -z "$brew_command" ] && [ -x /usr/local/bin/brew ]; then
            brew_command=/usr/local/bin/brew
        fi

        brew_prefix=""
        if [ -n "$brew_command" ]; then
            brew_prefix="$("$brew_command" --prefix 2>/dev/null || true)"
        fi

        if [ -n "$brew_prefix" ]; then
            for candidate in \
                "$brew_prefix"/opt/llvm/bin/clang++ \
                "$brew_prefix"/opt/llvm@*/bin/clang++; do
                compiler_supports_project_headers "$candidate" || true
                if [ -n "$selected_compiler" ]; then
                    break
                fi
            done
        fi
    fi

    # These fallbacks also cover a manually installed compiler already on PATH.
    if [ -z "$selected_compiler" ]; then
        compiler_supports_project_headers clang++ || true
    fi
    if [ -z "$selected_compiler" ]; then
        compiler_supports_project_headers c++ || true
    fi
fi

if [ -z "$selected_compiler" ]; then
    echo "Error: no compiler capable of building macOS C++20/Objective-C++ sources was found." >&2
    echo "Install/repair Xcode Command Line Tools or install Homebrew LLVM, then retry." >&2
    exit 1
fi

echo "Invisible Places macOS compiler: $selected_compiler"
echo "Invisible Places macOS SDK: $macos_sdk"
if [ -n "$selected_compiler_flags" ]; then
    echo "Invisible Places compiler compatibility flags: $selected_compiler_flags"
    echo "Invisible Places compiler runtime flags: $selected_linker_flags"
fi

# vcpkg performs a separate compiler probe before configuring some ports. Keep
# that probe on the same validated toolchain as the application configure.
export CXX="$selected_compiler"
export OBJCXX="$selected_compiler"
export SDKROOT="$macos_sdk"

if [ -n "$selected_compiler_flags" ]; then
    set -- \
        "-DCMAKE_CXX_FLAGS:STRING=$selected_compiler_flags" \
        "-DCMAKE_OBJCXX_FLAGS:STRING=$selected_compiler_flags" \
        "-DCMAKE_EXE_LINKER_FLAGS:STRING=$selected_linker_flags" \
        "$@"
fi

# Changing a compiler in-place makes CMake discard its cache during configure;
# that internal rerun can lose preset inputs such as the vcpkg toolchain. Detect
# a stale cross-machine cache first and start that one configure cleanly.
script_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
source_dir="$(dirname "$script_dir")"
binary_dir=""
expected_build_type=""
case "$preset" in
    macos-debug*)
        binary_dir="$source_dir/build/macos-debug"
        expected_build_type="Debug"
        ;;
    macos-release*)
        binary_dir="$source_dir/build/macos-release"
        expected_build_type="RelWithDebInfo"
        ;;
esac

fresh_configure=0
cache_file="$binary_dir/CMakeCache.txt"
if [ -n "$binary_dir" ] && [ -f "$cache_file" ]; then
    cached_compiler="$(sed -n 's/^CMAKE_CXX_COMPILER:[^=]*=//p' "$cache_file" | head -n 1)"
    cached_build_type="$(sed -n 's/^CMAKE_BUILD_TYPE:[^=]*=//p' "$cache_file" | head -n 1)"

    if [ "$cached_compiler" != "$selected_compiler" ] \
        || [ "$cached_build_type" != "$expected_build_type" ]; then
        fresh_configure=1
    fi

    case "$preset" in
        *-vcpkg)
            if ! grep -q '^CMAKE_TOOLCHAIN_FILE:[^=]*=' "$cache_file" \
                || ! grep -q '^VCPKG_MANIFEST_MODE:[^=]*=' "$cache_file"; then
                fresh_configure=1
            fi
            ;;
    esac
fi

if [ "$fresh_configure" -eq 1 ]; then
    echo "Resetting stale local CMake cache before applying the macOS preset."
    exec cmake \
        --fresh \
        --preset "$preset" \
        "-DCMAKE_CXX_COMPILER:FILEPATH=$selected_compiler" \
        "-DCMAKE_OBJCXX_COMPILER:FILEPATH=$selected_compiler" \
        "-DCMAKE_OSX_SYSROOT:PATH=$macos_sdk" \
        "$@"
fi

exec cmake \
    --preset "$preset" \
    "-DCMAKE_CXX_COMPILER:FILEPATH=$selected_compiler" \
    "-DCMAKE_OBJCXX_COMPILER:FILEPATH=$selected_compiler" \
    "-DCMAKE_OSX_SYSROOT:PATH=$macos_sdk" \
    "$@"
