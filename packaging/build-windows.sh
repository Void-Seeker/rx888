#!/usr/bin/env bash
#
# Cross-build libsddc for Windows from Linux, using the MSVC toolchain that
# msvc-wine installs. Produces sddc.dll and its import library for each
# architecture asked for, which is what PothosSDR and anything built against it
# expect -- an MSVC-ABI DLL, not a MinGW one.
#
#   MSVC_ROOT=~/my_msvc/opt/msvc ./packaging/build-windows.sh x64 x86
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MSVC_ROOT="${MSVC_ROOT:-$HOME/my_msvc/opt/msvc}"
OUT="${OUT:-$REPO/packaging/out/windows}"

ARCHES=("$@")
[ ${#ARCHES[@]} -gt 0 ] || ARCHES=(x64 x86)

say() { printf '\n== %s ==\n' "$*"; }
die() { echo "error: $*" >&2; exit 1; }

[ -d "$MSVC_ROOT" ] || die "no MSVC toolchain at $MSVC_ROOT (set MSVC_ROOT)"

# msvc-wine ships a wrapper directory per architecture holding cl, link and the
# rest, with the include and library paths already baked in.
arch_triple() {
    case "$1" in
        x64)   echo x86_64-pc-windows-msvc ;;
        x86)   echo i686-pc-windows-msvc ;;
        arm64) echo aarch64-pc-windows-msvc ;;
        *)     die "unknown architecture '$1' (use x64, x86 or arm64)" ;;
    esac
}

mkdir -p "$OUT"

for arch in "${ARCHES[@]}"; do
    triple="$(arch_triple "$arch")"
    bindir="$MSVC_ROOT/bin/$arch"
    [ -x "$bindir/link" ] || die "no MSVC wrappers for $arch at $bindir"

    say "$arch ($triple)"

    if ! rustup target list --installed | grep -qx "$triple"; then
        echo "  installing rust target $triple"
        rustup target add "$triple"
    fi

    # Rust drives the MSVC linker itself, so it just needs to find link.exe. The
    # wrappers already carry the right include and library paths, which is why
    # no vcvars dance is needed here.
    env_var="CARGO_TARGET_$(echo "$triple" | tr 'a-z-' 'A-Z_')_LINKER"

    ( cd "$REPO" && env \
        PATH="$bindir:$PATH" \
        "$env_var=$bindir/link" \
        cargo build --release --target "$triple" )

    d="$REPO/target/$triple/release"
    stage="$OUT/$arch"
    mkdir -p "$stage"

    # cdylib gives sddc.dll plus sddc.dll.lib; ship the import library under the
    # name a linker expects to be given
    for f in sddc.dll sddc.dll.lib sddc.pdb; do
        [ -f "$d/$f" ] && install -m644 "$d/$f" "$stage/"
    done
    [ -f "$stage/sddc.dll.lib" ] && cp "$stage/sddc.dll.lib" "$stage/sddc.lib"
    [ -f "$d/libsddc.h" ] && install -m644 "$d/libsddc.h" "$stage/"
    # the header is generated into the host target dir on a cross build
    [ -f "$stage/libsddc.h" ] || install -m644 "$REPO/target/release/libsddc.h" "$stage/" 2>/dev/null || true

    echo "  ->"; ls -1 "$stage" | sed 's/^/     /'
done

say "done"
echo "Windows binaries under $OUT"
echo
echo "Note: the device needs a WinUSB driver bound to it (Zadig, or the driver"
echo "package PothosSDR installs). Windows has no udev equivalent, so the udev"
echo "rule in this repo applies to Linux only."
