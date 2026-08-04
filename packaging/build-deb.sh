#!/usr/bin/env bash
#
# Build Debian packages for libsddc.
#
#   libsddc0     the shared library and the udev rule
#   libsddc-dev  header and linker symlink, for building against it
#
# Dependencies come from dpkg-shlibdeps reading the built binary, so they
# cannot drift from what was actually linked.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OUT:-$REPO/packaging/out}"
ARCH="$(dpkg --print-architecture)"
MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"

VERSION="${VERSION:-$(sed -n 's/^version *= *"\(.*\)"/\1/p' "$REPO/Cargo.toml" | head -1)}"
REVISION="${REVISION:-1}"
FULLVER="${VERSION}-${REVISION}"

# The SONAME the library is built with. Bumping it means a new libsddcN package
# name, which is why it is spelled out here.
SOVER=0
MAINTAINER="${MAINTAINER:-Void-Seeker <gausshunter@gmail.com>}"

say() { printf '\n== %s ==\n' "$*"; }

say "building libsddc $VERSION (SONAME libsddc.so.$SOVER)"
# A Rust cdylib carries no SONAME of its own. Without one, anything linking
# against it records the bare "libsddc.so" development symlink, which a runtime
# package must not ship and dpkg cannot resolve.
( cd "$REPO" && RUSTFLAGS="-C link-arg=-Wl,-soname,libsddc.so.$SOVER" cargo build --release )

LIBSRC="$REPO/target/release/libsddc.so"
HDRSRC="$REPO/target/release/libsddc.h"
for f in "$LIBSRC" "$HDRSRC"; do
    [ -f "$f" ] || { echo "missing build output: $f" >&2; exit 1; }
done

rm -rf "$OUT"; mkdir -p "$OUT"
STAGE="$OUT/stage"

say "staging"
L="$STAGE/libsddc$SOVER"
install -d "$L/usr/lib/$MULTIARCH" "$L/lib/udev/rules.d" "$L/DEBIAN" \
           "$L/usr/share/doc/libsddc$SOVER"
install -m644 "$LIBSRC" "$L/usr/lib/$MULTIARCH/libsddc.so.$SOVER"
install -m644 "$REPO/packaging/60-rx888.rules" "$L/lib/udev/rules.d/60-rx888.rules"

D="$STAGE/libsddc-dev"
install -d "$D/usr/lib/$MULTIARCH" "$D/usr/include" "$D/DEBIAN" \
           "$D/usr/share/doc/libsddc-dev"
install -m644 "$HDRSRC" "$D/usr/include/libsddc.h"
ln -sf "libsddc.so.$SOVER" "$D/usr/lib/$MULTIARCH/libsddc.so"

for p in "libsddc$SOVER" libsddc-dev; do
    install -m644 "$REPO/packaging/copyright" "$STAGE/$p/usr/share/doc/$p/copyright"
done

say "resolving dependencies"
tmp="$(mktemp -d)"; mkdir -p "$tmp/debian"
printf 'Source: libsddc\nPackage: tmp\nArchitecture: any\n' > "$tmp/debian/control"
: > "$tmp/debian/substvars"
LIB_DEPS="$( cd "$tmp" && dpkg-shlibdeps -O --warnings=0 \
    "$L/usr/lib/$MULTIARCH/libsddc.so.$SOVER" 2>/dev/null | sed 's/^shlibs:Depends=//' )"
rm -rf "$tmp"
echo "  libsddc$SOVER: $LIB_DEPS"

say "writing control files"
cat > "$L/DEBIAN/control" <<EOF
Package: libsddc$SOVER
Version: $FULLVER
Section: libs
Priority: optional
Architecture: $ARCH
Maintainer: $MAINTAINER
Depends: $LIB_DEPS
Recommends: udev
Description: Driver library for the RX888 family of SDR receivers
 Native driver for RX888, RX888 MkII and related SDDC hardware, speaking the
 RaspSDR register protocol. Uploads the device firmware on open and streams
 the ADC over USB 3.
 .
 This package contains the shared library and a udev rule granting the
 console user access to the device.
EOF

cat > "$L/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
if [ "$1" = configure ] && command -v udevadm >/dev/null 2>&1; then
    # so an already-plugged device picks up the new permissions
    udevadm control --reload-rules >/dev/null 2>&1 || true
    udevadm trigger --subsystem-match=usb --attr-match=idVendor=04b4 >/dev/null 2>&1 || true
fi
EOF
chmod 755 "$L/DEBIAN/postinst"

cat > "$L/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
if [ "$1" = remove ] && command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules >/dev/null 2>&1 || true
fi
EOF
chmod 755 "$L/DEBIAN/postrm"

cat > "$D/DEBIAN/control" <<EOF
Package: libsddc-dev
Version: $FULLVER
Section: libdevel
Priority: optional
Architecture: $ARCH
Maintainer: $MAINTAINER
Depends: libsddc$SOVER (= $FULLVER)
Description: Development files for the RX888 driver library
 Header and linker symlink for building against libsddc.
EOF

say "building packages"
for p in "libsddc$SOVER" libsddc-dev; do
    ( cd "$STAGE/$p" && find . -type f ! -path './DEBIAN/*' -printf '%P\0' \
        | xargs -0 md5sum > DEBIAN/md5sums 2>/dev/null || true )
    fakeroot dpkg-deb --build --root-owner-group "$STAGE/$p" \
        "$OUT/${p}_${FULLVER}_${ARCH}.deb" >/dev/null
done

say "done"
ls -1 "$OUT"/*.deb
