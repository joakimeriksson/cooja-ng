#!/bin/sh
# Fetch + verify + unpack GNU Lightning 2.2.3 into ./lightning-2.2.3, with the
# vendored mprotect fix applied.  Used by both test.yml and release.yml so the
# two recipes cannot drift.
#
# This exists because `curl -sL url | tar xz` fails SILENTLY: -s without -f
# turns an HTTP error into an empty tar stream, and the next step died with
# "cd: lightning-2.2.3: No such file" — which is how the first v0.2.0 release
# run lost its macos-x64 build to a transient ftp.gnu.org hiccup.  Now: fail on
# HTTP errors, retry, fall back to the GNU mirror network, and check the sha256
# (the same one Homebrew's formula pins) before unpacking anything.
set -eu
VER=2.2.3
SHA=c045c7a33a00affbfeb11066fa502c03992e474a62ba95977aad06dbc14c6829
TGZ=lightning-$VER.tar.gz
rm -f "$TGZ"
for url in "https://ftp.gnu.org/gnu/lightning/$TGZ" \
           "https://ftpmirror.gnu.org/gnu/lightning/$TGZ" \
           "https://mirrors.kernel.org/gnu/lightning/$TGZ"; do
    if curl -fsSL --retry 5 --retry-delay 3 --retry-all-errors "$url" -o "$TGZ"; then
        got=$(shasum -a 256 "$TGZ" | cut -c1-64)
        [ "$got" = "$SHA" ] && break
        echo "sha256 mismatch from $url: $got" >&2; rm -f "$TGZ"
    fi
done
[ -f "$TGZ" ] || { echo "could not fetch a valid $TGZ from any mirror" >&2; exit 1; }
rm -rf "lightning-$VER"; tar xzf "$TGZ"
patch -d "lightning-$VER" -p1 < "$(dirname "$0")/../patches/lightning-$VER-mprotect.patch"
echo "lightning-$VER ready (sha256 verified)"
