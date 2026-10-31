#!/bin/sh
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if [ -n "${SYDRAHYNTH_APP:-}" ]; then
    app_bundle=$SYDRAHYNTH_APP
elif [ -d "$repo_root/build-release/Sydrahynth_artefacts/Release/Standalone/Sydrahynth.app" ]; then
    app_bundle="$repo_root/build-release/Sydrahynth_artefacts/Release/Standalone/Sydrahynth.app"
elif [ -d "$repo_root/build/Sydrahynth_artefacts/RelWithDebInfo/Standalone/Sydrahynth.app" ]; then
    app_bundle="$repo_root/build/Sydrahynth_artefacts/RelWithDebInfo/Standalone/Sydrahynth.app"
else
    echo "Sydrahynth standalone app not found. Build it first with: cmake --build build" >&2
    exit 1
fi

if [ ! -d "$app_bundle" ]; then
    echo "Standalone app not found: $app_bundle" >&2
    exit 1
fi

if /usr/bin/pgrep -x Sydrahynth >/dev/null 2>&1; then
    /usr/bin/osascript -e 'tell application id "com.sydrahynth.Sydrahynth" to quit' >/dev/null
    attempts=0
    while /usr/bin/pgrep -x Sydrahynth >/dev/null 2>&1; do
        if [ "$attempts" -ge 20 ]; then
            echo "Sydrahynth did not quit; leaving the existing instance open." >&2
            exit 1
        fi
        sleep 1
        attempts=$((attempts + 1))
    done
fi

open -n "$app_bundle"
