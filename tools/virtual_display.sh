#!/usr/bin/env bash
# virtual_display.sh COMMAND [ARGUMENT...]: runs COMMAND with every window it opens on a virtual X display (Xvfb), not
# on the desktop: the rendering of a recording (`tools/virtual_display.sh tools/render_flight.sh SCENARIO FIRST_RUN
# DIRECTORY`) or a GUI flight looked at by screenshots, while somebody works at the machine. The display is the first
# free one from :99, 1920 x 1080; the container takes it as it takes any DISPLAY (scripts/container_run.sh), and the
# Wayland session's variable is dropped so that nothing falls back to the desktop.
#
# virtual_display.sh --install: names the host packages that are missing (xvfb, imagemagick, curl: the host's to
# install, docs/installation.md), then fetches VirtualGL 3.1.5 (its release's package, checked by its SHA-256) and
# unpacks it into log/tools/virtualgl/root. The desktop's GLX is not on a virtual display, and without VirtualGL the windows are
# drawn in software (Mesa's llvmpipe): the world window of a recording redraws some 8 times a second and the
# recording is refused (r3840 rendered so: "the world window stood still 86% of the time"). With it unpacked the
# recording windows draw on the GPU through EGL and are read back into their X windows (RECORDING_VIRTUALGL,
# scripts/video_recording_runtime.sh): the same flight rendered at a rate of 1.0 is a recording that passes, every
# frame within 0.07 s of its moment, 17.8 different frames a second of the world against the desktop's 24.
#
# What the display shows: `DISPLAY=:N import -window root picture.png` (the display is printed at the start); there
# is no window manager, the windows lie on one another at the corner. The display is stopped with COMMAND.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
virtualgl=log/tools/virtualgl
if [ $# -eq 0 ]; then
  sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  exit 2
fi
if [ "$1" = "--install" ]; then
  # The host's own packages are the host's to install: what is missing is named with its command.
  missing=()
  command -v Xvfb > /dev/null || missing+=(xvfb)
  command -v import > /dev/null || missing+=(imagemagick)
  command -v curl > /dev/null || missing+=(curl)
  if [ ${#missing[@]} -gt 0 ]; then
    echo "virtual_display.sh: install on the host first: sudo apt install ${missing[*]}" >&2
    exit 1
  fi
  package=virtualgl_3.1.5_amd64.deb
  sum=df3f7788ce41b182a47c0d298e5cd6d2d63579522cb41825970b7726e825485e
  mkdir -p "$virtualgl" && touch "$virtualgl/.keep"
  curl --fail --location --silent --show-error --output "$virtualgl/$package" \
    "https://github.com/VirtualGL/virtualgl/releases/download/3.1.5/$package" || exit 1
  echo "$sum  $virtualgl/$package" | sha256sum --check --quiet || { echo "virtual_display.sh: $package is not the release's" >&2; exit 1; }
  rm -rf "$virtualgl/root" && dpkg -x "$virtualgl/$package" "$virtualgl/root" && echo "VirtualGL 3.1.5 in $virtualgl/root"
  exit $?
fi
command -v Xvfb > /dev/null ||
  { echo "virtual_display.sh: Xvfb is not installed: sudo apt install xvfb (docs/installation.md)" >&2; exit 1; }
number=99
while [ -e "/tmp/.X11-unix/X$number" ] || [ -e "/tmp/.X$number-lock" ]; do number=$((number + 1)); done
Xvfb ":$number" -screen 0 1920x1080x24 -nolisten tcp > /dev/null 2>&1 &
server=$!
trap 'kill "$server" 2> /dev/null' EXIT
for _ in $(seq 1 50); do
  [ -e "/tmp/.X11-unix/X$number" ] && break
  sleep 0.1
done
[ -e "/tmp/.X11-unix/X$number" ] || { echo "virtual_display.sh: Xvfb :$number did not start" >&2; exit 1; }
drawn="in software (virtual_display.sh --install brings the GPU)"
gl=()
if [ -x "$virtualgl/root/opt/VirtualGL/bin/vglrun" ]; then
  gl=(RECORDING_VIRTUALGL="/workspace/$virtualgl/root")
  drawn="on the GPU through VirtualGL"
fi
echo "VIRTUAL_DISPLAY :$number (Xvfb, pid $server), the recording windows drawn $drawn"
env -u WAYLAND_DISPLAY DISPLAY=":$number" "${gl[@]}" "$@"
