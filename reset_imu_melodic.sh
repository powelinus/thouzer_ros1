#!/usr/bin/env bash
set -Eeuo pipefail

# Jalankan dari Ubuntu HOST, bukan dari shell Docker.
PROJECT="${PROJECT:-$HOME/thouzer_ws/thouzer_ros}"
SERVICE="${SERVICE:-thouzer}"
CONTAINER="${CONTAINER:-thouzer_ros}"
IMAGE="${IMAGE:-thouzer-melodic:mqtt}"
PKG=rt_usb_9axisimu_driver
SRC_IN_IMAGE="/root/catkin_ws/src/thouzer_ros/$PKG"
PKG_DIR="$PROJECT/$PKG"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$PROJECT/${PKG}.backup_${STAMP}"
TMP=""

fail() { echo "ERROR: $*" >&2; exit 1; }
cleanup() { if [[ -n "$TMP" ]]; then docker rm -f "$TMP" >/dev/null 2>&1 || true; fi; }
trap cleanup EXIT

command -v docker >/dev/null || fail "Docker tidak ditemukan. Jalankan di Ubuntu host, bukan di container."
command -v python3 >/dev/null || fail "Python 3 tidak ditemukan di host."
[[ -d "$PROJECT" ]] || fail "Folder project tidak ada: $PROJECT (atur PROJECT=/path/ke/thouzer_ros)"
[[ -f "$PROJECT/compose.yaml" || -f "$PROJECT/compose.yml" || -f "$PROJECT/docker-compose.yml" || -f "$PROJECT/docker-compose.yaml" ]] || fail "File Docker Compose tidak ditemukan di $PROJECT"
[[ -d "$PKG_DIR" ]] || fail "Folder IMU host tidak ditemukan: $PKG_DIR"
docker image inspect "$IMAGE" >/dev/null 2>&1 || fail "Image $IMAGE tidak ada"

# Hindari penghapusan data tanpa persetujuan pengguna.
echo "Project : $PROJECT"
echo "Backup  : $BACKUP"
echo "Image   : $IMAGE"
read -r -p "Lanjut backup dan ganti folder IMU dengan versi ROS1? [y/N] " answer
[[ "$answer" == y || "$answer" == Y ]] || exit 0

cd "$PROJECT"
TMP="$(docker create "$IMAGE")"
STAGING="$PROJECT/.${PKG}.ros1_${STAMP}"
mkdir -p "$STAGING"
if ! docker cp "$TMP:$SRC_IN_IMAGE/." "$STAGING/"; then
  rm -rf "$STAGING"
  fail "Tidak bisa menyalin package ROS1 dari image."
fi

grep -q '<buildtool_depend>catkin</buildtool_depend>' "$STAGING/package.xml" || fail "Package di image bukan catkin."
grep -q 'find_package(catkin' "$STAGING/CMakeLists.txt" || fail "CMakeLists di image bukan catkin."

# Perbaikan readFromDevice: akumulasi partial read dari serial CDC.
HEADER="$STAGING/include/rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"
[[ -f "$HEADER" ]] || fail "Header ROS1 tidak ditemukan: $HEADER"
python3 - "$HEADER" <<'PY'
from pathlib import Path
import re, sys
p = Path(sys.argv[1])
s = p.read_text()
m = re.search(r'\breadFromDevice\s*\(\s*unsigned\s+char\s*\*\s*buf\s*,\s*unsigned\s+int\s+buf_len\s*\)', s)
if not m:
    sys.exit('ERROR: Fungsi readFromDevice tidak ditemukan; folder asli tidak diubah.')
start = s.find('{', m.end())
if start == -1 or ';' in s[m.end():start]:
    sys.exit('ERROR: Definisi fungsi readFromDevice tidak ditemukan.')
level = 0
end = None
for i in range(start, len(s)):
    if s[i] == '{': level += 1
    elif s[i] == '}':
        level -= 1
        if level == 0:
            end = i
            break
if end is None:
    sys.exit('ERROR: Kurung penutup readFromDevice tidak ditemukan.')
body = s[start+1:end]
if 'port_fd_' not in body:
    sys.exit('ERROR: readFromDevice tidak menggunakan port_fd_; patch dibatalkan.')
new_body = '''
    if (port_fd_ < 0) {
      return -1;
    }
    unsigned int total = 0;
    while (total < buf_len)
    {
      int n = read(port_fd_, buf + total, buf_len - total);
      if (n <= 0) { return n; }
      total += n;
    }
    return total;
  '''
s = s[:start+1] + new_body + s[end:]
p.write_text(s)
print('OK: readFromDevice() patched pada source ROS1')
PY
# Hapus hanya metadata Git internal package, bukan .git di root project.
rm -rf "$STAGING/.git"

# Pastikan hanya launch dalam direktori launch/ yang dipakai.
if [[ -f "$STAGING/$PKG.launch" && -f "$STAGING/launch/$PKG.launch" ]]; then
  mv "$STAGING/$PKG.launch" "$STAGING/$PKG.launch.bak"
fi
[[ -f "$STAGING/launch/$PKG.launch" ]] || fail "Launch ROS1 tidak ditemukan di launch/$PKG.launch"

# Rename atomik setelah seluruh validasi staging selesai.
mv "$PKG_DIR" "$BACKUP"
mv "$STAGING" "$PKG_DIR"
echo "OK: folder lama disimpan di $BACKUP"

# Cek volume pada Compose agar source host yang baru benar-benar dipakai.
if ! grep -Eq '\./rt_usb_9axisimu_driver:/root/catkin_ws/src/thouzer_ros/rt_usb_9axisimu_driver' "$PROJECT"/compose.y*ml "$PROJECT"/docker-compose.y*ml 2>/dev/null; then
  fail "Bind mount IMU belum ditemukan di Compose. Tambahkan: ./rt_usb_9axisimu_driver:/root/catkin_ws/src/thouzer_ros/rt_usb_9axisimu_driver lalu jalankan ulang script atau docker compose up. Backup sudah dibuat."
fi

# Recreate WAJIB: rename direktori host membuat bind mount container lama menunjuk inode lama.
echo "Recreate container agar bind mount menunjuk folder ROS1 yang baru..."
docker compose up -d --no-deps --force-recreate "$SERVICE"

echo "Verifikasi package dan build di Docker..."
docker exec "$CONTAINER" bash -lc '
  set -e
  cd /root/catkin_ws
  grep -q "<buildtool_depend>catkin</buildtool_depend>" src/thouzer_ros/rt_usb_9axisimu_driver/package.xml
  source /opt/ros/melodic/setup.bash
  catkin build rt_usb_9axisimu_driver
  source devel/setup.bash
  rospack find rt_usb_9axisimu_driver
'

echo
 echo "SELESAI: IMU ROS1 Melodic berhasil dibuild."
echo "Backup lama: $BACKUP"
echo "Tes manual (di Docker): roslaunch rt_usb_9axisimu_driver rt_usb_9axisimu_driver.launch port:=/dev/ttyACM0"
echo "Catatan: konfigurasi openSerialPort() dari image ROS1 belum diubah otomatis; uji data IMU dahulu."
