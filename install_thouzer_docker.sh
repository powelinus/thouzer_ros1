#!/usr/bin/env bash
# THOUZER ROS1 Docker helper (v2: Python 2 pip constraints fix)
# Taruh file ini di root repository hasil clone: thouzer_ros1/
# OS host: Linux + Docker Engine. ROS Melodic (Python 2) berjalan di dalam image.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
IMAGE="${THOUZER_IMAGE:-thouzer-ros1:melodic}"
NAME="${THOUZER_CONTAINER:-thouzer-ros1-robot}"
USE_IMU="${THOUZER_USE_IMU:-false}"
IMU_PORT="${THOUZER_IMU_PORT:-/dev/ttyACM0}"
ROS_MASTER_URI_VALUE="${THOUZER_ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP_VALUE="${THOUZER_ROS_IP:-127.0.0.1}"
GENERATED_DIR="${ROOT}/.thouzer-docker"

say()  { printf '\033[1;36m[thouzer]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[peringatan]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
THOUZER ROS1 -> Docker (tanpa docker-compose bawaan)

Letakkan script ini di folder hasil clone lalu jalankan:
  chmod +x install_thouzer_docker.sh
  ./install_thouzer_docker.sh install    # build image, TIDAK menghidupkan robot
  ./install_thouzer_docker.sh start      # jalankan driver THOUZER di background
  ./install_thouzer_docker.sh logs       # log robot (Ctrl+C hanya keluar dari logs)
  ./install_thouzer_docker.sh shell      # terminal ROS di container robot
  ./install_thouzer_docker.sh rviz       # RViz; butuh desktop X11/XWayland
  ./install_thouzer_docker.sh ros rosnode list
  ./install_thouzer_docker.sh ros roslaunch thouzer_slam slam_gmapping.launch
  ./install_thouzer_docker.sh ros roslaunch thouzer_navigation thouzer_navigation.launch
  ./install_thouzer_docker.sh status
  ./install_thouzer_docker.sh stop
  ./install_thouzer_docker.sh remove
  ./install_thouzer_docker.sh rebuild    # build tanpa cache

Variabel opsional (contoh):
  THOUZER_USE_IMU=true THOUZER_IMU_PORT=/dev/ttyACM0 ./install_thouzer_docker.sh start
  THOUZER_NVIDIA=true ./install_thouzer_docker.sh rviz
  THOUZER_ROS_IP=192.168.1.10 ./install_thouzer_docker.sh start

Catatan:
  * Robot harus dapat diakses dari HOST melalui jaringan (default 192.168.212.1).
  * Koneksi MQTT & topic ada di thouzer_driver/config/mqtt.yaml.
  * IP LiDAR ada di thouzer_driver/launch/ether_urg.launch.
  * Default IMU nonaktif, sesuai docker-compose robot yang tersedia di repo.
  * Perubahan program selain file konfigurasi/launch perlu rebuild image.
  * ROS Melodic dan Python 2 telah EOL: gunakan di jaringan tepercaya.
EOF
}

check_repo() {
  [[ -f "$ROOT/docker/Dockerfile" ]] || die "docker/Dockerfile tidak ditemukan. Simpan script di root repo thouzer_ros1."
  [[ -f "$ROOT/.melodic_depends.rosinstall" ]] || die "File .melodic_depends.rosinstall tidak ditemukan."
  [[ -f "$ROOT/thouzer_bringup/launch/thouzer_robot.launch" ]] || die "Paket thouzer_bringup tidak ditemukan."
  [[ -f "$ROOT/thouzer_driver/config/mqtt.yaml" ]] || die "Konfigurasi MQTT tidak ditemukan."
}

check_docker() {
  [[ "$(uname -s)" == Linux ]] || die "Script ini membutuhkan Linux host (Docker --network host dan device passthrough)."
  command -v docker >/dev/null 2>&1 || die "Docker belum terpasang. Install Docker Engine, lalu ulangi."
  if ! docker info >/dev/null 2>&1; then
    die "Tidak bisa mengakses Docker daemon. Cek 'sudo systemctl status docker' dan izin user (grup docker)."
  fi
}

image_exists() { docker image inspect "$IMAGE" >/dev/null 2>&1; }
container_exists() { docker container inspect "$NAME" >/dev/null 2>&1; }
container_running() { [[ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null || true)" == true ]]; }

# Membuat Dockerfile terpisah: kode asli tidak diubah.
# Memperbarui kunci APT ROS untuk image lama; constraints mencegah duplicate pip requirement pada Python 2.
make_dockerfile() {
  mkdir -p "$GENERATED_DIR"
  local target="$GENERATED_DIR/Dockerfile"
  awk '
    /^FROM / && !inserted {
      print
      print "ENV DEBIAN_FRONTEND=noninteractive"
      print "# Perbarui key ROS pada image Melodic lama sebelum apt-get update."
      print "ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /tmp/thouzer-ros.key"
      print "RUN apt-key add /tmp/thouzer-ros.key && rm -f /tmp/thouzer-ros.key && \\"
      print "    rm -f /etc/apt/sources.list.d/ros1-latest.list /etc/apt/sources.list.d/ros-latest.list /etc/apt/sources.list.d/ros1-snapshots.list && \\"
      print "    echo \"deb http://packages.ros.org/ros/ubuntu bionic main\" > /etc/apt/sources.list.d/ros1-thouzer.list"
      inserted=1
      next
    }
    $0 == "        curl \\" {
      print
      print "        git \\"
      next
    }
    /pip install -r mqtt_bridge\/requirements.txt/ {
      print "    pip install --no-cache-dir \"pip<21\" \"setuptools<45\" && \\"
      print "    printf \"%s\\n\" \"paho-mqtt<2\" \"pymongo<4\" > /tmp/thouzer-python2-constraints.txt && \\"
      print "    pip install --no-cache-dir -c /tmp/thouzer-python2-constraints.txt -r mqtt_bridge/requirements.txt && \\"
      print "    rm -f /tmp/thouzer-python2-constraints.txt && \\"
      next
    }
    {
      gsub(/sudo apt-key add -/, "apt-key add -")
      gsub(/ros\/rosdistro\/master\/ros.asc/, "ros/rosdistro/master/ros.key")
      print
    }
  ' "$ROOT/docker/Dockerfile" > "$target"
  [[ -s "$target" ]] || die "Gagal membuat Dockerfile kompatibilitas."
  say "Dockerfile build: $target (Dockerfile repo tetap utuh)"
}

build_image() {
  check_repo
  check_docker
  if [[ "$(uname -m)" != x86_64 ]]; then
    warn "Host bukan x86_64. Image osrf/ros:melodic-desktop-full dapat tidak tersedia untuk arsitektur ini."
  fi
  make_dockerfile
  say "Membangun image $IMAGE (ROS Melodic / Ubuntu 18.04)..."
  local args=()
  if [[ "${1:-}" == --no-cache ]]; then args+=(--no-cache); fi
  if ! docker build "${args[@]}" -t "$IMAGE" -f "$GENERATED_DIR/Dockerfile" "$ROOT"; then
    die "Build gagal. ROS Melodic/Python 2 sudah EOL; baca pesan apt/pip/rosdep di atas. Image tidak dijalankan."
  fi
  say "Build berhasil. Jalankan: ./install_thouzer_docker.sh start"
}

# Root source masuk ke image saat build. Hanya konfigurasi & launch yang di-bind
# agar modifikasi IP/topics/maps dapat langsung terpakai saat container baru dimulai.
common_mounts() {
  mkdir -p "$ROOT/maps"
  MOUNTS=(
    --mount "type=bind,source=$ROOT/maps,target=/root/catkin_ws/maps"
    --mount "type=bind,source=$ROOT/thouzer_driver/config,target=/root/catkin_ws/src/thouzer_ros/thouzer_driver/config,readonly"
    --mount "type=bind,source=$ROOT/thouzer_driver/launch,target=/root/catkin_ws/src/thouzer_ros/thouzer_driver/launch,readonly"
    --mount "type=bind,source=$ROOT/thouzer_bringup/launch,target=/root/catkin_ws/src/thouzer_ros/thouzer_bringup/launch,readonly"
  )
}

start_robot() {
  check_repo
  check_docker
  image_exists || die "Image $IMAGE belum ada. Jalankan './install_thouzer_docker.sh install' dulu."
  if container_running; then
    say "Container $NAME sudah berjalan."
    docker ps --filter "name=^/${NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    return
  fi
  if container_exists; then
    say "Menghapus container lama (image dan folder host tidak dihapus)."
    docker rm "$NAME" >/dev/null
  fi

  common_mounts
  local device_args=()
  local imu_args=("use_imu:=false")
  case "${USE_IMU,,}" in
    true|yes|1)
      [[ -e "$IMU_PORT" ]] || die "Port IMU $IMU_PORT tidak ada pada host. Pastikan perangkat terhubung."
      device_args+=(--device "$IMU_PORT:$IMU_PORT")
      imu_args=("use_imu:=true" "imu_port:=$IMU_PORT")
      ;;
    false|no|0) ;;
    *) die "THOUZER_USE_IMU harus true/false." ;;
  esac

  say "Memulai $NAME: network host, IMU=$USE_IMU."
  docker run -d \
    --name "$NAME" --hostname "$NAME" --init \
    --network host \
    --env "ROS_MASTER_URI=$ROS_MASTER_URI_VALUE" \
    --env "ROS_IP=$ROS_IP_VALUE" \
    "${MOUNTS[@]}" "${device_args[@]}" \
    "$IMAGE" \
    roslaunch thouzer_bringup thouzer_robot.launch "${imu_args[@]}" publish_odom:=true >/dev/null
  say "Container dimulai. Cek: ./install_thouzer_docker.sh logs"
  warn "Pastikan jaringan host dapat mencapai IP robot dan MQTT di file config."
}

require_running() {
  check_docker
  container_running || die "Container $NAME tidak berjalan. Jalankan './install_thouzer_docker.sh start'."
}

# Menjalankan command ROS via docker exec perlu source environment (entrypoint
# hanya dieksekusi ketika docker run, tidak pada docker exec).
run_ros() {
  require_running
  [[ "$#" -gt 0 ]] || die "Contoh: ./install_thouzer_docker.sh ros rostopic list"
  docker exec -it "$NAME" bash -lc \
    'source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && exec "$@"' \
    thouzer-ros-exec "$@"
}

start_rviz() {
  check_repo
  check_docker
  image_exists || die "Image belum ada. Jalankan install dulu."
  [[ -n "${DISPLAY:-}" ]] || die "DISPLAY kosong; RViz membutuhkan X11 atau XWayland di host."
  [[ -d /tmp/.X11-unix ]] || die "Socket X11 /tmp/.X11-unix tidak ditemukan."
  common_mounts
  local gui_args=(--env "DISPLAY=$DISPLAY" --env QT_X11_NO_MITSHM=1
    --mount 'type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix')
  local xauth_file="${XAUTHORITY:-${HOME}/.Xauthority}"
  if [[ -f "$xauth_file" ]]; then
    gui_args+=(--mount "type=bind,source=$xauth_file,target=/root/.Xauthority,readonly" --env XAUTHORITY=/root/.Xauthority)
  else
    warn "Xauthority tidak ditemukan; jika RViz gagal membuka GUI, periksa izin X11/XWayland."
  fi
  if [[ -d /dev/dri ]]; then
    gui_args+=(--device /dev/dri:/dev/dri)
  fi
  if [[ "${THOUZER_NVIDIA:-false}" == true ]]; then
    gui_args+=(--gpus all --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display)
  else
    gui_args+=(--env LIBGL_ALWAYS_SOFTWARE=1)
  fi
  say "Menjalankan RViz sebagai container sementara (Ctrl+C untuk menutup)."
  if ! container_running; then
    warn "Container robot tidak berjalan. RViz tetap dibuka, tetapi topik robot belum tentu tersedia."
  fi
  docker run --rm -it --network host \
    --env "ROS_MASTER_URI=$ROS_MASTER_URI_VALUE" --env "ROS_IP=$ROS_IP_VALUE" \
    "${MOUNTS[@]}" "${gui_args[@]}" \
    "$IMAGE" roslaunch thouzer_bringup thouzer_remote_debugger.launch
}

main() {
  local action="${1:-install}"
  if [[ "$#" -gt 0 ]]; then shift; fi
  case "$action" in
    install|build) build_image ;;
    rebuild) build_image --no-cache ;;
    start) start_robot ;;
    logs) check_docker; container_exists || die "Container belum ada."; docker logs -f --tail 100 "$NAME" ;;
    shell) require_running; docker exec -it "$NAME" bash -lc 'source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && exec bash -i' ;;
    ros) run_ros "$@" ;;
    rviz) start_rviz ;;
    status)
      check_docker
      if container_exists; then
        docker ps -a --filter "name=^/${NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
      else
        say "Container $NAME tidak ditemukan."
      fi
      ;;
    stop)
      check_docker
      if container_running; then docker stop "$NAME"; else say "Container sudah berhenti/tidak ada."; fi
      ;;
    remove)
      check_docker
      if container_exists; then docker rm -f "$NAME"; else say "Container tidak ada."; fi
      ;;
    help|-h|--help) usage ;;
    *) usage >&2; die "Perintah tidak dikenal: $action" ;;
  esac
}

main "$@"
