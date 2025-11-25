#!/usr/bin/env bash
# manage_autorace_symlinks.sh
# 전체 과정: (1) 기존 항목 백업 또는 삭제 (2) 심볼릭 링크 생성 (3) (선택) catkin_make 빌드 및 소스
#
# 사용법 예시:
# 1) 기본 (백업 후 링크 생성, 빌드 안 함):
#    ./manage_autorace_symlinks.sh
#
# 2) 삭제하고 링크 생성 (주의: 복구 불가):
#    ./manage_autorace_symlinks.sh --delete --force
#
# 3) dry-run(무해한 실행 확인용):
#    ./manage_autorace_symlinks.sh --dry-run
#
# 4) 백업하고 링크 생성 후 catkin_make 실행 & 소스:
#    ./manage_autorace_symlinks.sh --build --source
#
# 옵션:
#   -n, --dry-run    : 실제로 파일을 변경하지 않고 어떤 동작이 일어날지 출력
#   -b, --backup     : 기존 폴더가 있으면 백업(기본 동작)
#   -d, --delete     : 기존 폴더가 있으면 삭제 (위험). --force 필요
#   -f, --force      : --delete를 묻지 않고 진행
#   -c, --build      : catkin_make 실행 (작업공간에서)
#   -s, --source     : 빌드 후 devel/setup.bash 소스 (자동으로 빌드 옵션과 함께 사용)
#   -h, --help       : 도움말 출력
#
# 경로는 /home/wego 기준으로 고정되어 있습니다 (요청대로).
# 필요하면 스크립트 내 변수만 수정하세요.

set -euo pipefail

# ======= 사용자 설정 (필요시 수정) =======
CATKIN_SRC="/home/wego/catkin_ws/src"
ORIG_DIR="/home/wego/autorace2025_vision"
PACKAGES=(stopline_pkg onelane_detection image_preprocessing color_detector camera_merger)
BACKUP_BASE="$HOME/backup_autorace2025_vision"
# ========================================

DRY_RUN=false
MODE="backup"   # "backup" (default) or "delete"
FORCE=false
DO_BUILD=false
DO_SOURCE=false

print_help() {
  sed -n '1,160p' "$0" | sed -n '1,120p'
}

# 간단한 출력 함수
info() { echo -e "[INFO] $*"; }
warn() { echo -e "[WARN] $*"; }
err()  { echo -e "[ERROR] $*" >&2; }

# 옵션 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--dry-run) DRY_RUN=true; shift ;;
    -b|--backup) MODE="backup"; shift ;;
    -d|--delete) MODE="delete"; shift ;;
    -f|--force) FORCE=true; shift ;;
    -c|--build) DO_BUILD=true; shift ;;
    -s|--source) DO_SOURCE=true; shift ;;
    -h|--help) print_help; exit 0 ;;
    *) err "Unknown option: $1"; print_help; exit 1 ;;
  esac
done

if [[ "$MODE" == "delete" && "$FORCE" != true ]]; then
  warn "삭제 모드가 선택되었습니다. 되돌릴 수 없습니다. 계속하려면 --force 옵션을 추가하세요."
  exit 1
fi

if [[ ! -d "$CATKIN_SRC" ]]; then
  err "catkin ws src 경로가 존재하지 않습니다: $CATKIN_SRC"
  exit 1
fi

if [[ ! -d "$ORIG_DIR" ]]; then
  err "원본 경로가 존재하지 않습니다: $ORIG_DIR"
  exit 1
fi

# 동작을 보여주는 dry-run 처리 함수
run_cmd() {
  if $DRY_RUN; then
    echo "[DRY-RUN] $*"
  else
    eval "$@"
  fi
}

timestamp() { date +"%Y%m%d_%H%M%S"; }

# 1) 현재 위치로 이동
info "작업 디렉터리: $CATKIN_SRC"
run_cmd "cd \"$CATKIN_SRC\""

# 2) 어떤 항목이 있는지 표시
info "확인: 대상 패키지 상태"
for pkg in "${PACKAGES[@]}"; do
  if [[ -L "$CATKIN_SRC/$pkg" ]]; then
    info "$pkg : 심볼릭 링크 -> $(readlink -f "$pkg")"
  elif [[ -d "$CATKIN_SRC/$pkg" ]]; then
    info "$pkg : 실제 디렉터리"
  elif [[ -e "$CATKIN_SRC/$pkg" ]]; then
    info "$pkg : 존재(파일 등)"
  else
    info "$pkg : 없음"
  fi
done

# 3) 백업 또는 삭제
if [[ "$MODE" == "backup" ]]; then
  BACKUP_DIR="$BACKUP_BASE/$(timestamp)"
  info "백업 모드: 존재하는 항목은 $BACKUP_DIR 으로 이동됩니다."
  if $DRY_RUN; then
    info "(dry-run) mkdir -p \"$BACKUP_DIR\""
  else
    mkdir -p "$BACKUP_DIR"
  fi

  for pkg in "${PACKAGES[@]}"; do
    if [[ -e "$pkg" && ! -L "$pkg" ]]; then
      info "Moving $pkg -> $BACKUP_DIR/"
      run_cmd "mv \"$pkg\" \"$BACKUP_DIR/\""
    elif [[ -L "$pkg" ]]; then
      info "$pkg 는 이미 심볼릭 링크입니다. 링크만 제거 후 새로 생성합니다."
      run_cmd "rm -f \"$pkg\""
    else
      info "$pkg 없음, 건너뜁니다."
    fi
  done

elif [[ "$MODE" == "delete" ]]; then
  info "삭제 모드: 존재하는 디렉터리를 완전 삭제합니다."
  for pkg in "${PACKAGES[@]}"; do
    if [[ -L "$pkg" ]]; then
      info "링크 제거: $pkg"
      run_cmd "rm -f \"$pkg\""
    elif [[ -d "$pkg" ]]; then
      info "디렉터리 삭제: $pkg"
      run_cmd "rm -rf \"$pkg\""
    else
      info "$pkg 없음, 건너뜁니다."
    fi
  done
fi

# 4) 심볼릭 링크 생성
info "심볼릭 링크 생성: 원본 -> $ORIG_DIR/<pkg>"
for pkg in "${PACKAGES[@]}"; do
  SRC_PATH="$ORIG_DIR/$pkg"
  if [[ ! -e "$SRC_PATH" ]]; then
    warn "원본이 존재하지 않음: $SRC_PATH (링크 생성 건너뜀)"
    continue
  fi
  # 이미 있는 항목(파일/디렉터리)을 재확인하고 건너뛰거나 덮어쓰기
  if [[ -e "$pkg" ]]; then
    warn "목표에 이미 항목이 존재합니다: $pkg (건너뜁니다)"
    continue
  fi
  info "ln -s \"$SRC_PATH\" \"$pkg\""
  run_cmd "ln -s \"$SRC_PATH\" \"$pkg\""
done

# 5) 결과 확인
info "최종 상태:"
run_cmd "ls -l"

# 6) (선택) 빌드
if $DO_BUILD; then
  info "catkin_make 실행 시작..."
  if $DRY_RUN; then
    info "(dry-run) cd /home/wego/catkin_ws && catkin_make"
  else
    pushd /home/wego/catkin_ws >/dev/null
    # 빌드 커맨드(사용자 환경에 따라 catkin build를 쓸 수도 있음)
    if command -v catkin_make >/dev/null 2>&1; then
      catkin_make
    else
      warn "catkin_make 명령을 찾을 수 없습니다. catkin build를 사용 중이면 수동으로 빌드하세요."
    fi
    popd >/dev/null
  fi
fi

# 7) (선택) 빌드 후 소스
if $DO_SOURCE; then
  if $DRY_RUN; then
    info "(dry-run) source /home/wego/catkin_ws/devel/setup.bash"
  else
    SETUP="/home/wego/catkin_ws/devel/setup.bash"
    if [[ -f "$SETUP" ]]; then
      # 소스는 현재 쉘에서만 유효하므로 스크립트는 안내만 합니다.
      info "빌드 후 setup.bash가 있으면 아래 명령을 현재 터미널에서 실행하세요:"
      echo "    source \"$SETUP\""
    else
      warn "setup.bash 파일을 찾을 수 없습니다: $SETUP"
    fi
  fi
fi

info "작업 완료."
if $DRY_RUN; then
  info "주의: dry-run 모드였음 (실제 변경 없음)."
fi