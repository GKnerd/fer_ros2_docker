#!/usr/bin/env bash
#
# nic_orin_config.sh -- tune NIC interrupt coalescing on the Jetson Orin.
#
# Lowers the interrupt coalescing latency on the interface that carries Franka
# FCI traffic, so the 1 kHz control loop is not delayed by interrupt batching.
#
# The settings are deliberately volatile: they live in the driver only and are
# lost on reboot, driver reload or interface removal. An operator has to enable
# them consciously before a control session.
#
# Usage: sudo ./nic_orin_config.sh [-i IFACE] [-n] [-h]

set -euo pipefail

readonly SCRIPT_NAME="${0##*/}"

# Defaults. Overridable by flag (interface) or environment (values).
IFACE="${IFACE:-eno1}"
DRY_RUN=0

# Coalescing knobs, in the order they are reported.
readonly PARAMS=(rx-usecs rx-frames tx-usecs tx-frames)

declare -A REQUESTED=(
    [rx-usecs]="${RX_USECS:-6}"
    [rx-frames]="${RX_FRAMES:-1}"
    [tx-usecs]="${TX_USECS:-32}"
    [tx-frames]="${TX_FRAMES:-1}"
)

declare -A BEFORE=()
declare -A AFTER=()

# Set while the link is intentionally down, so the EXIT trap can restore it.
LINK_IS_DOWN=0

log()  { printf '[%s] %s\n' "$SCRIPT_NAME" "$*"; }
warn() { printf '[%s] WARNING: %s\n' "$SCRIPT_NAME" "$*" >&2; }
die()  { printf '[%s] ERROR: %s\n' "$SCRIPT_NAME" "$*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: sudo $SCRIPT_NAME [-i IFACE] [-n] [-h]

Applies low-latency interrupt coalescing to a network interface and reports the
old and the new value of every parameter it touches.

  -i IFACE   interface to configure (default: ${IFACE})
  -n         dry run: report what would change, touch nothing
  -h         this help

Requested values, overridable by environment variable:
  RX_USECS=${REQUESTED[rx-usecs]}   RX_FRAMES=${REQUESTED[rx-frames]}
  TX_USECS=${REQUESTED[tx-usecs]}   TX_FRAMES=${REQUESTED[tx-frames]}

The settings are volatile and are lost on reboot or driver reload.
Exit status is 0 only if every requested value is in effect afterwards.
EOF
}

parse_args() {
    local opt
    while getopts ':i:nh' opt; do
        case "$opt" in
            i) IFACE="$OPTARG" ;;
            n) DRY_RUN=1 ;;
            h) usage; exit 0 ;;
            :) die "option -$OPTARG requires an argument" ;;
            *) usage >&2; die "unknown option -$OPTARG" ;;
        esac
    done
    shift $((OPTIND - 1))
    (($# == 0)) || die "unexpected argument: $1"
}

preflight() {
    local cmd key value

    for cmd in ethtool ip awk; do
        command -v "$cmd" >/dev/null 2>&1 || die "required command not found: $cmd"
    done

    for key in "${PARAMS[@]}"; do
        value="${REQUESTED[$key]}"
        [[ $value =~ ^[0-9]+$ ]] || die "requested $key is not a non-negative integer: '$value'"
    done

    [[ -e /sys/class/net/$IFACE ]] || die "no such network interface: $IFACE"

    if ((DRY_RUN == 0)) && ((EUID != 0)); then
        die "root privileges required to change interface settings, re-run with sudo"
    fi

    ethtool -c "$IFACE" >/dev/null 2>&1 ||
        die "$IFACE does not support interrupt coalescing, or it cannot be queried"
}

# read_coalesce ARRAY_NAME -- fill the named associative array from ethtool -c.
read_coalesce() {
    local -n dest="$1"
    local out key

    out=$(ethtool -c "$IFACE") || die "cannot read coalescing parameters from $IFACE"

    for key in "${PARAMS[@]}"; do
        dest["$key"]=$(awk -F':[[:space:]]*' -v k="$key" '$1 == k { print $2; exit }' <<<"$out")
        [[ -n ${dest[$key]} ]] || die "$IFACE does not report '$key'"
        [[ ${dest[$key]} != "n/a" ]] || die "the $IFACE driver does not support '$key'"
    done
}

restore_link() {
    ((LINK_IS_DOWN)) || return 0
    warn "aborting while $IFACE is down, restoring the link"
    ip link set "$IFACE" up ||
        warn "could not restore the link, run: sudo ip link set $IFACE up"
}

apply() {
    local args=() key

    for key in "${PARAMS[@]}"; do
        args+=("$key" "${REQUESTED[$key]}")
    done

    trap restore_link EXIT

    log "bringing $IFACE down"
    ip link set "$IFACE" down
    LINK_IS_DOWN=1

    log "applying: ethtool -C $IFACE ${args[*]}"
    ethtool -C "$IFACE" "${args[@]}" || die "ethtool rejected the requested parameters"

    log "bringing $IFACE up"
    ip link set "$IFACE" up
    LINK_IS_DOWN=0

    trap - EXIT
}

print_report() {
    local key status changed=0 failed=0

    printf '\n%-12s %10s %10s %10s   %s\n' PARAMETER BEFORE REQUESTED AFTER STATUS
    printf '%s\n' "-------------------------------------------------------------"

    for key in "${PARAMS[@]}"; do
        if ((DRY_RUN)); then
            if [[ ${BEFORE[$key]} == "${REQUESTED[$key]}" ]]; then
                status="already set"
            else
                status="would change"
                ((++changed))
            fi
            printf '%-12s %10s %10s %10s   %s\n' \
                "$key" "${BEFORE[$key]}" "${REQUESTED[$key]}" "-" "$status"
            continue
        fi

        if [[ ${AFTER[$key]} != "${REQUESTED[$key]}" ]]; then
            status="FAILED"
            ((++failed))
        elif [[ ${BEFORE[$key]} == "${AFTER[$key]}" ]]; then
            status="unchanged"
        else
            status="changed"
            ((++changed))
        fi

        printf '%-12s %10s %10s %10s   %s\n' \
            "$key" "${BEFORE[$key]}" "${REQUESTED[$key]}" "${AFTER[$key]}" "$status"
    done

    printf '\n'

    if ((DRY_RUN)); then
        log "dry run: $changed of ${#PARAMS[@]} parameters would change on $IFACE"
        return 0
    fi

    if ((failed)); then
        warn "$failed of ${#PARAMS[@]} parameters did not take effect on $IFACE"
        warn "the driver may clamp or ignore these values, check the requested range"
        return 1
    fi

    local revert=() key2
    for key2 in "${PARAMS[@]}"; do
        revert+=("$key2" "${BEFORE[$key2]}")
    done

    log "$changed of ${#PARAMS[@]} parameters changed on $IFACE"
    log "settings are volatile and will be lost on reboot or driver reload"
    log "to revert now: sudo ethtool -C $IFACE ${revert[*]}"
    return 0
}

main() {
    parse_args "$@"
    preflight

    log "reading current coalescing parameters from $IFACE"
    read_coalesce BEFORE

    if ((DRY_RUN)); then
        log "dry run, $IFACE will not be touched"
    else
        apply
        log "re-reading coalescing parameters from $IFACE"
        read_coalesce AFTER
    fi

    local rc=0
    print_report || rc=$?
    exit "$rc"
}

main "$@"
