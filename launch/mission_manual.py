#!/usr/bin/env python3
"""
Mission Script Manual: TR + BR
Jalankan: python3 mission_manual.py
Sistem: Full Manual (Tekan ENTER untuk lanjut tiap fase)
"""

import subprocess
import sys

# ============================================================
# Helper functions
# ============================================================
def run_br_phase(phase, desc=""):
    """Kirim Action Goal ke BR, blokir sampai selesai."""
    print(f"\n🚀 [BR Phase {phase}] {desc}")
    print("─" * 60)
    cmd = (
        f'ros2 action send_goal /br/build_stack '
        f'br_robot_interfaces/action/BuildStack '
        f'"{{mode: \'MOVE\', phase: \'{phase}\'}}" --feedback'
    )
    subprocess.run(cmd, shell=True)

def run_tr_phase(phase, desc=""):
    """Kirim Action Goal ke TR, blokir sampai selesai."""
    print(f"\n🚀 [TR Phase {phase}] {desc}")
    print("─" * 60)
    cmd = (
        f'ros2 action send_goal /tr/place_block '
        f'tr_robot_interfaces/action/PlaceBlock '
        f'"{{phase: \'{phase}\'}}" --feedback'
    )
    subprocess.run(cmd, shell=True)

def wait_enter(msg):
    """Jeda manual — user kontrol kapan lanjut."""
    print(f"\n⏸️  {msg}")
    input(">>> Tekan ENTER untuk lanjut...")

# ============================================================
# Mission Sequence
# ============================================================
def main():
    print("=" * 60)
    print("  ROBOCON 2027 — MISSION SCRIPT MANUAL (TR + BR)")
    print("=" * 60)
    print("⚠️  Pastikan semua terminal node sudah berjalan:")
    print("   - Webots (bringup.launch.py)")
    print("   - br_semi_auto + br_planner")
    print("   - semi_auto_controller + tr_planner")
    print()

    input(">>> Tekan ENTER untuk MEMULAI MISI...")

    # ============================================================
    # 1. BR Phase 1
    # ============================================================
    run_br_phase("1", "Navigasi ke titik awal + panggil TR")
    wait_enter("Cek BR di Webots — apakah sudah sampai & siap terima balok?")

    # ============================================================
    # 2. TR Phase 1
    # ============================================================
    run_tr_phase("1", "TR ambil balok & bawa ke BR")
    wait_enter("Cek TR di Webots — apakah balok sudah ditaruh di depan BR?")

    # ============================================================
    # 3. BR Phase 2
    # ============================================================
    run_br_phase("2", "BR ambil balok 1 & susun di zona build")
    wait_enter("Cek BR di Webots — apakah balok 1 sudah tersusun dengan rapi?")

    # ============================================================
    # 4. TR Phase 2
    # ============================================================
    run_tr_phase("2", "TR kembali ke HOME (siap untuk balok ke-2)")
    wait_enter("Cek TR di Webots — apakah sudah kembali ke posisi awal?")

    # ============================================================
    # 5. BR Phase 3
    # ============================================================
    run_br_phase("3", "BR kembali ke HOME (siap untuk balok ke-2)")
    wait_enter("Cek BR di Webots — apakah sudah di HOME & siap balok ke-2?")

    # ============================================================
    # 6. BR Phase 4
    # ============================================================
    run_br_phase("4", "BR ambil balok 2 & susun di level 2")
    wait_enter("Cek BR di Webots — apakah balok 2 sudah tersusun?")

    # ============================================================
    # SELESAI
    # ============================================================
    print("\n" + "=" * 60)
    print("  ✅ SEMUA RANGKAIAN MISI SELESAI (1-6)")
    print("=" * 60)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⛔ Misi dihentikan paksa oleh user (Ctrl+C).")
        sys.exit(1)