# Temporary tool changes to revert later

## capture_preset.py: DTR asserted (2026-10-15)

- **Change:** `open_port()` sets `port.dtr = True`. The original is `port.dtr = False`.
- **Why:** on 2026-10-15 the Videomancer shell stopped answering any command while DTR was low. Every capture timed out on `program info`. A raw probe with DTR high got replies at once. It started after a capture was interrupted by loading another vmprog, and a WSL reboot didn't clear it. The Videomancer itself had not been power-cycled yet.
- **Approved by:** Ron, as a temporary fix during the community defaults pass.
- **To revert:** power-cycle the Videomancer, set `port.dtr = False` back in `open_port()`, and run a `--dry-run` capture. If it answers, keep the original. If it still times out, the firmware really does need DTR, and this change should become permanent.
