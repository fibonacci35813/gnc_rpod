#!/usr/bin/env python3

"""Basic check for bsk and to see if it is available in this environment or not."""

from __future__ import annotations

def main() -> None:
    try:
        import Basilisk
        from Basilisk.utilities import macros
    except ImportError as exc:
        raise SystemExit(
            "Basilisk import failed. \n"
            "Install it with: pip install bsk"
            "Or activate the Basilisk environment if you have it installed locally."
        ) from exc
    print("Basilisk import: OK")
    print(f"Basilisk version: {Basilisk.__path__[0]}")
    print(f"1 second in nanoseconds: {macros.sec2nano(1.0)}")


if __name__ == "__main__":    
    main()
