#!/usr/bin/env python3
import sys
from piece_dataset.app import main

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'[오류] {exc}', file=sys.stderr)
        sys.exit(1)
