#!/usr/bin/env python3
"""Check Markdown math patterns that GitHub commonly mis-renders."""

from __future__ import annotations

import re
import sys
from pathlib import Path


DOCS_DIR = Path("docs")
STANDALONE_OPERATORS = {"=", "+", "-"}
CJK_PUNCT_INLINE_MATH = re.compile(
    r"[，。；：！？、]\$[^$\n]+\$"
)

BROKEN_LATEX_COMMAND = re.compile(
    r"(?<![A-Za-z\\])"
    r"(?:boxed|begin|end|mathbf|mathrm|operatorname|frac|sqrt|text|cdot|"
    r"times|rightarrow|leftrightarrow|lVert|rVert|psi|kappa|Delta|"
    r"sin|cos|tan)"
    r"(?=\\b|\\{)"
)

def check_file(path: Path) -> list[str]:
    errors: list[str] = []
    in_code_fence = False
    in_display_math = False
    display_start_line: int | None = None

    content = path.read_text(encoding="utf-8")

    for index, character in enumerate(content):
        code_point = ord(character)
        if (
            code_point == 0x7F
            or (code_point < 0x20 and character not in "\t\n\r")
        ):
            line_number = content.count("\n", 0, index) + 1
            errors.append(
                f"{path}:{line_number}: contains ASCII control character "
                f"0x{code_point:02x}; possible broken LaTeX escaping"
            )

    lines = content.splitlines()

    for line_number, line in enumerate(lines, start=1):
        stripped = line.strip()

        if stripped.startswith("```"):
            in_code_fence = not in_code_fence
            continue

        if in_code_fence:
            continue

        if "$$$" in line:
            errors.append(
                f"{path}:{line_number}: suspicious '$$$' sequence"
            )

        if "$$" in line:
            if stripped != "$$":
                errors.append(
                    f"{path}:{line_number}: "
                    "display-math delimiter '$$' must be on its own line"
                )
                continue

            in_display_math = not in_display_math
            if in_display_math:
                display_start_line = line_number
            else:
                display_start_line = None
            continue

        if in_display_math and stripped in STANDALONE_OPERATORS:
            errors.append(
                f"{path}:{line_number}: "
                f"standalone operator '{stripped}' inside display math; "
                "keep it on the same TeX line or use an aligned block"
            )

        if in_display_math:
            broken_command = BROKEN_LATEX_COMMAND.search(line)
            if broken_command:
                errors.append(
                    f"{path}:{line_number}: suspicious bare LaTeX token "
                    f"'{broken_command.group()}'; possible missing backslash"
                )

        if (
            not in_display_math
            and CJK_PUNCT_INLINE_MATH.search(line)
        ):
            errors.append(
                f"{path}:{line_number}: "
                "inline math immediately after CJK punctuation can render "
                "badly on GitHub; use backticks/plain text for simple symbols"
            )

    if in_display_math:
        errors.append(
            f"{path}:{display_start_line}: unclosed display-math block"
        )

    if in_code_fence:
        errors.append(
            f"{path}: unclosed fenced code block"
        )

    return errors


def main() -> int:
    paths = sorted(DOCS_DIR.rglob("*.md"))

    if not paths:
        print("PASS: no Markdown files found under docs/.")
        return 0

    errors: list[str] = []

    for path in paths:
        errors.extend(check_file(path))

    if errors:
        print("Markdown math check failed:")
        for error in errors:
            print(f"  - {error}")
        return 1

    print(
        f"PASS: checked {len(paths)} Markdown files; "
        "GitHub math patterns look safe."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
