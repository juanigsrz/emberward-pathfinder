import textwrap

import pytest


@pytest.fixture
def make_map(tmp_path):
    """Write a dedented map string to a temp file, return its path."""
    def _make(text, name="m.txt"):
        f = tmp_path / name
        f.write_text(textwrap.dedent(text).strip() + "\n")
        return str(f)
    return _make
