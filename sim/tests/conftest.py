import shutil
import tempfile

import pytest


@pytest.fixture
def sock_dir():
    """A short directory for unix sockets: macOS limits their paths to 104
    bytes, and pytest's tmp_path under /var/folders/... is longer."""
    d = tempfile.mkdtemp(prefix="v5t", dir="/tmp")
    yield d
    shutil.rmtree(d, ignore_errors=True)
