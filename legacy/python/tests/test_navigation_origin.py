"""Port forwarding changes the browser port, not its loopback origin trust."""
import pytest
from navigation.server import allowed_origin


@pytest.mark.parametrize('origin',[
    None,'http://localhost:50995','http://127.0.0.1:50995',
    'http://localhost:8765','http://127.0.0.1:8765','http://[::1]:50995',
    'https://localhost:50995','http://localhost',
])
def test_local_forwarded_origins_allowed(origin):
    assert allowed_origin(origin)


@pytest.mark.parametrize('origin',[
    'null','', 'http://example.com:50995','http://localhost.example.com:50995',
    'http://localhost@evil.example:50995','http://evil.example@localhost:50995',
    'http://localhost:bad','http://localhost:65536','file://localhost',
    'http://localhost:50995/path','http://localhost:50995?x=1',
    'http://localhost:50995#fragment','http://[::1',
])
def test_external_and_malformed_origins_rejected(origin):
    assert not allowed_origin(origin)
