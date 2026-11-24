"""A dead SSH listener must not prevent opening the station on another port."""
import errno
import json
from types import SimpleNamespace

import pytest

from tools.setup import pick_station_ports as ports


def test_dead_listener_and_other_station_are_skipped(monkeypatch):
    monkeypatch.setattr(ports, 'available', lambda port: port >= 18770)
    monkeypatch.setattr(ports, 'station_matches', lambda port, kit_id: False)
    assert ports.select([18768, 18769, 18770, 18771], 'kit') == ('available', [18770, 18771])


def test_matching_station_is_reused_even_after_a_free_port(monkeypatch):
    monkeypatch.setattr(ports, 'available', lambda port: port == 18768)
    monkeypatch.setattr(ports, 'station_matches', lambda port, kit_id: port == 18769 and kit_id == 'kit')
    assert ports.select([18768, 18769, 18770], 'kit') == ('existing', [18769])


@pytest.mark.parametrize('error', [errno.EADDRINUSE, errno.EPERM])
def test_bind_distinguishes_collision_from_socket_restrictions(monkeypatch, error):
    class Socket:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def bind(self, address):
            assert address == ('127.0.0.1', 18768)
            raise OSError(error, 'socket error')
    monkeypatch.setattr(ports.socket, 'socket', lambda *args: Socket())
    if error == errno.EADDRINUSE:
        assert not ports.available(18768)
    else:
        with pytest.raises(PermissionError):
            ports.available(18768)


@pytest.mark.parametrize('data,expected', [
    ({'mode': 'station', 'kit_id': 'kit'}, True),
    ({'mode': 'demo', 'kit_id': 'kit'}, False),
    ({'mode': 'station', 'kit_id': 'old'}, False),
    ('invalid json', False),
])
def test_only_same_release_station_can_be_reused(monkeypatch, data, expected):
    closed = []
    body = data if isinstance(data, str) else json.dumps(data)
    connection = SimpleNamespace(request=lambda *args: None,
        getresponse=lambda: SimpleNamespace(status=200, read=lambda size: body),
        close=lambda: closed.append(True))
    monkeypatch.setattr(ports.http.client, 'HTTPConnection', lambda *args, **kwargs: connection)
    assert ports.station_matches(18769, 'kit') is expected
    assert closed == [True]
