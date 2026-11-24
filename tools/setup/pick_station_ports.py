"""Select loopback ports on the notebook and Jetson without stopping listeners."""
import argparse
import errno
import http.client
import json
import socket


def available(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        try:
            listener.bind(('127.0.0.1', port))
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                return False
            raise
    return True


def station_matches(port, kit_id):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=.5)
    try:
        connection.request('GET', '/api/status')
        response = connection.getresponse()
        if response.status != 200:
            return False
        data = json.loads(response.read(1048576))
        return isinstance(data, dict) and data.get('mode') == 'station' and data.get('kit_id') == kit_id
    except (OSError, http.client.HTTPException, ValueError):
        return False
    finally:
        connection.close()


def select(ports, kit_id):
    free = []
    for port in ports:
        if available(port):
            free.append(port)
        elif station_matches(port, kit_id):
            return 'existing', [port]
    if not free:
        raise ValueError('사용 가능한 Pick Station 포트가 없습니다. 18768–18788 포트 사용을 확인해 주세요.')
    return 'available', free


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('side', choices=('local', 'remote'))
    parser.add_argument('kit_id')
    parser.add_argument('ports', type=int, nargs='*')
    args = parser.parse_args()
    ports = args.ports or list(range(18768, 18789))
    if any(not 1024 <= port <= 65535 for port in ports):
        parser.error('ports must be within 1024..65535')
    try:
        state, selected = select(ports, args.kit_id)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'웹 포트 확인 실패 · {exc}\n')
    print(state, *(selected[:1] if args.side == 'remote' else selected))


if __name__ == '__main__':
    main()
