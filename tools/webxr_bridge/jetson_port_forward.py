#!/usr/bin/env python3
"""Forward local TCP ports to the Jetson over SSH."""

from __future__ import annotations

import argparse
import getpass
import os
import select
import socketserver
import threading
from dataclasses import dataclass

import paramiko


BUFFER_SIZE = 16 * 1024


@dataclass(frozen=True)
class ForwardSpec:
    local_port: int
    remote_host: str
    remote_port: int


class ForwardHandler(socketserver.BaseRequestHandler):
    transport: paramiko.Transport
    remote_host: str
    remote_port: int

    def handle(self) -> None:
        try:
            channel = self.transport.open_channel(
                "direct-tcpip",
                (self.remote_host, self.remote_port),
                self.request.getpeername(),
            )
        except Exception as exc:
            print(f"open_channel failed: {exc}", flush=True)
            return

        if channel is None:
            print("open_channel returned None", flush=True)
            return

        try:
            while True:
                readable, _, _ = select.select([self.request, channel], [], [])
                if self.request in readable:
                    data = self.request.recv(BUFFER_SIZE)
                    if not data:
                        break
                    channel.sendall(data)
                if channel in readable:
                    data = channel.recv(BUFFER_SIZE)
                    if not data:
                        break
                    self.request.sendall(data)
        finally:
            channel.close()
            self.request.close()


def parse_forward(value: str) -> ForwardSpec:
    parts = value.split(":")
    if len(parts) == 2:
        return ForwardSpec(int(parts[0]), "127.0.0.1", int(parts[1]))
    if len(parts) == 3:
        return ForwardSpec(int(parts[0]), parts[1], int(parts[2]))
    raise argparse.ArgumentTypeError("Expected local_port:remote_port or local_port:host:remote_port")


def serve_forward(transport: paramiko.Transport, spec: ForwardSpec) -> socketserver.ThreadingTCPServer:
    class Handler(ForwardHandler):
        pass

    Handler.transport = transport
    Handler.remote_host = spec.remote_host
    Handler.remote_port = spec.remote_port

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    server = socketserver.ThreadingTCPServer(("127.0.0.1", spec.local_port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(
        f"127.0.0.1:{spec.local_port} -> {spec.remote_host}:{spec.remote_port}",
        flush=True,
    )
    return server


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.31.230")
    parser.add_argument("--user", default="yuanyou")
    parser.add_argument("--password-env", default="JETSON_PASS")
    parser.add_argument(
        "--forward",
        action="append",
        type=parse_forward,
        default=[],
        help="Port rule, e.g. 8088:8088 or 8088:127.0.0.1:8088",
    )
    args = parser.parse_args()

    password = os.environ.get(args.password_env)
    if password is None:
        password = getpass.getpass(f"{args.user}@{args.host} password: ")

    forwards = args.forward or [
        ForwardSpec(8088, "127.0.0.1", 8088),
        ForwardSpec(8765, "127.0.0.1", 8765),
    ]

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=args.host,
        username=args.user,
        password=password,
        look_for_keys=False,
        allow_agent=False,
    )

    transport = client.get_transport()
    if transport is None:
        raise RuntimeError("SSH transport is not available")
    transport.set_keepalive(15)

    servers = [serve_forward(transport, spec) for spec in forwards]
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        for server in servers:
            server.shutdown()
        client.close()


if __name__ == "__main__":
    main()
