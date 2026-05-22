#!/usr/bin/env python3
"""Small TCP relay for sharing a localhost HTTP proxy with a Quest headset."""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
from collections.abc import Iterable


BUFFER_SIZE = 64 * 1024


async def pipe(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    label: str,
) -> None:
    try:
        while True:
            data = await reader.read(BUFFER_SIZE)
            if not data:
                break
            writer.write(data)
            await writer.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        logging.debug("%s closed", label)
        writer.close()


async def relay_connection(
    client_reader: asyncio.StreamReader,
    client_writer: asyncio.StreamWriter,
    target_host: str,
    target_port: int,
    allowed_clients: set[str],
) -> None:
    peer = client_writer.get_extra_info("peername")
    peer_host = peer[0] if peer else "unknown"

    if allowed_clients and peer_host not in allowed_clients:
        logging.warning("Rejected connection from %s", peer_host)
        client_writer.close()
        await client_writer.wait_closed()
        return

    logging.info("Accepted connection from %s", peer_host)
    try:
        target_reader, target_writer = await asyncio.open_connection(
            target_host,
            target_port,
        )
    except OSError as exc:
        logging.error("Could not connect to target %s:%s: %s", target_host, target_port, exc)
        client_writer.close()
        await client_writer.wait_closed()
        return

    await asyncio.gather(
        pipe(client_reader, target_writer, f"{peer_host} -> proxy"),
        pipe(target_reader, client_writer, f"proxy -> {peer_host}"),
    )
    logging.info("Connection from %s closed", peer_host)


def parse_allowed_clients(values: Iterable[str]) -> set[str]:
    allowed: set[str] = set()
    for value in values:
        for item in value.split(","):
            item = item.strip()
            if item:
                allowed.add(item)
    return allowed


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Relay Quest LAN proxy traffic to a local-only HTTP proxy.",
    )
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=17890)
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=7890)
    parser.add_argument(
        "--allow",
        action="append",
        default=[],
        help="Allowed client IP. Can be passed multiple times or comma-separated.",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    allowed_clients = parse_allowed_clients(args.allow)

    server = await asyncio.start_server(
        lambda reader, writer: relay_connection(
            reader,
            writer,
            args.target_host,
            args.target_port,
            allowed_clients,
        ),
        args.listen_host,
        args.listen_port,
    )

    sockets = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    allow_text = ", ".join(sorted(allowed_clients)) if allowed_clients else "any client"
    logging.info(
        "Relay listening on %s, forwarding to %s:%s, allowed: %s",
        sockets,
        args.target_host,
        args.target_port,
        allow_text,
    )

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    async with server:
        await stop.wait()


if __name__ == "__main__":
    asyncio.run(main())
