#!/usr/bin/env python3
"""V2X E2E Simulator web viewer: serves the page and streams topic health + map layers over a WebSocket.

Run through ../run.sh, which sources ROS 2 and the simulator messages first.
"""
import argparse
import asyncio
import json
import logging
import os
import threading
import time

import rclpy
import yaml
from aiohttp import WSMsgType, web
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.serialization import deserialize_message

from decoders import Decoders
from geo import MapFrame
from monitor import TopicMonitor
from settings import Settings

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAYER_HZ = 10.0
HEALTH_PERIOD_S = 1.0

log = logging.getLogger('web-viewer')


class Viewer:
    def __init__(self, map_dir):
        with open(os.path.join(map_dir, 'lanelet2_meta.json')) as f:
            self.meta = json.load(f)
        with open(os.path.join(ROOT, 'config', 'traffic_lights.yaml'), encoding='utf-8') as f:
            self.traffic_lights = yaml.safe_load(f)
        with open(os.path.join(ROOT, 'config', 'sensors.yaml'), encoding='utf-8') as f:
            self.sensors = yaml.safe_load(f)['sensors']
        for sensor in self.sensors:     # YAML hex IDs arrive as ints; the page shows them as hex
            sensor['rsu_ids'] = [f'0x{i:x}' if isinstance(i, int) else str(i) for i in sensor.get('rsu_ids') or []]
        self.map_dir = map_dir
        self.settings = None
        self.decoders = Decoders(MapFrame(self.meta['utm_epsg'], self.meta['map_to_utm_offset']),
                                 self.meta, self.traffic_lights)
        self.monitor = TopicMonitor(renderable=lambda t: Decoders.layer_of(t) is not None)
        self.clients = {}            # websocket -> set of enabled topics
        self.decoded_seq = {}        # topic -> seq of the last decoded message
        self.decode_errors = {}

    # ------------------------------------------------------------------ HTTP
    async def index(self, request):
        return web.FileResponse(os.path.join(ROOT, 'web', 'index.html'))

    async def config(self, request):
        return web.json_response({
            'bounds': self.meta['bounds'],
            'map_source': self.meta['source'],
            'intersections': self.traffic_lights['intersections'],
            # RSU sensors of the default scenario of config/ (what the simulator builds); config/sensors.yaml without one
            'sensors': (self.settings.viewer_sensors() if self.settings else None) or self.sensors,
            'ego_topic': self.decoders.ego_topic,
            'utm_epsg': self.meta['utm_epsg'],
            'map_to_utm_offset': self.meta['map_to_utm_offset'],
        })

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=10, compress=True)
        await ws.prepare(request)
        self.clients[ws] = set()
        # Force a full resend of the layers to the new client.
        self.decoded_seq.clear()
        await ws.send_json({'type': 'health', 'topics': self.monitor.snapshot()})
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get('type') == 'enable':
                        self.clients[ws] = set(data.get('topics', []))
                        self.decoded_seq.clear()
        finally:
            self.clients.pop(ws, None)
        return ws

    # ------------------------------------------------------------------ streaming
    async def broadcast(self, payload):
        for ws in list(self.clients):
            if not ws.closed:
                try:
                    await ws.send_json(payload)
                except ConnectionResetError:
                    pass

    async def health_loop(self):
        while True:
            try:
                if self.clients:
                    await self.broadcast({'type': 'health', 'topics': self.monitor.snapshot(),
                                          'decode_errors': self.decode_errors})
            except Exception:  # never let the stream die
                log.exception('health update failed')
            await asyncio.sleep(HEALTH_PERIOD_S)

    async def layer_loop(self):
        while True:
            start = time.monotonic()
            try:
                await self.push_layers()
            except Exception:  # never let the stream die
                log.exception('layer update failed')
            await asyncio.sleep(max(0.0, 1.0 / LAYER_HZ - (time.monotonic() - start)))

    async def push_layers(self):
        """Decode the topics the clients show and send what changed since the last push."""
        if self.clients:
            enabled = set().union(*self.clients.values()) | {self.decoders.ego_topic}
            updates = {}
            latest = self.monitor.latest_messages()
            # Poses first: ego-relative objects need the current ego pose.
            latest.sort(key=lambda m: Decoders.layer_of(m[1]) != 'poses')
            for name, type_name, msg_class, data, seq in latest:
                if name not in enabled or Decoders.layer_of(type_name) is None:
                    continue
                if self.decoded_seq.get(name) == seq:
                    continue
                self.decoded_seq[name] = seq
                try:
                    updates[name] = self.decoders.decode(name, type_name, deserialize_message(data, msg_class))
                    self.decode_errors.pop(name, None)
                except Exception as e:  # keep streaming the other topics
                    self.decode_errors[name] = f'{e.__class__.__name__}: {e}'
                    log.exception('decoding %s failed', name)
            if updates:
                await self.broadcast({'type': 'layers', 'layers': updates})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='bind address (default: localhost only)')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--map-dir', default=os.path.join(ROOT, 'maps'))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')

    rclpy.init()
    viewer = Viewer(args.map_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(viewer.monitor)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()

    app = web.Application()
    app.router.add_get('/', viewer.index)
    app.router.add_get('/api/config', viewer.config)
    app.router.add_get('/ws', viewer.websocket)
    app.router.add_static('/map/', args.map_dir)
    app.router.add_static('/static/', os.path.join(ROOT, 'web'))
    viewer.settings = Settings(viewer.meta['utm_epsg'], viewer.meta['map_to_utm_offset'])
    viewer.settings.add_routes(app)

    async def start_loops(app):
        app['tasks'] = [asyncio.create_task(viewer.health_loop()), asyncio.create_task(viewer.layer_loop())]

    async def stop(app):
        for task in app['tasks']:
            task.cancel()
        executor.shutdown()
        viewer.monitor.destroy_node()
        rclpy.try_shutdown()

    app.on_startup.append(start_loops)
    app.on_cleanup.append(stop)
    log.info('V2X web viewer on http://%s:%d', args.host, args.port)
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == '__main__':
    main()
