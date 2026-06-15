import logging
import time
from typing import Dict, Tuple
import websockets.sync.client
from ..openpi_client import msgpack_numpy
import builtins
from tool.vla_profiler import profile_span, profile_timeline

class WebsocketClientPolicy():
    """Implements the Policy interface by communicating with a server over websocket.

    See WebsocketPolicyServer for a corresponding server implementation.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8000) -> None:
        self._uri = f"ws://{host}:{port}"
        self._packer = msgpack_numpy.Packer()
        self._ws, self._server_metadata = self._wait_for_server()
        self.host = host
        self.port = port
        self.network_latency_tolerance = 2.0 # seconds

    def get_server_metadata(self) -> Dict:
        return self._server_metadata

    def _wait_for_server(self) -> Tuple[websockets.sync.client.ClientConnection, Dict]:
        logging.info(f"Waiting for server at {self._uri}...")
        while True:
            try:
                self.conn = websockets.sync.client.connect(self._uri, compression=None, max_size=None)
                metadata = msgpack_numpy.unpackb(self.conn.recv())
                return self.conn, metadata
            except ConnectionRefusedError:
                logging.info("Still waiting for server...")
                time.sleep(5)

    #@profile_timeline(cat="network",name="network.infer",min_duration_ms=0.1,extra=lambda self, obs: {"host": self.host, "port": self.port}, )
    def infer(self, obs: Dict) -> Dict:  # noqa: UP006

        if self._ws.socket._closed or self._ws.socket.fileno() == -1:
            self._ws, self._server_metadata = self._wait_for_server()

        data = self._packer.pack(obs)
        self._ws.send(data)

        response = self.receive()
                
        if isinstance(response, str):
            # we're expecting bytes; if the server sends a string, it's an error.
            raise RuntimeError(f"Error in inference server:\n{response}")
        return msgpack_numpy.unpackb(response)
    
    #@profile_timeline(cat="network",name="network.receive",min_duration_ms=0.1 )
    def receive(self):
        try:
            response = self._ws.recv(timeout=self.network_latency_tolerance)
        except builtins.TimeoutError:
            self._ws.close()
            response = "模型" + str(self.network_latency_tolerance) +"秒内未返回结果"
        except Exception as e:
            response = "host:" + self.host+ " port:" + str(self.port)+ f" {type(e).__name__}: {str(e)}"

        return response

    def reset(self) -> None:
        pass
