import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from model_agent.openpi_client import action_chunk_broker
from model_agent.openpi_client import websocket_client_policy as _websocket_client_policy
import logging

class ModelAgent():
    def __init__(self,in_host,in_port,in_action_horizon):
        self.ws_client = _websocket_client_policy.WebsocketClientPolicy(
            host=in_host,
            port=in_port,
        )
        logging.info(f"Server metadata: {self.ws_client.get_server_metadata()}")
        self.policy = action_chunk_broker.ActionChunkBroker(
            policy=self.ws_client,
            action_horizon=in_action_horizon,
        )

    def infer(self,obs):
        response = self.policy.infer(obs)
        return response
        
